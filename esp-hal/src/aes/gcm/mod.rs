//! AES-GCM: in hardware on the DMA backend of chips with a GCM mode where the hardware can take the
//! buffers, and otherwise as AES-CTR on the running backend with GHASH on the CPU.

use core::{hint::black_box, ptr::NonNull};

#[cfg(aes_dma_mode_gcm)]
use crate::aes::BLOCK_SIZE;
use crate::aes::{
    Error,
    Key,
    Operation,
    cipher_modes::{CryptoBuffers, UnsafeCryptoBuffers},
};

mod operation;

/// The longest additional data one operation takes: one DMA descriptor.
const MAX_AAD_LEN: usize = 4_080;

/// The longest text one operation takes: five DMA descriptors, which cover TLS 1.3's largest
/// record of 16,385 bytes.
const MAX_TEXT_LEN: usize = 5 * 4_080;

/// The state of the hardware item of a GCM operation, which the backend reads and writes while it
/// owns the item.
#[cfg(aes_dma_mode_gcm)]
pub(crate) struct GcmState {
    pub(super) j0: [u8; BLOCK_SIZE],
    pub(super) aad: NonNull<[u8]>,
    /// The tag the backend writes when it encrypts, and checks when it decrypts.
    pub(super) tag: NonNull<[u8; 16]>,
    pub(super) result: Result<(), GcmError>,
    /// Whether a backend that cannot run the item refused it.
    pub(super) refused: bool,
}

// Safety: the pointers are only dereferenced while the work queue owns the item, and the handle
// borrows the data they point to for that time.
#[cfg(aes_dma_mode_gcm)]
unsafe impl Send for GcmState {}
#[cfg(aes_dma_mode_gcm)]
unsafe impl Sync for GcmState {}

#[cfg(aes_dma_mode_gcm)]
impl GcmState {
    /// Ends the item on a backend that cannot run it, which leaves the buffers untouched. The
    /// operation then runs as AES-CTR steps.
    pub(crate) fn refuse(&mut self) {
        self.refused = true;
    }
}

/// An error that ends an AES-GCM operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum GcmError {
    /// The authentication tag does not match. The output is zeroed.
    TagMismatch,

    /// [`AesGcmHandle::cancel`] cancelled the operation. An encryption's output can be partly
    /// written. When [`AesGcmHandle::wait`] or [`AesGcmHandle::wait_blocking`] returns this for a
    /// decryption, the output is zeroed.
    Cancelled,
}

/// An AES-GCM work queue user.
///
/// A nonce must never repeat under one key, because a repeated nonce breaks the authentication.
#[doc = cfg_select! {
    aes_dma_mode_gcm => {
        "[`AesDmaBackend`](crate::aes::dma::AesDmaBackend) runs an operation in hardware when the hardware can take its buffers. Otherwise, and on [`AesBackend`](crate::aes::AesBackend), the backend runs AES-CTR, and GHASH runs on the CPU in [`AesGcmHandle::poll`], [`AesGcmHandle::wait`] and [`AesGcmHandle::wait_blocking`]. The context computes the hash subkey on the first operation that runs this way, and keeps it until it is dropped."
    }
    _ => {
        "The backend runs AES-CTR, and GHASH runs on the CPU in [`AesGcmHandle::poll`], [`AesGcmHandle::wait`] and [`AesGcmHandle::wait_blocking`]. The context computes the hash subkey on its first operation, and keeps it until it is dropped."
    }
}]
#[instability::unstable]
pub struct AesGcmContext {
    inner: operation::Context,
}

impl AesGcmContext {
    /// Creates a new context that encrypts and decrypts with `key`.
    #[instability::unstable]
    pub fn new(key: impl Into<Key>) -> Self {
        Self {
            inner: operation::Context::new(key.into()),
        }
    }

    /// Starts encrypting `input` into `output`, and computing the tag over `aad` and the
    /// ciphertext.
    ///
    /// The tag is written to `tag` when the operation completes. The nonce must never repeat
    /// under this context's key.
    ///
    /// Dropping the returned handle before the operation completes cancels the operation.
    ///
    /// # Errors
    ///
    /// - [`Error::IncorrectBufferLength`] when `input` is empty or longer than 20,400 bytes, or
    ///   `aad` is longer than 4,080 bytes.
    /// - [`Error::BuffersNotEqual`] when `input` and `output` differ in length.
    #[instability::unstable]
    pub fn encrypt<'t>(
        &'t mut self,
        nonce: &[u8; 12],
        aad: &'t [u8],
        input: &'t [u8],
        output: &'t mut [u8],
        tag: &'t mut [u8; 16],
    ) -> Result<AesGcmHandle<'t>, Error> {
        validate(aad, input.len())?;
        let buffers = unsafe { CryptoBuffers::new(input, output)?.into_inner() };
        Ok(self.post(Operation::Encrypt, nonce, aad, buffers, NonNull::from(tag)))
    }

    /// Starts encrypting `buffer` in place, and computing the tag over `aad` and the
    /// ciphertext.
    ///
    /// Operates as [`Self::encrypt`] does.
    ///
    /// # Errors
    ///
    /// As [`Self::encrypt`].
    #[instability::unstable]
    pub fn encrypt_in_place<'t>(
        &'t mut self,
        nonce: &[u8; 12],
        aad: &'t [u8],
        buffer: &'t mut [u8],
        tag: &'t mut [u8; 16],
    ) -> Result<AesGcmHandle<'t>, Error> {
        validate(aad, buffer.len())?;
        let buffers = unsafe { CryptoBuffers::new_in_place(buffer).into_inner() };
        Ok(self.post(Operation::Encrypt, nonce, aad, buffers, NonNull::from(tag)))
    }

    /// Starts decrypting `input` into `output`, and checking `tag` over `aad` and `input`.
    ///
    /// When the tag does not match, the operation ends with [`GcmError::TagMismatch`] and
    /// `output` is zeroed.
    ///
    /// # Errors
    ///
    /// As [`Self::encrypt`].
    #[instability::unstable]
    pub fn decrypt<'t>(
        &'t mut self,
        nonce: &[u8; 12],
        aad: &'t [u8],
        input: &'t [u8],
        output: &'t mut [u8],
        tag: &'t [u8; 16],
    ) -> Result<AesGcmHandle<'t>, Error> {
        validate(aad, input.len())?;
        let buffers = unsafe { CryptoBuffers::new(input, output)?.into_inner() };
        Ok(self.post(Operation::Decrypt, nonce, aad, buffers, NonNull::from(tag)))
    }

    /// Starts decrypting `buffer` in place, and checking `tag` over `aad` and `buffer`.
    ///
    /// Operates as [`Self::decrypt`] does.
    ///
    /// # Errors
    ///
    /// As [`Self::encrypt`].
    #[instability::unstable]
    pub fn decrypt_in_place<'t>(
        &'t mut self,
        nonce: &[u8; 12],
        aad: &'t [u8],
        buffer: &'t mut [u8],
        tag: &'t [u8; 16],
    ) -> Result<AesGcmHandle<'t>, Error> {
        validate(aad, buffer.len())?;
        let buffers = unsafe { CryptoBuffers::new_in_place(buffer).into_inner() };
        Ok(self.post(Operation::Decrypt, nonce, aad, buffers, NonNull::from(tag)))
    }

    fn post<'t>(
        &'t mut self,
        mode: Operation,
        nonce: &[u8; 12],
        aad: &'t [u8],
        buffers: UnsafeCryptoBuffers,
        tag: NonNull<[u8; 16]>,
    ) -> AesGcmHandle<'t> {
        AesGcmHandle {
            inner: self.inner.post(mode, nonce, aad, buffers, tag),
        }
    }
}

fn validate(aad: &[u8], text_len: usize) -> Result<(), Error> {
    if text_len == 0 || text_len > MAX_TEXT_LEN || aad.len() > MAX_AAD_LEN {
        return Err(Error::IncorrectBufferLength);
    }
    Ok(())
}

/// The handle to a pending AES-GCM operation.
///
/// Dropping it before the operation completes cancels the operation.
#[instability::unstable]
pub struct AesGcmHandle<'t> {
    inner: operation::Handle<'t>,
}

impl AesGcmHandle<'_> {
    /// Polls the operation. Returns whether it has completed.
    #[instability::unstable]
    pub fn poll(&mut self) -> bool {
        self.inner.poll()
    }

    /// Polls the operation to completion, by busy-looping, and returns its result.
    #[instability::unstable]
    pub fn wait_blocking(self) -> Result<(), GcmError> {
        self.inner.wait_blocking()
    }

    /// Waits until the operation completes, and returns its result.
    #[instability::unstable]
    pub async fn wait(&mut self) -> Result<(), GcmError> {
        self.inner.wait().await
    }

    /// Cancels the operation, and waits until it leaves the work queue.
    ///
    /// After it returns, [`Self::wait`] or [`Self::wait_blocking`] gives the operation's result.
    #[doc = cfg_select! {
        aes_dma_mode_gcm => {
            "An operation that has not started ends with [`GcmError::Cancelled`]. Once the hardware has started an operation, it runs to completion, because the hardware cannot stop part way, and keeps its result. Any other operation that has not ended stops after the step the backend is processing, and ends with [`GcmError::Cancelled`]."
        }
        _ => {
            "An operation that has not ended stops after the step the backend is processing, and ends with [`GcmError::Cancelled`]."
        }
    }]
    #[instability::unstable]
    pub fn cancel(&mut self) -> impl Future<Output = ()> {
        self.inner.cancel()
    }
}

pub(super) unsafe fn zero(buffer: NonNull<[u8]>) {
    unsafe { buffer.cast::<u8>().write_bytes(0, buffer.len()) };
}

/// Compares two tags in constant time.
///
/// It must not return early: the time it takes must not depend on where the tags differ.
pub(super) fn tags_equal(a: &[u8; 16], b: &[u8; 16]) -> bool {
    let mut difference = 0u8;
    for (x, y) in a.iter().zip(b) {
        difference |= black_box(x ^ y);
    }
    black_box(difference) == 0
}
