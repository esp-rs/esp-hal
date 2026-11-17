//! AES-GCM, on chips with GCM in their AES-DMA hardware.

use core::{hint::black_box, ptr::NonNull};

use crate::{
    aes::{
        AES_WORK_QUEUE,
        AesOperation,
        BLOCK_SIZE,
        Error,
        Key,
        Operation,
        cipher_modes::{CryptoBuffers, UnsafeCryptoBuffers},
    },
    work_queue::{Handle, Status, WorkQueueFrontend},
};

/// The longest additional data one operation takes: one DMA descriptor.
const MAX_AAD_LEN: usize = 4_080;

/// The longest text one operation takes: five DMA descriptors, which cover TLS 1.3's largest
/// record of 16,385 bytes.
const MAX_TEXT_LEN: usize = 5 * 4_080;

/// The state of one GCM operation, which the backend reads and writes while it owns the item.
pub(crate) struct GcmState {
    pub(super) j0: [u8; BLOCK_SIZE],
    pub(super) aad: NonNull<[u8]>,
    /// The tag the backend writes when it encrypts, and checks when it decrypts.
    pub(super) tag: NonNull<[u8; 16]>,
    pub(super) result: Result<(), GcmError>,
}

// Safety: the pointers are only dereferenced while the work queue owns the item, and the handle
// borrows the data they point to for that time.
unsafe impl Send for GcmState {}
unsafe impl Sync for GcmState {}

impl GcmState {
    /// Ends the operation on a backend that cannot run GCM.
    pub(crate) fn set_unsupported(&mut self) {
        self.result = Err(GcmError::Unsupported);
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

    /// The running backend cannot process AES-GCM. Only
    /// [`AesDmaBackend`](crate::aes::dma::AesDmaBackend) can.
    Unsupported,

    /// [`AesGcmHandle::cancel`] cancelled the operation before it started. When
    /// [`AesGcmHandle::wait`] or [`AesGcmHandle::wait_blocking`] returns this for a decryption,
    /// the output is zeroed.
    Cancelled,
}

/// An AES-GCM work queue user.
///
/// Operations run on [`AesDmaBackend`](crate::aes::dma::AesDmaBackend), which must be started. On
/// [`AesBackend`](crate::aes::AesBackend), every operation ends with [`GcmError::Unsupported`]. A
/// nonce must never repeat under one key, because a repeated nonce breaks the authentication.
#[instability::unstable]
pub struct AesGcmContext {
    state: GcmState,
    frontend: WorkQueueFrontend<AesOperation>,
}

impl AesGcmContext {
    /// Creates a new context that encrypts and decrypts with `key`.
    #[instability::unstable]
    pub fn new(key: impl Into<Key>) -> Self {
        Self {
            state: GcmState {
                j0: [0; BLOCK_SIZE],
                aad: NonNull::slice_from_raw_parts(NonNull::dangling(), 0),
                tag: NonNull::dangling(),
                result: Ok(()),
            },
            frontend: WorkQueueFrontend::new(AesOperation {
                mode: Operation::Encrypt,
                cipher_mode: NonNull::dangling(),
                buffers: unsafe { CryptoBuffers::new_in_place(&mut []).into_inner() },
                key: key.into(),
                gcm: None,
            }),
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
    /// - [`Error::AadNotInRam`] when the DMA cannot read `aad`, for example from flash.
    /// - [`Error::BuffersNotEqual`] when `input` and `output` differ in length.
    #[cfg_attr(
        esp32s2,
        doc = "- [`Error::OutputInPsram`] when `output` is in PSRAM. `input` and `aad` can be in PSRAM."
    )]
    #[instability::unstable]
    pub fn encrypt<'t>(
        &'t mut self,
        nonce: &[u8; 12],
        aad: &'t [u8],
        input: &'t [u8],
        output: &'t mut [u8],
        tag: &'t mut [u8; 16],
    ) -> Result<AesGcmHandle<'t>, Error> {
        validate(aad, input.len(), output)?;
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
        validate(aad, buffer.len(), buffer)?;
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
        validate(aad, input.len(), output)?;
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
        validate(aad, buffer.len(), buffer)?;
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
        // A 96-bit nonce gives J0 = nonce || 0x00000001, without GHASH.
        self.state.j0[..12].copy_from_slice(nonce);
        self.state.j0[12..].copy_from_slice(&1u32.to_be_bytes());
        self.state.aad = NonNull::from(aad);
        self.state.tag = tag;
        self.state.result = Ok(());
        let state = NonNull::from(&mut self.state);

        let data = self.frontend.data_mut();
        data.mode = mode;
        data.buffers = buffers;
        data.gcm = Some(state);

        AesGcmHandle {
            handle: self.frontend.post(&AES_WORK_QUEUE),
            state,
            output: buffers.output,
            decrypt: mode == Operation::Decrypt,
        }
    }
}

fn validate(aad: &[u8], text_len: usize, output: &[u8]) -> Result<(), Error> {
    if text_len == 0 || text_len > MAX_TEXT_LEN || aad.len() > MAX_AAD_LEN {
        return Err(Error::IncorrectBufferLength);
    }

    // The backend copies a text in flash to the output first. Only the text has an output to be
    // copied into, so the DMA reads the additional data where it is.
    if !aad.is_empty() && !crate::soc::is_valid_memory_address(aad.as_ptr() as usize) {
        return Err(Error::AadNotInRam);
    }

    // The DMA of the ESP32-S2 fails to write a RAM descriptor that follows a PSRAM descriptor.
    // The last bytes of an AES-GCM output are written to the bounce block in RAM, so the check
    // must refuse an output in PSRAM.
    cfg_select! {
        esp32s2 => {
            if crate::psram::psram_range().contains(&(output.as_ptr() as usize)) {
                return Err(Error::OutputInPsram);
            }
        }
        _ => {
            let _ = output;
        }
    }

    Ok(())
}

/// The handle to a pending AES-GCM operation.
///
/// Dropping it before the operation completes cancels the operation.
#[instability::unstable]
pub struct AesGcmHandle<'t> {
    handle: Handle<'t, AesOperation>,
    state: NonNull<GcmState>,
    output: NonNull<[u8]>,
    decrypt: bool,
}

impl AesGcmHandle<'_> {
    /// Polls the operation. Returns whether it has completed.
    #[instability::unstable]
    pub fn poll(&mut self) -> bool {
        self.handle.poll()
    }

    /// Polls the operation to completion, by busy-looping, and returns its result.
    #[instability::unstable]
    pub fn wait_blocking(self) -> Result<(), GcmError> {
        let Self {
            handle,
            state,
            output,
            decrypt,
        } = self;
        result(handle.wait_blocking(), state, output, decrypt)
    }

    /// Waits until the operation completes, and returns its result.
    #[instability::unstable]
    pub async fn wait(&mut self) -> Result<(), GcmError> {
        let status = self.handle.wait().await;
        result(status, self.state, self.output, self.decrypt)
    }

    /// Cancels the operation, and waits until it leaves the work queue.
    ///
    /// An operation that has not started ends with [`GcmError::Cancelled`]. One that the backend
    /// has started runs to completion, because the hardware cannot stop part way, and keeps its
    /// result. [`Self::wait`] or [`Self::wait_blocking`] then returns the result.
    #[instability::unstable]
    pub fn cancel(&mut self) -> impl Future<Output = ()> {
        self.handle.cancel()
    }
}

fn result(
    status: Status,
    state: NonNull<GcmState>,
    output: NonNull<[u8]>,
    decrypt: bool,
) -> Result<(), GcmError> {
    match status {
        // The backend writes the result before it completes the item.
        Status::Completed => unsafe { state.as_ref() }.result,
        Status::Cancelled => {
            // Plaintext that no tag has verified must not reach the caller.
            if decrypt {
                unsafe { zero(output) };
            }
            Err(GcmError::Cancelled)
        }
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
