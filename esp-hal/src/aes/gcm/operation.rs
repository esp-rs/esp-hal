//! An AES-GCM operation.
//!
//! On a chip with GCM in hardware, the operation goes to the work queue first as one item, which
//! [`AesDmaBackend`](crate::aes::dma::AesDmaBackend) runs in hardware. A backend that cannot run
//! the item refuses it before it touches the buffers: [`AesBackend`](crate::aes::AesBackend)
//! always, and the DMA backend when the hardware cannot take the buffers. The operation then runs
//! as AES-CTR on the backend, with GHASH on the CPU, as every operation does on the other chips.
//!
//! That way is a chain of work queue steps. The first step encrypts J0, which masks the tag,
//! and on the context's first operation also the zero block, which gives the hash subkey H. Then
//! CTR steps of up to 4,080 bytes each produce the text, starting at counter 2.
//!
//! GHASH runs in [`Handle::poll`], [`Handle::wait`] and [`Handle::wait_blocking`], never in the
//! AES interrupt, and hashes one chunk while the backend processes the next. An encryption hashes
//! a chunk's ciphertext after its step completes. A decryption hashes a chunk's ciphertext before
//! its step is posted, because an in-place step overwrites it with plaintext.

use core::{marker::PhantomData, ptr::NonNull};

use ghash::{GHash, universal_hash::UniversalHash};

#[cfg(aes_dma_mode_gcm)]
use super::GcmState;
use super::{GcmError, tags_equal, zero};
use crate::{
    aes::{
        AES_WORK_QUEUE,
        AesOperation,
        BLOCK_SIZE,
        CipherState,
        Key,
        Operation,
        cipher_modes::{CryptoBuffers, Ctr, Ecb, UnsafeCryptoBuffers},
    },
    work_queue::{self, Status, WorkQueueFrontend},
};

/// The most text one CTR step takes.
const CHUNK: usize = 4_080;

pub(super) struct Context {
    /// GHASH keyed with H, from the end of the first operation's first step.
    ghash: Option<GHash>,
    cipher: CipherState,
    /// The blocks the first step encrypts in place: J0, after the zero block until H is known.
    blocks: [u8; 2 * BLOCK_SIZE],
    /// The hardware item's state, which the backend reads and writes while it owns the item.
    #[cfg(aes_dma_mode_gcm)]
    state: GcmState,
    frontend: WorkQueueFrontend<AesOperation>,
}

impl Context {
    pub(super) fn new(key: Key) -> Self {
        Self {
            ghash: None,
            cipher: CipherState::Ecb(Ecb),
            blocks: [0; 2 * BLOCK_SIZE],
            #[cfg(aes_dma_mode_gcm)]
            state: GcmState {
                j0: [0; BLOCK_SIZE],
                aad: NonNull::slice_from_raw_parts(NonNull::dangling(), 0),
                tag: NonNull::dangling(),
                result: Ok(()),
                refused: false,
            },
            frontend: WorkQueueFrontend::new(AesOperation {
                mode: Operation::Encrypt,
                cipher_mode: NonNull::dangling(),
                buffers: unsafe { CryptoBuffers::new_in_place(&mut []).into_inner() },
                key,
                #[cfg(aes_dma_mode_gcm)]
                gcm: None,
            }),
        }
    }

    pub(super) fn post<'t>(
        &'t mut self,
        mode: Operation,
        nonce: &[u8; 12],
        aad: &'t [u8],
        buffers: UnsafeCryptoBuffers,
        tag: NonNull<[u8; 16]>,
    ) -> Handle<'t> {
        // A 96-bit nonce gives J0 = nonce || 0x00000001, without GHASH.
        let mut j0 = [0; BLOCK_SIZE];
        j0[..12].copy_from_slice(nonce);
        j0[12..].copy_from_slice(&1u32.to_be_bytes());

        let mut handle = Handle {
            context: NonNull::from(self),
            step: None,
            decrypt: mode == Operation::Decrypt,
            j0,
            #[cfg(aes_dma_mode_gcm)]
            hardware: false,
            computes_h: false,
            aad: NonNull::from(aad),
            buffers,
            tag,
            ghash: None,
            mask: [0; BLOCK_SIZE],
            hashed: 0,
            posted: 0,
            done: 0,
            cancelled: false,
            result: None,
            _marker: PhantomData,
        };
        cfg_select! {
            aes_dma_mode_gcm => {
                handle.post_hardware();
            }
            _ => {
                handle.post_first_step();
            }
        }
        handle
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        clear_secrets(self);
    }
}

/// Zeroes the context's copies of H, the tag mask and the key stream.
fn clear_secrets(context: &mut Context) {
    unsafe {
        core::ptr::write_volatile(&mut context.blocks, [0; 2 * BLOCK_SIZE]);
        core::ptr::write_volatile(
            &mut context.cipher,
            CipherState::Ctr(Ctr::new([0; BLOCK_SIZE])),
        );
    }
    context.cipher = CipherState::Ecb(Ecb);
}

pub(super) struct Handle<'t> {
    context: NonNull<Context>,
    /// The step in the work queue. It is `None` only once the operation has ended.
    step: Option<work_queue::Handle<'t, AesOperation>>,
    decrypt: bool,
    j0: [u8; BLOCK_SIZE],
    /// Whether the item in the work queue is the hardware item.
    #[cfg(aes_dma_mode_gcm)]
    hardware: bool,
    /// Whether the first step also computes H.
    computes_h: bool,
    aad: NonNull<[u8]>,
    buffers: UnsafeCryptoBuffers,
    /// The tag the operation writes when it encrypts, and checks when it decrypts.
    tag: NonNull<[u8; 16]>,
    /// The hash so far, from the end of the first step.
    ghash: Option<GHash>,
    mask: [u8; BLOCK_SIZE],
    /// The text bytes hashed, posted and processed.
    hashed: usize,
    posted: usize,
    done: usize,
    cancelled: bool,
    result: Option<Result<(), GcmError>>,
    _marker: PhantomData<&'t mut Context>,
}

impl Handle<'_> {
    pub(super) fn poll(&mut self) -> bool {
        while self.result.is_none() {
            if !unwrap!(self.step.as_mut()).poll() {
                return false;
            }
            self.complete_step();
        }
        true
    }

    pub(super) fn wait_blocking(mut self) -> Result<(), GcmError> {
        loop {
            if let Some(result) = self.result {
                return result;
            }
            self.complete_step();
        }
    }

    pub(super) async fn wait(&mut self) -> Result<(), GcmError> {
        loop {
            if let Some(result) = self.result {
                return result;
            }
            let status = unwrap!(self.step.as_mut()).wait().await;
            self.step = None;
            self.step_completed(status);
        }
    }

    pub(super) async fn cancel(&mut self) {
        if self.result.is_some() {
            return;
        }
        self.cancelled = true;
        unwrap!(self.step.as_mut()).cancel().await;
        self.complete_step();
    }

    /// Waits for the step in the work queue, and moves the operation on.
    fn complete_step(&mut self) {
        let status = unwrap!(self.step.take()).wait_blocking();
        self.step_completed(status);
    }

    fn step_completed(&mut self, status: Status) {
        #[cfg(aes_dma_mode_gcm)]
        if core::mem::take(&mut self.hardware) && status == Status::Completed {
            let state = unsafe { &self.context.as_ref().state };
            if !state.refused {
                // The backend has written or checked the tag, and zeroed the output on a mismatch.
                self.result = Some(state.result);
                return;
            }
            if !self.cancelled {
                self.post_first_step();
                return;
            }
        }

        if self.cancelled || status == Status::Cancelled {
            self.end(Err(GcmError::Cancelled));
            return;
        }

        if self.ghash.is_none() {
            self.start_hash();
        } else {
            self.done = self.posted;
        }

        if self.done < self.buffers.input.len() {
            self.post_text();
        }
        self.hash_ahead();

        if self.done == self.buffers.input.len() {
            self.finish();
        }
    }

    /// Takes H and the tag mask from the first step, and hashes the additional data.
    fn start_hash(&mut self) {
        let context = unsafe { self.context.as_mut() };
        let mask_at = if self.computes_h {
            let mut h = [0; BLOCK_SIZE];
            h.copy_from_slice(&context.blocks[..BLOCK_SIZE]);
            context.ghash = Some(GHash::new(&h.into()));
            unsafe { core::ptr::write_volatile(&mut h, [0; BLOCK_SIZE]) };
            BLOCK_SIZE
        } else {
            0
        };
        self.mask
            .copy_from_slice(&context.blocks[mask_at..mask_at + BLOCK_SIZE]);
        unsafe { core::ptr::write_volatile(&mut context.blocks, [0; 2 * BLOCK_SIZE]) };

        let mut ghash = unwrap!(context.ghash.clone());
        ghash.update_padded(unsafe { self.aad.as_ref() });
        self.ghash = Some(ghash);

        let mut counter = self.j0;
        counter[12..].copy_from_slice(&2u32.to_be_bytes());
        context.cipher = CipherState::Ctr(Ctr::new(counter));
    }

    /// Posts the operation as the item that the hardware runs, if the backend can.
    #[cfg(aes_dma_mode_gcm)]
    fn post_hardware(&mut self) {
        // The work queue reads the context while the item is in it, and the handle touches no
        // part of the context until the item has left the queue.
        let context = unsafe { &mut *self.context.as_ptr() };
        context.state = GcmState {
            j0: self.j0,
            aad: self.aad,
            tag: self.tag,
            result: Ok(()),
            refused: false,
        };
        let data = context.frontend.data_mut();
        data.mode = if self.decrypt {
            Operation::Decrypt
        } else {
            Operation::Encrypt
        };
        data.buffers = self.buffers;
        data.gcm = Some(NonNull::from(&mut context.state));
        self.hardware = true;
        self.step = Some(context.frontend.post(&AES_WORK_QUEUE));
    }

    /// Posts the ECB step that encrypts J0, after the zero block if it computes H.
    fn post_first_step(&mut self) {
        let context = unsafe { self.context.as_mut() };
        self.computes_h = context.ghash.is_none();
        context.cipher = CipherState::Ecb(Ecb);
        let blocks = if self.computes_h {
            context.blocks[..BLOCK_SIZE].fill(0);
            context.blocks[BLOCK_SIZE..].copy_from_slice(&self.j0);
            &mut context.blocks[..]
        } else {
            context.blocks[..BLOCK_SIZE].copy_from_slice(&self.j0);
            &mut context.blocks[..BLOCK_SIZE]
        };
        let blocks = unsafe { CryptoBuffers::new_in_place(blocks).into_inner() };
        self.post_step(blocks);
    }

    /// Posts the next chunk of text. A decryption hashes the chunk first.
    fn post_text(&mut self) {
        let end = (self.done + CHUNK).min(self.buffers.input.len());
        if self.decrypt {
            self.hash_to(end);
        }
        let buffers = unsafe { self.buffers.byte_add(self.done) }.first_n(end - self.done);
        self.posted = end;
        self.post_step(buffers);
    }

    /// Posts a step over `buffers` with the context's cipher state.
    fn post_step(&mut self, buffers: UnsafeCryptoBuffers) {
        // The work queue reads the context while the step is in it, and the handle touches no
        // part of the context until the step has left the queue.
        let context = unsafe { &mut *self.context.as_ptr() };
        let data = context.frontend.data_mut();
        // The J0 step and AES-CTR both encrypt, whatever the operation does.
        data.mode = Operation::Encrypt;
        data.cipher_mode = NonNull::from(&mut context.cipher);
        data.buffers = buffers;
        #[cfg(aes_dma_mode_gcm)]
        {
            data.gcm = None;
        }
        self.step = Some(context.frontend.post(&AES_WORK_QUEUE));
    }

    /// Hashes the ciphertext the backend is not working on: up to the processed text when
    /// encrypting, and the chunk after the posted one when decrypting.
    fn hash_ahead(&mut self) {
        let end = if self.decrypt {
            (self.posted + CHUNK).min(self.buffers.input.len())
        } else {
            self.done
        };
        self.hash_to(end);
    }

    fn hash_to(&mut self, end: usize) {
        if end <= self.hashed {
            return;
        }
        let ciphertext = if self.decrypt {
            self.buffers.input
        } else {
            self.buffers.output
        };
        let chunk = unsafe {
            core::slice::from_raw_parts(
                ciphertext.cast::<u8>().add(self.hashed).as_ptr(),
                end - self.hashed,
            )
        };
        // Each chunk but the last is a whole number of blocks, so padding each one hashes the
        // text as one padded piece.
        unwrap!(self.ghash.as_mut()).update_padded(chunk);
        self.hashed = end;
    }

    /// Computes the tag over the hashed data, and writes or checks it.
    fn finish(&mut self) {
        let mut lengths = [0; BLOCK_SIZE];
        lengths[..8].copy_from_slice(&((self.aad.len() as u64) * 8).to_be_bytes());
        lengths[8..].copy_from_slice(&((self.buffers.input.len() as u64) * 8).to_be_bytes());

        let mut ghash = unwrap!(self.ghash.take());
        ghash.update_padded(&lengths);
        let mut tag: [u8; 16] = ghash.finalize().into();
        for (byte, mask) in tag.iter_mut().zip(&self.mask) {
            *byte ^= mask;
        }

        let result = if !self.decrypt {
            unsafe { self.tag.write(tag) };
            Ok(())
        } else if tags_equal(&tag, unsafe { self.tag.as_ref() }) {
            Ok(())
        } else {
            Err(GcmError::TagMismatch)
        };
        unsafe { core::ptr::write_volatile(&mut tag, [0; 16]) };

        self.end(result);
    }

    /// Ends the operation with `result`. No step is in the work queue.
    fn end(&mut self, result: Result<(), GcmError>) {
        // Plaintext that no tag has verified must not reach the caller.
        if self.decrypt && result.is_err() {
            unsafe { zero(self.buffers.output) };
        }
        self.ghash = None;
        unsafe { core::ptr::write_volatile(&mut self.mask, [0; BLOCK_SIZE]) };
        clear_secrets(unsafe { self.context.as_mut() });
        self.result = Some(result);
    }
}

impl Drop for Handle<'_> {
    fn drop(&mut self) {
        if self.result.is_none() {
            // Dropping the step waits until the work queue releases it.
            self.step = None;
            self.end(Err(GcmError::Cancelled));
        }
    }
}
