//! AES-GCM on the AES-DMA backend.
//!
//! One operation is one DMA transfer. The input chain holds the additional data, its zero padding,
//! the text, its zero padding and the length block. The hardware hashes them in that order.
//! The hardware writes whole blocks, so the last partial block of output goes to a bounce block.
//!
//! See <https://github.com/espressif/esp-idf/blob/ab95e70748f2eddac6f87217b0cf62533684a6ba/components/mbedtls/port/aes/esp_aes_gcm.c#L690-L789>
//! and <https://github.com/espressif/esp-idf/blob/ab95e70748f2eddac6f87217b0cf62533684a6ba/components/mbedtls/port/aes/dma/esp_aes_dma_core.c#L1366-L1600>.

use core::{hint::black_box, mem::ManuallyDrop, ptr::NonNull};

use super::{AesDma, AesDmaBackend, AesTransfer, CipherMode, DriverState};
use crate::{
    aes::{
        AES_WORK_QUEUE,
        AesOperation,
        BLOCK_SIZE,
        Error,
        Key,
        Mode,
        Operation,
        cipher_modes::{CryptoBuffers, UnsafeCryptoBuffers},
        read_words,
        write_words,
    },
    dma::{
        BurstConfig,
        DmaDescriptor,
        DmaEligiblePeripheral,
        DmaRxBuffer,
        DmaTxBuffer,
        NoBuffer,
        Preparation,
        aligned::InternalMemory,
        prepare_for_rx,
        prepare_for_tx,
    },
    work_queue::{Handle, Poll, Status, WorkQueueFrontend},
};

/// The longest additional data one operation takes: one DMA descriptor.
const MAX_AAD_LEN: usize = 4_080;

/// The longest text one operation takes: five DMA descriptors, which cover TLS 1.3's largest
/// record of 16,385 bytes.
const MAX_TEXT_LEN: usize = 5 * 4_080;

// A descriptor takes at most 4,080 bytes, or fewer where PSRAM needs a larger alignment. The
// input chain takes up to 2 descriptors for the additional data, 6 for the text and 3 of the
// backend's own blocks.
const INPUT_DESCRIPTORS: usize = 2 + 6 + 3;

// The output chain takes up to 6 descriptors for the text, a head and a tail for PSRAM, and the
// bounce block.
const OUTPUT_DESCRIPTORS: usize = 6 + 2 + 1;

/// A block the DMA reads or writes.
type Block = InternalMemory<[u8; BLOCK_SIZE]>;

/// The backend's descriptors and blocks for one GCM operation.
pub(super) struct GcmDma {
    input: [DmaDescriptor; INPUT_DESCRIPTORS],
    output: [DmaDescriptor; OUTPUT_DESCRIPTORS],
    // Each block is its own `InternalMemory`, so it has a cache line of its own where internal
    // memory is cached.
    aad_tail: Block,
    text_tail: Block,
    lengths: Block,
    output_tail: Block,
}

impl GcmDma {
    pub(super) const fn new() -> Self {
        Self {
            input: [DmaDescriptor::EMPTY; INPUT_DESCRIPTORS],
            output: [DmaDescriptor::EMPTY; OUTPUT_DESCRIPTORS],
            aad_tail: InternalMemory::new([0; BLOCK_SIZE]),
            text_tail: InternalMemory::new([0; BLOCK_SIZE]),
            lengths: InternalMemory::new([0; BLOCK_SIZE]),
            output_tail: InternalMemory::new([0; BLOCK_SIZE]),
        }
    }

    /// Zeroes the blocks, which hold the last partial blocks of the input and the output.
    fn clear(&mut self) {
        for block in [
            &mut self.aad_tail,
            &mut self.text_tail,
            &mut self.lengths,
            &mut self.output_tail,
        ] {
            unsafe { core::ptr::write_volatile(&mut *block.get_mut(), [0; BLOCK_SIZE]) };
        }
    }
}

/// The state of one GCM operation, which the backend reads and writes while it owns the item.
pub(crate) struct GcmState {
    j0: [u8; BLOCK_SIZE],
    aad: NonNull<[u8]>,
    /// The tag the backend writes when it encrypts, and checks when it decrypts.
    tag: NonNull<[u8; 16]>,
    result: Result<(), GcmError>,
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

    /// The running backend cannot process AES-GCM. Only [`AesDmaBackend`] can.
    Unsupported,

    /// [`AesGcmHandle::cancel`] cancelled the operation before it started. When
    /// [`AesGcmHandle::wait`] or [`AesGcmHandle::wait_blocking`] returns this for a decryption,
    /// the output is zeroed.
    Cancelled,
}

/// An AES-GCM work queue user.
///
/// Operations run on [`AesDmaBackend`], which must be started. On
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

fn validate(aad: &[u8], text_len: usize) -> Result<(), Error> {
    if text_len == 0 || text_len > MAX_TEXT_LEN || aad.len() > MAX_AAD_LEN {
        return Err(Error::IncorrectBufferLength);
    }

    // The backend copies a text in flash to the output first. Only the text has an output to be
    // copied into, so the DMA reads the additional data where it is.
    if !aad.is_empty() && !crate::soc::is_valid_memory_address(aad.as_ptr() as usize) {
        return Err(Error::AadNotInRam);
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

unsafe fn zero(buffer: NonNull<[u8]>) {
    unsafe { buffer.cast::<u8>().write_bytes(0, buffer.len()) };
}

/// Compares two tags in constant time.
///
/// It must not return early: the time it takes must not depend on where the tags differ.
fn tags_equal(a: &[u8; 16], b: &[u8; 16]) -> bool {
    let mut difference = 0u8;
    for (x, y) in a.iter().zip(b) {
        difference |= black_box(x ^ y);
    }
    black_box(difference) == 0
}

/// The GCM register values of one operation.
struct GcmRegisters {
    text_blocks: usize,
    aad_blocks: usize,
    remainder_bits: usize,
    j0: [u8; BLOCK_SIZE],
}

impl<'d> AesDma<'d> {
    /// Sets the number of additional-data blocks, and the number of valid bits in the text's last
    /// block. A full block sets 0.
    fn set_gcm_lengths(&self, aad_blocks: usize, remainder_bits: usize) {
        let regs = self.aes.regs();
        regs.aad_block_num()
            .write(|w| unsafe { w.aad_block_num().bits(aad_blocks as _) });
        regs.remainder_bit_num()
            .write(|w| unsafe { w.remainder_bit_num().bits(remainder_bits as _) });
    }

    fn write_j0(&self, j0: &[u8; BLOCK_SIZE]) {
        for (word, reg) in read_words(j0).zip(self.aes.regs().j0_mem_iter()) {
            reg.write(|w| unsafe { w.bits(word) });
        }
    }

    /// Starts the data phase of the operation that computed H.
    fn continue_transform(&self) {
        self.aes
            .regs()
            .continue_()
            .write(|w| w.continue_().set_bit());
    }

    fn read_tag(&self, tag: &mut [u8; 16]) {
        write_words(tag, |i| self.aes.regs().t0_mem(i).read().bits());
    }

    fn start_gcm_transfer(
        mut self,
        registers: &GcmRegisters,
        mut output: NoBuffer,
        mut input: NoBuffer,
        mode: Mode,
        key: &Key,
    ) -> AesTransfer<'d, NoBuffer, NoBuffer> {
        // AES has to be restarted after each calculation
        self.reset_aes();

        // Both channels are prepared before the key goes in, so that a DMA error leaves no key in
        // the registers.
        let peri = self.aes.aes.dma_peripheral();
        unwrap!(
            unsafe { self.channel.tx.prepare_transfer(peri, &mut input) }
                .and_then(|_| unsafe { self.channel.rx.prepare_transfer(peri, &mut output) })
        );

        self.aes.write_mode(mode);
        self.write_key(key);
        self.set_cipher_mode(CipherMode::Gcm);
        self.set_gcm_lengths(registers.aad_blocks, registers.remainder_bits);

        // In DMA mode, the first start computes H before the DMA delivers any data.
        self.enable_dma(true);
        self.start_transform();
        while !self.aes.is_idle() {}

        self.write_j0(&registers.j0);

        if let Err(error) = self
            .channel
            .tx
            .start_transfer()
            .and_then(|_| self.channel.rx.start_transfer())
        {
            // The key must not stay in the registers after a panic.
            self.reset_aes();
            panic!("AES-GCM DMA transfer failed to start: {:?}", error);
        }

        self.listen();
        self.set_num_block(registers.text_blocks as u32);
        self.continue_transform();

        DmaRxBuffer::into_view(output);
        DmaTxBuffer::into_view(input);
        AesTransfer {
            aes_dma: ManuallyDrop::new(self),
            rx_view: ManuallyDrop::new(()),
            tx_view: ManuallyDrop::new(()),
        }
    }
}

impl<'d> AesDmaBackend<'d> {
    /// Starts a GCM operation on the DMA.
    pub(super) fn start_gcm(
        &mut self,
        driver: AesDma<'d>,
        item: &mut AesOperation,
        state: NonNull<GcmState>,
    ) -> Poll {
        let len = item.buffers.input.len();

        if !crate::soc::is_valid_memory_address(item.buffers.input.addr().get()) {
            // The DMA cannot read the input, so the text is copied to the output in RAM and
            // processed in place.
            unsafe {
                item.buffers
                    .output
                    .cast::<u8>()
                    .copy_from_nonoverlapping(item.buffers.input.cast::<u8>(), len);
            }
            item.buffers.input = item.buffers.output;
        }

        let state = unsafe { state.as_ref() };
        let aad = state.aad;
        let text_full = len - len % BLOCK_SIZE;
        let aad_full = aad.len() - aad.len() % BLOCK_SIZE;

        let mut dma = self.gcm.get_mut();
        dma.clear();
        unsafe {
            copy_tail(&mut dma.aad_tail, aad, aad_full);
            copy_tail(&mut dma.text_tail, item.buffers.input, text_full);
        }
        // The length block holds the bit lengths of the additional data and the text, each as a
        // 64-bit big-endian number.
        dma.lengths.get_mut()[..8].copy_from_slice(&((aad.len() as u64) * 8).to_be_bytes());
        dma.lengths.get_mut()[8..].copy_from_slice(&((len as u64) * 8).to_be_bytes());

        let GcmDma {
            input: input_descriptors,
            output: output_descriptors,
            aad_tail,
            text_tail,
            lengths,
            output_tail,
        } = &mut *dma;

        let mut input = Chain::new(input_descriptors);
        if aad_full > 0 {
            input.tx(get_range(aad, 0, aad_full));
        }
        if aad.len() > aad_full {
            input.tx_block(aad_tail);
        }
        if text_full > 0 {
            input.tx(get_range(item.buffers.input, 0, text_full));
        }
        if len > text_full {
            input.tx_block(text_tail);
        }
        input.tx_block(lengths);
        let input = input.finish(false);

        let mut output = Chain::new(output_descriptors);
        if text_full > 0 {
            output.rx(
                #[cfg(dma_can_access_psram)]
                &mut self.unaligned_data_buffers,
                get_range(item.buffers.output, 0, text_full),
            );
        }
        if len > text_full {
            output.rx_block(output_tail);
        }
        let output = output.finish(true);

        #[cfg(soc_internal_memory_cached)]
        dma.writeback();

        let mode = if item.mode == Operation::Encrypt {
            item.key.encrypt_mode()
        } else {
            item.key.decrypt_mode()
        };

        let registers = GcmRegisters {
            text_blocks: len.div_ceil(BLOCK_SIZE),
            aad_blocks: aad.len().div_ceil(BLOCK_SIZE),
            remainder_bits: (len * 8) % 128,
            j0: state.j0,
        };
        let transfer = driver.start_gcm_transfer(&registers, output, input, mode, &item.key);

        self.state = DriverState::WaitingForDma {
            transfer,
            remaining_bytes: 0,
        };

        Poll::Pending(false)
    }

    /// Completes a GCM operation once the DMA is done.
    ///
    /// It runs in the AES interrupt when the queue is processed there. On a mismatch it zeroes up
    /// to 20,400 bytes of output.
    pub(super) fn finish_gcm(
        &mut self,
        item: &mut AesOperation,
        mut state: NonNull<GcmState>,
    ) -> Poll {
        let DriverState::WaitingForDma { transfer, .. } = &mut self.state else {
            unreachable!()
        };
        if !transfer.is_done() {
            return Poll::Pending(false);
        }

        let mut tag = [0; 16];
        // `wait` resets the block, which clears the key, H, J0 and tag registers, so the tag is
        // read first. The work queue never stops the backend during an operation, so every
        // operation the backend starts reaches this reset.
        transfer.aes_dma.read_tag(&mut tag);

        let DriverState::WaitingForDma { transfer, .. } =
            core::mem::replace(&mut self.state, DriverState::None)
        else {
            unreachable!()
        };
        let (driver, _, _) = transfer.wait();
        driver.clear_interrupt();

        #[cfg(dma_can_access_psram)]
        for buffer in self.unaligned_data_buffers.iter_mut() {
            if let Some(buffer) = buffer.as_mut() {
                buffer.write_back();
                // The buffer held part of the output, which can be plaintext.
                buffer.clear();
            }
            *buffer = None;
        }

        let output = item.buffers.output;
        let len = output.len();
        let text_full = len - len % BLOCK_SIZE;

        let mut dma = self.gcm.get_mut();
        #[cfg(soc_internal_memory_cached)]
        dma.invalidate();
        if len > text_full {
            unsafe {
                output.cast::<u8>().add(text_full).copy_from_nonoverlapping(
                    NonNull::from(&*dma.output_tail.get_ref()).cast::<u8>(),
                    len - text_full,
                );
            }
        }
        dma.clear();
        #[cfg(soc_internal_memory_cached)]
        dma.writeback();

        let state = unsafe { state.as_mut() };
        state.result = if item.mode == Operation::Encrypt {
            unsafe { state.tag.write(tag) };
            Ok(())
        } else if tags_equal(&tag, unsafe { state.tag.as_ref() }) {
            Ok(())
        } else {
            unsafe { zero(output) };
            Err(GcmError::TagMismatch)
        };
        unsafe { core::ptr::write_volatile(&mut tag, [0; 16]) };

        self.state = DriverState::Idle(driver);

        Poll::Ready(Status::Completed)
    }
}

/// Copies the bytes of `data` from `start` to its end into the start of `block`. The rest of
/// `block` stays zero.
unsafe fn copy_tail(block: &mut Block, data: NonNull<[u8]>, start: usize) {
    let len = data.len() - start;
    unsafe {
        block
            .get_mut()
            .as_mut_ptr()
            .copy_from_nonoverlapping(data.cast::<u8>().add(start).as_ptr(), len);
    }
}

fn get_range(data: NonNull<[u8]>, start: usize, end: usize) -> NonNull<[u8]> {
    NonNull::slice_from_raw_parts(unsafe { data.cast::<u8>().add(start) }, end - start)
}

/// Builds one descriptor chain from several segments.
struct Chain<'a> {
    descriptors: &'a mut [DmaDescriptor],
    used: usize,
    #[cfg(dma_can_access_psram)]
    accesses_psram: bool,
}

impl<'a> Chain<'a> {
    fn new(descriptors: &'a mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            used: 0,
            #[cfg(dma_can_access_psram)]
            accesses_psram: false,
        }
    }

    /// Links `count` descriptors from `first` after the chain so far.
    fn append(&mut self, first: usize, count: usize) {
        if first > 0 {
            let next = &raw mut self.descriptors[first];
            self.descriptors[first - 1].next = next;
        }
        self.used = first + count;
    }

    /// Counts the descriptors a prepared segment linked, from `first`.
    fn linked_from(&self, first: usize) -> usize {
        let mut count = 1;
        while !self.descriptors[first + count - 1].next.is_null() {
            count += 1;
        }
        count
    }

    fn tx(&mut self, data: NonNull<[u8]>) {
        let first = self.used;
        let (buffer, len) =
            unwrap!(unsafe { prepare_for_tx(&mut self.descriptors[first..], data, BLOCK_SIZE) });
        // `validate` bounds the lengths and the descriptor counts cover them in PSRAM, so the
        // chain always takes all of `data`.
        assert_eq!(len, data.len());
        cfg_select! {
            dma_can_access_psram => {
                self.accesses_psram |= buffer.0.accesses_psram;
            }
            _ => {
                let _ = buffer;
            }
        }
        let count = self.linked_from(first);
        self.append(first, count);
    }

    fn tx_block(&mut self, block: &mut Block) {
        let first = self.used;
        let descriptor = &mut self.descriptors[first];
        *descriptor = DmaDescriptor::EMPTY;
        descriptor.buffer = block.get_mut().as_mut_ptr();
        descriptor.set_size(BLOCK_SIZE);
        descriptor.set_length(BLOCK_SIZE);
        self.append(first, 1);
    }

    fn rx(
        &mut self,
        #[cfg(dma_can_access_psram)] align_buffers: &mut [Option<crate::dma::ManualWritebackBuffer>;
                 2],
        data: NonNull<[u8]>,
    ) {
        let first = self.used;
        let (buffer, len) = unsafe {
            prepare_for_rx(
                &mut self.descriptors[first..],
                #[cfg(dma_can_access_psram)]
                align_buffers,
                data,
            )
        };
        assert_eq!(len, data.len());
        cfg_select! {
            dma_can_access_psram => {
                self.accesses_psram |= buffer.0.accesses_psram;
            }
            _ => {
                let _ = buffer;
            }
        }
        let count = self.linked_from(first);
        self.append(first, count);
    }

    fn rx_block(&mut self, block: &mut Block) {
        let first = self.used;
        let descriptor = &mut self.descriptors[first];
        *descriptor = DmaDescriptor::EMPTY;
        descriptor.buffer = block.get_mut().as_mut_ptr();
        descriptor.set_size(BLOCK_SIZE);
        self.append(first, 1);
    }

    /// Sets the owner and the end-of-frame flags of the chain, and returns it as a DMA buffer.
    fn finish(self, rx: bool) -> NoBuffer {
        let last = self.used - 1;
        for (i, descriptor) in self.descriptors[..self.used].iter_mut().enumerate() {
            if rx {
                descriptor.reset_for_rx();
            } else {
                descriptor.reset_for_tx(i == last);
            }
        }
        NoBuffer(Preparation {
            start: self.descriptors.as_mut_ptr(),
            burst_transfer: BurstConfig::DEFAULT,
            check_owner: None,
            auto_write_back: rx,
            #[cfg(dma_can_access_psram)]
            accesses_psram: self.accesses_psram,
        })
    }
}
