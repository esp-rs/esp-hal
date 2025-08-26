#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Secure Hash Algorithm (SHA) Accelerator
//!
//! ## Overview
//! This SHA accelerator is a hardware device that speeds up the SHA algorithm
//! significantly, compared to a SHA algorithm implemented solely in software
//!
//! ## Configuration
//! This driver allows you to perform cryptographic hash operations using
//! various hash algorithms supported by the SHA peripheral, such as:
//! * SHA-1
//! * SHA-224
//! * SHA-256
//! * SHA-384
//! * SHA-512
//!
//! The driver supports two working modes:
//! * Typical SHA (CPU-driven)
//! * DMA-SHA (not supported yet)
//!
//! It provides functions to update the hash calculation with input data, finish
//! the hash calculation and retrieve the resulting hash value. The SHA
//! peripheral on ESP chips can handle large data streams efficiently, making it
//! suitable for cryptographic applications that require secure hashing.
//!
//! To use the SHA Peripheral Driver, you need to initialize it with the desired
//! SHA mode and the corresponding SHA peripheral. Once initialized, you can
//! update the hash calculation by providing input data, finish the calculation
//! to retrieve the hash value and repeat the process for a new hash calculation
//! if needed.
//!
//! ## Examples
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::sha::Sha;
//! # use esp_hal::sha::Sha256;
//! # use nb::block;
//! let mut source_data = "HELLO, ESPRESSIF!".as_bytes();
//! let mut sha = Sha::new(peripherals.SHA);
//! let mut hasher = sha.start::<Sha256>();
//! // Short hashes can be created by decreasing the output buffer to the
//! // desired length
//! let mut output = [0u8; 32];
//!
//! while !source_data.is_empty() {
//!     // All the HW Sha functions are infallible so unwrap is fine to use if
//!     // you use block!
//!     source_data = block!(hasher.update(source_data))?;
//! }
//!
//! // Finish can be called as many times as desired to get multiple copies of
//! // the output.
//! block!(hasher.finish(output.as_mut_slice()))?;
//!
//! # {after_snippet}
//! ```

use core::{borrow::Borrow, convert::Infallible, marker::PhantomData, mem::size_of};

/// Re-export digest for convenience
pub use digest::Digest;

use crate::{
    peripherals::SHA,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::GenericPeripheralGuard,
};

// ESP32 quirks:
// - Big endian text register (what about hash?)
// - Text and hash is in the same register -> needs an additional load operation to place the hash
//   in the text registers
// - Each algorithm has its own register cluster
// - No support for interleaved operation

/// The SHA Accelerator driver instance
pub struct Sha<'d> {
    sha: SHA<'d>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Sha as u8 }>,
}

impl<'d> Sha<'d> {
    /// Create a new instance of the SHA Accelerator driver.
    pub fn new(sha: SHA<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        Self { sha, _guard: guard }
    }

    /// Start a new digest.
    pub fn start<'a, A: ShaAlgorithm>(&'a mut self) -> ShaDigest<'d, A, &'a mut Self> {
        ShaDigest::new(self)
    }

    /// Start a new digest and take ownership of the driver.
    /// This is useful for storage outside a function body. i.e. in static or
    /// struct.
    pub fn start_owned<A: ShaAlgorithm>(self) -> ShaDigest<'d, A, Self> {
        ShaDigest::new(self)
    }

    /// Returns true if the hardware is processing the next message.
    fn is_busy(&self, algo: ShaAlgorithmKind) -> bool {
        algo.is_busy(&self.sha)
    }

    fn process_buffer(&self, state: &mut DigestState) {
        if state.first_run {
            state.algorithm.start(&self.sha);
            state.first_run = false;
        } else {
            state.algorithm.r#continue(&self.sha);
        }
    }

    fn write_data<'a>(
        &self,
        state: &mut DigestState,
        incoming: &'a [u8],
    ) -> nb::Result<&'a [u8], Infallible> {
        if state.message_buffer_is_full {
            if self.is_busy(state.algorithm) {
                // The message buffer is full and the hardware is still processing the previous
                // message. There's nothing to be done besides wait for the hardware.
                return Err(nb::Error::WouldBlock);
            } else {
                // Submit the full buffer.
                self.process_buffer(state);
                // The buffer is now free for filling.
                state.message_buffer_is_full = false;
            }
        }

        let chunk_len = state.algorithm.chunk_length();
        let mod_cursor = state.cursor % chunk_len;

        let (remaining, bound_reached) = state.alignment_helper.aligned_volatile_copy(
            m_mem(&self.sha, 0),
            incoming,
            chunk_len,
            mod_cursor,
        );

        state.cursor += incoming.len() - remaining.len();

        if bound_reached {
            // Message is full now. We don't have to wait for the result, just start the processing
            // or set the flag.
            _ = self.process_buffer_or_wait(state);
        }

        Ok(remaining)
    }

    fn process_buffer_or_wait(&self, state: &mut DigestState) -> nb::Result<(), Infallible> {
        if self.is_busy(state.algorithm) {
            // The message buffer is full and the hardware is still processing the
            // previous message. There's nothing to be done besides wait for the
            // hardware.
            state.message_buffer_is_full = true;
            return Err(nb::Error::WouldBlock);
        }

        // Send the full buffer.
        self.process_buffer(state);

        Ok(())
    }

    fn finish(&self, state: &mut DigestState, output: &mut [u8]) -> nb::Result<(), Infallible> {
        if state.message_buffer_is_full {
            // Wait for the hardware to become idle.
            if self.is_busy(state.algorithm) {
                return Err(nb::Error::WouldBlock);
            }

            // Start processing so that we can continue writing into SHA memory.
            self.process_buffer(state);
            state.message_buffer_is_full = false;
        }

        let chunk_len = state.algorithm.chunk_length();
        if state.finalize_state == FinalizeState::NotStarted {
            let cursor = state.cursor;
            self.update(state, &[0x80])?; // Append "1" bit
            state.finished_message_size = cursor;

            state.finalize_state = FinalizeState::FlushAlignBuffer;
        }

        if state.finalize_state == FinalizeState::FlushAlignBuffer {
            let flushed = state
                .alignment_helper
                .flush_to(m_mem(&self.sha, 0), state.cursor % chunk_len);

            state.finalize_state = FinalizeState::ZeroPadAlmostFull;
            if flushed > 0 {
                state.cursor += flushed;
                if state.cursor.is_multiple_of(chunk_len) {
                    self.process_buffer_or_wait(state)?;
                }
            }
        }

        let mut mod_cursor = state.cursor % chunk_len;
        if state.finalize_state == FinalizeState::ZeroPadAlmostFull {
            // Zero out remaining data if buffer is almost full (>=448/896), and process
            // buffer.
            //
            // In either case, we'll continue to the next state.
            state.finalize_state = FinalizeState::WriteMessageLength;
            let pad_len = chunk_len - mod_cursor;
            if pad_len < state.algorithm.message_length_bytes() {
                state.alignment_helper.volatile_write(
                    m_mem(&self.sha, 0),
                    0_u8,
                    pad_len,
                    mod_cursor,
                );
                state.cursor += pad_len;

                self.process_buffer_or_wait(state)?;
                mod_cursor = 0;
            }
        }

        if state.finalize_state == FinalizeState::WriteMessageLength {
            // In this state, we pad the remainder of the message block with 0s and append the
            // message length to the very end.
            // FIXME: this u64 should be u128 for 1024-bit block algos. Since cursor is only usize
            // (u32), this makes no difference currently, but may limit maximum message length in
            // the future.
            let message_len_bytes = size_of::<u64>();

            let pad_len = chunk_len - mod_cursor - message_len_bytes;
            // Fill remaining space with zeros
            state
                .alignment_helper
                .volatile_write(m_mem(&self.sha, 0), 0, pad_len, mod_cursor);

            // Write message length
            let length = state.finished_message_size as u64 * 8;
            state.alignment_helper.aligned_volatile_copy(
                m_mem(&self.sha, 0),
                &length.to_be_bytes(),
                chunk_len,
                chunk_len - message_len_bytes,
            );

            // Set up last state, start processing
            state.finalize_state = FinalizeState::ReadResult;
            self.process_buffer_or_wait(state)?;
        }

        if state.finalize_state == FinalizeState::ReadResult {
            if state.algorithm.is_busy(&self.sha) {
                return Err(nb::Error::WouldBlock);
            }
            if state.algorithm.load(&self.sha) {
                // Spin wait for result, 8-20 clock cycles according to manual
                while self.is_busy(state.algorithm) {}
            }

            state.alignment_helper.volatile_read_regset(
                h_mem(&self.sha, 0),
                output,
                core::cmp::min(output.len(), 32),
            );

            state.first_run = true;
            state.cursor = 0;
            state.alignment_helper.reset();
            state.finalize_state = FinalizeState::NotStarted;

            return Ok(());
        }

        Err(nb::Error::WouldBlock)
    }

    fn update<'a>(
        &self,
        state: &mut DigestState,
        incoming: &'a [u8],
    ) -> nb::Result<&'a [u8], Infallible> {
        state.finalize_state = FinalizeState::default();
        self.write_data(state, incoming)
    }
}

impl crate::private::Sealed for Sha<'_> {}

#[cfg(sha_dma)]
#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Sha<'_> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.sha.disable_peri_interrupt();

        self.sha.bind_peri_interrupt(handler.handler());
        self.sha.enable_peri_interrupt(handler.priority());
    }
}

// A few notes on this implementation with regards to 'memcpy',
// - The registers are *not* cleared after processing, so padding needs to be written out
// - Registers need to be written one u32 at a time, no u8 access
// - This means that we need to buffer bytes coming in up to 4 u8's in order to create a full u32

/// An active digest
///
/// This implementation might fail after u32::MAX/8 bytes, to increase please
/// see ::finish() length/self.cursor usage
pub struct ShaDigest<'d, A, S: Borrow<Sha<'d>>> {
    sha: S,
    state: DigestState,
    phantom: PhantomData<(&'d (), A)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
enum FinalizeState {
    #[default]
    NotStarted,
    FlushAlignBuffer,
    ZeroPadAlmostFull,
    WriteMessageLength,
    ReadResult,
}

#[derive(Clone, Debug)]
struct DigestState {
    algorithm: ShaAlgorithmKind,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished_message_size: usize,
    message_buffer_is_full: bool,
    finalize_state: FinalizeState,
}

impl DigestState {
    fn new(algorithm: ShaAlgorithmKind) -> Self {
        Self {
            algorithm,
            alignment_helper: AlignmentHelper::default(),
            cursor: 0,
            first_run: true,
            finished_message_size: 0,
            message_buffer_is_full: false,
            finalize_state: FinalizeState::default(),
        }
    }
}

impl<'d, A: ShaAlgorithm, S: Borrow<Sha<'d>>> ShaDigest<'d, A, S> {
    /// Creates a new digest
    #[allow(unused_mut)]
    pub fn new(mut sha: S) -> Self {
        #[cfg(not(esp32))]
        // Setup SHA Mode.
        sha.borrow()
            .sha
            .register_block()
            .mode()
            .write(|w| unsafe { w.mode().bits(A::ALGORITHM_KIND.mode_bits()) });

        Self {
            sha,
            state: DigestState::new(A::ALGORITHM_KIND),
            phantom: PhantomData,
        }
    }

    /// Restores a previously saved digest.
    #[cfg(not(esp32))]
    pub fn restore(sha: S, ctx: &mut Context<A>) -> Self {
        // Setup SHA Mode.
        sha.borrow()
            .sha
            .register_block()
            .mode()
            .write(|w| unsafe { w.mode().bits(A::ALGORITHM_KIND.mode_bits()) });

        // Restore the message buffer
        unsafe {
            core::ptr::copy_nonoverlapping(ctx.buffer.as_ptr(), m_mem(&sha.borrow().sha, 0), 32);
        }

        // Restore previously saved hash
        ctx.state.alignment_helper.volatile_write_regset(
            h_mem(&sha.borrow().sha, 0),
            &ctx.saved_digest,
            64,
        );

        Self {
            sha,
            state: ctx.state.clone(),
            phantom: PhantomData,
        }
    }

    /// Returns true if the hardware is processing the next message.
    pub fn is_busy(&self) -> bool {
        A::ALGORITHM_KIND.is_busy(&self.sha.borrow().sha)
    }

    /// Updates the SHA digest with the provided data buffer.
    pub fn update<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        self.sha.borrow().update(&mut self.state, incoming)
    }

    /// Finish of the calculation (if not already) and copy result to output
    /// After `finish()` is called `update()`s will contribute to a new hash
    /// which can be calculated again with `finish()`.
    ///
    /// Typically, output is expected to be the size of
    /// [ShaAlgorithm::DIGEST_LENGTH], but smaller inputs can be given to
    /// get a "short hash"
    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        self.sha.borrow().finish(&mut self.state, output)
    }

    /// Save the current state of the digest for later continuation.
    #[cfg(not(esp32))]
    pub fn save(&mut self, context: &mut Context<A>) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        context.state = self.state.clone();

        // Save the content of the current hash.
        self.state.alignment_helper.volatile_read_regset(
            h_mem(&self.sha.borrow().sha, 0),
            &mut context.saved_digest,
            64,
        );

        // Save the content of the current (probably partially written) message.
        unsafe {
            core::ptr::copy_nonoverlapping(
                m_mem(&self.sha.borrow().sha, 0),
                context.buffer.as_mut_ptr(),
                32,
            );
        }

        Ok(())
    }

    /// Discard the current digest and return the peripheral.
    pub fn cancel(self) -> S {
        self.sha
    }
}

#[cfg(not(esp32))]
/// Context for a SHA Accelerator driver instance
#[derive(Debug, Clone)]
pub struct Context<A: ShaAlgorithm> {
    state: DigestState,
    /// Buffered bytes (SHA_M_n_REG) to be processed.
    buffer: [u32; 32],
    /// Saved digest (SHA_H_n_REG) for interleaving operation
    saved_digest: [u8; 64],
    phantom: PhantomData<A>,
}

#[cfg(not(esp32))]
impl<A: ShaAlgorithm> Context<A> {
    /// Create a new empty context
    pub fn new() -> Self {
        Self {
            state: DigestState::new(A::ALGORITHM_KIND),
            buffer: [0; 32],
            saved_digest: [0; 64],
            phantom: PhantomData,
        }
    }

    /// Indicates if the SHA context is in the first run.
    ///
    /// Returns `true` if this is the first time processing data with the SHA
    /// instance, otherwise returns `false`.
    pub fn first_run(&self) -> bool {
        self.state.first_run
    }
}

#[cfg(not(esp32))]
impl<A: ShaAlgorithm> Default for Context<A> {
    fn default() -> Self {
        Self::new()
    }
}

/// This trait encapsulates the configuration for a specific SHA algorithm.
pub trait ShaAlgorithm: crate::private::Sealed {
    /// Constant containing the name of the algorithm as a string.
    const ALGORITHM: &'static str;

    /// Constant containing the kind of the algorithm.
    const ALGORITHM_KIND: ShaAlgorithmKind;

    /// The length of the chunk that the algorithm processes at a time.
    ///
    /// For example, in SHA-256, this would typically be 64 bytes.
    const CHUNK_LENGTH: usize;

    /// The length of the resulting digest produced by the algorithm.
    ///
    /// For example, in SHA-256, this would be 32 bytes.
    const DIGEST_LENGTH: usize;

    #[doc(hidden)]
    type DigestOutputSize: digest::generic_array::ArrayLength<u8> + 'static;
}

/// Note: digest has a blanket trait implementation for [digest::Digest] for any
/// element that implements FixedOutput + Default + Update + HashMarker
impl<'d, A: ShaAlgorithm, S: Borrow<Sha<'d>>> digest::HashMarker for ShaDigest<'d, A, S> {}

impl<'d, A: ShaAlgorithm, S: Borrow<Sha<'d>>> digest::OutputSizeUser for ShaDigest<'d, A, S> {
    type OutputSize = A::DigestOutputSize;
}

impl<'d, A: ShaAlgorithm, S: Borrow<Sha<'d>>> digest::Update for ShaDigest<'d, A, S> {
    fn update(&mut self, mut remaining: &[u8]) {
        while !remaining.is_empty() {
            remaining = nb::block!(Self::update(self, remaining)).unwrap();
        }
    }
}

impl<'d, A: ShaAlgorithm, S: Borrow<Sha<'d>>> digest::FixedOutput for ShaDigest<'d, A, S> {
    fn finalize_into(mut self, out: &mut digest::Output<Self>) {
        nb::block!(self.finish(out)).unwrap();
    }
}

for_each_sha_algorithm! {
    (algos $( ( $name:ident, $full_name:literal $sizes:tt $security:tt, $mode_bits:literal ) ),*) => {

        /// Specifies particular SHA algorithm.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[non_exhaustive]
        pub enum ShaAlgorithmKind {
            $(
                #[doc = concat!("The ", $full_name, " algorithm.")]
                $name,
            )*
        }
    };
}

for_each_sha_algorithm! {
    (algos $( ( $name:ident, $full_name:literal (sizes: $block_size:literal, $digest_words:literal, $message_length_bytes:literal) $security:tt, $mode_bits:literal ) ),*) => {
        impl ShaAlgorithmKind {
            #[cfg(not(esp32))]
            const fn mode_bits(self) -> u8 {
                match self {
                    $(ShaAlgorithmKind::$name => $mode_bits,)*
                }
            }

            const fn chunk_length(self) -> usize {
                match self {
                    $(ShaAlgorithmKind::$name => $block_size,)*
                }
            }

            /// Bytes needed to represent the length of the longest possible message.
            const fn message_length_bytes(self) -> usize {
                match self {
                    $(ShaAlgorithmKind::$name => $message_length_bytes,)*
                }
            }

            const fn digest_length(self) -> usize {
                match self {
                    $(ShaAlgorithmKind::$name => $digest_words,)*
                }
            }
        }
    };
}

impl ShaAlgorithmKind {
    fn start(self, sha: &crate::peripherals::SHA<'_>) {
        let regs = sha.register_block();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                match self {
                    ShaAlgorithmKind::Sha1 => regs.sha1_start().write(|w| w.sha1_start().set_bit()),
                    ShaAlgorithmKind::Sha256 => regs.sha256_start().write(|w| w.sha256_start().set_bit()),
                    ShaAlgorithmKind::Sha384 => regs.sha384_start().write(|w| w.sha384_start().set_bit()),
                    ShaAlgorithmKind::Sha512 => regs.sha512_start().write(|w| w.sha512_start().set_bit()),
                };
            } else {
                regs.start().write(|w| w.start().set_bit());
            }
        }
    }

    fn r#continue(self, sha: &crate::peripherals::SHA<'_>) {
        let regs = sha.register_block();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                match self {
                    ShaAlgorithmKind::Sha1 => regs.sha1_continue().write(|w| w.sha1_continue().set_bit()),
                    ShaAlgorithmKind::Sha256 => regs.sha256_continue().write(|w| w.sha256_continue().set_bit()),
                    ShaAlgorithmKind::Sha384 => regs.sha384_continue().write(|w| w.sha384_continue().set_bit()),
                    ShaAlgorithmKind::Sha512 => regs.sha512_continue().write(|w| w.sha512_continue().set_bit()),
                };
            } else {
                regs.continue_().write(|w| w.continue_().set_bit());
            }
        }
    }

    /// Starts loading the hash into the output registers.
    ///
    /// Returns whether the caller needs to wait for the hash to be loaded.
    fn load(self, _sha: &crate::peripherals::SHA<'_>) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let regs = _sha.register_block();
                match self {
                    ShaAlgorithmKind::Sha1 => regs.sha1_load().write(|w| w.sha1_load().set_bit()),
                    ShaAlgorithmKind::Sha256 => regs.sha256_load().write(|w| w.sha256_load().set_bit()),
                    ShaAlgorithmKind::Sha384 => regs.sha384_load().write(|w| w.sha384_load().set_bit()),
                    ShaAlgorithmKind::Sha512 => regs.sha512_load().write(|w| w.sha512_load().set_bit()),
                };

                true
            } else {
                // Return that no waiting is necessary
                false
            }
        }
    }

    fn is_busy(self, sha: &crate::peripherals::SHA<'_>) -> bool {
        let regs = sha.register_block();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let bit = match self {
                    ShaAlgorithmKind::Sha1 => regs.sha1_busy().read().sha1_busy(),
                    ShaAlgorithmKind::Sha256 => regs.sha256_busy().read().sha256_busy(),
                    ShaAlgorithmKind::Sha384 => regs.sha384_busy().read().sha384_busy(),
                    ShaAlgorithmKind::Sha512 => regs.sha512_busy().read().sha512_busy(),
                };
            } else {
                let bit = regs.busy().read().state();
            }
        }

        bit.bit_is_set()
    }
}

// This macro call creates Sha algorithm structs that implement the ShaAlgorithm trait.
for_each_sha_algorithm! {
    ( $name:ident, $full_name:literal (sizes: $block_size:literal, $digest_words:literal, $message_length_bytes:literal) (insecure_against: $($attack_kind:literal),*), $mode_bits:literal ) => {
        #[doc = concat!("Hardware-accelerated ", $full_name, " implementation")]
        ///
        /// This struct manages the context and state required for processing data using the selected hashing algorithm.
        ///
        /// The struct provides various functionalities such as initializing the hashing
        /// process, updating the internal state with new data, and finalizing the
        /// hashing operation to generate the final digest.
        $(
            #[doc = ""]
            #[doc = concat!(" > ⚠️ Note that this algorithm is known to be insecure against ", $attack_kind, " attacks.")]
        )*
        #[non_exhaustive]
        pub struct $name;

        impl crate::private::Sealed for $name {}

        impl ShaAlgorithm for $name {
            const ALGORITHM: &'static str = stringify!($name);
            const ALGORITHM_KIND: ShaAlgorithmKind = ShaAlgorithmKind::$name;

            const CHUNK_LENGTH: usize = Self::ALGORITHM_KIND.chunk_length();
            const DIGEST_LENGTH: usize = Self::ALGORITHM_KIND.digest_length();

            type DigestOutputSize = paste::paste!(digest::consts::[< U $digest_words >]);
        }
    };
}

fn h_mem(sha: &crate::peripherals::SHA<'_>, index: usize) -> *mut u32 {
    let sha = sha.register_block();
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            sha.text(index).as_ptr()
        } else {
            sha.h_mem(index).as_ptr()
        }
    }
}

fn m_mem(sha: &crate::peripherals::SHA<'_>, index: usize) -> *mut u32 {
    let sha = sha.register_block();
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            sha.text(index).as_ptr()
        } else {
            sha.m_mem(index).as_ptr()
        }
    }
}
