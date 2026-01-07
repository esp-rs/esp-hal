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
//!
//! ### Using the `Sha` driver
//!
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
//!
//! ### Using the `ShaBackend` driver
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::sha::{Sha1Context, ShaBackend};
//! #
//! let mut sha = ShaBackend::new(peripherals.SHA);
//! // Start the backend, which allows processing SHA operations.
//! let _backend = sha.start();
//!
//! // Create a new context to hash data with SHA-1.
//! let mut sha1_ctx = Sha1Context::new();
//!
//! // SHA-1 outputs a 20-byte digest.
//! let mut digest: [u8; 20] = [0; 20];
//!
//! // Process data. The `update` function returns a handle which can be used to wait
//! // for the operation to finish.
//! sha1_ctx.update(b"input data").wait_blocking();
//! sha1_ctx.update(b"input data").wait_blocking();
//! sha1_ctx.update(b"input data").wait_blocking();
//!
//! // Extract the final hash. This resets the context.
//! sha1_ctx.finalize(&mut digest).wait_blocking();
//!
//! // digest now contains the SHA-1 hash of the input.
//! #
//! # {after_snippet}
//! ```

#![allow(deprecated, reason = "generic_array 0.14 has been deprecated")]

use core::{borrow::Borrow, convert::Infallible, marker::PhantomData, mem::size_of, ptr::NonNull};

/// Re-export digest for convenience
pub use digest::Digest;

use crate::{
    peripherals::SHA,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::GenericPeripheralGuard,
    work_queue::{Handle, Poll, Status, VTable, WorkQueue, WorkQueueDriver, WorkQueueFrontend},
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
    (algos $( ( $name:ident, $full_name:literal (sizes: $block_size:literal, $digest_len:literal, $message_length_bytes:literal) $security:tt, $mode_bits:literal ) ),*) => {
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
                    $(ShaAlgorithmKind::$name => $digest_len,)*
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
    ( $name:ident, $full_name:literal (sizes: $block_size:literal, $digest_len:literal, $message_length_bytes:literal) (insecure_against: $($attack_kind:literal),*), $mode_bits:literal ) => {
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

            type DigestOutputSize = paste::paste!(digest::consts::[< U $digest_len >]);
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

#[derive(Clone)]
struct ShaOperation {
    operation: ShaOperationKind,
    // Buffer containing pieced-together message bytes, not necessarily a complete block. Not a fat
    // pointer because the driver will update the number of buffered bytes.
    buffer: NonNull<u8>,
    buffered_bytes: u8,
    message: NonNull<[u8]>,
    // Intermediate hash value. For SHA-512/t, this may also contain the initial hash (unverified
    // claim)
    #[cfg(not(esp32))]
    hw_state: NonNull<[u32]>,
    // TODO: we don't really need DigestState, especially not the alignment helper, but the driver
    // is formulated like this and so reusing what we have is simpler.
    state: DigestState,
}

impl ShaOperation {
    fn reset(&mut self) {
        self.state = DigestState::new(self.state.algorithm);
        self.buffered_bytes = 0;
    }
}

// Safety: ShaOperation is safe to share between threads, in the context of a WorkQueue. The
// WorkQueue ensures that only a single location can access the data. All the internals, except
// for the pointers, are Sync. The pointers are safe to share because they point at data that the
// SHA driver ensures can be accessed safely and soundly.
unsafe impl Sync for ShaOperation {}
// Safety: we will not hold on to the pointers when the work item leaves the queue.
unsafe impl Send for ShaOperation {}

static SHA_WORK_QUEUE: WorkQueue<ShaOperation> = WorkQueue::new();
const BLOCKING_SHA_VTABLE: VTable<ShaOperation> = VTable {
    post: |driver, item| {
        let driver = unsafe { ShaBackend::from_raw(driver) };

        // Ensure driver is initialized
        if let DriverState::Uninitialized(sha) = &driver.driver {
            driver.driver = DriverState::Initialized(Sha::new(unsafe { sha.clone_unchecked() }));
        };

        Some(driver.process(item))
    },
    poll: |driver, item| {
        let driver = unsafe { ShaBackend::from_raw(driver) };
        driver.process(item)
    },
    cancel: |_driver, _item| {
        // To achieve a decent performance in Typical SHA mode, we run the operations in a blocking
        // manner and so they can't be cancelled.
    },
    stop: |driver| {
        // Drop the SHA driver to conserve power when there is nothig to do (or when the driver was
        // stopped).
        let driver = unsafe { ShaBackend::from_raw(driver) };
        driver.deinitialize()
    },
};

/// Contains information about processing the current work item.
struct ProcessingState {
    message_partially_processed: bool,
    message_bytes_processed: usize,
}

enum DriverState<'d> {
    Uninitialized(SHA<'d>),
    Initialized(Sha<'d>),
}

#[derive(Clone, Copy, PartialEq)]
enum ShaOperationKind {
    Update,
    Finalize,
}

#[procmacros::doc_replace]
/// CPU-driven SHA processing backend.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::sha::{Sha1Context, ShaBackend};
/// #
/// let mut sha = ShaBackend::new(peripherals.SHA);
/// // Start the backend, which allows processing SHA operations.
/// let _backend = sha.start();
///
/// // Create a new context to hash data with SHA-1.
/// let mut sha1_ctx = Sha1Context::new();
///
/// // SHA-1 outputs a 20-byte digest.
/// let mut digest: [u8; 20] = [0; 20];
///
/// // Process data. The `update` function returns a handle which can be used to wait
/// // for the operation to finish.
/// sha1_ctx.update(b"input data").wait_blocking();
/// sha1_ctx.update(b"input data").wait_blocking();
/// sha1_ctx.update(b"input data").wait_blocking();
///
/// // Extract the final hash. This resets the context.
/// sha1_ctx.finalize(&mut digest).wait_blocking();
///
/// // digest now contains the SHA-1 hash of the input.
/// #
/// # {after_snippet}
/// ```
pub struct ShaBackend<'d> {
    driver: DriverState<'d>,
    processing_state: ProcessingState,
}
impl<'d> ShaBackend<'d> {
    /// Creates a new SHA backend.
    ///
    /// The backend needs to be [`start`][Self::start]ed before it can execute SHA operations.
    pub fn new(sha: SHA<'d>) -> Self {
        Self {
            driver: DriverState::Uninitialized(sha),
            processing_state: ProcessingState {
                message_partially_processed: false,
                message_bytes_processed: 0,
            },
        }
    }

    /// Registers the CPU-driven SHA driver to process SHA operations.
    ///
    /// The driver stops operating when the returned object is dropped.
    pub fn start(&mut self) -> ShaWorkQueueDriver<'_, 'd> {
        ShaWorkQueueDriver {
            inner: WorkQueueDriver::new(self, BLOCKING_SHA_VTABLE, &SHA_WORK_QUEUE),
        }
    }

    // WorkQueue callbacks. They may run in any context.

    unsafe fn from_raw<'any>(ptr: NonNull<()>) -> &'any mut Self {
        unsafe { ptr.cast::<ShaBackend<'_>>().as_mut() }
    }

    fn process(&mut self, item: &mut ShaOperation) -> Poll {
        // The hardware allows us to write the next message block while it's still processing the
        // current one. We can only do this if the current message block belongs to the same message
        // as the previous one. This is unfortunately not a safe operation in the work queue,
        // because we can't inspect the next work item during the processing of the current
        // one.
        // However, we can receive a message with multiple blocks and we can apply the optimization
        // while processing the message bytes here.
        // Another optimization we can do is skipping restoring the intermediate hash value if the
        // next operation is a continuation of the current one. We still need to save it between
        // operations, though.

        // In this function we process a number of message blocks.
        // There are two possible cases we need to handle:
        // - We receive message bytes that, including the buffered bytes, are enough for at least
        //   one complete SHA block. We need to copy whole blocks into the hardware, do the
        //   computation, save the state, and save the remaining data in the buffer.
        // - We receive message bytes that, including the buffered bytes, are not enough for a
        //   complete SHA block. This case marks the end of the message. We will need to apply
        //   padding, then finalize the computation.

        if item.operation == ShaOperationKind::Update {
            self.process_update(item)
        } else {
            self.process_finalize(item)
        }
    }

    #[cfg(not(esp32))]
    fn restore_state(driver: &mut Sha<'_>, item: &ShaOperation) {
        driver
            .sha
            .register_block()
            .mode()
            .write(|w| unsafe { w.mode().bits(item.state.algorithm.mode_bits()) });

        // Restore previously saved hash. Don't bother on first_run, the start operation will
        // use a hard-coded initial hash.
        if !item.state.first_run {
            for (i, reg) in driver.sha.register_block().h_mem_iter().enumerate() {
                reg.write(|w| unsafe { w.bits(item.hw_state.as_ref()[i]) });
            }
        }
    }

    fn process_update(&mut self, item: &mut ShaOperation) -> Poll {
        let driver = if let DriverState::Initialized(sha) = &mut self.driver {
            sha
        } else {
            unreachable!()
        };

        // The message can be longer than a SHA block, so we track the processed bytes.
        if !self.processing_state.message_partially_processed {
            self.processing_state = ProcessingState {
                message_partially_processed: true,
                message_bytes_processed: 0,
            };

            #[cfg(not(esp32))]
            Self::restore_state(driver, item);

            let buffered = unsafe {
                core::slice::from_raw_parts(item.buffer.as_ptr(), item.buffered_bytes as usize)
            };
            debug!(
                "update: restored state with {} buffered bytes",
                buffered.len()
            );

            // This is never supposed to block or even start processing, we're writing an incomplete
            // block into idle hardware.
            debug_assert!(buffered.len() < item.state.algorithm.chunk_length());
            nb::block!(driver.write_data(&mut item.state, buffered)).unwrap();
        }

        let remaining_message =
            unsafe { &item.message.as_ref()[self.processing_state.message_bytes_processed..] };

        // Even if we don't have a full chunk's worth of message, this function was called because
        // with the buffered data we have enough. If `message_processed == 0`, we have written a
        // partial block loaded in the hardware and will need to fill it out with message bytes.
        if remaining_message.len() >= item.state.algorithm.chunk_length()
            || self.processing_state.message_bytes_processed == 0
        {
            if let Ok(remaining) = driver.write_data(&mut item.state, remaining_message) {
                let total_consumed = item.message.len() - remaining.len();
                self.processing_state.message_bytes_processed = total_consumed;
            }

            // Request recall, SHA doesn't have an interrupt.
            return Poll::Pending(true);
        } else {
            // It's not necessary to wait here in the first call - no operation can be running when
            // we only restored a partial buffer.

            // Unfortunately we don't see the future - we can't tell if the next operation can be
            // overlapped with this one, so we need to wait for processing to complete so that we
            // can save the context - ESP32 is an exception, as only one context can use the
            // hardware at a time.
            #[cfg(not(esp32))]
            if driver.is_busy(item.state.algorithm) {
                // Request recall, SHA doesn't have an interrupt.
                return Poll::Pending(true);
            }
        }

        // Save hash. If `!first_run`, we did not start an operation so there is no hash to save.
        #[cfg(not(esp32))]
        if !item.state.first_run {
            for (i, reg) in driver.sha.register_block().h_mem_iter().enumerate() {
                unsafe { item.hw_state.as_mut()[i] = reg.read().bits() };
            }
        }

        // We can only process complete blocks before finalization. Write back the unprocessed bytes
        // to the item's buffer.
        if !remaining_message.is_empty() {
            debug!(
                "Writing back {} unprocessed bytes to buffer",
                remaining_message.len()
            );
            unsafe {
                // Safety: the frontend ensures that the buffer is large enough to hold the
                // remaining message.
                core::ptr::copy_nonoverlapping(
                    remaining_message.as_ptr(),
                    item.buffer.as_ptr(),
                    remaining_message.len(),
                );
            }
        }
        item.buffered_bytes = remaining_message.len() as u8;
        self.processing_state.message_partially_processed = false;

        Poll::Ready(Status::Completed)
    }

    fn process_finalize(&mut self, item: &mut ShaOperation) -> Poll {
        let driver = if let DriverState::Initialized(sha) = &mut self.driver {
            sha
        } else {
            unreachable!()
        };

        if !self.processing_state.message_partially_processed {
            // We don't need to track the byte count here, just that we've restored the hash and
            // written the buffered data. `process_finalize` ignores `message_bytes_processed`.
            self.processing_state.message_partially_processed = true;

            #[cfg(not(esp32))]
            Self::restore_state(driver, item);

            let buffered = unsafe { item.message.as_ref() };
            debug!(
                "finalize: restored state with {} buffered bytes",
                buffered.len()
            );

            // This is never supposed to block or even start processing, we're writing an incomplete
            // block into idle hardware.
            debug_assert!(buffered.len() < item.state.algorithm.chunk_length());

            nb::block!(driver.write_data(&mut item.state, buffered)).unwrap();
        }

        // Safety: caller must ensure that result buffer is large enough.
        let result = unsafe {
            core::slice::from_raw_parts_mut(
                item.buffer.as_ptr(),
                item.state.algorithm.digest_length(),
            )
        };
        if driver.finish(&mut item.state, result).is_err() {
            return Poll::Pending(true);
        }

        self.processing_state.message_partially_processed = false;

        item.reset();

        Poll::Ready(Status::Completed)
    }

    fn deinitialize(&mut self) {
        if let DriverState::Initialized(ref sha) = self.driver {
            self.driver = DriverState::Uninitialized(unsafe { sha.sha.clone_unchecked() });
        }
    }
}

/// An active work queue driver.
///
/// This object must be kept around, otherwise SHA operations will never complete.
pub struct ShaWorkQueueDriver<'t, 'd> {
    inner: WorkQueueDriver<'t, ShaBackend<'d>, ShaOperation>,
}

impl<'t, 'd> ShaWorkQueueDriver<'t, 'd> {
    /// Finishes processing the current work queue item, then stops the driver.
    pub fn stop(self) -> impl Future<Output = ()> {
        self.inner.stop()
    }
}

#[cfg(esp32)]
enum SoftwareHasher {
    Sha1(sha1::Sha1),
    Sha256(sha2::Sha256),
    Sha384(sha2::Sha384),
    Sha512(sha2::Sha512),
}

// Common implementation, to be hidden behind algo-dependent contexts.
#[cfg_attr(not(esp32), derive(Clone))]
struct ShaContext<const CHUNK_BYTES: usize, const DIGEST_WORDS: usize> {
    frontend: WorkQueueFrontend<ShaOperation>,
    buffer: [u8; CHUNK_BYTES],

    #[cfg(not(esp32))] // Saved H_MEM registers
    state: [u32; 16],

    // ESP32 can't save (or rather, restore) the hash context, so we need to fall back to software
    // if a context already uses the hardware.
    #[cfg(esp32)]
    use_software: Option<SoftwareHasher>,
}

// ESP32 can't save the context, so it can't support interleaved operation. To support the digest
// API, we need to limit the number of concurrent contexts - to 1. Thankfully ESP32 has no DSA/HMAC,
// so we don't need to worry about figuring THAT edge case out.
// Currently we fall back to a software implementation if multiple contexts are created. Instead, we
// could save the hash like we do on other MCUs, and continue with software if another context has
// been created in the mean time. This would prevent a case where a context could indefinitely
// reserve the hardware.
#[cfg(esp32)]
use portable_atomic::{AtomicBool, Ordering};
#[cfg(esp32)]
static ACCELERATOR_IN_USE: AtomicBool = AtomicBool::new(false);

impl<const CHUNK_BYTES: usize, const DIGEST_WORDS: usize> ShaContext<CHUNK_BYTES, DIGEST_WORDS> {
    fn new(algorithm: ShaAlgorithmKind) -> Self {
        #[cfg(esp32)]
        let use_software = if ACCELERATOR_IN_USE.swap(true, Ordering::SeqCst) {
            let hasher = match algorithm {
                ShaAlgorithmKind::Sha1 => SoftwareHasher::Sha1(sha1::Sha1::new()),
                ShaAlgorithmKind::Sha256 => SoftwareHasher::Sha256(sha2::Sha256::new()),
                ShaAlgorithmKind::Sha384 => SoftwareHasher::Sha384(sha2::Sha384::new()),
                ShaAlgorithmKind::Sha512 => SoftwareHasher::Sha512(sha2::Sha512::new()),
            };
            Some(hasher)
        } else {
            None
        };

        Self {
            frontend: WorkQueueFrontend::new(ShaOperation {
                operation: ShaOperationKind::Update,
                buffer: NonNull::dangling(),
                message: NonNull::from(&mut []),
                #[cfg(not(esp32))]
                hw_state: NonNull::from(&mut []),
                buffered_bytes: 0,
                state: DigestState::new(algorithm),
            }),
            buffer: [0; CHUNK_BYTES],
            #[cfg(not(esp32))]
            state: [0; 16],

            #[cfg(esp32)]
            use_software,
        }
    }

    fn update<'t>(&'t mut self, data: &'t [u8]) -> ShaHandle<'t> {
        debug!(
            "Update {:?} with {} bytes",
            self.frontend.data_mut().state.algorithm,
            data.len()
        );
        #[cfg(esp32)]
        if let Some(hasher) = self.use_software.as_mut() {
            Self::update_using_software(hasher, data);
            return ShaHandle(self.frontend.post_completed(&SHA_WORK_QUEUE));
        }

        let op_data = self.frontend.data_mut();

        // We want to pass complete blocks to the worker. If we pass an incomplete block, it must be
        // the last one. Because update can take any number of bytes, we need to buffer a block's
        // worth, truncate data to the nearest block size, then save the rest.
        // Buffering data here, and handling head/data/tail processing is way too annoying.
        // Therefore, we implement the following strategy:
        // - If we can buffer the data, we do.
        // - If we can't, we post a work item to the queue. The work item includes the buffer, the
        //   data, and the state pointers.
        // - The worker will load and process data as it sees fit. The worker will put the remaining
        //   data into the buffer, and update the state and buffer length.
        // - The worker is responsible for updating the buffer and state. It will set `first` to
        //   false if it saves an intermediate result into `state`.
        // This means chunking and remainder handling is the responsibility of the worker.
        let buffered = op_data.buffered_bytes as usize;
        if data.len() + buffered < CHUNK_BYTES {
            op_data.buffered_bytes += data.len() as u8;
            op_data.message = NonNull::from(data); // Ensure message.len() returns the consumed length

            self.buffer[buffered..][..data.len()].copy_from_slice(data);
            return ShaHandle(self.frontend.post_completed(&SHA_WORK_QUEUE));
        }

        // If a block is complete, we send it to the hardware. If the data we send is a whole number
        // of complete blocks, the driver will leave `buffer` empty. This ensures that we
        // never start a finalize operation with a complete block.

        op_data.operation = ShaOperationKind::Update;
        op_data.message = NonNull::from(data);
        op_data.buffer = NonNull::from(&mut self.buffer).cast();
        #[cfg(not(esp32))]
        {
            op_data.hw_state = NonNull::from(&mut self.state);
        }

        ShaHandle(self.frontend.post(&SHA_WORK_QUEUE))
    }

    fn finalize<'t>(&'t mut self, result: &mut [u8]) -> ShaHandle<'t> {
        debug!(
            "Finalize {:?} into buffer of {} bytes",
            self.frontend.data_mut().state.algorithm,
            result.len()
        );
        #[cfg(esp32)]
        if let Some(hasher) = self.use_software.as_mut() {
            Self::finalize_using_software(hasher, result);
            return ShaHandle(self.frontend.post_completed(&SHA_WORK_QUEUE));
        }

        let op_data = self.frontend.data_mut();

        debug_assert!((op_data.buffered_bytes as usize) < op_data.state.algorithm.chunk_length());

        op_data.operation = ShaOperationKind::Finalize;
        op_data.message = NonNull::from(&mut self.buffer[..op_data.buffered_bytes as usize]);
        op_data.buffer = NonNull::from(result).cast();
        #[cfg(not(esp32))]
        {
            op_data.hw_state = NonNull::from(&mut self.state);
        }

        ShaHandle(self.frontend.post(&SHA_WORK_QUEUE))
    }

    #[cfg(esp32)]
    fn update_using_software(hasher: &mut SoftwareHasher, data: &[u8]) {
        match hasher {
            SoftwareHasher::Sha1(sha) => sha.update(data),
            SoftwareHasher::Sha256(sha) => sha.update(data),
            SoftwareHasher::Sha384(sha) => sha.update(data),
            SoftwareHasher::Sha512(sha) => sha.update(data),
        }
    }

    #[cfg(esp32)]
    fn finalize_using_software(hasher: &mut SoftwareHasher, result: &mut [u8]) {
        match hasher {
            SoftwareHasher::Sha1(sha) => {
                let output = sha.finalize_reset();
                result.copy_from_slice(output.as_slice())
            }
            SoftwareHasher::Sha256(sha) => {
                let output = sha.finalize_reset();
                result.copy_from_slice(output.as_slice())
            }
            SoftwareHasher::Sha384(sha) => {
                let output = sha.finalize_reset();
                result.copy_from_slice(output.as_slice())
            }
            SoftwareHasher::Sha512(sha) => {
                let output = sha.finalize_reset();
                result.copy_from_slice(output.as_slice())
            }
        }
    }
}

#[cfg(esp32)]
impl<const CHUNK_BYTES: usize, const DIGEST_WORDS: usize> Drop
    for ShaContext<CHUNK_BYTES, DIGEST_WORDS>
{
    fn drop(&mut self) {
        ACCELERATOR_IN_USE.store(false, Ordering::Release);
    }
}

/// A handle for an in-progress operation, returned by [`update`](Sha1Context::update) or
/// [`finalize`](Sha1Context::finalize).
pub struct ShaHandle<'t>(Handle<'t, ShaOperation>);

impl ShaHandle<'_> {
    /// Polls the status of the work item.
    ///
    /// This function returns `true` if the item has been processed.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.0.poll()
    }

    /// Polls the work item to completion, by busy-looping.
    ///
    /// This function returns immediately if `poll` returns `true`.
    #[inline]
    pub fn wait_blocking(self) -> Status {
        self.0.wait_blocking()
    }

    /// Waits until the work item is completed.
    #[inline]
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        self.0.wait()
    }

    /// Cancels the work item and asynchronously waits until it is removed from the work queue.
    #[inline]
    pub fn cancel(&mut self) -> impl Future<Output = ()> {
        self.0.cancel()
    }
}

/// Error type returned by [`finalize_into_slice`](Sha1Context::finalize_into_slice).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FinalizeError {
    /// The provided buffer is smaller than the digest output of the algorithm.
    BufferTooSmall,
}

// Now implement the actual public types.
// Helper macro to limit the scope of `paste`
macro_rules! impl_worker_context {
    ($name:ident, $full_name:literal, $algo:expr, $digest_len:literal ) => {
        #[doc = concat!("A ", $full_name, " context.")]
        #[cfg_attr(not(esp32), derive(Clone))]
        pub struct $name(ShaContext<{ $algo.chunk_length() }, { $algo.digest_length() / 4 }>);

        impl $name {
            /// Creates a new context.
            ///
            /// The context represents the in-progress processing of a single message. You need to
            /// feed message bytes to [`Self::update`], then finalize the process using
            /// [`Self::finalize`].
            ///
            /// Any number of contexts can be created, to hash any number of messages concurrently.
            pub fn new() -> Self {
                Self(ShaContext::new($algo))
            }

            /// Hashes `data`.
            pub fn update<'t>(&'t mut self, data: &'t [u8]) -> ShaHandle<'t> {
                self.0.update(data)
            }

            /// Finishes the hashing process, writes the final hash to `result`.
            ///
            /// Resets the context to an empty state.
            pub fn finalize<'t>(
                &'t mut self,
                result: &mut [u8; { $algo.digest_length() }],
            ) -> ShaHandle<'t> {
                self.0.finalize(result)
            }

            /// Finishes the hashing process, writes the final hash to `result`.
            ///
            /// Resets the context to an empty state.
            ///
            /// Returns [`FinalizeError::BufferTooSmall`] if `result` is not large enough to hold
            /// the final hash.
            pub fn finalize_into_slice<'t>(
                &'t mut self,
                result: &mut [u8],
            ) -> Result<ShaHandle<'t>, FinalizeError> {
                if result.len() < $algo.digest_length() {
                    return Err(FinalizeError::BufferTooSmall);
                }

                Ok(self.0.finalize(result))
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        // Implementing these implies Digest, too
        impl digest::HashMarker for $name {}

        impl digest::OutputSizeUser for $name {
            type OutputSize = paste::paste!(digest::consts::[< U $digest_len >]);
        }

        impl digest::Update for $name {
            fn update(&mut self, data: &[u8]) {
                Self::update(self, data).wait_blocking();
            }
        }

        impl digest::FixedOutput for $name {
            fn finalize_into(mut self, out: &mut digest::Output<Self>) {
                Self::finalize(&mut self, out.as_mut()).wait_blocking();
            }
        }
    };
}

for_each_sha_algorithm! {
    ( $name:ident, $full_name:literal (sizes: $block_size:literal, $digest_len:literal, $message_length_bytes:literal) $security:tt, $mode_bits:literal ) => {
        paste::paste! {
            impl_worker_context!( [<$name Context>], $full_name, ShaAlgorithmKind::$name, $digest_len );
        }
    };
}
