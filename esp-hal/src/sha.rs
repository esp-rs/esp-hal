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
//!    * SHA-1
//!    * SHA-224
//!    * SHA-256
//!    * SHA-384
//!    * SHA-512
//!
//! The driver supports two working modes:
//!    * Typical SHA
//!    * DMA-SHA
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
//! ## Implementation State
//! - DMA-SHA Mode is not supported.

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

        state.cursor = state.cursor.wrapping_add(incoming.len() - remaining.len());

        if bound_reached {
            // Message is full now.

            if self.is_busy(state.algorithm) {
                // The message buffer is full and the hardware is still processing the previous
                // message. There's nothing to be done besides wait for the hardware.
                state.message_buffer_is_full = true;
            } else {
                // Send the full buffer.
                self.process_buffer(state);
            }
        }

        Ok(remaining)
    }

    fn finish(&self, state: &mut DigestState, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // Store message length for padding
        let length = (state.cursor as u64 * 8).to_be_bytes();
        nb::block!(self.update(state, &[0x80]))?; // Append "1" bit

        let chunk_len = state.algorithm.chunk_length();

        // Flush partial data, ensures aligned cursor
        {
            while self.is_busy(state.algorithm) {}
            if state.message_buffer_is_full {
                self.process_buffer(state);

                state.message_buffer_is_full = false;
                while self.is_busy(state.algorithm) {}
            }

            let flushed = state
                .alignment_helper
                .flush_to(m_mem(&self.sha, 0), state.cursor % chunk_len);
            state.cursor = state.cursor.wrapping_add(flushed);

            if flushed > 0 && state.cursor.is_multiple_of(chunk_len) {
                self.process_buffer(state);
                while self.is_busy(state.algorithm) {}
            }
        }
        debug_assert!(state.cursor.is_multiple_of(4));

        let mut mod_cursor = state.cursor % chunk_len;
        if (chunk_len - mod_cursor) < chunk_len / 8 {
            // Zero out remaining data if buffer is almost full (>=448/896), and process
            // buffer
            let pad_len = chunk_len - mod_cursor;
            state
                .alignment_helper
                .volatile_write(m_mem(&self.sha, 0), 0_u8, pad_len, mod_cursor);
            self.process_buffer(state);
            state.cursor = state.cursor.wrapping_add(pad_len);

            debug_assert_eq!(state.cursor % chunk_len, 0);
            mod_cursor = 0;

            // Spin-wait for finish
            while self.is_busy(state.algorithm) {}
        }

        let pad_len = chunk_len - mod_cursor - size_of::<u64>();

        state
            .alignment_helper
            .volatile_write(m_mem(&self.sha, 0), 0, pad_len, mod_cursor);

        state.alignment_helper.aligned_volatile_copy(
            m_mem(&self.sha, 0),
            &length,
            chunk_len,
            chunk_len - size_of::<u64>(),
        );

        self.process_buffer(state);
        // Spin-wait for final buffer to be processed
        while self.is_busy(state.algorithm) {}

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

        Ok(())
    }

    fn update<'a>(
        &self,
        state: &mut DigestState,
        incoming: &'a [u8],
    ) -> nb::Result<&'a [u8], Infallible> {
        state.finished = false;

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

#[derive(Clone, Debug)]
struct DigestState {
    algorithm: ShaAlgorithmKind,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished: bool,
    message_buffer_is_full: bool,
}

impl DigestState {
    fn new(algorithm: ShaAlgorithmKind) -> Self {
        Self {
            algorithm,
            alignment_helper: AlignmentHelper::default(),
            cursor: 0,
            first_run: true,
            finished: false,
            message_buffer_is_full: false,
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

    /// Indicates if the SHA context has finished processing the data.
    ///
    /// Returns `true` if the SHA calculation is complete, otherwise returns.
    pub fn finished(&self) -> bool {
        self.state.finished
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

/// This macro implements the Sha<'a, Dm> trait for a specified Sha algorithm
/// and a set of parameters
macro_rules! impl_sha {
    ($name: ident, $digest_length: literal) => {
        /// A SHA implementation struct.
        ///
        /// This struct is generated by the macro and represents a specific SHA hashing
        /// algorithm (e.g., SHA-256, SHA-1). It manages the context and state required
        /// for processing data using the selected hashing algorithm.
        ///
        /// The struct provides various functionalities such as initializing the hashing
        /// process, updating the internal state with new data, and finalizing the
        /// hashing operation to generate the final digest.
        #[non_exhaustive]
        pub struct $name;

        impl crate::private::Sealed for $name {}

        impl $crate::sha::ShaAlgorithm for $name {
            const ALGORITHM: &'static str = stringify!($name);
            const ALGORITHM_KIND: ShaAlgorithmKind = ShaAlgorithmKind::$name;

            const CHUNK_LENGTH: usize = Self::ALGORITHM_KIND.chunk_length();

            const DIGEST_LENGTH: usize = $digest_length;

            // We use paste to append `U` to the digest size to match a const defined in
            // digest
            type DigestOutputSize = paste::paste!(digest::consts::[< U $digest_length >]);
        }
    };
}

/// Specifies particular SHA algorithm.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ShaAlgorithmKind {
    /// The SHA-1 algorithm.
    ///
    /// Note that this algorithm is known to be insecure against collision attacks and length
    /// extension attacks.
    #[cfg(sha_algo_sha_1)]
    Sha1,

    /// The SHA-224 algorithm.
    ///
    /// Note that this algorithm is known to be insecure against length extension attacks.
    #[cfg(sha_algo_sha_224)]
    Sha224,

    /// The SHA-256 algorithm.
    ///
    /// Note that this algorithm is known to be insecure against length extension attacks.
    #[cfg(sha_algo_sha_256)]
    Sha256,

    /// The SHA-384 algorithm.
    #[cfg(sha_algo_sha_384)]
    Sha384,

    /// The SHA-512 algorithm.
    ///
    /// Note that this algorithm is known to be insecure against length extension attacks.
    #[cfg(sha_algo_sha_512)]
    Sha512,

    /// The SHA-512/224 algorithm.
    #[cfg(sha_algo_sha_512_224)]
    Sha512_224,

    /// The SHA-512/256 algorithm.
    #[cfg(sha_algo_sha_512_256)]
    Sha512_256,
    // TODO
    // #[allow(non_camel_case_types)]
    // #[cfg(sha_algo_sha_512_t)]
    // Sha512_t(u16),
}

impl ShaAlgorithmKind {
    #[cfg(not(esp32))]
    const fn mode_bits(self) -> u8 {
        match self {
            #[cfg(sha_algo_sha_1)]
            ShaAlgorithmKind::Sha1 => 0,
            #[cfg(sha_algo_sha_224)]
            ShaAlgorithmKind::Sha224 => 1,
            #[cfg(sha_algo_sha_256)]
            ShaAlgorithmKind::Sha256 => 2,
            #[cfg(sha_algo_sha_384)]
            ShaAlgorithmKind::Sha384 => 3,
            #[cfg(sha_algo_sha_512)]
            ShaAlgorithmKind::Sha512 => 4,
            #[cfg(sha_algo_sha_512_224)]
            ShaAlgorithmKind::Sha512_224 => 5,
            #[cfg(sha_algo_sha_512_256)]
            ShaAlgorithmKind::Sha512_256 => 6,
        }
    }

    const fn chunk_length(self) -> usize {
        match self {
            #[cfg(sha_algo_sha_1)]
            ShaAlgorithmKind::Sha1 => 64,
            #[cfg(sha_algo_sha_224)]
            ShaAlgorithmKind::Sha224 => 64,
            #[cfg(sha_algo_sha_256)]
            ShaAlgorithmKind::Sha256 => 64,
            #[cfg(sha_algo_sha_384)]
            ShaAlgorithmKind::Sha384 => 128,
            #[cfg(sha_algo_sha_512)]
            ShaAlgorithmKind::Sha512 => 128,
            #[cfg(sha_algo_sha_512_224)]
            ShaAlgorithmKind::Sha512_224 => 128,
            #[cfg(sha_algo_sha_512_256)]
            ShaAlgorithmKind::Sha512_256 => 128,
        }
    }

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

// All the hash algorithms introduced in FIPS PUB 180-4 Spec.
// – SHA-1
// – SHA-224
// – SHA-256
// – SHA-384
// – SHA-512
// – SHA-512/224
// – SHA-512/256
// – SHA-512/t (not implemented yet)
// Two working modes
// – Typical SHA
// – DMA-SHA (not implemented yet)
#[cfg(sha_algo_sha_1)]
impl_sha!(Sha1, 20);
#[cfg(sha_algo_sha_224)]
impl_sha!(Sha224, 28);
#[cfg(sha_algo_sha_256)]
impl_sha!(Sha256, 32);
#[cfg(sha_algo_sha_384)]
impl_sha!(Sha384, 48);
#[cfg(sha_algo_sha_512)]
impl_sha!(Sha512, 64);
#[cfg(sha_algo_sha_512_224)]
impl_sha!(Sha512_224, 28);
#[cfg(sha_algo_sha_512_256)]
impl_sha!(Sha512_256, 32);
// TODO: Allow/Implement SHA512_(u16)

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
