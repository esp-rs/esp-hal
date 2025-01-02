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
#![doc = crate::before_snippet!()]
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
//!     source_data = block!(hasher.update(source_data)).unwrap();
//! }
//!
//! // Finish can be called as many times as desired to get multiple copies of
//! // the output.
//! block!(hasher.finish(output.as_mut_slice())).unwrap();
//!
//! # }
//! ```
//! ## Implementation State
//! - DMA-SHA Mode is not supported.

use core::{borrow::BorrowMut, convert::Infallible, marker::PhantomData, mem::size_of};

/// Re-export digest for convenience
#[cfg(feature = "digest")]
pub use digest::Digest;

#[cfg(not(esp32))]
use crate::peripherals::Interrupt;
use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::SHA,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::GenericPeripheralGuard,
};

/// The SHA Accelerator driver instance
pub struct Sha<'d> {
    sha: PeripheralRef<'d, SHA>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Sha as u8 }>,
}

impl<'d> Sha<'d> {
    /// Create a new instance of the SHA Accelerator driver.
    pub fn new(sha: impl Peripheral<P = SHA> + 'd) -> Self {
        crate::into_ref!(sha);
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
}

impl crate::private::Sealed for Sha<'_> {}

#[cfg(not(esp32))]
impl crate::interrupt::InterruptConfigurable for Sha<'_> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::SHA);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::SHA, handler.handler()) };
        unwrap!(crate::interrupt::enable(Interrupt::SHA, handler.priority()));
    }
}

// A few notes on this implementation with regards to 'memcpy',
// - It seems that ptr::write_bytes already acts as volatile, while ptr::copy_*
//   does not (in this case)
// - The registers are *not* cleared after processing, so padding needs to be
//   written out
// - This component uses core::intrinsics::volatile_* which is unstable, but is
//   the only way to
// efficiently copy memory with volatile
// - For this particular registers (and probably others), a full u32 needs to be
//   written partial
// register writes (i.e. in u8 mode) does not work
//   - This means that we need to buffer bytes coming in up to 4 u8's in order
//     to create a full u32

/// An active digest
///
/// This implementation might fail after u32::MAX/8 bytes, to increase please
/// see ::finish() length/self.cursor usage
pub struct ShaDigest<'d, A, S: BorrowMut<Sha<'d>>> {
    sha: S,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished: bool,
    message_buffer_is_full: bool,
    phantom: PhantomData<(&'d (), A)>,
}

impl<'d, A: ShaAlgorithm, S: BorrowMut<Sha<'d>>> ShaDigest<'d, A, S> {
    /// Creates a new digest
    #[allow(unused_mut)]
    pub fn new(mut sha: S) -> Self {
        #[cfg(not(esp32))]
        // Setup SHA Mode.
        sha.borrow_mut()
            .sha
            .mode()
            .write(|w| unsafe { w.mode().bits(A::MODE_AS_BITS) });

        Self {
            sha,
            alignment_helper: AlignmentHelper::default(),
            cursor: 0,
            first_run: true,
            finished: false,
            message_buffer_is_full: false,
            phantom: PhantomData,
        }
    }

    /// Restores a previously saved digest.
    #[cfg(not(esp32))]
    pub fn restore(mut sha: S, ctx: &mut Context<A>) -> Self {
        // Setup SHA Mode.
        sha.borrow_mut()
            .sha
            .mode()
            .write(|w| unsafe { w.mode().bits(A::MODE_AS_BITS) });

        // Restore the message buffer
        unsafe {
            core::ptr::copy_nonoverlapping(
                ctx.buffer.as_ptr(),
                m_mem(&sha.borrow_mut().sha, 0),
                32,
            );
        }

        let mut ah = ctx.alignment_helper.clone();

        // Restore previously saved hash
        ah.volatile_write_regset(h_mem(&sha.borrow_mut().sha, 0), &ctx.saved_digest, 64);

        Self {
            sha,
            alignment_helper: ah,
            cursor: ctx.cursor,
            first_run: ctx.first_run,
            finished: ctx.finished,
            message_buffer_is_full: ctx.message_buffer_is_full,
            phantom: PhantomData,
        }
    }

    /// Returns true if the hardware is processing the next message.
    pub fn is_busy(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                A::is_busy(&self.sha.borrow().sha)
            } else {
                self.sha.borrow().sha.busy().read().state().bit_is_set()
            }
        }
    }

    /// Updates the SHA digest with the provided data buffer.
    pub fn update<'a>(&mut self, incoming: &'a [u8]) -> Option<&'a [u8]> {
        self.finished = false;

        self.write_data(incoming)
    }

    /// Finish of the calculation (if not already) and copy result to output
    /// After `finish()` is called `update()`s will contribute to a new hash
    /// which can be calculated again with `finish()`.
    ///
    /// Typically, output is expected to be the size of
    /// [ShaAlgorithm::DIGEST_LENGTH], but smaller inputs can be given to
    /// get a "short hash"
    pub fn finish(&mut self, output: &mut [u8]) -> Option<()> {
        // Store message length for padding
        let length = (self.cursor as u64 * 8).to_be_bytes();
        // Append "1" bit
        if self.update(&[0x80]).is_none() {
            return None;
        }

        // Flush partial data, ensures aligned cursor
        {
            while self.is_busy() {}
            if self.message_buffer_is_full {
                self.process_buffer();
                self.message_buffer_is_full = false;
                while self.is_busy() {}
            }

            let flushed = self.alignment_helper.flush_to(
                m_mem(&self.sha.borrow_mut().sha, 0),
                (self.cursor % A::CHUNK_LENGTH) / self.alignment_helper.align_size(),
            );
            self.cursor = self.cursor.wrapping_add(flushed);

            if flushed > 0 && self.cursor % A::CHUNK_LENGTH == 0 {
                self.process_buffer();
                while self.is_busy() {}
            }
        }
        debug_assert!(self.cursor % 4 == 0);

        let mod_cursor = self.cursor % A::CHUNK_LENGTH;
        if (A::CHUNK_LENGTH - mod_cursor) < A::CHUNK_LENGTH / 8 {
            // Zero out remaining data if buffer is almost full (>=448/896), and process
            // buffer
            let pad_len = A::CHUNK_LENGTH - mod_cursor;
            self.alignment_helper.volatile_write_bytes(
                m_mem(&self.sha.borrow_mut().sha, 0),
                0_u8,
                pad_len / self.alignment_helper.align_size(),
                mod_cursor / self.alignment_helper.align_size(),
            );
            self.process_buffer();
            self.cursor = self.cursor.wrapping_add(pad_len);

            debug_assert_eq!(self.cursor % A::CHUNK_LENGTH, 0);

            // Spin-wait for finish
            while self.is_busy() {}
        }

        let mod_cursor = self.cursor % A::CHUNK_LENGTH; // Should be zero if branched above
        let pad_len = A::CHUNK_LENGTH - mod_cursor - size_of::<u64>();

        self.alignment_helper.volatile_write_bytes(
            m_mem(&self.sha.borrow_mut().sha, 0),
            0,
            pad_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.alignment_helper.aligned_volatile_copy(
            m_mem(&self.sha.borrow_mut().sha, 0),
            &length,
            A::CHUNK_LENGTH / self.alignment_helper.align_size(),
            (A::CHUNK_LENGTH - size_of::<u64>()) / self.alignment_helper.align_size(),
        );

        self.process_buffer();
        // Spin-wait for final buffer to be processed
        while self.is_busy() {}

        // ESP32 requires additional load to retrieve output
        #[cfg(esp32)]
        {
            A::load(&mut self.sha.borrow_mut().sha);

            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
        }

        self.alignment_helper.volatile_read_regset(
            h_mem(&self.sha.borrow_mut().sha, 0),
            output,
            core::cmp::min(output.len(), 32) / self.alignment_helper.align_size(),
        );

        self.first_run = true;
        self.cursor = 0;
        self.alignment_helper.reset();

        Some(())
    }

    /// Save the current state of the digest for later continuation.
    #[cfg(not(esp32))]
    pub fn save(&mut self, context: &mut Context<A>) -> Option<()> {
        if self.is_busy() {
            return None;
        }

        context.alignment_helper = self.alignment_helper.clone();
        context.cursor = self.cursor;
        context.first_run = self.first_run;
        context.finished = self.finished;
        context.message_buffer_is_full = self.message_buffer_is_full;

        // Save the content of the current hash.
        self.alignment_helper.volatile_read_regset(
            h_mem(&self.sha.borrow_mut().sha, 0),
            &mut context.saved_digest,
            64 / self.alignment_helper.align_size(),
        );

        // Save the content of the current (probably partially written) message.
        unsafe {
            core::ptr::copy_nonoverlapping(
                m_mem(&self.sha.borrow_mut().sha, 0),
                context.buffer.as_mut_ptr(),
                32,
            );
        }

        Some(())
    }

    /// Discard the current digest and return the peripheral.
    pub fn cancel(self) -> S {
        self.sha
    }

    /// Processes the data buffer and updates the hash state.
    ///
    /// This method is platform-specific and differs for ESP32 and non-ESP32
    /// platforms.
    fn process_buffer(&mut self) {
        #[cfg(not(esp32))]
        if self.first_run {
            // Set SHA_START_REG
            self.sha
                .borrow_mut()
                .sha
                .start()
                .write(|w| unsafe { w.bits(1) });
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            self.sha
                .borrow_mut()
                .sha
                .continue_()
                .write(|w| unsafe { w.bits(1) });
        }

        #[cfg(esp32)]
        if self.first_run {
            A::start(&mut self.sha.borrow_mut().sha);
            self.first_run = false;
        } else {
            A::r#continue(&mut self.sha.borrow_mut().sha);
        }
    }

    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> Option<&'a [u8]> {
        if self.message_buffer_is_full {
            if self.is_busy() {
                // The message buffer is full and the hardware is still processing the previous
                // message. There's nothing to be done besides wait for the hardware.
                return None;
            } else {
                // Submit the full buffer.
                self.process_buffer();
                // The buffer is now free for filling.
                self.message_buffer_is_full = false;
            }
        }

        let mod_cursor = self.cursor % A::CHUNK_LENGTH;
        let chunk_len = A::CHUNK_LENGTH;

        let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(
            m_mem(&self.sha.borrow().sha, 0),
            incoming,
            chunk_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.cursor = self.cursor.wrapping_add(incoming.len() - remaining.len());

        if bound_reached {
            // Message is full now.

            if self.is_busy() {
                // The message buffer is full and the hardware is still processing the previous
                // message. There's nothing to be done besides wait for the hardware.
                self.message_buffer_is_full = true;
            } else {
                // Send the full buffer.
                self.process_buffer();
            }
        }

        Some(remaining)
    }
}

#[cfg(not(esp32))]
/// Context for a SHA Accelerator driver instance
#[derive(Debug, Clone)]
pub struct Context<A: ShaAlgorithm> {
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished: bool,
    message_buffer_is_full: bool,
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
            cursor: 0,
            first_run: true,
            finished: false,
            message_buffer_is_full: false,
            alignment_helper: AlignmentHelper::default(),
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
        self.first_run
    }

    /// Indicates if the SHA context has finished processing the data.
    ///
    /// Returns `true` if the SHA calculation is complete, otherwise returns.
    pub fn finished(&self) -> bool {
        self.finished
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

    /// The length of the chunk that the algorithm processes at a time.
    ///
    /// For example, in SHA-256, this would typically be 64 bytes.
    const CHUNK_LENGTH: usize;

    /// The length of the resulting digest produced by the algorithm.
    ///
    /// For example, in SHA-256, this would be 32 bytes.
    const DIGEST_LENGTH: usize;

    #[cfg(feature = "digest")]
    #[doc(hidden)]
    type DigestOutputSize: digest::generic_array::ArrayLength<u8> + 'static;

    #[cfg(not(esp32))]
    #[doc(hidden)]
    const MODE_AS_BITS: u8;

    #[cfg(esp32)]
    #[doc(hidden)]
    // Initiate the operation
    fn start(sha: &mut crate::peripherals::SHA);

    #[cfg(esp32)]
    #[doc(hidden)]
    // Continue the operation
    fn r#continue(sha: &mut crate::peripherals::SHA);

    #[cfg(esp32)]
    #[doc(hidden)]
    // Calculate the final hash
    fn load(sha: &mut crate::peripherals::SHA);

    #[cfg(esp32)]
    #[doc(hidden)]
    // Check if peripheral is busy
    fn is_busy(sha: &crate::peripherals::SHA) -> bool;
}

/// implement digest traits if digest feature is present.
/// Note: digest has a blanket trait implementation for [digest::Digest] for any
/// element that implements FixedOutput + Default + Update + HashMarker
#[cfg(feature = "digest")]
impl<'d, A: ShaAlgorithm, S: BorrowMut<Sha<'d>>> digest::HashMarker for ShaDigest<'d, A, S> {}

#[cfg(feature = "digest")]
impl<'d, A: ShaAlgorithm, S: BorrowMut<Sha<'d>>> digest::OutputSizeUser for ShaDigest<'d, A, S> {
    type OutputSize = A::DigestOutputSize;
}

#[cfg(feature = "digest")]
impl<'d, A: ShaAlgorithm, S: BorrowMut<Sha<'d>>> digest::Update for ShaDigest<'d, A, S> {
    fn update(&mut self, data: &[u8]) {
        let mut remaining = data.as_ref();
        while !remaining.is_empty() {
            remaining = nb::block!(Self::update(self, remaining)).unwrap();
        }
    }
}

#[cfg(feature = "digest")]
impl<'d, A: ShaAlgorithm, S: BorrowMut<Sha<'d>>> digest::FixedOutput for ShaDigest<'d, A, S> {
    fn finalize_into(mut self, out: &mut digest::Output<Self>) {
        nb::block!(self.finish(out)).unwrap();
    }
}

/// This macro implements the Sha<'a, Dm> trait for a specified Sha algorithm
/// and a set of parameters
macro_rules! impl_sha {
    ($name: ident, $mode_bits: tt, $digest_length: tt, $chunk_length: tt) => {
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

            const CHUNK_LENGTH: usize = $chunk_length;

            const DIGEST_LENGTH: usize = $digest_length;

            #[cfg(not(esp32))]
            const MODE_AS_BITS: u8 = $mode_bits;

            #[cfg(feature = "digest")]
            // We use paste to append `U` to the digest size to match a const defined in
            // digest
            type DigestOutputSize = paste::paste!(digest::consts::[< U $digest_length >]);

            #[cfg(esp32)]
            fn start(sha: &mut crate::peripherals::SHA) {
                paste::paste! {
                    sha.[< $name:lower _start >]().write(|w| w.[< $name:lower _start >]().set_bit());
                }
            }

            #[cfg(esp32)]
            fn r#continue(sha: &mut crate::peripherals::SHA) {
                paste::paste! {
                    sha.[< $name:lower _continue >]().write(|w| w.[< $name:lower _continue >]().set_bit());
                }
            }

            #[cfg(esp32)]
            fn load(sha: &mut crate::peripherals::SHA) {
                paste::paste! {
                    sha.[< $name:lower _load >]().write(|w| w.[< $name:lower _load >]().set_bit());
                }
            }

            #[cfg(esp32)]
            fn is_busy(sha: &crate::peripherals::SHA) -> bool {
                paste::paste! {
                    sha.[< $name:lower _busy >]().read().[< $name:lower _busy >]().bit_is_set()
                }
            }
        }
    };
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
//
// TODO: Allow/Implement SHA512_(u16)
impl_sha!(Sha1, 0, 20, 64);
#[cfg(not(esp32))]
impl_sha!(Sha224, 1, 28, 64);
impl_sha!(Sha256, 2, 32, 64);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_sha!(Sha384, 3, 48, 128);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_sha!(Sha512, 4, 64, 128);
#[cfg(any(esp32s2, esp32s3))]
impl_sha!(Sha512_224, 5, 28, 128);
#[cfg(any(esp32s2, esp32s3))]
impl_sha!(Sha512_256, 6, 32, 128);

fn h_mem(sha: &crate::peripherals::SHA, index: usize) -> *mut u32 {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            sha.text(index).as_ptr()
        } else {
            sha.h_mem(index).as_ptr()
        }
    }
}

fn m_mem(sha: &crate::peripherals::SHA, index: usize) -> *mut u32 {
    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            sha.text(index).as_ptr()
        } else {
            sha.m_mem(index).as_ptr()
        }
    }
}
