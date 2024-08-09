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
//! # use core::option::Option::None;
//! # use nb::block;
//! let source_data = "HELLO, ESPRESSIF!".as_bytes();
//! let mut remaining = source_data;
//! let mut hasher = Sha256::new(peripherals.SHA);
//! // Short hashes can be created by decreasing the output buffer to the
//! // desired length
//! let mut output = [0u8; 32];
//!
//! while remaining.len() > 0 {
//!     // All the HW Sha functions are infallible so unwrap is fine to use if
//!     // you use block!
//!     remaining = block!(hasher.update(remaining)).unwrap();
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

#![allow(missing_docs)] // TODO: Remove when able

use core::{convert::Infallible, marker::PhantomData};

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::SHA,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::PeripheralClockControl,
};

/// Context for a SHA Accelerator driver instance
#[derive(Debug, Clone)]
pub struct Context<DM: crate::Mode> {
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished: bool,
    /// Buffered bytes (SHA_M_n_REG) to be processed.
    buffer: [u32; 32],
    /// Saved digest (SHA_H_n_REG) for interleaving operation
    #[cfg(not(esp32))]
    saved_digest: Option<[u8; 64]>,
    phantom: PhantomData<DM>,
}

impl crate::private::Sealed for Context<crate::Blocking> {}

#[cfg(not(esp32))]
impl crate::InterruptConfigurable for Context<crate::Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::SHA, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::SHA, handler.priority())
                .unwrap();
        }
    }
}

impl<DM: crate::Mode> Context<DM> {
    pub fn first_run(&self) -> bool {
        self.first_run
    }

    pub fn finished(&self) -> bool {
        self.finished
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

// This implementation might fail after u32::MAX/8 bytes, to increase please see
// ::finish() length/self.cursor usage
pub trait Sha<'d, DM: crate::Mode>: core::ops::DerefMut<Target = Context<DM>> {
    /// Constant containing the name of the algorithm as a string.
    const ALGORITHM: &'static str;

    /// Setup SHA Mode
    #[cfg(not(esp32))]
    fn mode_as_bits() -> u8;

    fn chunk_length(&self) -> usize;

    fn digest_length(&self) -> usize;

    /// ESP32 requires that a control register to be written to calculate the
    /// final SHA hash.
    #[cfg(esp32)]
    fn load_reg(&self);

    /// ESP32 uses a different register per hash mode.
    #[cfg(esp32)]
    fn is_busy(&self) -> bool;

    #[cfg(not(esp32))]
    fn is_busy(&self) -> bool {
        let sha = unsafe { crate::peripherals::SHA::steal() };
        sha.busy().read().bits() != 0
    }

    #[cfg(esp32)]
    fn process_buffer(&mut self);

    #[cfg(not(esp32))]
    fn process_buffer(&mut self) {
        let sha = unsafe { crate::peripherals::SHA::steal() };
        // Setup SHA Mode before processing current buffer.
        sha.mode()
            .write(|w| unsafe { w.mode().bits(Self::mode_as_bits()) });
        if self.first_run {
            // Set SHA_START_REG
            sha.start().write(|w| unsafe { w.bits(1) });
            self.first_run = false;
        } else {
            // Restore previously saved hash if interleaving operation
            if let Some(ref saved_digest) = self.saved_digest.take() {
                self.alignment_helper.volatile_write_regset(
                    sha.h_mem(0).as_ptr(),
                    saved_digest,
                    64,
                );
            }
            // SET SHA_CONTINUE_REG
            sha.continue_().write(|w| unsafe { w.bits(1) });
        }
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        let sha = unsafe { crate::peripherals::SHA::steal() };
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        // Flush aligned buffer in memory before flushing alignment_helper
        unsafe {
            core::ptr::copy_nonoverlapping(
                self.buffer.as_ptr(),
                #[cfg(esp32)]
                sha.text(0).as_ptr(),
                #[cfg(not(esp32))]
                sha.m_mem(0).as_ptr(),
                32,
            );
        }

        let ctx = self.deref_mut();
        let flushed = ctx.alignment_helper.flush_to(
            #[cfg(esp32)]
            sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            sha.m_mem(0).as_ptr(),
            (ctx.cursor % chunk_len) / ctx.alignment_helper.align_size(),
        );

        self.cursor = self.cursor.wrapping_add(flushed);
        if flushed > 0 && self.cursor % chunk_len == 0 {
            self.process_buffer();
            while self.is_busy() {}
        }

        Ok(())
    }

    // This function ensures that incoming data is aligned to u32 (due to issues
    // with cpy_mem<u8>)
    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        let sha = unsafe { crate::peripherals::SHA::steal() };
        let mod_cursor = self.cursor % self.chunk_length();

        let chunk_len = self.chunk_length();

        let ctx = self.deref_mut();
        // Buffer the incoming bytes into u32 aligned words.
        let (remaining, bound_reached) = ctx.alignment_helper.aligned_volatile_copy(
            ctx.buffer.as_mut_ptr(),
            incoming,
            chunk_len / ctx.alignment_helper.align_size(),
            mod_cursor / ctx.alignment_helper.align_size(),
        );

        self.cursor = self.cursor.wrapping_add(incoming.len() - remaining.len());

        // If bound reached we write the buffer to memory and process it.
        if bound_reached {
            unsafe {
                core::ptr::copy_nonoverlapping(
                    self.buffer.as_ptr(),
                    #[cfg(esp32)]
                    sha.text(0).as_ptr(),
                    #[cfg(not(esp32))]
                    sha.m_mem(0).as_ptr(),
                    32,
                );
            }
            self.process_buffer();

            // Wait until buffer has completely processed
            while self.is_busy() {}
            // Save the content of the current hash for interleaving operation.
            #[cfg(not(esp32))]
            {
                let mut saved_digest = [0u8; 64];
                self.alignment_helper.volatile_read_regset(
                    sha.h_mem(0).as_ptr(),
                    &mut saved_digest,
                    64 / self.alignment_helper.align_size(),
                );
                self.saved_digest.replace(saved_digest);
            }
        }

        Ok(remaining)
    }

    fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        self.finished = false;

        let remaining = self.write_data(buffer)?;

        Ok(remaining)
    }

    // Finish of the calculation (if not alreaedy) and copy result to output
    // After `finish()` is called `update()`s will contribute to a new hash which
    // can be calculated again with `finish()`.
    //
    // Typically output is expected to be the size of digest_length(), but smaller
    // inputs can be given to get a "short hash"
    fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // The main purpose of this function is to dynamically generate padding for the
        // input. Padding: Append "1" bit, Pad zeros until 512/1024 filled
        // then set the message length in the LSB (overwriting the padding)
        // If not enough free space for length+1, add length at end of a new zero'd
        // block

        let sha = unsafe { crate::peripherals::SHA::steal() };

        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        // Store message length for padding
        let length = (self.cursor as u64 * 8).to_be_bytes();
        nb::block!(Sha::update(self, &[0x80]))?; // Append "1" bit
        nb::block!(self.flush_data())?; // Flush partial data, ensures aligned cursor
        debug_assert!(self.cursor % 4 == 0);

        let mod_cursor = self.cursor % chunk_len;
        if (chunk_len - mod_cursor) < core::mem::size_of::<u64>() {
            // Zero out remaining data if buffer is almost full (>=448/896), and process
            // buffer
            let pad_len = chunk_len - mod_cursor;
            let ctx = self.deref_mut();
            ctx.alignment_helper.volatile_write_bytes(
                #[cfg(esp32)]
                sha.text(0).as_ptr(),
                #[cfg(not(esp32))]
                sha.m_mem(0).as_ptr(),
                0_u8,
                pad_len / ctx.alignment_helper.align_size(),
                mod_cursor / ctx.alignment_helper.align_size(),
            );
            self.process_buffer();
            self.cursor = self.cursor.wrapping_add(pad_len);

            debug_assert_eq!(self.cursor % chunk_len, 0);

            // Spin-wait for finish
            while self.is_busy() {}
        }

        let mod_cursor = self.cursor % chunk_len; // Should be zero if branched above
        let pad_len = chunk_len - mod_cursor - core::mem::size_of::<u64>();

        let ctx = self.deref_mut();
        ctx.alignment_helper.volatile_write_bytes(
            #[cfg(esp32)]
            sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            sha.m_mem(0).as_ptr(),
            0_u8,
            pad_len / ctx.alignment_helper.align_size(),
            mod_cursor / ctx.alignment_helper.align_size(),
        );

        ctx.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32)]
            sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            sha.m_mem(0).as_ptr(),
            &length,
            chunk_len / ctx.alignment_helper.align_size(),
            (chunk_len - core::mem::size_of::<u64>()) / ctx.alignment_helper.align_size(),
        );

        self.process_buffer();
        // Spin-wait for final buffer to be processed
        while self.is_busy() {}

        // ESP32 requires additional load to retrieve output
        #[cfg(esp32)]
        {
            self.load_reg();
            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
        }

        self.alignment_helper.volatile_read_regset(
            #[cfg(esp32)]
            sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            sha.h_mem(0).as_ptr(),
            output,
            core::cmp::min(output.len(), 32) / self.alignment_helper.align_size(),
        );

        self.first_run = true;
        self.cursor = 0;
        self.alignment_helper.reset();

        Ok(())
    }

    /// Create a new instance in [crate::Blocking] mode.
    #[cfg_attr(not(esp32), doc = "Optionally an interrupt handler can be bound.")]
    fn new_internal(sha: impl Peripheral<P = SHA> + 'd) -> (PeripheralRef<'d, SHA>, Context<DM>) {
        crate::into_ref!(sha);

        PeripheralClockControl::reset(crate::system::Peripheral::Sha);
        PeripheralClockControl::enable(crate::system::Peripheral::Sha);

        (
            sha,
            Context {
                cursor: 0,
                first_run: true,
                finished: false,
                alignment_helper: AlignmentHelper::default(),
                buffer: [0u32; 32],
                #[cfg(not(esp32))]
                saved_digest: None,
                phantom: PhantomData,
            },
        )
    }
}

/// This macro implements the Sha<'a, DM> trait for a specified Sha algorithm
/// and a set of parameters
macro_rules! impl_sha {
    ($name: ident, $mode_bits: tt, $digest_length: tt, $chunk_length: tt) => {
        pub struct $name<'d, DM: crate::Mode>(PeripheralRef<'d, SHA>, Context<DM>);

        impl<'d> $name<'d, crate::Blocking> {
            /// Create a new instance in [crate::Blocking] mode.
            #[cfg_attr(not(esp32), doc = "Optionally an interrupt handler can be bound.")]
            pub fn new(sha: impl Peripheral<P = SHA> + 'd) -> $name<'d, crate::Blocking> {
                let (sha, ctx) = Self::new_internal(sha);
                Self(sha, ctx)
            }
        }

        /// Automatically implement Deref + DerefMut to get access to inner context
        impl<'a, DM: crate::Mode> core::ops::Deref for $name<'a, DM> {
            type Target = Context<DM>;

            fn deref(&self) -> &Self::Target {
                &self.1
            }
        }

        impl<'a, DM: crate::Mode> core::ops::DerefMut for $name<'a, DM> {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.1
            }
        }

        /// Implement Default to create hasher out of thin air
        /// TODO: Ensure safety is when using multiple Sha at once.
        impl<'d> core::default::Default for $name<'d, crate::Blocking> {
            fn default() -> Self {
                let sha = unsafe { crate::peripherals::SHA::steal() };
                let (sha, ctx) = Self::new_internal(sha);
                Self(sha, ctx)
            }
        }

        impl<'d> $crate::sha::Sha<'d, crate::Blocking> for $name<'d, crate::Blocking> {
            const ALGORITHM: &'static str = stringify!($name);

            #[cfg(not(esp32))]
            fn mode_as_bits() -> u8 {
                $mode_bits
            }

            fn chunk_length(&self) -> usize {
                $chunk_length
            }

            fn digest_length(&self) -> usize {
                $digest_length
            }

            // ESP32 uses different registers for its operation
            #[cfg(esp32)]
            fn load_reg(&self) {
                paste::paste! {
                    unsafe { self.0.[< $name:lower _load >]().write(|w| w.bits(1)) };
                }
            }

            #[cfg(esp32)]
            fn is_busy(&self) -> bool {
                paste::paste! {
                    self.0.[< $name:lower _busy >]().read().[< $name:lower _busy >]().bit_is_set()
                }
            }

            #[cfg(esp32)]
            fn process_buffer(&mut self) {
                paste::paste! {
                    if self.first_run {
                        self.0.[< $name:lower _start >]().write(|w| unsafe { w.bits(1) });
                        self.first_run = false;
                    } else {
                        self.0.[< $name:lower _continue >]().write(|w| unsafe { w.bits(1) });
                    }
                }
            }
        }

        /// implement digest traits if digest feature is present.
        /// Note: digest has a blanket trait implementation for [digest::Digest] for any
        /// element that implements FixedOutput + Default + Update + HashMarker
        #[cfg(feature = "digest")]
        impl<'d, DM: crate::Mode> digest::HashMarker for $name<'d, DM> {}

        #[cfg(feature = "digest")]
        impl<'a, DM: crate::Mode> digest::OutputSizeUser for $name<'a, DM> {
            // We use paste to append `U` to the digest size to match a const defined in
            // digest
            paste::paste! {
            type OutputSize = digest::consts::[< U $digest_length >];
            }
        }

        #[cfg(feature = "digest")]
        impl<'a> digest::Update for $name<'a, crate::Blocking> {
            fn update(&mut self, data: &[u8]) {
                let mut remaining = data.as_ref();
                while remaining.len() > 0 {
                    remaining = nb::block!(Sha::update(self, remaining)).unwrap();
                }
            }
        }

        #[cfg(feature = "digest")]
        impl<'a> digest::FixedOutput for $name<'a, crate::Blocking> {
            fn finalize_into(mut self, out: &mut digest::Output<Self>) {
                nb::block!(self.finish(out)).unwrap()
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
