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
//! # use esp_hal::sha::ShaMode;
//! # use core::option::Option::None;
//! # use nb::block;
//! let source_data = "HELLO, ESPRESSIF!".as_bytes();
//! let mut remaining = source_data;
//! let mut hasher = Sha::new(peripherals.SHA, ShaMode::SHA256);
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

/// The SHA Accelerator driver instance
pub struct Sha<'d, DM: crate::Mode> {
    sha: PeripheralRef<'d, SHA>,
    mode: ShaMode,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    cursor: usize,
    first_run: bool,
    finished: bool,
    phantom: PhantomData<DM>,
}

/// Hash Algorithm Mode
#[derive(Debug, Clone, Copy)]
pub enum ShaMode {
    SHA1,
    #[cfg(not(esp32))]
    SHA224,
    SHA256,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    SHA384,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    SHA512,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_224,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_256,
    // SHA512_(u16) // Max 511
}

// TODO: Maybe make Sha Generic (Sha<Mode>) in order to allow for better
// compiler optimizations? (Requires complex const generics which isn't stable
// yet)

#[cfg(not(esp32))]
fn mode_as_bits(mode: ShaMode) -> u8 {
    match mode {
        ShaMode::SHA1 => 0,
        ShaMode::SHA224 => 1,
        ShaMode::SHA256 => 2,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA384 => 3,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA512 => 4,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA512_224 => 5,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA512_256 => 6,
        // _ => 0 // TODO: SHA512/t
    }
}

impl<'d> Sha<'d, crate::Blocking> {
    /// Create a new instance in [crate::Blocking] mode.
    #[cfg_attr(not(esp32), doc = "Optionally an interrupt handler can be bound.")]
    pub fn new(sha: impl Peripheral<P = SHA> + 'd, mode: ShaMode) -> Self {
        crate::into_ref!(sha);

        PeripheralClockControl::enable(crate::system::Peripheral::Sha);

        // Setup SHA Mode
        #[cfg(not(esp32))]
        sha.mode()
            .write(|w| unsafe { w.mode().bits(mode_as_bits(mode)) });

        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            finished: false,
            alignment_helper: AlignmentHelper::default(),
            phantom: PhantomData,
        }
    }
}

impl<'d> crate::private::Sealed for Sha<'d, crate::Blocking> {}

#[cfg(not(esp32))]
impl<'d> crate::InterruptConfigurable for Sha<'d, crate::Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::SHA, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::SHA, handler.priority())
                .unwrap();
        }
    }
}

// TODO: Allow/Implement SHA512_(u16)

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
impl<'d, DM: crate::Mode> Sha<'d, DM> {
    pub fn first_run(&self) -> bool {
        self.first_run
    }

    pub fn finished(&self) -> bool {
        self.finished
    }

    #[cfg(not(esp32))]
    fn process_buffer(&mut self) {
        if self.first_run {
            // Set SHA_START_REG
            self.sha.start().write(|w| unsafe { w.bits(1) });
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            self.sha.continue_().write(|w| unsafe { w.bits(1) });
        }
    }

    #[cfg(esp32)]
    fn process_buffer(&mut self) {
        if self.first_run {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_start().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA256 => self.sha.sha256_start().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA384 => self.sha.sha384_start().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA512 => self.sha.sha512_start().write(|w| unsafe { w.bits(1) }),
            }
            self.first_run = false;
        } else {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_continue().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA256 => self.sha.sha256_continue().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA384 => self.sha.sha384_continue().write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA512 => self.sha.sha512_continue().write(|w| unsafe { w.bits(1) }),
            }
        }
    }

    fn chunk_length(&self) -> usize {
        match self.mode {
            ShaMode::SHA1 | ShaMode::SHA256 => 64,
            #[cfg(not(esp32))]
            ShaMode::SHA224 => 64,
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            _ => 128,
        }
    }

    #[cfg(esp32)]
    fn is_busy(&self) -> bool {
        match self.mode {
            ShaMode::SHA1 => self.sha.sha1_busy().read().sha1_busy().bit_is_set(),
            ShaMode::SHA256 => self.sha.sha256_busy().read().sha256_busy().bit_is_set(),
            ShaMode::SHA384 => self.sha.sha384_busy().read().sha384_busy().bit_is_set(),
            ShaMode::SHA512 => self.sha.sha512_busy().read().sha512_busy().bit_is_set(),
        }
    }

    #[cfg(not(esp32))]
    fn is_busy(&self) -> bool {
        self.sha.busy().read().bits() != 0
    }

    pub fn digest_length(&self) -> usize {
        match self.mode {
            ShaMode::SHA1 => 20,
            #[cfg(not(esp32))]
            ShaMode::SHA224 => 28,
            ShaMode::SHA256 => 32,
            #[cfg(any(esp32, esp32s2, esp32s3))]
            ShaMode::SHA384 => 48,
            #[cfg(any(esp32, esp32s2, esp32s3))]
            ShaMode::SHA512 => 64,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_224 => 28,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_256 => 32,
        }
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        let flushed = self.alignment_helper.flush_to(
            #[cfg(esp32)]
            self.sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            self.sha.m_mem(0).as_ptr(),
            (self.cursor % chunk_len) / self.alignment_helper.align_size(),
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
        let mod_cursor = self.cursor % self.chunk_length();

        let chunk_len = self.chunk_length();

        let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32)]
            self.sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            self.sha.m_mem(0).as_ptr(),
            incoming,
            chunk_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.cursor = self.cursor.wrapping_add(incoming.len() - remaining.len());

        if bound_reached {
            self.process_buffer();
        }

        Ok(remaining)
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
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
    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // The main purpose of this function is to dynamically generate padding for the
        // input. Padding: Append "1" bit, Pad zeros until 512/1024 filled
        // then set the message length in the LSB (overwriting the padding)
        // If not enough free space for length+1, add length at end of a new zero'd
        // block

        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        // Store message length for padding
        let length = (self.cursor as u64 * 8).to_be_bytes();
        nb::block!(self.update(&[0x80]))?; // Append "1" bit
        nb::block!(self.flush_data())?; // Flush partial data, ensures aligned cursor
        debug_assert!(self.cursor % 4 == 0);

        let mod_cursor = self.cursor % chunk_len;
        if (chunk_len - mod_cursor) < core::mem::size_of::<u64>() {
            // Zero out remaining data if buffer is almost full (>=448/896), and process
            // buffer
            let pad_len = chunk_len - mod_cursor;
            self.alignment_helper.volatile_write_bytes(
                #[cfg(esp32)]
                self.sha.text(0).as_ptr(),
                #[cfg(not(esp32))]
                self.sha.m_mem(0).as_ptr(),
                0_u8,
                pad_len / self.alignment_helper.align_size(),
                mod_cursor / self.alignment_helper.align_size(),
            );
            self.process_buffer();
            self.cursor = self.cursor.wrapping_add(pad_len);

            debug_assert_eq!(self.cursor % chunk_len, 0);

            // Spin-wait for finish
            while self.is_busy() {}
        }

        let mod_cursor = self.cursor % chunk_len; // Should be zero if branched above
        let pad_len = chunk_len - mod_cursor - core::mem::size_of::<u64>();

        self.alignment_helper.volatile_write_bytes(
            #[cfg(esp32)]
            self.sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            self.sha.m_mem(0).as_ptr(),
            0_u8,
            pad_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32)]
            self.sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            self.sha.m_mem(0).as_ptr(),
            &length,
            chunk_len / self.alignment_helper.align_size(),
            (chunk_len - core::mem::size_of::<u64>()) / self.alignment_helper.align_size(),
        );

        self.process_buffer();
        // Spin-wait for final buffer to be processed
        while self.is_busy() {}

        // ESP32 requires additional load to retrieve output
        #[cfg(esp32)]
        {
            match self.mode {
                ShaMode::SHA1 => unsafe { self.sha.sha1_load().write(|w| w.bits(1)) },
                ShaMode::SHA256 => unsafe { self.sha.sha256_load().write(|w| w.bits(1)) },
                ShaMode::SHA384 => unsafe { self.sha.sha384_load().write(|w| w.bits(1)) },
                ShaMode::SHA512 => unsafe { self.sha.sha512_load().write(|w| w.bits(1)) },
            }

            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
        }

        self.alignment_helper.volatile_read_regset(
            #[cfg(esp32)]
            self.sha.text(0).as_ptr(),
            #[cfg(not(esp32))]
            self.sha.h_mem(0).as_ptr(),
            output,
            core::cmp::min(output.len(), 32) / self.alignment_helper.align_size(),
        );

        self.first_run = true;
        self.cursor = 0;
        self.alignment_helper.reset();

        Ok(())
    }
}
