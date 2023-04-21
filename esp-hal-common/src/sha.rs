use core::convert::Infallible;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    // peripherals::generic::{Readable, Reg, RegisterSpec, Resettable, Writable},
    peripherals::SHA,
    reg_access::AlignmentHelper,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
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

pub struct Sha<'d> {
    sha: PeripheralRef<'d, SHA>,
    mode: ShaMode,
    alignment_helper: AlignmentHelper,
    cursor: usize,
    first_run: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum ShaMode {
    SHA1       = 0,
    #[cfg(not(esp32))]
    SHA224     = 1,
    SHA256     = 2,
    #[cfg(any(esp32s2, esp32s3, esp32))]
    SHA384     = 3,
    #[cfg(any(esp32s2, esp32s3, esp32))]
    SHA512     = 4,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_224 = 5,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_256 = 6,
    // SHA512_(u16) // Max 511
}

// TODO: Allow/Implemenet SHA512_(u16)

// A few notes on this implementation with regards to 'memcpy',
// - It seems that ptr::write_bytes already acts as volatile, while ptr::copy_*
//   does not (in this case)
// - The registers are *not* cleared after processing, so padding needs to be
//   written out
// - This component uses core::intrinsics::volatile_* which is unstable, but is
//   the only way to efficiently copy memory with volatile
// - For this particular registers (and probably others), a full u32 needs to be
//   written. Partial register writes (i.e. in u8 mode) does not work
// - This means that we need to buffer bytes coming in up to 4 u8's in order to
//   create a full u32

// This implementation might fail after u32::MAX/8 bytes, to increase please see
// ::finish() length/self.cursor usage
impl<'d> Sha<'d> {
    pub fn new(
        sha: impl Peripheral<P = SHA> + 'd,
        mode: ShaMode,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(sha);

        peripheral_clock_control.enable(PeripheralEnable::Sha);

        // Setup SHA Mode
        #[cfg(not(esp32))]
        sha.mode.write(|w| unsafe { w.mode().bits(mode as u8) });

        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            alignment_helper: AlignmentHelper::default(),
        }
    }

    #[cfg(esp32)]
    pub fn setmode(&mut self, _mode: ShaMode) {
        
    }

    #[cfg(not(esp32))]
    pub fn setmode(&mut self, mode: ShaMode) {
        // Setup SHA Mode
        self.sha
            .mode
            .write(|w| unsafe { w.mode().bits(mode as u8) });
    }

    #[cfg(not(esp32))]
    #[inline]
    fn process_buffer(&mut self) {
        // FIXME: SHA_START_REG & SHA_CONTINUE_REG are wrongly marked as RO (they are
        // WO)
        if self.first_run {
            // Set SHA_START_REG
            // self.sha.start.write(|w| w.start().set_bit());
            unsafe {
                self.sha.start.as_ptr().write_volatile(1u32);
            }
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            // self.sha.continue_.write(|w| w.continue_op().set_bit());
            unsafe {
                self.sha.continue_.as_ptr().write_volatile(1u32);
            }
        }
    }

    #[cfg(esp32)]
    #[inline]
    fn process_buffer(&mut self) {
        if self.first_run {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_start.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA256 => self.sha.sha256_start.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA384 => self.sha.sha384_start.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA512 => self.sha.sha512_start.write(|w| unsafe { w.bits(1) }),
            }
            self.first_run = false;
        } else {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_continue.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA256 => self.sha.sha256_continue.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA384 => self.sha.sha384_continue.write(|w| unsafe { w.bits(1) }),
                ShaMode::SHA512 => self.sha.sha512_continue.write(|w| unsafe { w.bits(1) }),
            }
        }
    }

    #[inline]
    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA256 => 64,
            #[cfg(not(esp32))]
            ShaMode::SHA224 => 64,
            #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
            _ => 128,
        };
    }

    #[cfg(esp32)]
    #[inline]
    fn is_busy(&self) -> bool {
        match self.mode {
            ShaMode::SHA1 => self.sha.sha1_busy.read().sha1_busy().bit_is_set(),
            ShaMode::SHA256 => self.sha.sha256_busy.read().sha256_busy().bit_is_set(),
            ShaMode::SHA384 => self.sha.sha384_busy.read().sha384_busy().bit_is_set(),
            ShaMode::SHA512 => self.sha.sha512_busy.read().sha512_busy().bit_is_set(),
        }
    }

    #[cfg(not(esp32))]
    #[inline]
    fn is_busy(&self) -> bool {
        self.sha.busy.read().bits() != 0
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

    #[inline]
    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        let flushed = self.alignment_helper.flush_to(
            #[cfg(esp32)]
            &mut self.sha.text,
            #[cfg(not(esp32))]
            &mut self.sha.m_mem,
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
    #[inline]
    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        let mod_cursor = self.cursor % self.chunk_length();

        let chunk_len = self.chunk_length();

        let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32)]
            &mut self.sha.text,
            #[cfg(not(esp32))]
            &mut self.sha.m_mem,
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

    // #[cfg(esp32)]
    // #[inline]
    // fn input_ptr(&mut self) -> &mut [Reg<impl RegisterSpec<Ux = u32> + Resettable
    // + Writable>] { &mut self.sha.text
    // }
    //
    // #[cfg(any(esp32s3, esp32c3, esp32c6))]
    // #[inline]
    // fn input_ptr(&mut self) -> &mut [Reg<impl RegisterSpec<Ux = u8> + Resettable
    // + Writable>] { &mut self.sha.m_mem
    // }
    //
    // #[cfg(esp32s2)]
    // #[inline]
    // fn input_ptr(&mut self) -> &mut [Reg<impl RegisterSpec<Ux = u32> + Resettable
    // + Writable>] { &mut self.sha.m_mem
    // }
    //
    // #[cfg(esp32)]
    // #[inline]
    // fn output_ptr(&mut self) -> &[Reg<impl RegisterSpec<Ux = u32> + Readable>] {
    // &self.sha.text
    // }
    //
    // #[cfg(any(esp32s3, esp32c3, esp32c6))]
    // #[inline]
    // fn output_ptr(&mut self) -> &[Reg<impl RegisterSpec<Ux = u8> + Readable>] {
    // &mut self.sha.h_mem
    // }
    //
    // #[cfg(esp32s2)]
    // #[inline]
    // fn output_ptr(&mut self) -> &[Reg<impl RegisterSpec<Ux = u32> + Readable>] {
    // &mut self.sha.h_mem
    // }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

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
                &mut self.sha.text,
                #[cfg(not(esp32))]
                &mut self.sha.m_mem,
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
            &mut self.sha.text,
            #[cfg(not(esp32))]
            &mut self.sha.m_mem,
            0_u8,
            pad_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32)]
            &mut self.sha.text,
            #[cfg(not(esp32))]
            &mut self.sha.m_mem,
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
                ShaMode::SHA1 => unsafe { self.sha.sha1_load.write(|w| w.bits(1)) },
                ShaMode::SHA256 => unsafe { self.sha.sha256_load.write(|w| w.bits(1)) },
                ShaMode::SHA384 => unsafe { self.sha.sha384_load.write(|w| w.bits(1)) },
                ShaMode::SHA512 => unsafe { self.sha.sha512_load.write(|w| w.bits(1)) },
            }

            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
        }

        self.alignment_helper.volatile_read_regset(
            #[cfg(esp32)]
            &self.sha.text[0],
            #[cfg(not(esp32))]
            &self.sha.h_mem[0],
            output,
            core::cmp::min(output.len(), 32) / self.alignment_helper.align_size(),
        );

        self.first_run = true;
        self.cursor = 0;
        self.alignment_helper.reset();

        Ok(())
    }
}
