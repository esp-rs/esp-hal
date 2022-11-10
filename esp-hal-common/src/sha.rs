use core::{convert::Infallible, ptr::slice_from_raw_parts};

use esp_println::println;

use nb::block;
use crate::pac::SHA;

// All the hash algorithms introduced in FIPS PUB 180-4 Spec.
// – SHA-1
// – SHA-224
// – SHA-256
// – SHA-384
// – SHA-512
// – SHA-512/224
// – SHA-512/256
// – SHA-512/t
// Two working modes
// – Typical SHA
// – DMA-SHA (not implemented yet)

#[derive(Debug)]
pub struct Sha {
    sha: SHA,
    mode: ShaMode,
    cursor: usize,
    first_run: bool,
    finished: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum ShaMode {
    SHA1,
    #[cfg(not(esp32))]
    SHA224,
    SHA256,
    SHA384,
    #[cfg(any(esp32s2, esp32s3, esp32))]
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

#[cfg(esp32)]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        Self {
            sha,
            mode,
            first_run: true,
            cursor: 0,
            finished: false,
        }
    }

    fn process_buffer(&mut self) {
        if self.first_run {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_start().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA256 => self.sha.sha256_start().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA384 => self.sha.sha384_start.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA512 => self.sha.sha512_start.write(|w| unsafe { w.bits(1u32) }),
            }
            self.first_run = false;
        } else {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_continue().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA256 => self.sha.sha256_continue.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA384 => self.sha.sha384_continue.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA512 => self.sha.sha512_continue.write(|w| unsafe { w.bits(1u32) }),
            }
        }
    }

    fn is_busy(&mut self) -> bool {
        match self.mode {
            // FIXME: These are marked WO, while being RO
            ShaMode::SHA1 => unsafe { self.sha.sha1_busy.as_ptr().read() == 0 },
            ShaMode::SHA256 => unsafe { self.sha.sha256_busy.as_ptr().read() == 0 },
            ShaMode::SHA384 => unsafe { self.sha.sha384_busy.as_ptr().read() == 0 },
            ShaMode::SHA512 => unsafe { self.sha.sha512_busy.as_ptr().read() == 0 },
        }
    }

    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA256 => 64,
            ShaMode::SHA384 | ShaMode::SHA512 => 128,
        };
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();
        let to_go = chunk_len - (self.cursor % chunk_len);
        let to_read = if buffer.len() > to_go {
            to_go
        } else {
            buffer.len()
        };
        let (chunk, buffer) = buffer.split_at(to_read);

        unsafe {
            let m_cursor_ptr = self.sha.text_[0].as_ptr().add(self.cursor % chunk_len);
            core::ptr::copy_nonoverlapping(chunk.as_ptr(), m_cursor_ptr as *mut u8, chunk.len());
            self.cursor = self.cursor.wrapping_add(chunk.len());
        }

        // Finished writing current chunk, start accelerator
        if self.cursor % chunk_len == 0 {
            self.process_buffer();
        }

        Ok(buffer)
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        if !self.finished {
            if self.cursor % self.chunk_length() != 0 {
                self.process_buffer();
            }

            match self.mode {
                // FIXME: These are marked WO, while being RO, also inconsistent func/reg
                ShaMode::SHA1 => unsafe { self.sha.sha1_load.write(|w| w.bits(1u32)) },
                ShaMode::SHA256 => unsafe { self.sha.sha256_load().write(|w| w.bits(1u32)) },
                ShaMode::SHA384 => unsafe { self.sha.sha384_load.write(|w| w.bits(1u32)) },
                ShaMode::SHA512 => unsafe { self.sha.sha512_load.write(|w| w.bits(1u32)) },
            }

            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
            self.finished = true;
        }

        unsafe {
            // Read SHA1=Text[0:4] | SHA256=Text[0:8] | SHA384=Text[0:11] |
            // SHA512=Text[0:15] TODO: limit read len to digest size
            core::ptr::copy_nonoverlapping(
                self.sha.text_.as_ptr() as *const u8,
                output.as_mut_ptr(),
                output.len(),
            );
        }
        Ok(())
    }
}

fn mode_as_bits(mode: ShaMode) -> u8 {
    match mode {
        ShaMode::SHA1 => 0,
        ShaMode::SHA224 => 1,
        ShaMode::SHA256 => 2,
        ShaMode::SHA384 => 3,
        ShaMode::SHA512 => 4,
        ShaMode::SHA512_224 => 5,
        ShaMode::SHA512_256 => 6,
        // _ => 0 // TODO: SHA512/t
    }
}

// TODO: this is different for the esp32-hal (sha.text instead of m_mem &
// seperate start/continue/busy (also has sha.load=finish reg) regs per algo)
#[cfg(any(esp32s2, esp32s3, em_memsp32c2, esp32c3))]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        sha.mode.write(|w| unsafe {
            // TODO: Allow/Implemenet SHA512_(u16)
            w.mode().bits(mode_as_bits(mode))
        });
        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            finished: false,
        }
    }

    pub fn first_run(&self) -> bool {
        self.first_run
    }

    pub fn finished(&self) -> bool {
        self.finished
    }

    fn process_buffer(&mut self) {
        // FIXME: SHA_START_REG & SHA_CONTINUE_REG are wrongly marked as RO (they are
        // WO)
        if self.first_run {
            // Set SHA_START_REG
            unsafe {
                // TODO: not sure what to correct unwrapping is (? is infallible but can't
                // convert to Result for some reason)
                let ptr = self.sha.start.as_ptr().as_mut().unwrap();
                *ptr = 1u32;
            }
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            unsafe {
                // TODO: not sure what to correct unwrapping is (? is infallible but can't
                // convert to Result for some reason)_
                let ptr = self.sha.continue_.as_ptr().as_mut().unwrap();
                *ptr = 1u32;
            }
        }
    }

    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA224 | ShaMode::SHA256 => 64,
            _ => 128,
        };
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        // NOTE: m_mem is 64,u8; but datasheet (esp32s2/esp32s3@>=SHA384) says 128, u8
        // Load data into M_n_REG
        let chunk_len = self.chunk_length();

        // Read only enough to fill m_mem
        let to_go = chunk_len - (self.cursor % chunk_len);
        let to_read = if buffer.len() > to_go {
            to_go
        } else {
            buffer.len()
        };
        let (chunk, buffer) = buffer.split_at(to_read);

        unsafe {
            let m_cursor_ptr = self.sha.m_mem[0].as_ptr() as *mut u8;
            core::ptr::copy_nonoverlapping(
                chunk.as_ptr(),
                m_cursor_ptr.add(self.cursor % chunk_len),
                chunk.len(),
            );
            self.cursor = self.cursor.wrapping_add(chunk.len());
            println!("Buffer {:?} {:02x?}", self.sha.m_mem.as_ptr() as *const u8, slice_from_raw_parts(self.sha.m_mem.as_ptr() as *const u8, 128).as_ref());
        }

        // Finished writing current chunk, start accelerator
        if self.cursor % chunk_len == 0 {
            self.process_buffer();
        }

        Ok(buffer)
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // TODO: should we enforce full hash readout in output?
        // TODO: `finished` latch could also be done by freeing self, although only a
        // single read-out would be allowed :thinking:

        // Pad messagee:
        // Append "1" bit
        // Second[SHA-1/2xx], append k zero bits, where k is the smallest, non-negative
        // solution to the equation m + 1 + k ≡ 448 mod 512 Second[SHA-3xx/5xx],
        // Second, append k zero bits, where k is the smallest, non-negative solution to
        // the equation m + 1 + k ≡ 896 mod 1024; Last, append the
        // {SHA12=>64/SHA35=>128}-bit block of value equal to the number m expressed
        // using a binary representation

        // Translation: Append "1" bit, Pad zeros until 512/1024 filled
        // then set the message length in the LSB (overwriting the padding)
        // If not enough free space for length+1, add a new zero'd block add add length
        // there

        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        if !self.finished {
            // Bit-length of original message + "1" bit
            let length = (self.cursor * 8 + 1).to_be_bytes();
            block!(self.update(&[0x80]))?; // Append "1" bit
            let mut mod_cursor = self.cursor % chunk_len;

            // TODO: verify this works (should be 8 or 15 bytes free for this not to
            // trigger)
            println!(
                "[SHA] adding padding?: chunkl={}, cursor={}",
                chunk_len, mod_cursor
            );
            if chunk_len - mod_cursor < chunk_len / 8 {
                println!("[SHA] Adding padding!");
                // Zero out remaining data if buffer is almost full (>=448/896), and process
                // buffer
                let pad_len = chunk_len - mod_cursor;
                unsafe {
                    let m_cursor_ptr = self.sha.m_mem[0].as_ptr() as *mut u8;
                    core::ptr::write_bytes::<u8>(m_cursor_ptr.add(mod_cursor), 0, pad_len);
                }
                self.process_buffer();
                self.cursor = self.cursor.wrapping_add(pad_len);
                mod_cursor = self.cursor % chunk_len; // Should be zero if branched above

                // Spin-wait for finish 
                while self.sha.busy.read().bits() != 0 {}
            }

            unsafe {
                let m_cursor_ptr = self.sha.m_mem[0].as_ptr() as *mut u8;
                // Pad zeros
                core::ptr::write_bytes::<u8>(
                    m_cursor_ptr.add(mod_cursor),
                    0,
                    chunk_len - length.len() - mod_cursor,
                );
                // Write length (BE) to end
                // FIXME: length does nothing for some reason
                println!(
                    "{:02x?}, {}, {:?}, cursor={}, {:?} {:?}",
                    length,
                    chunk_len - length.len(),
                    length.as_ptr(),
                    mod_cursor,
                    m_cursor_ptr,
                    (self.sha.m_mem.as_ptr() as *mut u8).add(1),
                );

                core::ptr::copy_nonoverlapping::<u8>(length.as_ptr(), (self.sha.m_mem[0].as_ptr() as *mut u8).add(chunk_len - length.len()), length.len());
            }

            self.process_buffer();

            // Spin-wait for final buffer to be processed
            while self.sha.busy.read().bits() != 0 {}

            self.finished = true;
        }

        unsafe {
            // TODO: limit read len to digest size
            core::ptr::copy_nonoverlapping(
                self.sha.h_mem.as_ptr() as *const u8,
                output.as_mut_ptr(),
                output.len(),
            );
        }
        Ok(())
    }

    pub fn free(self) -> SHA {
        self.sha
    }
}
