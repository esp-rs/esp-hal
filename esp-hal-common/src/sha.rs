use crate::pac::SHA;
use core::convert::Infallible;

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
    finished: bool
}

#[derive(Debug, Clone, Copy)]
pub enum ShaMode {
    SHA1,
    SHA224,
    SHA256,
    SHA384,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_224,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_256,
    // SHA512_(u16) // Max 511
}


// TODO: this is different for the esp32-hal (sha.text instead of m_mem & seperate start/continue/busy (also has sha.load=finish reg) regs per algo)
#[cfg(any(esp32s2, esp32s3, esp32c2, esp32c3))]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        sha.mode.write(|w| unsafe {
            // TODO: Allow SHA512_(u16)
            w.mode().bits(mode as u8)
        });
        Self { sha, mode, cursor: 0, finished: false }
    }

    fn process_buffer(&mut self, first_start: bool) {
        // FIXME: SHA_START_REG & SHA_CONTINUE_REG are wrongly marked as RO (they are WO)
        if first_start {
            // Set SHA_START_REG
            unsafe {
                // TODO: not sure what to correct unwrapping is (? is infallible but can't convert to Result for some reason)
                let ptr = self.sha.start.as_ptr().as_mut().unwrap();
                *ptr = 1u32;
            }
        } else {
            // SET SHA_CONTINUE_REG
            unsafe {
                // TODO: not sure what to correct unwrapping is (? is infallible but can't convert to Result for some reason)
                let ptr = self.sha.continue_.as_ptr().as_mut().unwrap();
                *ptr = 1u32;
            }
            
        }
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> 
    {   
        if self.sha.busy.read().state().bit() {
            return Err(nb::Error::WouldBlock)
        }
        
        // NOTE: m_mem is 64,u8; but datasheet (esp32s2/esp32s3@>=SHA384) says 128, u8
        // Load data into M_n_REG
        let chunk_len = match self.mode {
            ShaMode::SHA1 | ShaMode::SHA224 | ShaMode::SHA256 => 64,
            _ => 128,
        };

        let first_start = self.cursor == 0; // TODO: could overflow if >4294967296 bytes (4.2GB)
        
        // Read only enough to fill m_mem 
        let to_go = chunk_len - (self.cursor % chunk_len);
        let to_read = if buffer.len() > to_go { to_go } else { buffer.len() };
        let (chunk, buffer) = buffer.split_at(to_read);

        unsafe {
            let m_cursor_ptr = self.sha.m_mem[0].as_ptr().add(self.cursor % chunk_len);
            core::ptr::copy_nonoverlapping(chunk.as_ptr(), m_cursor_ptr, chunk.len());
            self.cursor = self.cursor.wrapping_add(chunk.len());
        }

        // Finished writing current chunk, start accelerator
        if self.cursor % chunk_len == 0 {
            self.process_buffer(first_start);
        }
 
        Ok(buffer)
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> 
    {
        // TODO: should we enforce full hash readout in output?
        // TODO: `finished` latch could also be done by freeing self, although only a single read-out would be allowed :thinking:

        // Pad messagee:
        // Append "1" bit
        // Second[SHA-1/2xx], append k zero bits, where k is the smallest, non-negative solution to the equation m + 1 + k ≡ 448 mod 512
        // Second[SHA-3xx/5xx], Second, append k zero bits, where k is the smallest, non-negative solution to the equation m + 1 + k ≡ 896 mod 1024;
        // Last, append the {SHA12=>64/SHA35=>128}-bit block of value equal to the number m expressed using a binary representation

        // Translation: Append "1" bit, Pad zeros until 512/1024 filled
        // then set the message length in the LSB (overwriting the padding)
        // If not enough free space for length+1, add a new zero'd block add add length there

        if self.sha.busy.read().state().bit() {
            return Err(nb::Error::WouldBlock)
        }

        let chunk_len = match self.mode {
            ShaMode::SHA1 | ShaMode::SHA224 | ShaMode::SHA256 => 64,
            _ => 128,
        };

        if !self.finished {
            // Bit-length of original message + "1" bit
            let length = (self.cursor * 8 + 1).to_be_bytes();

            self.update(&[0x80])?; // Append "1" bit
            let mod_cursor = self.cursor % chunk_len;

            // TODO: verify this works (should be 8 or 15 bytes free for this not to trigger)
            if chunk_len - mod_cursor < chunk_len / 8 {
                // Zero out remaining data if buffer is almost full (>=448/896), and process buffer
                
                let pad_len = chunk_len - mod_cursor;
                unsafe {
                    let m_cursor_ptr = self.sha.m_mem[0].as_ptr().add(mod_cursor);
                    core::ptr::write_bytes::<u8>(m_cursor_ptr, 0, pad_len);
                }
                self.process_buffer(self.cursor < chunk_len);
                self.cursor = self.cursor.wrapping_add(pad_len);
            }

            let mod_cursor = self.cursor % chunk_len; // Should be zero if branched above

            // Spin-wait for potential branch above to finish
            while self.sha.busy.read().state().bit() {} 

            unsafe {
                let m_cursor_ptr = self.sha.m_mem[0].as_ptr().add(mod_cursor);
                // Pad zeros
                m_cursor_ptr.write_bytes(0, chunk_len-length.len()-mod_cursor);
                // Write length (BE) to end
                let m_cursor_ptr = self.sha.m_mem[0].as_ptr().add(chunk_len-length.len());
                core::ptr::copy_nonoverlapping(length.as_ptr(), m_cursor_ptr, length.len());
            }
            
            self.process_buffer(self.cursor < chunk_len);

            // Spin-wait for final buffer to be processed
            while self.sha.busy.read().state().bit() { }

            self.finished = true;
        }
        
        unsafe {
            core::ptr::copy_nonoverlapping(self.sha.h_mem.as_ptr() as *const u8, output.as_mut_ptr(), output.len());
        }
        Ok(())
    }


    pub fn free(self) -> SHA {
        self.sha
    }
}
