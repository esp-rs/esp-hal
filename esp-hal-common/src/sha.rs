use core::{convert::Infallible, ptr::slice_from_raw_parts, intrinsics::size_of};

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

// The alignment helper helps you write to registers that only excepts u32 using u8 (bytes)
// It keeps a write buffer of 4 u8 (could in theory be 3 but less convient)
// And if the incoming data is not convertable to u32 (i.e. not a multiple of 4 in length) it will
// store the remainder in the buffer until the next call
//
// It assumes incoming `dst` are aligned to desired layout (in future ptr.is_aligned can be used)
struct AlignmentHelper {
    buf: [u8; size_of::<u32>()],
    buf_fill: usize
}

impl AlignmentHelper {
    fn flush(&mut self) -> &[u8]
    {
        let (data, _) = self.buf.split_at(self.buf_fill);
        data
    }

    pub fn flush_to(&mut self, dst: *mut u32) {
        if self.buf_fill != 0 {
            for i in self.buf_fill..size_of::<u32>() {
                self.buf[i] = 0;
            }

            unsafe { dst.write_volatile(u32::from_ne_bytes(self.buf)) };
        }
    }

    pub fn aligned_volatile_copy<'a>(&mut self, dst: *mut u32, src: &'a [u8], dst_bound: usize) -> &'a [u8] {
        assert!(dst_bound > 0);

        let mut cursor = 0;
        if self.buf_fill != 0 {
            // First prepend existing data
            for i in 0..(size_of::<u32>() - self.buf_fill) {
                match src.get(i) {
                    Some(v) => { 
                        self.buf[self.buf_fill+i] = *v ;
                        self.buf_fill += 1;
                    },
                    None => { return &[] } // Used up entire buffer before filling buff_fil
                }
            }

            unsafe {
                core::intrinsics::volatile_copy_nonoverlapping_memory(dst, self.buf.as_ptr() as *const u32, 1);
                cursor += 1; 
            };
            self.buf_fill = 0;
        }

        let (to_write, remaining) = src.split_at(core::cmp::min(dst_bound-cursor, src.len()/size_of::<u32>())*size_of::<u32>());
        unsafe {
            core::intrinsics::volatile_copy_nonoverlapping_memory(dst.add(cursor), to_write.as_ptr() as *const u32, to_write.len()/size_of::<u32>());
        }

        // If it's data we can't store we don't need to try and align it, just wait for next write
        // Generally this applies when (src/4*4) != src
        if remaining.len() < 4 {
            self.buf.copy_from_slice(remaining);
            self.buf_fill = remaining.len();
            return &[];
        }

        return remaining;
    }
}


#[derive(Debug)]
pub struct Sha {
    sha: SHA,
    mode: ShaMode,
    write_buf: [u8; 4],
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

// TODO: Fix esp32 impl to also have u32 alignment (extract alignment code into seperate thing)
// TODO: Allow/Implemenet SHA512_(u16)

// A few notes on this implementation with regards to 'memcpy',
// - It seems that ptr::write_bytes already acts as volatile, while ptr::copy_* does not (in this case)
// - The registers are *not* cleared after processing, so padding needs to be written out
// - This component uses core::intrinsics::volatile_* which is unstable, but is the only way to
// efficiently copy memory with volatile
// - For this particular registers (and probably others), a full u32 needs to be written partial
// register writes (i.e. in u8 mode) does not work
//   - This means that we need to buffer bytes coming in up to 4 u8's in order to create a full u32
//     - The implementation for this is messy atm (the entire write_data function is for this purpose), it would be good to create a uniform interface
//     for writing into aligned registers 

// This only supports inputs up to u32::MAX in byte size, to increase please see ::finish()
// length/self.cursor usage
#[cfg(any(esp32s2, esp32s3, em_memsp32c2, esp32c3))]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        sha.mode.write(|w| unsafe {
            w.mode().bits(mode_as_bits(mode))
        });
        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            finished: false,
            write_buf: [0u8; 4]
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
                println!("[SHA] Starting engine as first run");
                self.sha.start.as_ptr().write_volatile(1u32);
            }
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            unsafe {
                println!("[SHA] Starting engine as continue");
                self.sha.continue_.as_ptr().write_volatile(1u32);
            }
        }
    }

    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA224 | ShaMode::SHA256 => 64,
            _ => 128,
        };
    }

    pub fn digest_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 => 20,
            ShaMode::SHA224 => 28,
            ShaMode::SHA256 => 32, 
            ShaMode::SHA384 => 48,
            ShaMode::SHA512 => 64,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_224 => 28,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_256 => 32, 
        }
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }
        
        let write_buf_fill = self.cursor % 4;
        println!("Flushing {} bytes from buffer", write_buf_fill);
        if write_buf_fill == 0 {
            return Ok(())
        }

        let chunk_len = self.chunk_length();
        for i in write_buf_fill..self.write_buf.len() {
            self.write_buf[i] = 0u8;
        }

        let write_buf_cursor = (self.cursor - write_buf_fill) % chunk_len;
        unsafe {
            let ptr = (self.sha.m_mem.as_ptr() as *mut u32).add(write_buf_cursor/4);
            println!("[Flsuh]: Writing {:02x?} to {:?}", self.write_buf, ptr);
            core::ptr::write_volatile(ptr, u32::from_ne_bytes(self.write_buf));
        }
        self.cursor = self.cursor.wrapping_add(4-write_buf_fill);
        block!(self.check_flush())?;
        Ok(())
    }

    // This function ensures that incoming data is aligned to u32 (due to issues with cpy_mem<u8>)
    // The first return value will be written into the current cursor if exists
    // If the second return value (always multiple of size<u32>) can be written to the rest of
    // the buffer
    // The final return value is the remaining data that cannot be used
    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible>
    {
        let chunk_len = self.chunk_length();
        let mod_cursor = self.cursor % chunk_len;
        //  (Option<u32>, &'a [u8], &'a [u8])
        if mod_cursor % 4 == 0 {
            // No data in write_buf, just write as much as we can from incoming
            let split_pt = core::cmp::min(chunk_len-mod_cursor, incoming.len());
            let (mut chunk, remaining) = incoming.split_at(split_pt);
            
            // Add full chunk to cursor for next call
            self.cursor = self.cursor.wrapping_add(chunk.len());
            // Incoming data is not aligned, store left over in write_buf
            if chunk.len() % 4 != 0 {
                let (nchunk, to_store) = chunk.split_at(chunk.len() - (chunk.len() % 4));
                chunk = nchunk;
                to_store.iter().enumerate().for_each(|(i, v)| {
                    if let Some(sv) = self.write_buf.get_mut(i) {
                        *sv = *v;
                    }
                });
            }

            if chunk.len() == 0{
                return Ok(remaining)
            }
            unsafe {
                // Volatile write to output
                let ptr = (self.sha.m_mem[0].as_ptr() as *mut u32).add(mod_cursor/4);
                println!("[Update]: Writing {:02x?} ({}) to {:?}", chunk, chunk.len(), ptr);
                core::intrinsics::volatile_copy_nonoverlapping_memory::<u32>(ptr, chunk.as_ptr() as *mut u32, chunk.len()/4);
            }

            Ok(remaining) 
        } else {
            // Current cursor not aligned, first load as much into write_buf as possible
            let write_buf_fill = mod_cursor % 4;
            let write_buf_cursor = (self.cursor - write_buf_fill) % chunk_len;
            for i in 0..(self.write_buf.len() - write_buf_fill) {
                match incoming.get(i) {
                    Some(v) => {
                        self.write_buf[write_buf_fill+i] = *v;
                        self.cursor = self.cursor.wrapping_add(1);
                    },
                    None => { return Ok(&[]) }
                }
            }
            // Not returned, so write_buf_fill is now 4
            unsafe {
                let ptr = (self.sha.m_mem.as_ptr() as *mut u32).add(write_buf_cursor/4);
                println!("[Update]: Writing {:02x?} to {:?}", self.write_buf, ptr);
                core::ptr::write_volatile(ptr, u32::from_ne_bytes(self.write_buf));
            }
            

            let mod_cursor = self.cursor % chunk_len;
            // Skip over previously written bytes
            let (_, incoming) = incoming.split_at(core::cmp::min(incoming.len(), 4-write_buf_fill));
            let (mut chunk, remaining) = incoming.split_at(core::cmp::min(incoming.len(), chunk_len-mod_cursor));

            // Add full chunk to cursor for next call
            self.cursor = self.cursor.wrapping_add(chunk.len());

             // Remaining data is also not aligned, store left over in write_buf
            if chunk.len() % 4 != 0 {
                let (nchunk, to_store) = chunk.split_at(chunk.len() - (chunk.len() % 4));
                chunk = nchunk;
                to_store.iter().enumerate().for_each(|(i, v)| {
                    if let Some(sv) = self.write_buf.get_mut(i) {
                        *sv = *v;
                    }
                });
            }
            
            if chunk.len() == 0{
                return Ok(remaining)
            }
            unsafe {
                // Volatile write to output 
                let ptr = (self.sha.m_mem[0].as_ptr() as *mut u32).add(mod_cursor/4);
                println!("[Update]: Writing {:02x?} ({}) to {:?}", chunk, chunk.len(), ptr);
                core::intrinsics::volatile_copy_nonoverlapping_memory::<u32>(ptr, chunk.as_ptr() as *mut u32, chunk.len()/4);
            }
             
            Ok(remaining)
        }
    }
    
    fn check_flush(&mut self) -> nb::Result<(), Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        // Finished writing current chunk, start accelerator
        if self.cursor % self.chunk_length() == 0 {
            self.process_buffer();
        }
        Ok(())
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        // NOTE: m_mem is 64,u8; but datasheet (esp32s2/esp32s3@>=SHA384) says 128, u8
        // Load data into M_n_REG
        let remaining = self.write_data(buffer)?;
        block!(self.check_flush())?;

        Ok(remaining)
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // TODO: should we enforce full hash readout in output?

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
            println!("Message len: {}", self.cursor);
            let length = (self.cursor * 8).to_be_bytes();
            block!(self.update(&[0x80]))?; // Append "1" bit
            block!(self.flush_data())?; // Flush partial data, ensures aligned cursor 

            let mod_cursor = self.cursor % chunk_len;

            // TODO: verify this works (should be 8 or 15 bytes free for this not to
            // trigger)
            println!(
                "[SHA] adding padding?: chunkl={}, cursor={}",
                chunk_len, mod_cursor
            );
            if chunk_len - mod_cursor < chunk_len / 8 {
                println!("[SHA] Adding padding!");
                // TODO: I think the default is zero after proc_buffer, so might not need most of
                // this

                // Zero out remaining data if buffer is almost full (>=448/896), and process
                // buffer
                let pad_len = chunk_len - mod_cursor;
                unsafe {
                    let m_cursor_ptr = (self.sha.m_mem[0].as_ptr() as *mut u32).add(mod_cursor/4);
                    println!("[Finish]: Writing [{:02x?},...] ({}) to {:?}", 0, pad_len, m_cursor_ptr);
                    //core::ptr::write_bytes::<u32>(m_cursor_ptr, 0, pad_len/4);
                    core::intrinsics::volatile_set_memory(m_cursor_ptr, 0, pad_len/4);
                }
                self.process_buffer();
                self.cursor = self.cursor.wrapping_add(pad_len);

                // Spin-wait for finish 
                while self.sha.busy.read().bits() != 0 {}
            }

            let mod_cursor = self.cursor % chunk_len; // Should be zero if branched above
            unsafe {
                let m_cursor_ptr = self.sha.m_mem[0].as_ptr() as *mut u32;
                // Pad zeros 
                let pad_ptr = m_cursor_ptr.add(mod_cursor/4);
                let pad_len = (chunk_len - mod_cursor) - 4; 

                println!("[Finish]: Writing [{:02x?},...] ({}) to {:?}", 0, pad_len, pad_ptr);
                //core::intrinsics::volatile_set_memory(pad_ptr, 0, pad_len/4);
                core::ptr::write_bytes(pad_ptr, 0, pad_len/4);

                // Write length (BE) to end
                println!(
                    "{:02x?}, {}, {:?}, cursor={}, {:?} {:?}",
                    length,
                    chunk_len - length.len(),
                    length.as_ptr(),
                    self.cursor,
                    m_cursor_ptr,
                    self.sha.m_mem.as_ptr().add(1),
                );
               
                let len_mem_ptr = (self.sha.m_mem.as_ptr() as *mut u32).add((chunk_len/4) -1);
                println!("[Finish]: Writing {:02x?} to {:?}", length, len_mem_ptr);
                // Needs volatile writes
                core::intrinsics::volatile_copy_memory::<u32>(len_mem_ptr, length.as_ptr() as *mut u32, length.len()/4);
            }

            self.process_buffer();

            // Spin-wait for final buffer to be processed
            while self.sha.busy.read().bits() != 0 {}

            self.finished = true;
        }

        unsafe {
            // TODO: limit read len to digest size
            core::intrinsics::volatile_copy_nonoverlapping_memory::<u32>(
                output.as_mut_ptr() as *mut u32,
                self.sha.h_mem.as_ptr() as *const u32,
                core::cmp::min(self.digest_length(), output.len())/4,
            );
        }
        Ok(())
    }

    pub fn free(self) -> SHA {
        self.sha
    }
}
