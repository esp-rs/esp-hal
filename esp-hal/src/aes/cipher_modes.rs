//! Software implementations of the supported block cipher operating modes.
//!
//! These may be used by the Typical AES operating mode, as well as by the DMA-enabled driver where
//! the DMA does not support a particular operating mode in hardware.

use super::{BLOCK_SIZE, CryptoBuffers};

/// Electronic codebook mode.
pub(super) struct Ecb {}
impl Ecb {
    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: CryptoBuffers,
        process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        buffer.for_data_chunks(BLOCK_SIZE, process_block);
    }
}

/// Cipher block chaining mode.
pub(super) struct Cbc {
    pub(super) iv: [u8; BLOCK_SIZE],
}
impl Cbc {
    pub(super) fn encrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let iv = self.iv.as_mut_ptr();

        buffer.for_data_chunks(BLOCK_SIZE, |plaintext, ciphertext, len| {
            // Block input is feedback/IV mixed with plaintext. We can let this overwrite the IV
            // because we'll overwrite this again with the ciphertext as the next IV.
            xor_into(iv, plaintext, len);

            // Block output is ciphertext directly
            process_block(iv, ciphertext, len);

            // Feed back the ciphertext for the next iteration.
            copy(iv, ciphertext, len);
        });
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let iv = self.iv.as_mut_ptr();

        buffer.for_data_chunks(BLOCK_SIZE, |ciphertext, plaintext, len| {
            // Block output is plaintext, mixed with IV
            process_block(iv, plaintext, len);
            xor_into(plaintext, iv, len);

            // Next IV is the previous ciphertext
            copy(iv, ciphertext, len);
        });
    }
}

/// Output feedback mode.
pub(super) struct Ofb {
    pub(super) iv: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Ofb {
    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            if offset == 0 {
                // Out of bytes, generate next key.
                let iv = &mut self.iv as *mut _;
                process_block(iv, iv, BLOCK_SIZE);
            }

            // Calculate ciphertext by mixing the key with the plaintext.
            unsafe { *ciphertext = *plaintext ^ self.iv[offset] };
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }
}

/// Counter mode.
pub(super) struct Ctr {
    /// The nonce + counter. Note that the security of this relies on the nonce being random.
    pub(super) nonce: [u8; BLOCK_SIZE],
    /// The key produced by the block cipher.
    pub(super) buffer: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Ctr {
    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        fn increment(nonce: &mut [u8]) {
            for byte in nonce.iter_mut().rev() {
                *byte = byte.wrapping_add(1);
                if *byte != 0 {
                    break;
                }
            }
        }

        let mut offset = self.offset;
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            if offset == 0 {
                let nonce = self.nonce.as_ptr();
                let buffer = self.buffer.as_mut_ptr();
                // Block input is feedback/IV. Block output is feedback/IV.
                process_block(nonce, buffer, BLOCK_SIZE);
                increment(&mut self.nonce);
            }

            // Calculate ciphertext by mixing the key with the plaintext.
            unsafe { *ciphertext = *plaintext ^ self.buffer[offset] };
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }
}

/// Cipher feedback with 8-bit shift mode.
pub(super) struct Cfb8 {
    pub(super) iv: [u8; BLOCK_SIZE],
}
impl Cfb8 {
    pub(super) fn encrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            ov.copy_from_slice(&self.iv);

            let iv = self.iv.as_mut_ptr();
            process_block(iv, iv, BLOCK_SIZE);

            unsafe {
                let out = self.iv[0] ^ *plaintext;
                *ciphertext = out;

                // Shift the key by a byte.
                self.iv[0..BLOCK_SIZE - 1].copy_from_slice(&ov[1..BLOCK_SIZE]);
                self.iv[BLOCK_SIZE - 1] = *ciphertext;
            }
        });
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |ciphertext, plaintext, _| {
            ov.copy_from_slice(&self.iv);

            let iv = self.iv.as_mut_ptr();
            process_block(iv, iv, BLOCK_SIZE);

            unsafe {
                let out = self.iv[0] ^ *ciphertext;
                *plaintext = out;

                // Shift the key by a byte.
                self.iv[0..BLOCK_SIZE - 1].copy_from_slice(&ov[1..BLOCK_SIZE]);
                self.iv[BLOCK_SIZE - 1] = *ciphertext;
            }
        });
    }
}

/// Cipher feedback with 128-bit shift mode.
pub(super) struct Cfb128 {
    pub(super) iv: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Cfb128 {
    pub(super) fn encrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            if offset == 0 {
                let iv = self.iv.as_mut_ptr();
                process_block(iv, iv, BLOCK_SIZE);
            }

            unsafe {
                self.iv[offset] ^= *plaintext;
                *ciphertext = self.iv[offset];
            }
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(*const u8, *mut u8, usize),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |ciphertext, plaintext, _| {
            if offset == 0 {
                let iv = self.iv.as_mut_ptr();
                process_block(iv, iv, BLOCK_SIZE);
            }

            unsafe {
                let c = *ciphertext;
                *plaintext = self.iv[offset] ^ c;
                self.iv[offset] = c;
            }
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }
}

// Utilities

impl CryptoBuffers {
    fn for_data_chunks(self, chunk_size: usize, mut cb: impl FnMut(*const u8, *mut u8, usize)) {
        let input = pointer_chunks(self.input, self.text_length, chunk_size);
        let output = pointer_chunks_mut(self.output, self.text_length, chunk_size);

        for (input, output, len) in input
            .zip(output)
            .map(|((input, len), (output, _))| (input, output, len))
        {
            cb(input, output, len)
        }
    }
}

fn pointer_chunks<T>(
    mut ptr: *const T,
    mut len: usize,
    chunk: usize,
) -> impl Iterator<Item = (*const T, usize)> + Clone {
    core::iter::from_fn(move || {
        let advance = if len > chunk {
            chunk
        } else if len > 0 {
            len
        } else {
            return None;
        };

        let retval = (ptr, advance);

        unsafe { ptr = ptr.add(advance) };
        len -= advance;
        Some(retval)
    })
}

fn pointer_chunks_mut<T>(
    ptr: *mut T,
    len: usize,
    chunk: usize,
) -> impl Iterator<Item = (*mut T, usize)> + Clone {
    pointer_chunks(ptr.cast_const(), len, chunk).map(|(ptr, chunk_len)| (ptr.cast_mut(), chunk_len))
}

fn xor_into(mut out: *mut u8, mut a: *const u8, len: usize) {
    let end = unsafe { out.add(len) };
    while out < end {
        unsafe {
            *out ^= *a;
            a = a.add(1);
            out = out.add(1);
        }
    }
}

fn copy(out: *mut u8, from: *const u8, len: usize) {
    unsafe {
        out.copy_from(from, len);
    }
}
