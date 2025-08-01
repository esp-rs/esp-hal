//! Software implementations of the supported block cipher operating modes.
//!
//! These may be used by the Typical AES operating mode, as well as by the DMA-enabled driver where
//! the DMA does not support a particular operating mode in hardware.

use core::ptr::NonNull;

use super::{BLOCK_SIZE, CryptoBuffers};

/// Electronic codebook mode.
pub(super) struct Ecb {}
impl Ecb {
    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        buffer.for_data_chunks(BLOCK_SIZE, |input, output, len| {
            process_block(
                NonNull::slice_from_raw_parts(input, len),
                NonNull::slice_from_raw_parts(output, len),
            )
        });
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
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let iv = NonNull::from(self.iv.as_mut());

        buffer.for_data_chunks(BLOCK_SIZE, |plaintext, ciphertext, len| {
            // Block input is feedback/IV mixed with plaintext. We can let this overwrite the IV
            // because we'll overwrite this again with the ciphertext as the next IV.
            xor_into(iv.cast(), plaintext, len);

            // Block output is ciphertext directly
            process_block(iv, NonNull::slice_from_raw_parts(ciphertext, len));

            // Feed back the ciphertext for the next iteration.
            copy(iv.cast(), ciphertext, len);
        });
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let iv = NonNull::from(self.iv.as_mut());

        buffer.for_data_chunks(BLOCK_SIZE, |ciphertext, plaintext, len| {
            // Block output is plaintext, mixed with IV
            process_block(iv, NonNull::slice_from_raw_parts(plaintext, len));
            xor_into(plaintext, iv.cast(), len);

            // Next IV is the previous ciphertext
            copy(iv.cast(), ciphertext, len);
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
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            if offset == 0 {
                // Out of bytes, generate next key.
                let iv = NonNull::from(self.iv.as_mut());
                process_block(iv, iv);
            }

            // Calculate ciphertext by mixing the key with the plaintext.
            unsafe { ciphertext.write(plaintext.read() ^ self.iv[offset]) };
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
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
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
                let nonce = NonNull::from(self.nonce.as_mut());
                let buffer = NonNull::from(self.buffer.as_mut());
                // Block input is feedback/IV. Block output is feedback/IV.
                process_block(nonce, buffer);
                increment(&mut self.nonce);
            }

            // Calculate ciphertext by mixing the key with the plaintext.
            unsafe { ciphertext.write(plaintext.read() ^ self.buffer[offset]) };
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
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            ov.copy_from_slice(&self.iv);

            let iv = NonNull::from(self.iv.as_mut());
            process_block(iv, iv);

            unsafe {
                let out = self.iv[0] ^ plaintext.read();
                ciphertext.write(out);

                // Shift the key by a byte.
                self.iv[0..BLOCK_SIZE - 1].copy_from_slice(&ov[1..BLOCK_SIZE]);
                self.iv[BLOCK_SIZE - 1] = ciphertext.read();
            }
        });
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |ciphertext, plaintext, _| {
            ov.copy_from_slice(&self.iv);

            let iv = NonNull::from(self.iv.as_mut());
            process_block(iv, iv);

            unsafe {
                let c = ciphertext.read();
                let out = self.iv[0] ^ c;
                plaintext.write(out);

                // Shift the key by a byte.
                self.iv[0..BLOCK_SIZE - 1].copy_from_slice(&ov[1..BLOCK_SIZE]);
                self.iv[BLOCK_SIZE - 1] = c;
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
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            if offset == 0 {
                let iv = NonNull::from(self.iv.as_mut());
                process_block(iv, iv);
            }

            unsafe {
                self.iv[offset] ^= plaintext.read();
                ciphertext.write(self.iv[offset]);
            }
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: CryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |ciphertext, plaintext, _| {
            if offset == 0 {
                let iv = NonNull::from(self.iv.as_mut());
                process_block(iv, iv);
            }

            unsafe {
                let c = ciphertext.read();
                plaintext.write(self.iv[offset] ^ c);
                self.iv[offset] = c;
            }
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }
}

// Utilities

impl CryptoBuffers {
    fn for_data_chunks(
        self,
        chunk_size: usize,
        mut cb: impl FnMut(NonNull<u8>, NonNull<u8>, usize),
    ) {
        let input = pointer_chunks(self.input, chunk_size);
        let output = pointer_chunks(self.output, chunk_size);

        for (input, output, len) in input
            .zip(output)
            .map(|((input, len), (output, _))| (input, output, len))
        {
            cb(input, output, len)
        }
    }
}

fn pointer_chunks<T>(
    ptr: NonNull<[T]>,
    chunk: usize,
) -> impl Iterator<Item = (NonNull<T>, usize)> + Clone {
    let mut len = ptr.len();
    let mut ptr = ptr.cast::<T>();
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

fn xor_into(mut out: NonNull<u8>, mut a: NonNull<u8>, len: usize) {
    let end = unsafe { out.add(len) };
    while out < end {
        unsafe {
            out.write(out.read() ^ a.read());
            a = a.add(1);
            out = out.add(1);
        }
    }
}

fn copy(out: NonNull<u8>, from: NonNull<u8>, len: usize) {
    unsafe {
        out.copy_from(from, len);
    }
}
