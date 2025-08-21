//! Software implementations of the supported block cipher operating modes.
//!
//! These may be used by the Typical AES operating mode, as well as by the DMA-enabled driver where
//! the DMA does not support a particular operating mode in hardware.

use core::{marker::PhantomData, ptr::NonNull};

use super::{BLOCK_SIZE, Error};

#[derive(Clone, Copy)]
pub(crate) struct CryptoBuffers<'a> {
    buffers: UnsafeCryptoBuffers,
    _marker: PhantomData<&'a mut ()>,
}

impl<'a> CryptoBuffers<'a> {
    pub fn new(input: &'a [u8], output: &'a mut [u8]) -> Result<Self, Error> {
        if input.len() != output.len() {
            return Err(Error::BuffersNotEqual);
        }
        Ok(Self {
            buffers: UnsafeCryptoBuffers {
                input: NonNull::from(input),
                output: NonNull::from(output),
            },
            _marker: PhantomData,
        })
    }

    pub fn new_in_place(data: &'a mut [u8]) -> Self {
        let ptr = NonNull::from(data);
        Self {
            buffers: UnsafeCryptoBuffers {
                input: ptr,
                output: ptr,
            },
            _marker: PhantomData,
        }
    }

    pub(super) unsafe fn into_inner(self) -> UnsafeCryptoBuffers {
        self.buffers
    }
}

#[derive(Clone, Copy)]
pub(super) struct UnsafeCryptoBuffers {
    pub input: NonNull<[u8]>,
    pub output: NonNull<[u8]>,
}
impl UnsafeCryptoBuffers {
    pub fn in_place(&self) -> bool {
        self.input.addr() == self.output.addr()
    }

    #[cfg(aes_dma)]
    pub(crate) unsafe fn byte_add(self, bytes: usize) -> Self {
        UnsafeCryptoBuffers {
            input: unsafe { self.input.byte_add(bytes) },
            output: unsafe { self.output.byte_add(bytes) },
        }
    }
}

/// Electronic codebook mode.
#[derive(Clone)]
pub struct Ecb;
impl Ecb {
    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
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
#[derive(Clone)]
pub struct Cbc {
    pub(super) iv: [u8; BLOCK_SIZE],
}
impl Cbc {
    /// Creates a new context object.
    pub fn new(iv: [u8; BLOCK_SIZE]) -> Self {
        Self { iv }
    }

    pub(super) fn encrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
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
        buffer: UnsafeCryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let iv = NonNull::from(self.iv.as_mut()).cast::<u8>();

        if buffer.in_place() {
            let mut temp_buffer = [0; 16];
            let temp = NonNull::from(&mut temp_buffer).cast::<u8>();
            buffer.for_data_chunks(BLOCK_SIZE, |ciphertext, plaintext, len| {
                copy(temp, ciphertext, len);

                // Block output is plaintext, mixed with IV
                process_block(
                    NonNull::slice_from_raw_parts(ciphertext, len),
                    NonNull::slice_from_raw_parts(plaintext, len),
                );
                xor_into(plaintext, iv, len);

                // Next IV is the previous ciphertext
                copy(iv, temp, len);
            });
        } else {
            buffer.for_data_chunks(BLOCK_SIZE, |ciphertext, plaintext, len| {
                // Block output is plaintext, mixed with IV
                process_block(
                    NonNull::slice_from_raw_parts(ciphertext, len),
                    NonNull::slice_from_raw_parts(plaintext, len),
                );
                xor_into(plaintext, iv, len);

                // Next IV is the previous ciphertext
                copy(iv, ciphertext, len);
            });
        }
    }
}

/// Output feedback mode.
#[derive(Clone)]
pub struct Ofb {
    pub(super) iv: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Ofb {
    /// Creates a new context object.
    pub fn new(iv: [u8; BLOCK_SIZE]) -> Self {
        Self { iv, offset: 0 }
    }

    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut offset = self.offset;
        buffer.for_data_chunks(1, |input, output, _| {
            if offset == 0 {
                // Out of bytes, generate next key.
                let iv = NonNull::from(self.iv.as_mut());
                process_block(iv, iv);
            }

            // Calculate ciphertext by mixing the key with the plaintext.
            unsafe { output.write(input.read() ^ self.iv[offset]) };
            offset = (offset + 1) % BLOCK_SIZE;
        });
        self.offset = offset;
    }

    #[cfg(aes_dma)]
    pub(super) fn flush(&mut self, buffer: UnsafeCryptoBuffers) -> usize {
        let mut offset = self.offset;
        buffer
            .first_n((BLOCK_SIZE - offset) % BLOCK_SIZE)
            .for_data_chunks(1, |input, output, _| {
                unsafe { output.write(input.read() ^ self.iv[offset]) };
                offset += 1;
            });
        let flushed = offset - self.offset;
        self.offset = offset % BLOCK_SIZE;
        flushed
    }
}

/// Counter mode.
#[derive(Clone)]
pub struct Ctr {
    /// The nonce + counter. Note that the security of this relies on the nonce being random.
    pub(super) nonce: [u8; BLOCK_SIZE],
    /// The key produced by the block cipher.
    pub(super) buffer: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Ctr {
    /// Creates a new context object.
    pub fn new(nonce: [u8; BLOCK_SIZE]) -> Self {
        Self {
            nonce,
            buffer: [0; BLOCK_SIZE],
            offset: 0,
        }
    }

    pub(super) fn encrypt_decrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
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

    #[cfg(aes_dma)]
    pub(super) fn flush(&mut self, buffer: UnsafeCryptoBuffers) -> usize {
        let mut offset = self.offset;
        buffer
            .first_n((BLOCK_SIZE - offset) % BLOCK_SIZE)
            .for_data_chunks(1, |plaintext, ciphertext, _| {
                unsafe { ciphertext.write(plaintext.read() ^ self.buffer[offset]) };
                offset += 1;
            });
        let flushed = offset - self.offset;
        self.offset = offset % BLOCK_SIZE;
        flushed
    }
}

/// Cipher feedback with 8-bit shift mode.
#[derive(Clone)]
pub struct Cfb8 {
    pub(super) iv: [u8; BLOCK_SIZE],
}
impl Cfb8 {
    /// Creates a new context object.
    pub fn new(iv: [u8; BLOCK_SIZE]) -> Self {
        Self { iv }
    }

    pub(super) fn encrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |plaintext, ciphertext, _| {
            process_block(NonNull::from(self.iv.as_mut()), NonNull::from(ov.as_mut()));

            unsafe {
                let out = ov[0] ^ plaintext.read();
                ciphertext.write(out);

                // Shift IV by a byte.
                self.iv.copy_within(1.., 0);
                self.iv[BLOCK_SIZE - 1] = out;
            }
        });
    }

    pub(super) fn decrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
        mut process_block: impl FnMut(NonNull<[u8]>, NonNull<[u8]>),
    ) {
        let mut ov = [0; BLOCK_SIZE];
        buffer.for_data_chunks(1, |ciphertext, plaintext, _| {
            process_block(NonNull::from(self.iv.as_mut()), NonNull::from(ov.as_mut()));

            unsafe {
                let c = ciphertext.read();
                plaintext.write(ov[0] ^ c);

                // Shift IV by a byte.
                self.iv.copy_within(1.., 0);
                self.iv[BLOCK_SIZE - 1] = c;
            }
        });
    }
}

/// Cipher feedback with 128-bit shift mode.
#[derive(Clone)]
pub struct Cfb128 {
    pub(super) iv: [u8; BLOCK_SIZE],
    pub(super) offset: usize,
}
impl Cfb128 {
    /// Creates a new context object.
    pub fn new(iv: [u8; BLOCK_SIZE]) -> Self {
        Self { iv, offset: 0 }
    }

    pub(super) fn encrypt(
        &mut self,
        buffer: UnsafeCryptoBuffers,
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
        buffer: UnsafeCryptoBuffers,
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

    #[cfg(aes_dma)]
    pub(super) fn flush_encrypt(&mut self, buffer: UnsafeCryptoBuffers) -> usize {
        let mut offset = self.offset;
        buffer
            .first_n((BLOCK_SIZE - offset) % BLOCK_SIZE)
            .for_data_chunks(1, |plaintext, ciphertext, _| {
                unsafe {
                    self.iv[offset] ^= plaintext.read();
                    ciphertext.write(self.iv[offset]);
                }
                offset += 1;
            });
        let flushed = offset - self.offset;
        self.offset = offset % BLOCK_SIZE;
        flushed
    }

    #[cfg(aes_dma)]
    pub(super) fn flush_decrypt(&mut self, buffer: UnsafeCryptoBuffers) -> usize {
        let mut offset = self.offset;
        buffer
            .first_n((BLOCK_SIZE - offset) % BLOCK_SIZE)
            .for_data_chunks(1, |ciphertext, plaintext, _| {
                unsafe {
                    let c = ciphertext.read();
                    plaintext.write(self.iv[offset] ^ c);
                    self.iv[offset] = c;
                }
                offset += 1;
            });
        let flushed = offset - self.offset;
        self.offset = offset % BLOCK_SIZE;
        flushed
    }
}

// Utilities

impl UnsafeCryptoBuffers {
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

    #[cfg(aes_dma)]
    fn first_n(self, n: usize) -> UnsafeCryptoBuffers {
        let len = n.min(self.input.len());
        Self {
            input: NonNull::slice_from_raw_parts(self.input.cast(), len),
            output: NonNull::slice_from_raw_parts(self.output.cast(), len),
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
