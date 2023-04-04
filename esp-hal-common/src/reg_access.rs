//! Utils
//!
//! # Overview
//!
//! Collection of struct which helps you write to registers.

use crate::peripherals::generic::{Readable, Reg, RegisterSpec, Resettable, Writable};

const U32_ALIGN_SIZE: usize = core::mem::size_of::<u32>();

const U32_FROM_BYTES: fn([u8; 4]) -> u32 = u32::from_ne_bytes;
const U32_TO_BYTES: fn(u32) -> [u8; 4] = u32::to_ne_bytes;

// The alignment helper helps you write to registers that only accepts u32
// using regular u8s (bytes). It keeps a write buffer of 4 u8 (could in theory
// be 3 but less convient). And if the incoming data is not convertable to u32
// (i.e. not a multiple of 4 in length) it will store the remainder in the
// buffer until the next call.
//
// It assumes incoming `dst` are aligned to desired layout (in future
// ptr.is_aligned can be used). It also assumes that writes are done in FIFO
// order.
#[derive(Debug)]
pub(crate) struct AlignmentHelper {
    buf: [u8; U32_ALIGN_SIZE],
    buf_fill: usize,
}

impl AlignmentHelper {
    pub fn default() -> AlignmentHelper {
        AlignmentHelper {
            buf: [0u8; U32_ALIGN_SIZE],
            buf_fill: 0,
        }
    }

    pub fn reset(&mut self) {
        self.buf_fill = 0;
    }

    pub const fn align_size(&self) -> usize {
        U32_ALIGN_SIZE
    }

    // This function will write any remaining buffer to dst and return the
    // amount of *bytes* written (0 means no write). If the buffer is not
    // aligned to the size of the register destination, it will append the '0'
    // value.
    pub fn flush_to<T, Ux>(&mut self, dst: &mut [Reg<T>], offset: usize) -> usize
    where
        T: RegisterSpec<Ux = Ux> + Resettable + Writable,
    {
        if self.buf_fill != 0 {
            for i in self.buf_fill..U32_ALIGN_SIZE {
                self.buf[i] = 0;
            }

            let dst_ptr = unsafe { (dst[0].as_ptr() as *mut u32).add(offset) };
            unsafe {
                dst_ptr.write_volatile(U32_FROM_BYTES(self.buf));
            }

            let ret = self.align_size() - self.buf_fill;
            self.buf_fill = 0;
            ret
        } else {
            0
        }
    }

    // This function is similar to `volatile_set_memory` but will prepend data that
    // was previously ingested and ensure aligned (u32) writes.
    pub fn volatile_write_bytes<T, Ux>(
        &mut self,
        dst: &mut [Reg<T>],
        val: u8,
        count: usize,
        offset: usize,
    ) where
        T: RegisterSpec<Ux = Ux> + Resettable + Writable,
    {
        let dst_ptr = unsafe { (dst[0].as_ptr() as *mut u32).add(offset) };

        let mut cursor = if self.buf_fill != 0 {
            for i in self.buf_fill..U32_ALIGN_SIZE {
                self.buf[i] = val;
            }

            unsafe {
                dst_ptr.write_volatile(U32_FROM_BYTES(self.buf));
            }

            self.buf_fill = 0;
            1
        } else {
            0
        };

        while cursor < count {
            unsafe {
                dst_ptr
                    .add(cursor)
                    .write_volatile(U32_FROM_BYTES([0_u8; 4]));
            }
            cursor += 1;
        }
    }

    // This function is similar to `volatile_copy_nonoverlapping_memory`,
    // however it buffers up to a u32 in order to always write to registers in
    // an aligned way. Additionally it will keep stop writing when the end of
    // the register (defined by `dst_bound` relative to `dst`) and returns the
    // remaining data (if not possible to write everything), and if it wrote
    // till dst_bound or exited early (due to lack of data).
    pub fn aligned_volatile_copy<'a, T, Ux>(
        &mut self,
        dst: &mut [Reg<T>],
        src: &'a [u8],
        dst_bound: usize,
        offset: usize,
    ) -> (&'a [u8], bool)
    where
        T: RegisterSpec<Ux = Ux> + Resettable + Writable,
    {
        assert!(dst_bound > 0);

        let dst_ptr = unsafe { (dst[0].as_ptr() as *mut u32).add(offset) };

        let mut nsrc = src;
        let mut cursor = 0;
        if self.buf_fill != 0 {
            // First prepend existing data
            let max_fill = U32_ALIGN_SIZE - self.buf_fill;
            let (nbuf, src) = src.split_at(core::cmp::min(src.len(), max_fill));
            nsrc = src;
            for i in 0..max_fill {
                match nbuf.get(i) {
                    Some(v) => {
                        self.buf[self.buf_fill] = *v;
                        self.buf_fill += 1;
                    }
                    None => return (&[], false), // Used up entire buffer before filling buff_fil
                }
            }

            unsafe {
                dst_ptr.write_volatile(U32_FROM_BYTES(self.buf));
            }
            cursor += 1;

            self.buf_fill = 0;
        }

        if dst_bound <= offset + cursor {
            return (nsrc, true);
        }

        let (to_write, remaining) = nsrc.split_at(core::cmp::min(
            (dst_bound - offset - cursor) * U32_ALIGN_SIZE,
            (nsrc.len() / U32_ALIGN_SIZE) * U32_ALIGN_SIZE,
        ));

        if to_write.len() > 0 {
            for (i, v) in to_write.chunks_exact(U32_ALIGN_SIZE).enumerate() {
                unsafe {
                    dst_ptr
                        .add(i + cursor)
                        .write_volatile(U32_FROM_BYTES(v.try_into().unwrap()));
                }
            }
        }

        // If it's data we can't store we don't need to try and align it, just wait for
        // next write Generally this applies when (src/4*4) != src
        let was_bounded = (offset + cursor + to_write.len() / U32_ALIGN_SIZE) == dst_bound;

        if remaining.len() > 0 && remaining.len() < 4 {
            for i in 0..remaining.len() {
                self.buf[i] = remaining[i];
            }

            self.buf_fill = remaining.len();

            return (&[], was_bounded);
        }

        return (remaining, was_bounded);
    }

    pub fn volatile_write_regset<T, Ux>(&mut self, dst: &mut Reg<T>, src: &[u8], dst_bound: usize)
    where
        T: RegisterSpec<Ux = Ux> + Resettable + Writable,
    {
        assert!(dst_bound > 0);
        assert!(src.len() <= dst_bound * 4);

        let dst_ptr = dst.as_ptr() as *mut u32;

        if src.len() > 0 {
            for (i, v) in src.chunks_exact(U32_ALIGN_SIZE).enumerate() {
                unsafe {
                    dst_ptr
                        .add(i)
                        .write_volatile(U32_FROM_BYTES(v.try_into().unwrap()));
                }
            }
        }
    }

    pub fn volatile_read_regset<T, Ux>(&self, src: &Reg<T>, dst: &mut [u8], dst_bound: usize)
    where
        T: RegisterSpec<Ux = Ux> + Readable,
    {
        assert!(dst.len() >= dst_bound * 4);

        let src_ptr = src.as_ptr() as *const u32;

        let chunks = dst.chunks_exact_mut(U32_ALIGN_SIZE);
        for (i, chunk) in chunks.enumerate() {
            let read_val: [u8; U32_ALIGN_SIZE] =
                unsafe { U32_TO_BYTES(src_ptr.add(i).read_volatile()) };
            chunk.copy_from_slice(&read_val);
        }
    }
}
