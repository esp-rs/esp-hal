use bytemuck::AnyBitPattern;

use crate::soc::efuse::{Efuse, EfuseBlock};

/// The bit field for get access to efuse data
#[derive(Clone, Copy)]
pub struct EfuseField {
    blk: EfuseBlock,
    bit_off: u16,
    bit_len: u16,
}

impl EfuseField {
    pub(crate) const fn new(blk: EfuseBlock, bit_off: u16, bit_len: u16) -> Self {
        Self {
            blk,
            bit_off,
            bit_len,
        }
    }
}

impl Efuse {
    /// Read field value in a little-endian order
    #[inline(always)]
    pub fn read_field_le<T: AnyBitPattern>(field: EfuseField) -> T {
        let mut output = core::mem::MaybeUninit::<T>::uninit();
        // represent output value as a bytes slice
        let mut bytes = unsafe {
            core::slice::from_raw_parts_mut(
                output.as_mut_ptr() as *mut u8,
                core::mem::size_of::<T>(),
            )
        };
        // get block address
        let block_address = field.blk.address();

        let bit_off = field.bit_off as usize;
        let bit_end = (field.bit_len as usize).min(bytes.len() * 8) + bit_off;

        let mut last_word_off = bit_off / 32;
        let mut last_word = unsafe { block_address.add(last_word_off).read_volatile() };
        let word_bit_off = bit_off % 32;
        let word_bit_ext = 32 - word_bit_off;
        let mut word_off = last_word_off;

        for bit_off in (bit_off..bit_end).step_by(32) {
            let word_bit_len = 32.min(bit_end - bit_off);

            if word_off != last_word_off {
                // read new word
                last_word_off = word_off;
                last_word = unsafe { block_address.add(last_word_off).read_volatile() };
            }

            let mut word = last_word >> word_bit_off;

            word_off += 1;

            if word_bit_len > word_bit_ext {
                // read the next word
                last_word_off = word_off;
                last_word = unsafe { block_address.add(last_word_off).read_volatile() };
                // append bits from a beginning of the next word
                word |= last_word.wrapping_shl((32 - word_bit_off) as u32);
            };

            if word_bit_len < 32 {
                // mask only needed bits of a word
                word &= u32::MAX >> (32 - word_bit_len);
            }

            // get data length in bytes (ceil)
            let byte_len = (word_bit_len + 7) / 8;
            // represent word as a byte slice
            let word_bytes =
                unsafe { core::slice::from_raw_parts(&word as *const u32 as *const u8, byte_len) };

            // copy word bytes to output value bytes
            bytes[..byte_len].copy_from_slice(word_bytes);

            // move read window forward
            bytes = &mut bytes[byte_len..];
        }

        // fill untouched bytes with zeros
        bytes.fill(0);

        unsafe { output.assume_init() }
    }

    /// Read field value in a big-endian order
    #[inline(always)]
    pub fn read_field_be<T: AnyBitPattern>(field: EfuseField) -> T {
        // read value in a little-endian order
        let mut output = Self::read_field_le::<T>(field);
        // represent output value as a byte slice
        let bytes = unsafe {
            core::slice::from_raw_parts_mut(
                &mut output as *mut T as *mut u8,
                core::mem::size_of::<T>(),
            )
        };
        // reverse byte order
        bytes.reverse();
        output
    }

    /// Read bit value.
    ///
    /// This function panics if the field's bit length is not equal to 1.
    #[inline(always)]
    pub fn read_bit(field: EfuseField) -> bool {
        assert_eq!(field.bit_len, 1);
        Self::read_field_le::<u8>(field) != 0
    }
}
