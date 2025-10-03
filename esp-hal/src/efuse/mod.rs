#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # eFuse (one-time programmable configuration)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the chip, allowing access to various chip-specific
//! information such as:
//!
//!   * MAC address
//!   * Chip revision
//!
//! and more. It is useful for retrieving chip-specific configuration and
//! identification data during runtime.
//!
//! The `Efuse` struct represents the eFuse peripheral and is responsible for
//! reading various eFuse fields and values.
//!
//! ## Examples
//!
//! ### Read data from the eFuse storage.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::efuse::{Efuse, SECURE_VERSION};
//!
//! let mac_address = Efuse::read_base_mac_address();
//!
//! println!(
//!     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
//!     mac_address[0],
//!     mac_address[1],
//!     mac_address[2],
//!     mac_address[3],
//!     mac_address[4],
//!     mac_address[5]
//! );
//!
//! println!("MAC address {:02x?}", Efuse::mac_address());
//! println!("Flash Encryption {:?}", Efuse::flash_encryption());
//!
//! // Besides the helper methods, various eFuse field constants can also be read with a lower-level API:
//!
//! println!(
//!     "Secure version {:?}",
//!     Efuse::read_field_le::<u16>(SECURE_VERSION)
//! );
//! # {after_snippet}
//! ```

use core::{cmp, mem, slice, sync::atomic::Ordering};

use bytemuck::AnyBitPattern;
use portable_atomic::AtomicU8;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
pub(crate) mod implem;

pub use implem::*;

/// The bit field for get access to efuse data
#[derive(Debug, Clone, Copy)]
#[instability::unstable]
pub struct EfuseField {
    /// The block
    pub(crate) block: EfuseBlock,
    /// Word number - this is just informational
    pub(crate) _word: u32,
    /// Starting bit in the efuse block
    pub(crate) bit_start: u32,
    /// Number of bits
    pub(crate) bit_count: u32,
}

impl EfuseField {
    pub(crate) const fn new(block: u32, word: u32, bit_start: u32, bit_count: u32) -> Self {
        Self {
            block: EfuseBlock::from_repr(block).unwrap(),
            _word: word,
            bit_start,
            bit_count,
        }
    }
}

/// A struct representing the eFuse functionality of the chip.
#[instability::unstable]
pub struct Efuse;

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    #[instability::unstable]
    pub fn read_base_mac_address() -> [u8; 6] {
        let mut mac_addr = [0u8; 6];

        let mac0 = Self::read_field_le::<[u8; 4]>(crate::efuse::MAC0);
        let mac1 = Self::read_field_le::<[u8; 2]>(crate::efuse::MAC1);

        // MAC address is stored in big endian, so load the bytes in reverse:
        mac_addr[0] = mac1[1];
        mac_addr[1] = mac1[0];
        mac_addr[2] = mac0[3];
        mac_addr[3] = mac0[2];
        mac_addr[4] = mac0[1];
        mac_addr[5] = mac0[0];

        mac_addr
    }

    /// Read field value in a little-endian order
    #[inline(always)]
    #[instability::unstable]
    pub fn read_field_le<T: AnyBitPattern>(field: EfuseField) -> T {
        let EfuseField {
            block,
            bit_start,
            bit_count,
            ..
        } = field;

        // Represent output value as a bytes slice:
        let mut output = mem::MaybeUninit::<T>::uninit();
        let mut bytes = unsafe {
            slice::from_raw_parts_mut(output.as_mut_ptr() as *mut u8, mem::size_of::<T>())
        };

        let bit_off = bit_start as usize;
        let bit_end = cmp::min(bit_count as usize, bytes.len() * 8) + bit_off;

        let mut last_word_off = bit_off / 32;
        let mut last_word = unsafe { block.address().add(last_word_off).read_volatile() };

        let word_bit_off = bit_off % 32;
        let word_bit_ext = 32 - word_bit_off;

        let mut word_off = last_word_off;
        for bit_off in (bit_off..bit_end).step_by(32) {
            if word_off != last_word_off {
                // Read a new word:
                last_word_off = word_off;
                last_word = unsafe { block.address().add(last_word_off).read_volatile() };
            }

            let mut word = last_word >> word_bit_off;
            word_off += 1;

            let word_bit_len = cmp::min(bit_end - bit_off, 32);
            if word_bit_len > word_bit_ext {
                // Read the next word:
                last_word_off = word_off;
                last_word = unsafe { block.address().add(last_word_off).read_volatile() };
                // Append bits from a beginning of the next word:
                word |= last_word.wrapping_shl((32 - word_bit_off) as u32);
            };

            if word_bit_len < 32 {
                // Mask only needed bits of a word:
                word &= u32::MAX >> (32 - word_bit_len);
            }

            // Represent word as a byte slice:
            let byte_len = word_bit_len.div_ceil(8);
            let word_bytes =
                unsafe { slice::from_raw_parts(&word as *const u32 as *const u8, byte_len) };

            // Copy word bytes to output value bytes:
            bytes[..byte_len].copy_from_slice(word_bytes);

            // Move read window forward:
            bytes = &mut bytes[byte_len..];
        }

        // Fill untouched bytes with zeros:
        bytes.fill(0);

        unsafe { output.assume_init() }
    }

    /// Read bit value.
    ///
    /// This function panics if the field's bit length is not equal to 1.
    #[inline(always)]
    #[instability::unstable]
    pub fn read_bit(field: EfuseField) -> bool {
        assert_eq!(field.bit_count, 1);
        Self::read_field_le::<u8>(field) != 0
    }

    /// Set the base mac address
    ///
    /// The new value will be returned by `read_mac_address` instead of the one
    /// hard-coded in eFuse. This does not persist across device resets.
    ///
    /// Can only be called once. Returns `Err(SetMacError::AlreadySet)`
    /// otherwise.
    #[instability::unstable]
    pub fn set_mac_address(mac: [u8; 6]) -> Result<(), SetMacError> {
        if MAC_OVERRIDE_STATE
            .compare_exchange(0, 1, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return Err(SetMacError::AlreadySet);
        }

        unsafe {
            MAC_OVERRIDE = mac;
        }

        MAC_OVERRIDE_STATE.store(2, Ordering::Relaxed);

        Ok(())
    }

    /// Get base mac address
    ///
    /// By default this reads the base mac address from eFuse, but it can be
    /// overridden by `set_mac_address`.
    #[instability::unstable]
    pub fn mac_address() -> [u8; 6] {
        if MAC_OVERRIDE_STATE.load(Ordering::Relaxed) == 2 {
            unsafe { MAC_OVERRIDE }
        } else {
            Self::read_base_mac_address()
        }
    }

    /// Returns the hardware revision
    ///
    /// The chip version is calculated using the following
    /// formula: MAJOR * 100 + MINOR. (if the result is 1, then version is v0.1)
    pub fn chip_revision() -> u16 {
        Self::major_chip_version() as u16 * 100 + Self::minor_chip_version() as u16
    }
}

// Indicates the state of setting the mac address
// 0 -- unset
// 1 -- in the process of being set
// 2 -- set
//
// Values other than 0 indicate that we cannot attempt setting the mac address
// again, and values other than 2 indicate that we should read the mac address
// from eFuse.
#[cfg_attr(not(feature = "unstable"), allow(unused))]
static MAC_OVERRIDE_STATE: AtomicU8 = AtomicU8::new(0);
#[cfg_attr(not(feature = "unstable"), allow(unused))]
static mut MAC_OVERRIDE: [u8; 6] = [0; 6];

/// Error indicating issues with setting the MAC address.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[instability::unstable]
pub enum SetMacError {
    /// The MAC address has already been set and cannot be changed.
    AlreadySet,
}
