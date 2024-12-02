use core::ops::Range;

use portable_atomic::{AtomicU8, Ordering};

pub use self::implementation::*;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
mod implementation;

mod efuse_field;

#[cfg(any(feature = "quad-psram", feature = "octal-psram"))]
mod psram_common;

// Using static mut should be fine since we are only writing to it once during
// initialization. As other tasks and interrupts are not running yet, the worst
// that can happen is, that the user creates a DMA buffer before initializing
// the HAL. This will access the PSRAM range, returning an empty range - which
// is, at that point, true. The user has no (safe) means to allocate in PSRAM
// before initializing the HAL.
#[cfg(any(feature = "quad-psram", feature = "octal-psram"))]
static mut MAPPED_PSRAM: MappedPsram = MappedPsram { memory_range: 0..0 };

fn psram_range_internal() -> Range<usize> {
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "quad-psram", feature = "octal-psram"))] {
            unsafe { MAPPED_PSRAM.memory_range.clone() }
        } else {
            0..0
        }
    }
}

const DRAM: Range<usize> = self::constants::SOC_DRAM_LOW..self::constants::SOC_DRAM_HIGH;

#[cfg(any(feature = "quad-psram", feature = "octal-psram"))]
pub struct MappedPsram {
    memory_range: Range<usize>,
}

// Indicates the state of setting the mac address
// 0 -- unset
// 1 -- in the process of being set
// 2 -- set
//
// Values other than 0 indicate that we cannot attempt setting the mac address
// again, and values other than 2 indicate that we should read the mac address
// from eFuse.
static MAC_OVERRIDE_STATE: AtomicU8 = AtomicU8::new(0);
static mut MAC_OVERRIDE: [u8; 6] = [0; 6];

/// Error indicating issues with setting the MAC address.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum SetMacError {
    /// The MAC address has already been set and cannot be changed.
    AlreadySet,
}

impl self::efuse::Efuse {
    /// Set the base mac address
    ///
    /// The new value will be returned by `read_mac_address` instead of the one
    /// hard-coded in eFuse. This does not persist across device resets.
    ///
    /// Can only be called once. Returns `Err(SetMacError::AlreadySet)`
    /// otherwise.
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
    pub fn mac_address() -> [u8; 6] {
        if MAC_OVERRIDE_STATE.load(Ordering::Relaxed) == 2 {
            unsafe { MAC_OVERRIDE }
        } else {
            Self::read_base_mac_address()
        }
    }
}

#[allow(unused)]
pub(crate) fn is_valid_ram_address(address: usize) -> bool {
    addr_in_range(address, DRAM)
}

#[allow(unused)]
pub(crate) fn is_slice_in_dram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, DRAM)
}

#[allow(unused)]
pub(crate) fn is_valid_psram_address(address: usize) -> bool {
    addr_in_range(address, psram_range_internal())
}

#[allow(unused)]
pub(crate) fn is_slice_in_psram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, psram_range_internal())
}

#[allow(unused)]
pub(crate) fn is_valid_memory_address(address: usize) -> bool {
    is_valid_ram_address(address) || is_valid_psram_address(address)
}

fn slice_in_range<T>(slice: &[T], range: Range<usize>) -> bool {
    let slice = slice.as_ptr_range();
    let start = slice.start as usize;
    let end = slice.end as usize;
    // `end` is >= `start`, so we don't need to check that `end > range.start`
    // `end` is also one past the last element, so it can be equal to the range's
    // end which is also one past the memory region's last valid address.
    addr_in_range(start, range.clone()) && end <= range.end
}

fn addr_in_range(addr: usize, range: Range<usize>) -> bool {
    range.contains(&addr)
}
