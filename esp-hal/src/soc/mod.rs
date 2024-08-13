#![allow(missing_docs)] // TODO: Remove when able

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

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum SetMacError {
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
    /// overriden by `set_mac_address`.
    pub fn get_mac_address() -> [u8; 6] {
        if MAC_OVERRIDE_STATE.load(Ordering::Relaxed) == 2 {
            unsafe { MAC_OVERRIDE }
        } else {
            Self::read_base_mac_address()
        }
    }
}

#[allow(unused)]
pub(crate) fn is_valid_ram_address(address: u32) -> bool {
    (self::constants::SOC_DRAM_LOW..=self::constants::SOC_DRAM_HIGH).contains(&address)
}

#[allow(unused)]
pub(crate) fn is_valid_psram_address(address: u32) -> bool {
    #[cfg(psram)]
    {
        let start = crate::psram::psram_vaddr_start() as u32;
        let end = start + crate::psram::PSRAM_BYTES as u32;
        (start..=end).contains(&address)
    }
    #[cfg(not(psram))]
    false
}

#[allow(unused)]
pub(crate) fn is_valid_memory_address(address: u32) -> bool {
    is_valid_ram_address(address) || is_valid_psram_address(address)
}
