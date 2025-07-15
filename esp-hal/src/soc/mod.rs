#![cfg_attr(not(feature = "rt"), expect(unused))]

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

#[cfg(feature = "psram")]
mod psram_common;

// Using static mut should be fine since we are only writing to it once during
// initialization. As other tasks and interrupts are not running yet, the worst
// that can happen is, that the user creates a DMA buffer before initializing
// the HAL. This will access the PSRAM range, returning an empty range - which
// is, at that point, true. The user has no (safe) means to allocate in PSRAM
// before initializing the HAL.
#[cfg(feature = "psram")]
static mut MAPPED_PSRAM: MappedPsram = MappedPsram { memory_range: 0..0 };

pub(crate) fn psram_range() -> Range<usize> {
    cfg_if::cfg_if! {
        if #[cfg(feature = "psram")] {
            #[allow(static_mut_refs)]
            unsafe { MAPPED_PSRAM.memory_range.clone() }
        } else {
            0..0
        }
    }
}

#[cfg(feature = "psram")]
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
#[cfg_attr(not(feature = "unstable"), allow(unused))]
static MAC_OVERRIDE_STATE: AtomicU8 = AtomicU8::new(0);
#[cfg_attr(not(feature = "unstable"), allow(unused))]
static mut MAC_OVERRIDE: [u8; 6] = [0; 6];

/// Error indicating issues with setting the MAC address.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub enum SetMacError {
    /// The MAC address has already been set and cannot be changed.
    AlreadySet,
}

#[cfg_attr(not(feature = "unstable"), allow(unused))]
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
    addr_in_range(address, memory_range!("DRAM"))
}

#[allow(unused)]
pub(crate) fn is_slice_in_dram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, memory_range!("DRAM"))
}

#[allow(unused)]
pub(crate) fn is_valid_psram_address(address: usize) -> bool {
    addr_in_range(address, psram_range())
}

#[allow(unused)]
pub(crate) fn is_slice_in_psram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, psram_range())
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

pub(crate) fn addr_in_range(addr: usize, range: Range<usize>) -> bool {
    range.contains(&addr)
}

#[cfg(feature = "rt")]
#[cfg(riscv)]
#[unsafe(export_name = "hal_main")]
fn hal_main(a0: usize, a1: usize, a2: usize) -> ! {
    unsafe extern "Rust" {
        // This symbol will be provided by the user via `#[entry]`
        fn main(a0: usize, a1: usize, a2: usize) -> !;
    }

    setup_stack_guard();

    unsafe {
        main(a0, a1, a2);
    }
}

#[cfg(xtensa)]
#[cfg(feature = "rt")]
#[unsafe(no_mangle)]
#[cfg_attr(esp32s3, unsafe(link_section = ".rwtext"))]
unsafe extern "C" fn ESP32Reset() -> ! {
    unsafe {
        configure_cpu_caches();
    }

    /// The ESP32 has a first stage bootloader that handles loading program data
    /// into the right place therefore we skip loading it again. This function
    /// is called by xtensa-lx-rt in Reset.
    #[doc(hidden)]
    #[unsafe(no_mangle)]
    pub extern "Rust" fn __init_data() -> bool {
        false
    }

    // These symbols come from `memory.x`
    unsafe extern "C" {
        static mut _rtc_fast_bss_start: u32;
        static mut _rtc_fast_bss_end: u32;
        static mut _rtc_fast_persistent_start: u32;
        static mut _rtc_fast_persistent_end: u32;

        static mut _rtc_slow_bss_start: u32;
        static mut _rtc_slow_bss_end: u32;
        static mut _rtc_slow_persistent_start: u32;
        static mut _rtc_slow_persistent_end: u32;

        static mut _stack_start_cpu0: u32;

        static mut __stack_chk_guard: u32;
    }

    // set stack pointer to end of memory: no need to retain stack up to this point
    unsafe {
        xtensa_lx::set_stack_pointer(core::ptr::addr_of_mut!(_stack_start_cpu0));
    }

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    unsafe {
        xtensa_lx_rt::zero_bss(
            core::ptr::addr_of_mut!(_rtc_fast_bss_start),
            core::ptr::addr_of_mut!(_rtc_fast_bss_end),
        );
        xtensa_lx_rt::zero_bss(
            core::ptr::addr_of_mut!(_rtc_slow_bss_start),
            core::ptr::addr_of_mut!(_rtc_slow_bss_end),
        );
    }
    if matches!(
        crate::system::reset_reason(),
        None | Some(crate::rtc_cntl::SocResetReason::ChipPowerOn)
    ) {
        unsafe {
            xtensa_lx_rt::zero_bss(
                core::ptr::addr_of_mut!(_rtc_fast_persistent_start),
                core::ptr::addr_of_mut!(_rtc_fast_persistent_end),
            );
            xtensa_lx_rt::zero_bss(
                core::ptr::addr_of_mut!(_rtc_slow_persistent_start),
                core::ptr::addr_of_mut!(_rtc_slow_persistent_end),
            );
        }
    }

    setup_stack_guard();

    crate::interrupt::setup_interrupts();

    // continue with default reset handler
    unsafe { xtensa_lx_rt::Reset() }
}

#[cfg(feature = "rt")]
#[unsafe(export_name = "__stack_chk_fail")]
unsafe extern "C" fn stack_chk_fail() {
    panic!("Stack corruption detected");
}

#[cfg(feature = "rt")]
fn setup_stack_guard() {
    unsafe extern "C" {
        static mut __stack_chk_guard: u32;
    }

    unsafe {
        let stack_chk_guard = core::ptr::addr_of_mut!(__stack_chk_guard);
        // we _should_ use a random value but we don't have a good source for random
        // numbers here
        stack_chk_guard.write_volatile(esp_config::esp_config_int!(
            u32,
            "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
        ));
    }
}
