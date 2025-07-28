#![cfg_attr(not(feature = "rt"), expect(unused))]

use core::ops::Range;

pub use self::implementation::*;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
mod implementation;

#[allow(unused)]
pub(crate) fn is_valid_ram_address(address: usize) -> bool {
    addr_in_range(address, memory_range!("DRAM"))
}

#[allow(unused)]
pub(crate) fn is_slice_in_dram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, memory_range!("DRAM"))
}

#[allow(unused)]
#[cfg(psram)]
pub(crate) fn is_valid_psram_address(address: usize) -> bool {
    addr_in_range(address, crate::psram::psram_range())
}

#[allow(unused)]
#[cfg(psram)]
pub(crate) fn is_slice_in_psram<T>(slice: &[T]) -> bool {
    slice_in_range(slice, crate::psram::psram_range())
}

#[allow(unused)]
pub(crate) fn is_valid_memory_address(address: usize) -> bool {
    if is_valid_ram_address(address) {
        return true;
    }
    #[cfg(psram)]
    if is_valid_psram_address(address) {
        return true;
    }

    false
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
