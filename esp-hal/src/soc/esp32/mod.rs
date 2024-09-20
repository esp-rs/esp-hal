//! # SOC (System-on-Chip) module (ESP32)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32` chip.

use core::ptr::addr_of_mut;

use crate::rtc_cntl::SocResetReason;

pub mod cpu_control;
pub mod efuse;
pub mod gpio;
pub mod peripherals;
#[cfg(feature = "quad-psram")]
pub mod psram;
pub mod radio_clocks;
pub mod trng;

/// The name of the chip ("esp32") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32"
    };
}

pub use chip;

pub(crate) mod constants {
    /// The base clock frequency for the I2S peripheral (Hertz).
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for I2S operations.
    pub const I2S_DEFAULT_CLK_SRC: u32 = 2;
    /// The starting address of the Remote Control (RMT) module's RAM.
    pub const RMT_RAM_START: usize = 0x3ff56800;
    /// The size, in bytes, of each RMT channel's dedicated RAM.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 64;
    /// The lower bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_LOW: u32 = 0x3FFA_E000;
    /// The upper bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_HIGH: u32 = 0x4000_0000;
    /// A reference clock tick of 1 MHz.
    pub const REF_TICK: fugit::HertzU32 = fugit::HertzU32::MHz(1);
}

/// Function initializes ESP32 specific memories (RTC slow and fast) and
/// then calls original Reset function
///
/// ENTRY point is defined in memory.x
/// *Note: the pre_init function is called in the original reset handler
/// after the initializations done in this function*
#[doc(hidden)]
#[no_mangle]
pub unsafe extern "C" fn ESP32Reset() -> ! {
    // These symbols come from `memory.x`
    extern "C" {
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
    xtensa_lx::set_stack_pointer(addr_of_mut!(_stack_start_cpu0));

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    xtensa_lx_rt::zero_bss(
        addr_of_mut!(_rtc_fast_bss_start),
        addr_of_mut!(_rtc_fast_bss_end),
    );
    xtensa_lx_rt::zero_bss(
        addr_of_mut!(_rtc_slow_bss_start),
        addr_of_mut!(_rtc_slow_bss_end),
    );
    if matches!(
        crate::reset::get_reset_reason(),
        None | Some(SocResetReason::ChipPowerOn)
    ) {
        xtensa_lx_rt::zero_bss(
            addr_of_mut!(_rtc_fast_persistent_start),
            addr_of_mut!(_rtc_fast_persistent_end),
        );
        xtensa_lx_rt::zero_bss(
            addr_of_mut!(_rtc_slow_persistent_start),
            addr_of_mut!(_rtc_slow_persistent_end),
        );
    }

    unsafe {
        let stack_chk_guard = core::ptr::addr_of_mut!(__stack_chk_guard);
        // we _should_ use a random value but we don't have a good source for random
        // numbers here
        stack_chk_guard.write_volatile(0xdeadbabe);
    }

    // continue with default reset handler
    xtensa_lx_rt::Reset();
}

/// The ESP32 has a first stage bootloader that handles loading program data
/// into the right place therefore we skip loading it again.
#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub extern "Rust" fn __init_data() -> bool {
    false
}
