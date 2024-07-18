//! # SOC (System-on-Chip) module (ESP32-S3)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S3` chip.
//!
//! Also few constants are defined in this module for `ESP32-S3` chip:
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source

use core::ptr::addr_of_mut;

use self::peripherals::{LPWR, TIMG0, TIMG1};
use crate::{
    rtc_cntl::{Rtc, SocResetReason},
    timer::timg::Wdt,
};

pub mod cpu_control;
pub mod efuse;
pub mod gpio;
pub mod peripherals;
#[cfg(psram)]
pub mod psram;
pub mod radio_clocks;
pub mod trng;

pub mod ulp_core;

/// The name of the chip ("esp32s3") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32s3"
    };
}

pub use chip;

pub(crate) mod constants {
    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    pub const RMT_RAM_START: usize = 0x60016800;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: u8 = 1;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);

    pub const SOC_DRAM_LOW: u32 = 0x3FC8_8000;
    pub const SOC_DRAM_HIGH: u32 = 0x3FD0_0000;

    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

#[doc(hidden)]
#[link_section = ".rwtext"]
pub unsafe fn configure_cpu_caches() {
    // this is just the bare minimum we need to run code from flash
    // consider implementing more advanced configurations
    // see https://github.com/apache/incubator-nuttx/blob/master/arch/xtensa/src/esp32s3/esp32s3_start.c

    extern "C" {
        fn rom_config_instruction_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );
    }

    // ideally these should be configurable
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE: u32 = 0x8000; // ESP32S3_INSTRUCTION_CACHE_32KB
    const CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS: u8 = 8; // ESP32S3_INSTRUCTION_CACHE_8WAYS
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE: u8 = 32; // ESP32S3_INSTRUCTION_CACHE_LINE_32B

    // Configure the mode of instruction cache: cache size, cache line size.
    rom_config_instruction_cache_mode(
        CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE,
        CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS,
        CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE,
    );
}

/// Function initializes ESP32S3 specific memories (RTC slow and fast) and
/// then calls original Reset function
///
/// ENTRY point is defined in memory.x
/// *Note: the pre_init function is called in the original reset handler
/// after the initializations done in this function*
#[doc(hidden)]
#[no_mangle]
#[link_section = ".rwtext"]
pub unsafe extern "C" fn ESP32Reset() -> ! {
    configure_cpu_caches();

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

    crate::interrupt::setup_interrupts();

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

#[export_name = "__post_init"]
unsafe fn post_init() {
    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(LPWR::steal());
    rtc.rwdt.disable();

    Wdt::<TIMG0, crate::Blocking>::set_wdt_enabled(false);
    Wdt::<TIMG1, crate::Blocking>::set_wdt_enabled(false);
}

// TODO: This need to implement eratta workarrounds:
// https://github.com/espressif/esp-idf/blob/master/components/esp_rom/patches/esp_rom_cache_esp32s2_esp32s3.c#L92-L102
#[cfg(esp32s3)]
#[doc(hidden)]
#[link_section = ".rwtext"]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }
    Cache_WriteBack_Addr(addr, size);
}
