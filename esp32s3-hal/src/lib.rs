//! `no_std` HAL for the ESP32-S3 from Espressif.
//!
//! Implements a number of the traits defined by the various packages in the
//! [embedded-hal] repository.
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal
//!
//! ### Cargo Features
//!
//! - `async` - Enable support for asynchronous operation, with interfaces
//!   provided by [embedded-hal-async] and [embedded-io-async]
//! - `debug` - Enable debug features in the HAL (used for development)
//! - `defmt` - Enable [`defmt::Format`] on certain types
//! - `direct-boot` - Use the direct boot image format
//! - `eh1` - Implement the traits defined in the `1.0.0-xxx` pre-releases of
//!   [embedded-hal], [embedded-hal-nb], and [embedded-io]
//! - `embassy` - Enable support for [embassy], a modern asynchronous embedded
//!   framework. One of `embassy-time-*` features must also be enabled when
//!   using this feature.
//! - `embassy-executor-interrupt` - Use the multicore-aware interrupt-mode
//!   embassy executor
//! - `embassy-executor-thread` - Use the multicore-aware thread-mode embassy
//!   executor
//! - `embassy-time-systick` - Enable the [embassy] time driver using the
//!   `SYSTIMER` peripheral
//! - `embassy-time-timg0` - Enable the [embassy] time driver using the `TIMG0`
//!   peripheral
//! - `log` - enable log output using the `log` crate
//! - `opsram-2m` - Use externally connected Octal PSRAM (2MB)
//! - `opsram-4m` - Use externally connected Octal PSRAM (4MB)
//! - `opsram-8m` - Use externally connected Octal PSRAM (8MB)
//! - `opsram-16m`- Use externally connected Octal PSRAM (16MB)
//! - `psram-2m` - Use externally connected PSRAM (2MB)
//! - `psram-4m` - Use externally connected PSRAM (4MB)
//! - `psram-8m` - Use externally connected PSRAM (8MB)
//! - `rt` - Runtime support
//! - `ufmt` - Implement the [`ufmt_write::uWrite`] trait for the UART driver
//! - `vectored` - Enable interrupt vectoring
//!
//! #### Default Features
//!
//! The `rt` and `vectored` features are enabled by default.
//!
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [embedded-io-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io-async
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-nb]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-nb
//! [embedded-io]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io
//! [embassy]: https://github.com/embassy-rs/embassy
//! [`ufmt_write::uWrite`]: https://docs.rs/ufmt-write/latest/ufmt_write/trait.uWrite.html
//! [`defmt::Format`]: https://docs.rs/defmt/0.3.5/defmt/trait.Format.html
//!
//! ### Supported Image Formats
//!
//! This HAL supports building multiple different application image formats. You
//! can read about each below.
//!
//! The ESP-IDF Bootloader format is used unless some other format is specified
//! via its feature.
//!
//! #### ESP-IDF Bootloader
//!
//! Use the second-stage bootloader from [ESP-IDF] and its associated
//! application image format. See the [App Image Format] documentation for more
//! information about this format.
//!
//! [ESP-IDF]: https://github.com/espressif/esp-idf
//! [App Image Format]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/app_image_format.html
//!
//! #### Direct Boot
//!
//! This device additionally supports direct-boot, which allows an application
//! to be executed directly from flash, without using the second-stage
//! bootloader. For more information please see the
//! [esp32c3-direct-boot-example] in the Espressif organization on GitHub.
//!
//! [esp32c3-direct-boot-example]: https://github.com/espressif/esp32c3-direct-boot-example

#![no_std]
#![cfg_attr(
    feature = "direct-boot",
    feature(asm_experimental_arch),
    feature(naked_functions)
)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

pub use esp_hal_common::*;

#[cfg(all(feature = "rt", feature = "direct-boot"))]
#[doc(hidden)]
#[no_mangle]
#[link_section = ".init"]
#[naked]
unsafe extern "C" fn init() {
    core::arch::asm!("call0 startup_direct_boot", options(noreturn));
}

#[cfg(all(feature = "rt", feature = "direct-boot"))]
#[doc(hidden)]
#[no_mangle]
pub unsafe fn startup_direct_boot() -> ! {
    // These symbols are from `memory.x`
    extern "C" {
        static mut _rtc_fast_bss_start: u32;
        static mut _rtc_fast_bss_end: u32;

        static mut _rtc_slow_bss_start: u32;
        static mut _rtc_slow_bss_end: u32;

        // Boundaries of the .rtc_fast.text section
        static mut _rtc_fast_text_start: u32;
        static mut _rtc_fast_text_end: u32;
        static mut _irtc_fast_text: u32;

        // Boundaries of the .rtc_fast.data section
        static mut _rtc_fast_data_start: u32;
        static mut _rtc_fast_data_end: u32;
        static mut _irtc_fast_data: u32;

        // Boundaries of the .rtc_slow.text section
        static mut _rtc_slow_text_start: u32;
        static mut _rtc_slow_text_end: u32;
        static mut _irtc_slow_text: u32;

        // Boundaries of the .rtc_slow.data section
        static mut _rtc_slow_data_start: u32;
        static mut _rtc_slow_data_end: u32;
        static mut _irtc_slow_data: u32;

        static mut _stack_end_cpu0: u32;
    }

    // set stack pointer to end of memory: no need to retain stack up to this point
    xtensa_lx::set_stack_pointer(&mut _stack_end_cpu0);

    // copy rtc data from flash to destinations
    r0::init_data(
        &mut _rtc_fast_data_start,
        &mut _rtc_fast_data_end,
        &_irtc_fast_data,
    );

    r0::init_data(
        &mut _rtc_fast_text_start,
        &mut _rtc_fast_text_end,
        &_irtc_fast_text,
    );

    r0::init_data(
        &mut _rtc_slow_data_start,
        &mut _rtc_slow_data_end,
        &_irtc_slow_data,
    );

    r0::init_data(
        &mut _rtc_slow_text_start,
        &mut _rtc_slow_text_end,
        &_irtc_slow_text,
    );

    // Initialize RTC RAM
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_slow_bss_start, &mut _rtc_slow_bss_end);

    // first of all copy rwtext
    extern "C" {
        // Boundaries of the .iram section
        static mut _srwtext: u32;
        static mut _erwtext: u32;
        static mut _irwtext: u32;
    }
    r0::init_data(&mut _srwtext, &mut _erwtext, &_irwtext);

    // do some configurations for compatability with the 2nd stage bootloader
    // this is a workaround and ideally we should deal with these settings in other
    // places
    (&*crate::peripherals::TIMG0::PTR)
        .int_ena_timers
        .modify(|_, w| w.t0_int_ena().set_bit().t1_int_ena().set_bit());
    (&*crate::peripherals::TIMG1::PTR)
        .int_ena_timers
        .modify(|_, w| w.t0_int_ena().set_bit().t1_int_ena().set_bit());

    (&*crate::peripherals::RTC_CNTL::PTR)
        .swd_wprotect
        .write(|w| w.bits(0x8f1d312a));
    (&*crate::peripherals::RTC_CNTL::PTR)
        .swd_conf
        .modify(|_, w| w.swd_disable().set_bit());
    (&*crate::peripherals::RTC_CNTL::PTR)
        .swd_wprotect
        .write(|w| w.bits(0));

    (&*crate::peripherals::SYSTEM::PTR)
        .sysclk_conf
        .modify(|_, w| w.soc_clk_sel().bits(1));

    esp_hal_common::xtensa_lx_rt::Reset();
}

#[cfg(feature = "rt")]
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
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE: u32 = 0x4000; // ESP32S3_INSTRUCTION_CACHE_16KB
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
#[cfg(feature = "rt")]
#[doc(hidden)]
#[no_mangle]
#[link_section = ".rwtext"]
pub unsafe extern "C" fn ESP32Reset() -> ! {
    configure_cpu_caches();

    // These symbols come from `memory.x`
    extern "C" {
        static mut _rtc_fast_bss_start: u32;
        static mut _rtc_fast_bss_end: u32;

        static mut _rtc_slow_bss_start: u32;
        static mut _rtc_slow_bss_end: u32;

        static mut _stack_end_cpu0: u32;
    }

    // set stack pointer to end of memory: no need to retain stack up to this point
    esp_hal_common::xtensa_lx::set_stack_pointer(&mut _stack_end_cpu0);

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    esp_hal_common::xtensa_lx_rt::zero_bss(&mut _rtc_slow_bss_start, &mut _rtc_slow_bss_end);

    // continue with default reset handler
    esp_hal_common::xtensa_lx_rt::Reset();
}

/// The ESP32 has a first stage bootloader that handles loading program data
/// into the right place therefore we skip loading it again.
#[doc(hidden)]
#[no_mangle]
#[rustfmt::skip]
pub extern "Rust" fn __init_data() -> bool {
    #[cfg(feature = "direct-boot")]
    let res = true;

    #[cfg(not(feature = "direct-boot"))]
    let res = false;

    res
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    use esp_hal_common::{
        peripherals::{RTC_CNTL, TIMG0, TIMG1},
        timer::Wdt,
    };

    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(RTC_CNTL::steal());
    rtc.rwdt.disable();

    Wdt::<TIMG0>::set_wdt_enabled(false);
    Wdt::<TIMG1>::set_wdt_enabled(false);
}
