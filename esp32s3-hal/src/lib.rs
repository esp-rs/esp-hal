#![no_std]
#![cfg_attr(feature = "direct-boot", feature(naked_functions))]
#![cfg_attr(feature = "direct-boot", feature(asm_experimental_arch))]

pub use embedded_hal as ehal;
#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;
#[doc(inline)]
pub use esp_hal_common::{
    analog::adc::implementation as adc,
    clock,
    cpu_control::CpuControl,
    dma::{self, gdma},
    efuse,
    gpio,
    i2c,
    i2s,
    interrupt,
    ledc,
    macros,
    mcpwm,
    otg_fs,
    peripherals,
    prelude,
    pulse_control,
    sha,
    spi,
    system,
    systimer,
    timer,
    twai,
    uart,
    utils,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    Rtc,
    Rwdt,
    Uart,
    UsbSerialJtag,
};

pub use self::gpio::IO;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SensExt};
}

#[no_mangle]
extern "C" fn EspDefaultHandler(_level: u32, _interrupt: peripherals::Interrupt) {}

#[no_mangle]
extern "C" fn DefaultHandler() {}

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
    xtensa_lx_rt::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    xtensa_lx_rt::zero_bss(&mut _rtc_slow_bss_start, &mut _rtc_slow_bss_end);

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

    xtensa_lx_rt::Reset();
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
    xtensa_lx::set_stack_pointer(&mut _stack_end_cpu0);

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    xtensa_lx_rt::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    xtensa_lx_rt::zero_bss(&mut _rtc_slow_bss_start, &mut _rtc_slow_bss_end);

    // continue with default reset handler
    xtensa_lx_rt::Reset();
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
