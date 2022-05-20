#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{
    i2c,
    interrupt,
    pac,
    prelude,
    pulse_control,
    ram,
    spi,
    usb_serial_jtag,
    utils,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    RtcCntl,
    Serial,
    Timer,
    UsbSerialJtag,
};

pub use self::gpio::IO;

pub mod gpio;

#[no_mangle]
extern "C" fn DefaultHandler(_level: u32, _interrupt: pac::Interrupt) {}

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
    false
}

fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}
