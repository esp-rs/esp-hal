#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{
    clock,
    efuse,
    gpio as gpio_types,
    i2c::{self, I2C},
    interrupt,
    pac,
    prelude,
    pulse_control,
    ram,
    serial,
    spi,
    systimer,
    utils,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    RtcCntl,
    Serial,
    Timer,
};

pub use self::gpio::IO;

pub mod adc;
pub mod dac;
pub mod gpio;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SensExt};
}

#[no_mangle]
extern "C" fn DefaultHandler(_level: u32, _interrupt: pac::Interrupt) {}

/// Function initializes ESP32 specific memories (RTC slow and fast) and
/// then calls original Reset function
///
/// ENTRY point is defined in memory.x
/// *Note: the pre_init function is called in the original reset handler
/// after the initializations done in this function*
#[cfg(feature = "rt")]
#[doc(hidden)]
#[no_mangle]
pub unsafe extern "C" fn ESP32Reset() -> ! {
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
    int_enable as u8
        | ((nmi_enable as u8) << 1)
        | (int_enable as u8) << 2
        | ((nmi_enable as u8) << 3)
}
