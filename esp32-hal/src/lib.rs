#![no_std]

pub use embedded_hal as ehal;
#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;
#[doc(inline)]
pub use esp_hal_common::{
    aes,
    analog::adc::implementation as adc,
    analog::dac::implementation as dac,
    clock,
    cpu_control::CpuControl,
    dma,
    dma::pdma,
    efuse,
    entry,
    gpio,
    i2c,
    i2s,
    interrupt,
    ledc,
    macros,
    mcpwm,
    pcnt,
    peripherals,
    prelude,
    pulse_control,
    sha,
    spi,
    system,
    timer,
    trapframe,
    uart,
    utils,
    xtensa_lx,
    xtensa_lx_rt,
    Cpu,
    Delay,
    PulseControl,
    Rng,
    Rtc,
    Rwdt,
    Uart,
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
    false
}
