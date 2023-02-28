#![no_std]

pub use embedded_hal as ehal;
#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;
pub use esp_hal_common::{
    aes,
    analog::adc::implementation as adc,
    clock,
    dma,
    dma::gdma,
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
    riscv,
    sha,
    spi,
    system,
    systimer,
    timer,
    trapframe,
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
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

extern "C" {
    // Boundaries of the .iram section
    static mut _srwtext: u32;
    static mut _erwtext: u32;
    static mut _irwtext: u32;

    // Boundaries of the .bss section
    static mut _ebss: u32;
    static mut _sbss: u32;

    // Boundaries of the rtc .bss section
    static mut _rtc_fast_bss_start: u32;
    static mut _rtc_fast_bss_end: u32;

    // Boundaries of the .rtc_fast.text section
    static mut _srtc_fast_text: u32;
    static mut _ertc_fast_text: u32;
    static mut _irtc_fast_text: u32;

    // Boundaries of the .rtc_fast.data section
    static mut _rtc_fast_data_start: u32;
    static mut _rtc_fast_data_end: u32;
    static mut _irtc_fast_data: u32;
}

#[cfg(feature = "direct-boot")]
#[doc(hidden)]
#[esp_hal_common::esp_riscv_rt::pre_init]
unsafe fn init() {
    r0::init_data(&mut _srwtext, &mut _erwtext, &_irwtext);

    r0::init_data(
        &mut _rtc_fast_data_start,
        &mut _rtc_fast_data_end,
        &_irtc_fast_data,
    );

    r0::init_data(&mut _srtc_fast_text, &mut _ertc_fast_text, &_irtc_fast_text);

    esp_hal_common::disable_apm_filter();
}

#[allow(unreachable_code)]
#[export_name = "_mp_hook"]
#[doc(hidden)]
pub fn mp_hook() -> bool {
    unsafe {
        r0::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    }

    #[cfg(feature = "direct-boot")]
    return true;

    // no init data when using normal boot - but we need to zero out BSS
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
    }

    false
}

#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: peripherals::Interrupt) {}
