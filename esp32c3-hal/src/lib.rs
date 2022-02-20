#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{pac, prelude, Delay, Serial, Timer};
#[cfg(not(feature = "normalboot"))]
use riscv_rt::pre_init;

pub mod gpio;
pub mod i2c;
pub mod rtc_cntl;

pub use esp_hal_common::{interrupt, ram, Cpu};

pub use self::{gpio::IO, i2c::I2C, rtc_cntl::RtcCntl};

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

#[cfg(not(feature = "normalboot"))]
#[pre_init]
#[cfg(not(feature = "normalboot"))]
#[doc(hidden)]
unsafe fn init() {
    r0::init_data(&mut _srwtext, &mut _erwtext, &_irwtext);

    r0::init_data(
        &mut _rtc_fast_data_start,
        &mut _rtc_fast_data_end,
        &_irtc_fast_data,
    );

    r0::init_data(&mut _srtc_fast_text, &mut _ertc_fast_text, &_irtc_fast_text);
}

#[allow(unreachable_code)]
#[export_name = "_mp_hook"]
#[doc(hidden)]
pub fn mp_hook() -> bool {
    unsafe {
        r0::zero_bss(&mut _rtc_fast_bss_start, &mut _rtc_fast_bss_end);
    }

    #[cfg(not(feature = "normalboot"))]
    return true;

    // no init data when using normal boot - but we need to zero out BSS
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
    }

    false
}

fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}
