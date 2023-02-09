#![no_std]

pub use embedded_hal as ehal;
#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;
#[doc(inline)]
pub use esp_hal_common::{
    analog::adc::implementation as adc,
    clock,
    dma::{self, gdma},
    efuse,
    entry,
    gpio,
    i2c,
    interrupt,
    ledc,
    macros,
    peripherals,
    prelude,
    riscv,
    sha,
    spi,
    system,
    systimer,
    timer,
    trapframe,
    uart,
    Cpu,
    Delay,
    Rng,
    Rtc,
    Rwdt,
    Uart,
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
}

#[cfg(feature = "direct-boot")]
#[doc(hidden)]
#[esp_hal_common::esp_riscv_rt::pre_init]
unsafe fn init() {
    r0::init_data(&mut _srwtext, &mut _erwtext, &_irwtext);
}

#[allow(unreachable_code)]
#[export_name = "_mp_hook"]
#[doc(hidden)]
pub fn mp_hook() -> bool {
    if cfg!(feature = "direct-boot") {
        true
    } else {
        unsafe {
            r0::zero_bss(&mut _sbss, &mut _ebss);
        }

        false
    }
}

#[no_mangle]
extern "C" fn EspDefaultHandler(_interrupt: peripherals::Interrupt) {}
