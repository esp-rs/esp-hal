#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{gpio as hal_gpio, pac, prelude, Serial, Timer};

pub mod rtc_cntl;

pub mod gpio;

use riscv_rt::pre_init;
pub use rtc_cntl::RtcCntl;

extern "C" {
    // Boundaries of the .iram section
    static mut _siramdata: u32;
    static mut _eiramdata: u32;
    static mut _iramdata: u32;

    // Boundaries of the .bss section
    static mut _ebss: u32;
    static mut _sbss: u32;
}

#[pre_init]
unsafe fn init() {
    // no init data for iram when using normal boot - the boot loader does it for us
    #[cfg(not(feature = "normalboot"))]
    r0::init_data(&mut _siramdata, &mut _eiramdata, &_iramdata);
}

#[export_name = "_mp_hook"]
pub extern "Rust" fn mp_hook() -> bool {
    #[cfg(not(feature = "normalboot"))]
    return true;

    // no init data when using normal boot - but we need to zero out BSS
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
    }

    false
}
