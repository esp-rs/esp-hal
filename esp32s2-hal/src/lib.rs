#![no_std]

pub use embedded_hal as ehal;
pub use esp32s2 as pac;
pub use esp_hal_common::{prelude, Serial, Timer};

#[no_mangle]
extern "C" fn DefaultHandler(_level: u32, _interrupt: pac::Interrupt) {}
