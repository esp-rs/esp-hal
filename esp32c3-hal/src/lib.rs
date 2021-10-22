#![no_std]

pub use embedded_hal as ehal;
pub use esp32c3 as pac;
pub use esp_hal_common::{prelude, Serial, Timer};

pub mod rtc_cntl;

pub use rtc_cntl::RtcCntl;
