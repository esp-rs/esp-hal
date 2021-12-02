#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{gpio as hal_gpio, pac, prelude, Serial, Timer};

pub mod rtc_cntl;

pub mod gpio;

pub use rtc_cntl::RtcCntl;
