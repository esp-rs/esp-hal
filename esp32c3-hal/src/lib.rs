#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{pac, prelude, Delay, Serial, Timer};

pub mod gpio;
pub mod rtc_cntl;

pub use self::{gpio::IO, rtc_cntl::RtcCntl};
