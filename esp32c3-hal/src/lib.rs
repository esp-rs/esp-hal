#![no_std]

pub use embedded_hal as ehal;
pub use esp_hal_common::{pac, prelude, Serial, Timer};

pub mod gpio;
pub mod rtc_cntl;

pub use gpio::IO;
pub use rtc_cntl::RtcCntl;
