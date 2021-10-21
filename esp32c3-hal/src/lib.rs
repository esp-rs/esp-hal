#![no_std]

pub use embedded_hal as ehal;
pub use esp32c3 as pac;

pub mod prelude;
pub mod rtc_cntl;
pub mod serial;
pub mod timer;

pub use rtc_cntl::RtcCntl;
pub use serial::Serial;
pub use timer::Timer;
