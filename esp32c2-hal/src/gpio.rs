//! General Purpose I/Os
//!
//! To get access to the pins, you first need to convert them into a HAL
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `IO::new`.
//!
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio5.into_push_pull_output();
//! ```
use esp_hal_common::gpio::{types::*, *};

// FIXME: add alternate functions
gpio! {
    Function1,
    SingleCore,

    Gpio0:  ( gpio0,  0,  gpio[0], IO, RTC, Bank0, None),
    Gpio1:  ( gpio1,  1,  gpio[1], IO, RTC, Bank0, None),
    Gpio2:  ( gpio2,  2,  gpio[2], IO, RTC, Bank0, None),
    Gpio3:  ( gpio3,  3,  gpio[3], IO, RTC, Bank0, None),
    Gpio4:  ( gpio4,  4,  gpio[4], IO, RTC, Bank0, None),
    Gpio5:  ( gpio5,  5,  gpio[5], IO, RTC, Bank0, None),
    Gpio6:  ( gpio6,  6,  gpio[6], IO,   0, Bank0, None),
    Gpio7:  ( gpio7,  7,  gpio[7], IO,   0, Bank0, None),
    Gpio8:  ( gpio8,  8,  gpio[8], IO,   0, Bank0, None),
    Gpio9:  ( gpio9,  9,  gpio[9], IO,   0, Bank0, None),
    Gpio10: (gpio10, 10, gpio[10], IO,   0, Bank0, None),
    Gpio18: (gpio18, 18, gpio[18], IO,   0, Bank0, None),
    Gpio19: (gpio19, 19, gpio[19], IO,   0, Bank0, None),
    Gpio20: (gpio20, 20, gpio[20], IO,   0, Bank0, None),
}
