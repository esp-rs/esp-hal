#![no_std]

// By default, we don't want probe-rs to interfere with test timings by halting
// cores and polling RTT. The tests don't produce output most of the time
// anyway. The only cases where output can be interesting are: during
// development, and when a test fails. In these cases, you can enable
// the `defmt` feature to get the output.

use esp_hal as _;

#[cfg(not(feature = "defmt"))]
#[defmt::global_logger]
struct Logger;

#[cfg(not(feature = "defmt"))]
unsafe impl defmt::Logger for Logger {
    fn acquire() {}
    unsafe fn flush() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
}

#[cfg(feature = "defmt")]
use defmt_rtt as _;
// Make sure esp_backtrace is not removed.
use esp_backtrace as _;

#[macro_export]
macro_rules! i2c_pins {
    ($peripherals:expr) => {{
        // Order: (SDA, SCL)
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($peripherals.pins.gpio2, $peripherals.pins.gpio3)
            } else if #[cfg(esp32)] {
                ($peripherals.pins.gpio32, $peripherals.pins.gpio33)
            } else if #[cfg(esp32c6)] {
                ($peripherals.pins.gpio6, $peripherals.pins.gpio7)
            } else if #[cfg(esp32h2)] {
                ($peripherals.pins.gpio12, $peripherals.pins.gpio22)
            } else if #[cfg(esp32c2)] {
                ($peripherals.pins.gpio18, $peripherals.pins.gpio9)
            } else {
                ($peripherals.pins.gpio4, $peripherals.pins.gpio5)
            }
        }
    }};
}

#[macro_export]
macro_rules! common_test_pins {
    ($peripherals:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($peripherals.pins.gpio9, $peripherals.pins.gpio10)
            } else if #[cfg(esp32)] {
                ($peripherals.pins.gpio26, $peripherals.pins.gpio27)
            } else {
                ($peripherals.pins.gpio2, $peripherals.pins.gpio3)
            }
        }
    }};
}

// A GPIO that's not connected to anything. We use the BOOT pin for this, but
// beware: it has a pullup.
#[macro_export]
macro_rules! unconnected_pin {
    ($peripherals:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                $peripherals.pins.gpio0
            } else if #[cfg(esp32c6)] {
                $peripherals.pins.gpio9
            } else if #[cfg(esp32h2)] {
                $peripherals.pins.gpio9
            } else if #[cfg(esp32c2)] {
                $peripherals.pins.gpio8
            } else {
                $peripherals.pins.gpio9
            }
        }
    }};
}
