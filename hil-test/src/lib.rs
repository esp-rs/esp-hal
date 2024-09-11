#![no_std]

// By default, we don't want probe-rs to interfere with test timings by halting
// cores and polling RTT. The tests don't produce output most of the time
// anyway. The only cases where output can be interesting are: during
// development, and when a test fails. In these cases, you can enable
// the `defmt` feature to get the output.

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
    ($io:expr) => {{
        // Order: (SDA, SCL)
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($io.gpio2, $io.gpio3)
            } else if #[cfg(esp32)] {
                ($io.gpio32, $io.gpio33)
            } else if #[cfg(esp32c6)] {
                ($io.gpio6, $io.gpio7)
            } else if #[cfg(esp32h2)] {
                ($io.gpio12, $io.gpio22)
            } else if #[cfg(esp32c2)] {
                ($io.gpio18, $io.gpio9)
            } else {
                ($io.gpio4, $io.gpio5)
            }
        }
    }};
}

#[macro_export]
macro_rules! common_test_pins {
    ($io:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($io.gpio9, $io.gpio10)
            }
            else if #[cfg(esp32)] {
                ($io.gpio26, $io.gpio27)
            }
            else {
                ($io.gpio2, $io.gpio3)
            }
        }
    }};
}
