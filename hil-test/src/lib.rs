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

// Define type aliases and macros for conditional pin assignments
#[macro_export]
macro_rules! i2c_pins {
    ($io:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                // For ESP32-S2 and ESP32-S3, use GPIO 2(SDA) and 3(SCL) for I2C
                ($io.pins.gpio2, $io.pins.gpio3)
            } else if #[cfg(esp32c6)] {
                // For ESP32-C6, use GPIO 6(SDA) and 7(SCL) for I2C
                ($io.pins.gpio6, $io.pins.gpio7)
            } else if #[cfg(esp32h2)] {
                // For ESP32-H2, use GPIO 4(SDA) and 22(SCL) for I2C
                ($io.pins.gpio4, $io.pins.gpio22)
            } else if #[cfg(esp32c2)] {
                // For ESP32-C2, use GPIO 18(SDA) and 19(SCL) for I2C
                ($io.pins.gpio18, $io.pins.gpio19)
            } else {
                // ESP32 and ESP32C3, use GPIO 4(SDA) and 7(SCL)
                ($io.pins.gpio4, $io.pins.gpio7)
            }
        }
    }};
}

#[macro_export]
macro_rules! common_test_pins {
    ($io:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(not(any(esp32s2, esp32s3)))] {
                // For other targets, use GPIO 2 and 3 for common tests
                ($io.pins.gpio2, $io.pins.gpio3)
            } else if #[cfg(any(esp32s2, esp32s3))] {
                // For ESP32-S2 and ESP32-S3, use GPIO 9 and 10 for common tests
                ($io.pins.gpio9, $io.pins.gpio10)
            }
        }
    }};
}
