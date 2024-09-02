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

use cfg_if::cfg_if;
// Define type aliases and macros for conditional pin assignments
cfg_if! {
    if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
        // Type alias for I2C pin configuration on S2 and S3 using GPIOs 2 and 3
        pub type I2C_SCL_Pin = esp_hal::gpio::Gpio2;
        pub type I2C_SDA_Pin = esp_hal::gpio::Gpio3;

        // SPI pin aliases for avoiding conflicts with I2C on S2 and S3
        pub type MFU_Pin1 = esp_hal::gpio::Gpio9;
        pub type MFU_Pin2 = esp_hal::gpio::Gpio10;
    } else if #[cfg(any(feature = "esp32c6"))] {
        // Type alias for I2C pin configuration on C6 using GPIOs 6 and 7
        pub type I2C_SCL_Pin = esp_hal::gpio::Gpio7;
        pub type I2C_SDA_Pin = esp_hal::gpio::Gpio6;

        /// SPI_MISO pin alias for avoiding conflicts with I2C on C6
        pub type SPI_MISO_Pin = esp_hal::gpio::Gpio4;
        /// SPI_MOSI pin alias for avoiding conflicts with I2C on C6
        pub type SPI_MOSI_Pin = esp_hal::gpio::Gpio5;

        /// Most
        pub type MFU_Pin1 = esp_hal::gpio::Gpio2;
        pub type MFU_Pin2 = esp_hal::gpio::Gpio3;
    } else if #[cfg(any(feature = "esp32h2"))] {
        // Type alias for I2C pin configuration on H2 using GPIOs 6 and 7
        pub type I2C_SCL_Pin = esp_hal::gpio::Gpio6;
        pub type I2C_SDA_Pin = esp_hal::gpio::Gpio7;

        pub type MFU_Pin1 = esp_hal::gpio::Gpio2;
        pub type MFU_Pin2 = esp_hal::gpio::Gpio3;
        // No special pin assignments needed for SPI on H2
    } else {
        // Default case for other chips
        pub type I2C_SCL_Pin = esp_hal::gpio::Gpio4;
        pub type I2C_SDA_Pin = esp_hal::gpio::Gpio5;
        
        pub type MFU_Pin1 = esp_hal::gpio::Gpio2;
        pub type MFU_Pin2 = esp_hal::gpio::Gpio3;
    }
}
