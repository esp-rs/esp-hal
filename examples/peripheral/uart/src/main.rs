//! Example of sending a software break signal from a UART in
//! Blocking mode.
//!
//! The following wiring is assumed:
//! - TX => GPIO1 (ESP32), GPIO20 (ESP32-C2), GPIO21 (ESP32-C3), GPIO16 (ESP32-C6), GPIO24
//!   (ESP32-H2), GPIO43 (ESP32-S2/S3)

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let tx_pin = peripherals.GPIO1;
        } else if #[cfg(feature = "esp32c2")] {
            let tx_pin = peripherals.GPIO20;
        } else if #[cfg(feature = "esp32c3")] {
            let tx_pin = peripherals.GPIO21;
        } else if #[cfg(feature = "esp32c6")] {
            let tx_pin = peripherals.GPIO16;
        } else if #[cfg(feature = "esp32h2")] {
            let tx_pin = peripherals.GPIO24;
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let tx_pin = peripherals.GPIO43;
        }
    }

    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_tx(tx_pin);
    let delay = Delay::new();

    loop {
        // Send a break signal for 20 bits
        uart.send_break(20);
        delay.delay_millis(500);
    }
}
