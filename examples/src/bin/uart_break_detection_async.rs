//! Async UART break detection example.
//!
//! The following wiring is assumed:
//! - RX => GPIO16

//% CHIPS: esp32
//% FEATURES: embassy esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_rx(peripherals.GPIO16)
        .into_async();

    loop {
        uart.wait_for_break_async().await;
        esp_println::print!("\nBREAK");
    }
}
