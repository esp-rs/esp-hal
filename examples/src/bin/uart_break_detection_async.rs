//! Async UART break detection example.
//!
//! The following wiring is assumed:
//! - TX => GPIO17
//! - RX => GPIO16

//% CHIPS: esp32
//% FEATURES: embassy embassy-generic-timers esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::uart::{Config as UartConfig, DataBits, StopBits, Uart};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .baudrate(19200)
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::Stop1)
        .rx_fifo_full_threshold(1); // interrupt every time a byte is received
    let mut uart = Uart::new(
        peripherals.UART1,
        uart_config,
        peripherals.GPIO16, // RX
        peripherals.GPIO17, // TX
    )
    .expect("Failed to initialize UART")
    .into_async();

    loop {
        uart.wait_for_break_async().await;
        esp_println::print!("\nBREAK");

        let mut buf = [0u8; 11];
        let len = uart.read_async(&mut buf).await.unwrap();
        for byte in buf.iter().take(len) {
            esp_println::print!(" {:02X}", byte);
        }
    }
}