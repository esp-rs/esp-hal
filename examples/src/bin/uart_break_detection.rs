//! Blocking UART break detection example.
//!
//! The following wiring is assumed:
//! - TX => GPIO17
//! - RX => GPIO16

//% CHIPS: esp32
//% FEATURES: embassy embassy-generic-timers esp-hal/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    entry,
    gpio::{Level, Output},
    uart::{Config as UartConfig, DataBits, StopBits, Uart},
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .baudrate(19200)
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::Stop1)
        .rx_fifo_full_threshold(1);
    let mut uart = Uart::new(
        peripherals.UART1,
        uart_config,
        peripherals.GPIO16, // RX
        peripherals.GPIO17, // TX
    )
    .expect("Failed to initialize UART");

    loop {
        uart.wait_for_break();
        esp_println::print!("\nBREAK");
        
        while let Ok(byte) = uart.read_byte() {
            esp_println::print!(" {:02X}", byte);
        }
    }
}