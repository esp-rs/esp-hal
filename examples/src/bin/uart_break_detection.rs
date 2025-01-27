//! Blocking UART break detection example.
//!
//! The following wiring is assumed:
//! - TX => GPIO17
//! - RX => GPIO16

//% CHIPS: esp32

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    uart::{Config as UartConfig, DataBits, Parity, RxConfig, StopBits, Uart},
};

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
        .with_rx(RxConfig::default().with_fifo_full_threshold(1));
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_rx(peripherals.GPIO16)
        .with_tx(peripherals.GPIO17);

    loop {
        uart.wait_for_break();
        esp_println::print!("\nBREAK");
    }
}
