//! This shows how to configure UART
//! You can short the TX and RX pin and see it reads what was written.
//! Additionally you can connect a logic analyzer to TX and see how the changes
//! of the configuration change the output signal.
//!
//! The following wiring is assumed:
//! - TX => GPIO4
//! - RX => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, uart::Uart};
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut serial1 = Uart::new(
        peripherals.UART1,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
    )
    .unwrap();

    let delay = Delay::new();

    println!("Start");
    loop {
        serial1.write_byte(0x42).ok();
        let read = block!(serial1.read_byte());

        match read {
            Ok(read) => println!("Read 0x{:02x}", read),
            Err(err) => println!("Error {:?}", err),
        }

        delay.delay_millis(250);
    }
}
