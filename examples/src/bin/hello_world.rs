//! This shows how to write text to UART0.
//!
//! You can see the output with `espflash` if you provide the `--monitor`
//! option.
//!
//! Depending on the chip, you will need to ensure that you are connected to
//! the UART USB port, and not the USB-SERIAL-JTAG port. If you want to test
//! printing over USB-SERIAL-JTAG, try the usb_serial_jtag example instead.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, uart::Uart};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    // Default pins for Uart/Serial communication
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio1, peripherals.pins.gpio3);
        } else if #[cfg(feature = "esp32c2")] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio20, peripherals.pins.gpio19);
        } else if #[cfg(feature = "esp32c3")] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio21, peripherals.pins.gpio20);
        } else if #[cfg(feature = "esp32c6")] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio16, peripherals.pins.gpio17);
        } else if #[cfg(feature = "esp32h2")] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio24, peripherals.pins.gpio23);
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let (mut tx_pin, mut rx_pin) = (peripherals.pins.gpio43, peripherals.pins.gpio44);
        }
    }

    let mut uart0 = Uart::new(peripherals.UART0, &mut rx_pin, &mut tx_pin).unwrap();

    loop {
        writeln!(uart0, "Hello world!").unwrap();
        delay.delay(1.secs());
    }
}
