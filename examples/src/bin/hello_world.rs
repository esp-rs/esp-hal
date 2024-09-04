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
use esp_hal::{delay::Delay, gpio::Io, prelude::*, uart::Uart};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Default pins for Uart/Serial communication
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio1, io.pins.gpio3);
        } else if #[cfg(feature = "esp32c2")] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio20, io.pins.gpio19);
        } else if #[cfg(feature = "esp32c3")] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio21, io.pins.gpio20);
        } else if #[cfg(feature = "esp32c6")] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio16, io.pins.gpio17);
        } else if #[cfg(feature = "esp32h2")] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio24, io.pins.gpio23);
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let (mut tx_pin, mut rx_pin) = (io.pins.gpio43, io.pins.gpio44);
        }
    }

    let mut uart0 =
        Uart::new_with_default_pins(peripherals.UART0, &mut tx_pin, &mut rx_pin).unwrap();

    loop {
        writeln!(uart0, "Hello world!").unwrap();
        delay.delay(1.secs());
    }
}
