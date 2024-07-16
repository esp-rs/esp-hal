//! This shows how to output text via USB Serial/JTAG.
//!
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    usb_serial_jtag::UsbSerialJtag,
    Blocking,
};

static USB_SERIAL: Mutex<RefCell<Option<UsbSerialJtag<'static, Blocking>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
    usb_serial.set_interrupt_handler(usb_device);
    usb_serial.listen_rx_packet_recv_interrupt();

    critical_section::with(|cs| USB_SERIAL.borrow_ref_mut(cs).replace(usb_serial));

    loop {
        critical_section::with(|cs| {
            writeln!(
                USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                "Hello world!"
            )
            .ok();
        });

        delay.delay(1.secs());
    }
}

#[handler]
fn usb_device() {
    critical_section::with(|cs| {
        let mut usb_serial = USB_SERIAL.borrow_ref_mut(cs);
        let usb_serial = usb_serial.as_mut().unwrap();

        writeln!(usb_serial, "USB serial interrupt").unwrap();

        while let nb::Result::Ok(c) = usb_serial.read_byte() {
            writeln!(usb_serial, "Read byte: {:02x}", c).unwrap();
        }

        usb_serial.reset_rx_packet_recv_interrupt();
    });
}
