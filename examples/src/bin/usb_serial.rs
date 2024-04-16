//! CDC-ACM serial port example using polling in a busy loop.
//!
//! This example should be built in release mode.

//% CHIPS: esp32s2 esp32s3

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    gpio::IO,
    otg_fs::{UsbBus, USB},
    peripherals::Peripherals,
    prelude::*,
};
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let usb = USB::new(peripherals.USB0, io.pins.gpio19, io.pins.gpio20);
    let usb_bus = UsbBus::new(usb, unsafe { &mut *addr_of_mut!(EP_MEMORY) });

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(USB_CLASS_CDC)
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
