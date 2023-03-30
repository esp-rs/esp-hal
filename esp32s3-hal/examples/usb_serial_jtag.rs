//! This shows how to output text via USB Serial/JTAG.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.
//! PLEASE NOTE: USB Serial input on ESP32-S3 is not currently working
//! See https://github.com/esp-rs/esp-hal/issues/269 for details

#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use critical_section::Mutex;
use esp32s3_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals, USB_DEVICE},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    UsbSerialJtag,
};
use esp_backtrace as _;
use nb::block;

static USB_SERIAL: Mutex<RefCell<Option<UsbSerialJtag<USB_DEVICE>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let mut usb_serial =
        UsbSerialJtag::new(peripherals.USB_DEVICE, &mut system.peripheral_clock_control);

    usb_serial.listen_rx_packet_recv_interrupt();

    timer0.start(1u64.secs());

    critical_section::with(|cs| USB_SERIAL.borrow_ref_mut(cs).replace(usb_serial));

    interrupt::enable(
        peripherals::Interrupt::USB_DEVICE,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    loop {
        critical_section::with(|cs| {
            writeln!(
                USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
                "Hello world!"
            )
            .ok();
        });

        block!(timer0.wait()).unwrap();
    }
}

#[interrupt]
fn USB_DEVICE() {
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
