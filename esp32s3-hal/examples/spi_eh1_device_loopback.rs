//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO12
//! MISO    GPIO11
//! MOSI    GPIO13
//! CS 1    GPIO4
//! CS 2    GPIO5
//! CS 3    GPIO6
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s3_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    spi::{Spi, SpiBusController, SpiMode},
    timer::TimerGroup,
    Delay,
    Rtc,
    Serial,
};
use panic_halt as _;
use xtensa_lx_rt::entry;

use embedded_hal_1::spi::blocking::SpiDevice;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Serial::new(peripherals.UART0);

    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio12;
    let miso = io.pins.gpio11;
    let mosi = io.pins.gpio13;

    let spi_controller = SpiBusController::from_spi(Spi::new_no_cs(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        1000u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    ));
    let mut spi_device_1 = spi_controller.add_device(io.pins.gpio4);
    let mut spi_device_2 = spi_controller.add_device(io.pins.gpio5);
    let mut spi_device_3 = spi_controller.add_device(io.pins.gpio6);

    let mut delay = Delay::new(&clocks);
    writeln!(serial0, "=== SPI example with embedded-hal-1 traits ===").unwrap();

    loop {
        // --- Symmetric transfer (Read as much as we write) ---
        write!(serial0, "Starting symmetric transfer...").unwrap();
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        spi_device_1.transfer(&mut read[..], &write[..]).unwrap();
        assert_eq!(write, read);
        spi_device_2.transfer(&mut read[..], &write[..]).unwrap();
        spi_device_3.transfer(&mut read[..], &write[..]).unwrap();
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);

        // --- Asymmetric transfer (Read more than we write) ---
        write!(serial0, "Starting asymetric transfer (read > write)...").unwrap();
        let mut read: [u8; 4] = [0x00; 4];

        spi_device_1
            .transfer(&mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
        spi_device_2
            .transfer(&mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        spi_device_3
            .transfer(&mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);

        // --- Symmetric transfer with huge buffer ---
        // Only your RAM is the limit!
        write!(serial0, "Starting huge transfer...").unwrap();
        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }
        let mut read = [0x00u8; 4096];

        spi_device_1
            .transfer(&mut read[..], &write[..])
            .expect("Huge transfer failed");
        assert_eq!(write, read);
        spi_device_2
            .transfer(&mut read[..], &write[..])
            .expect("Huge transfer failed");
        spi_device_3
            .transfer(&mut read[..], &write[..])
            .expect("Huge transfer failed");
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);

        // --- Symmetric transfer with huge buffer in-place (No additional allocation
        // needed) ---
        write!(serial0, "Starting huge transfer (in-place)...").unwrap();
        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }

        spi_device_1
            .transfer_in_place(&mut write[..])
            .expect("Huge transfer failed");
        for byte in 0..write.len() {
            assert_eq!(write[byte], byte as u8);
        }
        spi_device_2
            .transfer_in_place(&mut write[..])
            .expect("Huge transfer failed");
        spi_device_3
            .transfer_in_place(&mut write[..])
            .expect("Huge transfer failed");
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);
    }
}
