//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS 1    GPIO5
//! CS 2    GPIO6
//! CS 3    GPIO7
//!
//! Folowing pins are used for ESP32:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS 1    GPIO5
//! CS 2    GPIO13
//! CS 3    GPIO14
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embedded-hal

#![no_std]
#![no_main]

use core::cell::RefCell;

use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::RefCellDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Io},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    system::SystemControl,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;

    let spi_bus = Spi::new(peripherals.SPI2, 1000.kHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sclk),
        Some(mosi),
        Some(miso),
        gpio::NO_PIN,
    );
    let spi_bus = RefCell::new(spi_bus);
    let mut spi_device_1 =
        RefCellDevice::new_no_delay(&spi_bus, io.pins.gpio5.into_push_pull_output());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let mut spi_device_2 =
                RefCellDevice::new_no_delay(&spi_bus, io.pins.gpio13.into_push_pull_output());
            let mut spi_device_3 =
                RefCellDevice::new_no_delay(&spi_bus, io.pins.gpio14.into_push_pull_output());
        } else {
            let mut spi_device_2 =
                RefCellDevice::new_no_delay(&spi_bus, io.pins.gpio6.into_push_pull_output());
            let mut spi_device_3 =
                RefCellDevice::new_no_delay(&spi_bus, io.pins.gpio7.into_push_pull_output());
        }
    }

    let delay = Delay::new(&clocks);
    println!("=== SPI example with embedded-hal-1 traits ===");

    loop {
        // --- Symmetric transfer (Read as much as we write) ---
        print!("Starting symmetric transfer...");
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        spi_device_1.transfer(&mut read[..], &write[..]).unwrap();
        assert_eq!(write, read);
        spi_device_2.transfer(&mut read[..], &write[..]).unwrap();
        spi_device_3.transfer(&mut read[..], &write[..]).unwrap();
        println!(" SUCCESS");
        delay.delay_millis(250);

        // --- Asymmetric transfer (Read more than we write) ---
        print!("Starting asymetric transfer (read > write)...");
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
        println!(" SUCCESS");
        delay.delay_millis(250);

        // --- Symmetric transfer with huge buffer ---
        // Only your RAM is the limit!
        print!("Starting huge transfer...");
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
        println!(" SUCCESS");
        delay.delay_millis(250);

        // --- Symmetric transfer with huge buffer in-place (No additional allocation
        // needed) ---
        print!("Starting huge transfer (in-place)...");
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
        println!(" SUCCESS");
        delay.delay_millis(250);
    }
}
