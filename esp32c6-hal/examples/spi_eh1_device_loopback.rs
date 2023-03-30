//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO6
//! MISO    GPIO2
//! MOSI    GPIO7
//! CS 1    GPIO3
//! CS 2    GPIO4
//! CS 3    GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

#![no_std]
#![no_main]

use embedded_hal_1::spi::SpiDevice;
use esp32c6_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiBusController, SpiMode},
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio6;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio7;

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
    let mut spi_device_1 = spi_controller.add_device(io.pins.gpio3);
    let mut spi_device_2 = spi_controller.add_device(io.pins.gpio4);
    let mut spi_device_3 = spi_controller.add_device(io.pins.gpio5);

    let mut delay = Delay::new(&clocks);
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
        delay.delay_ms(250u32);

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
        delay.delay_ms(250u32);

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
        delay.delay_ms(250u32);

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
        delay.delay_ms(250u32);
    }
}
