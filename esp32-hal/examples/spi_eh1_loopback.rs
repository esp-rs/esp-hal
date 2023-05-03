//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO19
//! MISO    GPIO25
//! MOSI    GPIO23
//! CS      GPIO22
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

#![no_std]
#![no_main]

use embedded_hal_1::spi::SpiBus;
use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;

    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio19;
    let miso = io.pins.gpio25;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio22;

    let mut spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        1000u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);
    println!("=== SPI example with embedded-hal-1 traits ===");

    loop {
        // --- Symmetric transfer (Read as much as we write) ---
        print!("Starting symmetric transfer...");
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        SpiBus::transfer(&mut spi, &mut read[..], &write[..]).expect("Symmetric transfer failed");
        assert_eq!(write, read);
        println!(" SUCCESS");
        delay.delay_ms(250u32);

        // --- Asymmetric transfer (Read more than we write) ---
        print!("Starting asymetric transfer (read > write)...");
        let mut read: [u8; 4] = [0x00; 4];

        SpiBus::transfer(&mut spi, &mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
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

        SpiBus::transfer(&mut spi, &mut read[..], &write[..]).expect("Huge transfer failed");
        assert_eq!(write, read);
        println!(" SUCCESS");
        delay.delay_ms(250u32);

        // --- Symmetric transfer with huge buffer in-place (No additional allocation
        // needed) ---
        print!("Starting huge transfer (in-place)...");
        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }

        SpiBus::transfer_in_place(&mut spi, &mut write[..]).expect("Huge transfer failed");
        for byte in 0..write.len() {
            assert_eq!(write[byte], byte as u8);
        }
        println!(" SUCCESS");
        delay.delay_ms(250u32);
    }
}
