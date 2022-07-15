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

use core::fmt::Write;

use embedded_hal_1::spi::blocking::{
    SpiBus, SpiBusWrite, SpiBusRead, SpiBusFlush,
};
use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    Delay,
    RtcCntl,
    Serial,
    Timer,
};
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0, clocks.apb_clock);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    timer0.disable();
    rtc_cntl.set_wdt_global_enable(false);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio19;
    let miso = io.pins.gpio25;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio22;

    let mut spi = Spi::new(
        peripherals.SPI2,
        sclk,
        Some(mosi),
        Some(miso),
        Some(cs),
        1000u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);
    writeln!(serial0, "=== SPI example with embedded-hal-1 traits ===").unwrap();

    loop {
        write!(serial0, "Starting symmetric transfer...").unwrap();
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        SpiBus::transfer(&mut spi, &mut read[..], &write[..]).expect("Symmetric transfer failed");
        assert_eq!(write, read);
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);


        write!(serial0, "Starting assymetric transfer (read > write)...").unwrap();
        let mut read: [u8; 4] = [0x00; 4];

        SpiBus::transfer(&mut spi, &mut read[0..2], &write[..]).expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);


        // Only your RAM is the limit!
        write!(serial0, "Starting huge transfer...").unwrap();
        let mut write = [0x55u8; 256];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }
        let mut read = [0x00u8; 256];

        SpiBus::transfer(&mut spi, &mut read[..], &write[..]).expect("Huge transfer failed");
        //SpiBus::transfer_in_place(&mut spi, &mut write[..]).expect("Huge transfer failed");
        assert_eq!(write, read);
        writeln!(serial0, " SUCCESS").unwrap();
        delay.delay_ms(250u32);
    }
}
