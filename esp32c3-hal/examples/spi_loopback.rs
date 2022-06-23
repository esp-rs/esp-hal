//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO6
//! MISO    GPIO2
//! MOSI    GPIO7
//! CS      GPIO10
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

use esp32c3_hal::{
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
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio6;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio7;
    let cs = io.pins.gpio10;

    let mut spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        Some(miso),
        Some(cs),
        100u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        writeln!(serial0, "{:x?}", data).ok();

        delay.delay_ms(250u32);
    }
}
