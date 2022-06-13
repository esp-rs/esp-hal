//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO36
//! MISO    GPIO37
//! MOSI    GPIO35
//! CS      GPIO34
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

use esp32s2_hal::{
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
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    timer0.disable();
    rtc_cntl.set_wdt_global_enable(false);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio36;
    let miso = io.pins.gpio37;
    let mosi = io.pins.gpio35;
    let cs = io.pins.gpio34;

    let mut spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
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
