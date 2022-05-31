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

use esp32c3_hal::{gpio::IO, pac::Peripherals, prelude::*, Delay, RtcCntl, Serial, Timer};
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

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

    peripherals
        .SYSTEM
        .sysclk_conf
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(1) });

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio6;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio7;
    let cs = io.pins.gpio10;

    let mut spi = esp32c3_hal::spi::Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        100u32.kHz(),
        embedded_hal::spi::MODE_0,
        &mut peripherals.SYSTEM,
    );

    let mut delay = Delay::new(peripherals.SYSTIMER);

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        writeln!(serial0, "{:x?}", data).ok();

        delay.delay_ms(250u32);
    }
}
