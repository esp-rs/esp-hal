//! This shows how to output text via USB Serial/JTAG.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    UsbSerialJtag,
};
use esp_backtrace as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut delay = Delay::new(&clocks);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    loop {
        writeln!(UsbSerialJtag, "Hello world!").ok();
        delay.delay_ms(500u32);
    }
}
