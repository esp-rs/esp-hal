#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{pac::Peripherals, prelude::*, Delay, RtcCntl, Timer, UsbSerialJtag};
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let mut delay = Delay::new(peripherals.SYSTIMER);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);

    // Disable watchdog timers
    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    loop {
        writeln!(UsbSerialJtag, "Hello world!").ok();
        delay.delay_ms(500u32);
    }
}
