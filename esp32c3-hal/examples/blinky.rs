#![no_std]
#![no_main]

use esp32c3_hal::{pac, prelude::*, RtcCntl, Timer, IO};
use nb::block;
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();

    let mut led = io.pins.gpio2.into_push_pull_output();
    led.set_high().unwrap();

    timer0.start(10_000_000u64);

    loop {
        led.toggle().unwrap();
        block!(timer0.wait()).unwrap();
    }
}
