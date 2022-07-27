//! This shows how to write text to serial0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    RtcCntl,
    Serial,
};
use nb::block;
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut serial0 = Serial::new(peripherals.UART0);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_global_enable(false);
    wdt0.disable();
    wdt1.disable();

    timer0.start(1u64.secs());

    loop {
        writeln!(serial0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}
