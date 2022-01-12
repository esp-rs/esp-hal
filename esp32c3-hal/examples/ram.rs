#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    pac::{self, UART0},
    prelude::*,
    RtcCntl,
    Serial,
    Timer,
};
use nb::block;
use panic_halt as _;
use riscv_rt::{entry};

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();

    timer0.start(10_000_000u64);

    writeln!(serial0, "Function located at {:p}", say_hello as *const ()).unwrap();

    loop {
        say_hello(&mut serial0);
        block!(timer0.wait()).unwrap();
    }
}

#[link_section = ".iram"]
#[inline(never)]
fn say_hello(serial0: &mut Serial<UART0>) {
    writeln!(serial0, "Hello world!").unwrap();
}
