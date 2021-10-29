#![no_std]
#![no_main]

use esp32c3_hal::{pac, prelude::*, RtcCntl, Serial, Timer};
use nb::block;
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let rtccntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();
    let mut timer0 = Timer::new(peripherals.TIMG0);

    rtccntl.set_super_wdt_enable(false);
    rtccntl.set_wdt_enable(false);
    timer0.disable();

    loop {
        let byte = block!(serial0.read()).unwrap();
        block!(serial0.write(byte)).unwrap();
    }
}
