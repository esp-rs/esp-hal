//! This shows how to use the TIMG peripheral interrupts.
//! There is TIMG0 and TIMG1 each of them containing a general purpose timer and
//! a watchdog timer.

#![no_std]
#![no_main]

use core::cell::RefCell;

use bare_metal::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals, TIMG0, TIMG1},
    prelude::*,
    timer::{Timer, Timer0, TimerGroup},
    Rtc,
};
use panic_halt as _;
use riscv_rt::entry;

static mut TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static mut TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timer_group1.timer0;
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    interrupt::enable(pac::Interrupt::TG0_T0_LEVEL, interrupt::Priority::Priority1).unwrap();
    timer0.start(500u64.millis());
    timer0.listen();

    interrupt::enable(pac::Interrupt::TG1_T0_LEVEL, interrupt::Priority::Priority1).unwrap();
    timer1.start(1u64.secs());
    timer1.listen();

    riscv::interrupt::free(|_cs| unsafe {
        TIMER0.get_mut().replace(Some(timer0));
        TIMER1.get_mut().replace(Some(timer1));
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn TG0_T0_LEVEL() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("Interrupt 1");

        let mut timer0 = TIMER0.borrow(*cs).borrow_mut();
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
        timer0.start(500u64.millis());
    });
}

#[interrupt]
fn TG1_T0_LEVEL() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("Interrupt 11");

        let mut timer1 = TIMER1.borrow(*cs).borrow_mut();
        let timer1 = timer1.as_mut().unwrap();

        timer1.clear_interrupt();
        timer1.start(1u64.secs());
    });
}
