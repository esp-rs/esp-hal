//! This shows how to use the SYSTIMER peripheral including interrupts.
//! It's an additional timer besides the TIMG peripherals.

#![no_std]
#![no_main]

use core::cell::RefCell;

use esp32s3_hal::{
    clock::ClockControl,
    interrupt,
    interrupt::Priority,
    pac::{self, Peripherals},
    prelude::*,
    systimer::{Alarm, SystemTimer, Target},
    timer::TimerGroup,
    Delay,
    RtcCntl,
};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut ALARM0: SpinLockMutex<RefCell<Option<Alarm<Target, 0>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut ALARM1: SpinLockMutex<RefCell<Option<Alarm<Target, 1>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut ALARM2: SpinLockMutex<RefCell<Option<Alarm<Target, 2>>>> =
    SpinLockMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc_cntl.set_wdt_global_enable(false);

    let syst = SystemTimer::new(peripherals.SYSTIMER);

    let alarm0 = syst.alarm0;
    alarm0.set_target(40_000_000);
    alarm0.enable_interrupt();

    let alarm1 = syst.alarm1;
    alarm1.set_target(41_111_111);
    alarm1.enable_interrupt();

    let alarm2 = syst.alarm2;
    alarm2.set_target(42_222_222 * 2);
    alarm2.enable_interrupt();

    unsafe {
        (&ALARM0).lock(|data| (*data).replace(Some(alarm0)));
        (&ALARM1).lock(|data| (*data).replace(Some(alarm1)));
        (&ALARM2).lock(|data| (*data).replace(Some(alarm2)));
    }

    interrupt::enable(pac::Interrupt::SYSTIMER_TARGET0, Priority::Priority1).unwrap();
    interrupt::enable(pac::Interrupt::SYSTIMER_TARGET1, Priority::Priority2).unwrap();
    interrupt::enable(pac::Interrupt::SYSTIMER_TARGET2, Priority::Priority2).unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
        delay.delay_ms(500u32);
    }
}

#[interrupt]
fn SYSTIMER_TARGET0() {
    esp_println::println!("Interrupt lvl1 (alarm0)");

    unsafe {
        (&ALARM0).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}

#[interrupt]
fn SYSTIMER_TARGET1() {
    esp_println::println!("Interrupt lvl2 (alarm1)");

    unsafe {
        (&ALARM1).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}

#[interrupt]
fn SYSTIMER_TARGET2() {
    esp_println::println!("Interrupt lvl2 (alarm2)");

    unsafe {
        (&ALARM2).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}
