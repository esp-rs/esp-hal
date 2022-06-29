#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp32s3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals, UART0},
    prelude::*,
    systimer::{Alarm, SystemTimer, Target},
    Cpu,
    Delay,
    RtcCntl,
    Serial,
    Timer,
};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut SERIAL: SpinLockMutex<RefCell<Option<Serial<UART0>>>> =
    SpinLockMutex::new(RefCell::new(None));
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

    let mut timer0 = Timer::new(peripherals.TIMG0, clocks.apb_clock);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let serial0 = Serial::new(peripherals.UART0).unwrap();

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    timer0.disable();
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
        (&SERIAL).lock(|data| (*data).replace(Some(serial0)));
        (&ALARM0).lock(|data| (*data).replace(Some(alarm0)));
        (&ALARM1).lock(|data| (*data).replace(Some(alarm1)));
        (&ALARM2).lock(|data| (*data).replace(Some(alarm2)));
    }

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET0,
        interrupt::CpuInterrupt::Interrupt0LevelPriority1,
    );

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET1,
        interrupt::CpuInterrupt::Interrupt19LevelPriority2,
    );

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET2,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    unsafe {
        xtensa_lx::interrupt::enable_mask(1 << 19 | 1 << 0 | 1 << 23);
    }

    loop {
        delay.delay_ms(500u32);
    }
}

#[no_mangle]
pub fn level1_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt lvl1 (alarm0)").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt0LevelPriority1,
    );

    unsafe {
        (&ALARM0).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}

#[no_mangle]
pub fn level2_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt lvl2 (alarm1)").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt19LevelPriority2,
    );

    unsafe {
        (&ALARM1).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}

#[no_mangle]
pub fn level3_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt lvl3 (alarm2)").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );

    unsafe {
        (&ALARM2).lock(|data| {
            let mut alarm = data.borrow_mut();
            let alarm = alarm.as_mut().unwrap();
            alarm.clear_interrupt();
        });
    }
}
