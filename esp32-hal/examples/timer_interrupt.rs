#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp32_hal::{
    pac::{self, Peripherals, TIMG0, TIMG1, UART0},
    prelude::*,
    RtcCntl,
    Serial,
    Timer,
};
use esp_hal_common::{interrupt, Cpu};
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut SERIAL: SpinLockMutex<RefCell<Option<Serial<UART0>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER0: SpinLockMutex<RefCell<Option<Timer<TIMG0>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER1: SpinLockMutex<RefCell<Option<Timer<TIMG1>>>> =
    SpinLockMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the TIMG watchdog timer.
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);
    let serial0 = Serial::new(peripherals.UART0).unwrap();
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    timer0.disable();
    timer1.disable();
    rtc_cntl.set_wdt_global_enable(false);

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG0_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt20LevelPriority2,
    );
    timer0.start(50_000_000u64);
    timer0.listen();

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG1_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );
    timer1.start(100_000_000u64);
    timer1.listen();

    unsafe {
        (&SERIAL).lock(|data| (*data).replace(Some(serial0)));
        (&TIMER0).lock(|data| (*data).replace(Some(timer0)));
        (&TIMER1).lock(|data| (*data).replace(Some(timer1)));
    }

    unsafe {
        xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 20,
        );
        xtensa_lx::interrupt::enable_mask(
            1 << 23,
        );
    }

    loop {}
}

#[no_mangle]
pub fn level2_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt Level 2").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt20LevelPriority2,
    );

    unsafe {
        (&TIMER0).lock(|data| {
            let mut timer0 = data.borrow_mut();
            let timer0 = timer0.as_mut().unwrap();
            timer0.clear_interrupt();
            timer0.start(50_000_000u64);
        });
    }
}

#[no_mangle]
pub fn level3_interrupt() {
    unsafe {
        (&SERIAL).lock(|data| {
            let mut serial = data.borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Interrupt Level 3").ok();
        });
    }

    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );

    unsafe {
        (&TIMER1).lock(|data| {
            let mut timer1 = data.borrow_mut();
            let timer1 = timer1.as_mut().unwrap();
            timer1.clear_interrupt();
            timer1.start(100_000_000u64);
        });
    }
}
