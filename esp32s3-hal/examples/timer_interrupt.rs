#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp32s3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals, TIMG0, TIMG1, UART0},
    prelude::*,
    timer::{Timer0, Timer1, TimerGroup},
    Cpu,
    RtcCntl,
    Serial,
};
use esp_hal_common::Timer;
use panic_halt as _;
use xtensa_lx::mutex::{Mutex, SpinLockMutex};
use xtensa_lx_rt::entry;

static mut SERIAL: SpinLockMutex<RefCell<Option<Serial<UART0>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER00: SpinLockMutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER01: SpinLockMutex<RefCell<Option<Timer<Timer1<TIMG0>>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER10: SpinLockMutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> =
    SpinLockMutex::new(RefCell::new(None));
static mut TIMER11: SpinLockMutex<RefCell<Option<Timer<Timer1<TIMG1>>>>> =
    SpinLockMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the TIMG watchdog timer.
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer00 = timer_group0.timer0;
    let mut timer01 = timer_group0.timer1;
    let mut wdt0 = timer_group0.wdt;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer10 = timer_group1.timer0;
    let mut timer11 = timer_group1.timer1;
    let mut wdt1 = timer_group1.wdt;

    let serial0 = Serial::new(peripherals.UART0);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt0.disable();
    wdt1.disable();
    rtc_cntl.set_wdt_global_enable(false);

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG0_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt20LevelPriority2,
    );
    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG0_T1_LEVEL,
        interrupt::CpuInterrupt::Interrupt20LevelPriority2,
    );
    timer00.start(500u64.millis());
    timer00.listen();
    timer01.start(2500u64.millis());
    timer01.listen();

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG1_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );
    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG1_T1_LEVEL,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );
    timer10.start(1u64.secs());
    timer10.listen();
    timer11.start(3u64.secs());
    timer11.listen();

    unsafe {
        (&SERIAL).lock(|data| (*data).replace(Some(serial0)));
        (&TIMER00).lock(|data| (*data).replace(Some(timer00)));
        (&TIMER01).lock(|data| (*data).replace(Some(timer01)));
        (&TIMER10).lock(|data| (*data).replace(Some(timer10)));
        (&TIMER11).lock(|data| (*data).replace(Some(timer11)));
    }

    unsafe {
        xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(1 << 20);
        xtensa_lx::interrupt::enable_mask(1 << 23);
    }

    loop {}
}

#[no_mangle]
pub fn level2_interrupt() {
    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt20LevelPriority2,
    );

    unsafe {
        (&TIMER00).lock(|data| {
            let mut timer = data.borrow_mut();
            let timer = timer.as_mut().unwrap();

            if timer.is_interrupt_set() {
                timer.clear_interrupt();
                timer.start(500u64.millis());

                (&SERIAL).lock(|data| {
                    let mut serial = data.borrow_mut();
                    let serial = serial.as_mut().unwrap();
                    writeln!(serial, "Interrupt Level 2 - Timer0").ok();
                });
            }
        });

        (&TIMER01).lock(|data| {
            let mut timer = data.borrow_mut();
            let timer = timer.as_mut().unwrap();

            if timer.is_interrupt_set() {
                timer.clear_interrupt();
                timer.start(2500u64.millis());

                (&SERIAL).lock(|data| {
                    let mut serial = data.borrow_mut();
                    let serial = serial.as_mut().unwrap();
                    writeln!(serial, "Interrupt Level 2 - Timer1").ok();
                });
            }
        });
    }
}

#[no_mangle]
pub fn level3_interrupt() {
    interrupt::clear(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt23LevelPriority3,
    );

    unsafe {
        (&TIMER10).lock(|data| {
            let mut timer = data.borrow_mut();
            let timer = timer.as_mut().unwrap();

            if timer.is_interrupt_set() {
                timer.clear_interrupt();
                timer.start(1u64.secs());

                (&SERIAL).lock(|data| {
                    let mut serial = data.borrow_mut();
                    let serial = serial.as_mut().unwrap();
                    writeln!(serial, "Interrupt Level 3 - Timer0").ok();
                });
            }
        });

        (&TIMER11).lock(|data| {
            let mut timer = data.borrow_mut();
            let timer = timer.as_mut().unwrap();

            if timer.is_interrupt_set() {
                timer.clear_interrupt();
                timer.start(3u64.secs());

                (&SERIAL).lock(|data| {
                    let mut serial = data.borrow_mut();
                    let serial = serial.as_mut().unwrap();
                    writeln!(serial, "Interrupt Level 3 - Timer1").ok();
                });
            }
        });
    }
}
