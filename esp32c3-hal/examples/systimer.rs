//! This shows how to use the SYSTIMER peripheral including interrupts.
//! It's an additional timer besides the TIMG peripherals.

#![no_std]
#![no_main]

use core::cell::RefCell;

use bare_metal::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    systimer::{Alarm, SystemTimer, Target},
    timer::TimerGroup,
    Cpu,
    RtcCntl,
};
use panic_halt as _;
use riscv_rt::entry;

static mut ALARM0: Mutex<RefCell<Option<Alarm<Target, 0>>>> = Mutex::new(RefCell::new(None));
static mut ALARM1: Mutex<RefCell<Option<Alarm<Target, 1>>>> = Mutex::new(RefCell::new(None));
static mut ALARM2: Mutex<RefCell<Option<Alarm<Target, 2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_global_enable(false);
    wdt0.disable();
    wdt1.disable();

    let syst = SystemTimer::new(peripherals.SYSTIMER);

    esp_println::println!("SYSTIMER Current value = {}", SystemTimer::now());

    let alarm0 = syst.alarm0;
    alarm0.set_target(40_000_000);
    alarm0.enable_interrupt();

    let alarm1 = syst.alarm1;
    alarm1.set_target(41_111_111);
    alarm1.enable_interrupt();

    let alarm2 = syst.alarm2;
    alarm2.set_target(42_222_222 * 2);
    alarm2.enable_interrupt();

    interrupt::enable(
        pac::Interrupt::SYSTIMER_TARGET0,
        interrupt::Priority::Priority1,
    )
    .unwrap();
    interrupt::enable(
        pac::Interrupt::SYSTIMER_TARGET1,
        interrupt::Priority::Priority1,
    )
    .unwrap();
    interrupt::enable(
        pac::Interrupt::SYSTIMER_TARGET2,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    riscv::interrupt::free(|_cs| unsafe {
        ALARM0.get_mut().replace(Some(alarm0));
        ALARM1.get_mut().replace(Some(alarm1));
        ALARM2.get_mut().replace(Some(alarm2));
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn SYSTIMER_TARGET0() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("Interrupt 1 = {}", SystemTimer::now());

        let mut alarm = ALARM0.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt1);
        alarm.clear_interrupt();
    });
}

#[interrupt]
fn SYSTIMER_TARGET1() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("Interrupt 2 = {}", SystemTimer::now());

        let mut alarm = ALARM1.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt2);
        alarm.clear_interrupt();
    });
}

#[interrupt]
fn SYSTIMER_TARGET2() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("Interrupt 3 = {}", SystemTimer::now());

        let mut alarm = ALARM2.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt3);
        alarm.clear_interrupt();
    });
}
