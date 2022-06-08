#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use bare_metal::Mutex;
use esp32c3_hal::{
    pac::{self, Peripherals, UART0},
    prelude::*,
    RtcCntl,
    Serial,
    Timer,
};
use esp_hal_common::{
    interrupt::{self},
    systimer::{Alarm, SystemTimer, Target},
    Cpu,
};
use panic_halt as _;
use riscv_rt::entry;

static mut SERIAL: Mutex<RefCell<Option<Serial<UART0>>>> = Mutex::new(RefCell::new(None));
static mut ALARM0: Mutex<RefCell<Option<Alarm<Target, 0>>>> = Mutex::new(RefCell::new(None));
static mut ALARM1: Mutex<RefCell<Option<Alarm<Target, 1>>>> = Mutex::new(RefCell::new(None));
static mut ALARM2: Mutex<RefCell<Option<Alarm<Target, 2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    writeln!(serial0, "SYSTIMER Demo start!").ok();

    let mut syst = SystemTimer::new(peripherals.SYSTIMER);

    writeln!(serial0, "SYSTIMER Current value = {}", syst.now()).ok();

    let alarm0 = syst.alarm0().unwrap();
    alarm0.set_target(40_000_000);
    alarm0.enable_interrupt();

    let alarm1 = syst.alarm1().unwrap();
    alarm1.set_target(41_111_111);
    alarm1.enable_interrupt();

    let alarm2 = syst.alarm2().unwrap();
    alarm2.set_target(42_222_222 * 2);
    alarm2.enable_interrupt();

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET0,
        interrupt::CpuInterrupt::Interrupt1,
    );
    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET1,
        interrupt::CpuInterrupt::Interrupt2,
    );
    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::SYSTIMER_TARGET2,
        interrupt::CpuInterrupt::Interrupt3,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt2,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt3,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::Priority::Priority1,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt2,
        interrupt::Priority::Priority1,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt3,
        interrupt::Priority::Priority1,
    );

    riscv::interrupt::free(|_cs| unsafe {
        SERIAL.get_mut().replace(Some(serial0));
        ALARM0.get_mut().replace(Some(alarm0));
        ALARM1.get_mut().replace(Some(alarm1));
        ALARM2.get_mut().replace(Some(alarm2));
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[no_mangle]
pub fn interrupt1() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "Interrupt 1").ok();

        let mut alarm = ALARM0.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt1);
        alarm.clear_interrupt();
    });
}

#[no_mangle]
pub fn interrupt2() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "Interrupt 2").ok();

        let mut alarm = ALARM1.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt2);
        alarm.clear_interrupt();
    });
}

#[no_mangle]
pub fn interrupt3() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "Interrupt 3").ok();

        let mut alarm = ALARM2.borrow(*cs).borrow_mut();
        let alarm = alarm.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt3);
        alarm.clear_interrupt();
    });
}
