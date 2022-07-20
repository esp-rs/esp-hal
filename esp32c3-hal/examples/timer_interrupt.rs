#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use bare_metal::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals, TIMG0, TIMG1, UART0},
    prelude::*,
    timer::{Timer0, TimerGroup},
    Cpu,
    RtcCntl,
    Serial,
};
use esp_hal_common::Timer;
use panic_halt as _;
use riscv_rt::entry;

static mut SERIAL: Mutex<RefCell<Option<Serial<UART0>>>> = Mutex::new(RefCell::new(None));
static mut TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static mut TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timer_group1.timer0;
    let mut wdt1 = timer_group1.wdt;

    let serial0 = Serial::new(peripherals.UART0);

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    wdt0.disable();
    wdt1.disable();

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG0_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt1,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt1,
        interrupt::Priority::Priority1,
    );

    timer0.start(500u64.millis());
    timer0.listen();

    interrupt::enable(
        Cpu::ProCpu,
        pac::Interrupt::TG1_T0_LEVEL,
        interrupt::CpuInterrupt::Interrupt11,
    );
    interrupt::set_kind(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt11,
        interrupt::InterruptKind::Level,
    );
    interrupt::set_priority(
        Cpu::ProCpu,
        interrupt::CpuInterrupt::Interrupt11,
        interrupt::Priority::Priority1,
    );

    timer1.start(1u64.secs());
    timer1.listen();

    riscv::interrupt::free(|_cs| unsafe {
        SERIAL.get_mut().replace(Some(serial0));
        TIMER0.get_mut().replace(Some(timer0));
        TIMER1.get_mut().replace(Some(timer1));
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

        let mut timer0 = TIMER0.borrow(*cs).borrow_mut();
        let timer0 = timer0.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt1);
        timer0.clear_interrupt();

        timer0.start(500u64.millis());
    });
}

#[no_mangle]
pub fn interrupt11() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "Interrupt 11").ok();

        let mut timer1 = TIMER1.borrow(*cs).borrow_mut();
        let timer1 = timer1.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt11);
        timer1.clear_interrupt();

        timer1.start(1u64.secs());
    });
}
