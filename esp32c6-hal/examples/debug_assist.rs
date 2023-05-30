//! This shows debug-assist
//!
//! Uncomment the functionality you want to test

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32c6_hal::{
    assist_debug::DebugAssist,
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

static DA: Mutex<RefCell<Option<DebugAssist>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut da = DebugAssist::new(
        peripherals.ASSIST_DEBUG,
        &mut system.peripheral_clock_control,
    );

    // uncomment the functionality you want to test

    // 512K of RAM, 32K reserved for cache
    // RAM starts at 0x40800000 + 32K = 0x40808000
    // RAM length = 512K - 32K = 480K = 0x78000
    // RAM end = 0x40808000 + 0x78000 = 0x40880000

    // da.enable_sp_monitor(0x4087ee00, 0x40880000);
    // 0x40880000 - 0x4087ee00 = 0x1200
    // da.enable_region0_monitor(0x4087ee00, 0x4087ef00, true, true);
    // 0x4087ef00 - 0x4087ee00 = 0x100
    da.enable_region1_monitor(0x4087ee00, 0x4087ef00, true, true);
    // 0x4087ef00 - 0x4087ee00 = 0x100

    critical_section::with(|cs| DA.borrow_ref_mut(cs).replace(da));

    interrupt::enable(
        peripherals::Interrupt::ASSIST_DEBUG,
        interrupt::Priority::Priority3,
    )
    .unwrap();

    unsafe {
        riscv::interrupt::enable();
    }

    eat_up_stack(0);

    loop {}
}

#[allow(unconditional_recursion)]
fn eat_up_stack(v: u32) {
    println!("Iteration {v}");
    eat_up_stack(v + 1);
}

#[interrupt]
fn ASSIST_DEBUG() {
    critical_section::with(|cs| {
        println!("\n\nDEBUG_ASSIST interrupt");
        let mut da = DA.borrow_ref_mut(cs);
        let da = da.as_mut().unwrap();

        if da.is_sp_monitor_interrupt_set() {
            println!("SP MONITOR TRIGGERED");
            da.clear_sp_monitor_interrupt();
            let pc = da.get_sp_monitor_pc();
            println!("PC = 0x{:x}", pc);
        }

        if da.is_region0_monitor_interrupt_set() {
            println!("REGION0 MONITOR TRIGGERED");
            da.clear_region0_monitor_interrupt();
            let pc = da.get_region_monitor_pc();
            println!("PC = 0x{:x}", pc);
        }

        if da.is_region1_monitor_interrupt_set() {
            println!("REGION1 MONITOR TRIGGERED");
            da.clear_region1_monitor_interrupt();
            let pc = da.get_region_monitor_pc();
            println!("PC = 0x{:x}", pc);
        }

        loop {}
    });
}
