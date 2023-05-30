//! This shows debug-assist
//!
//! Uncomment the functionality you want to test

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32h2_hal::{
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

    // 320k of RAM, 16k reserved for cache
    // RAM starts at 0x40800000 + 16k = 0x40804000
    // RAM length = 320k - 16k = 304k = 0x4c000
    // RAM end = 0x40804000 + 0x4c000 = 0x40850000

    // Uncomment the functionality you want to test

    // Monitor the SP so as to prevent stack overflow or erroneous push/pop. When
    // the SP exceeds the minimum or maximum threshold, the module will record the
    // PC pointer and generate an interrup
    da.enable_sp_monitor(0x40850002, 0x40860000);
    // 0x40850000 - 0x1200 = 0x4084ee00

    // Monitor reads/writes performed by the CPU over data bus and peripheral bus
    // in a certain address space, i.e., memory region. Whenever the bus reads or
    // writes in the specified address space, an interrupt will be triggered
    // da.enable_region0_monitor(0x4084ee00, 0x4084ef00, true, true);
    // da.enable_region1_monitor(0x4084ee00, 0x4084ef00, true, true);
    // 0x4084ee00 + 0x100 = 0x4084ef00

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
