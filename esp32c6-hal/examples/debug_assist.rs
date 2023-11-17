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
};
use esp_backtrace as _;
use esp_println::println;

static DA: Mutex<RefCell<Option<DebugAssist>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut da = DebugAssist::new(peripherals.ASSIST_DEBUG);

    extern "C" {
        // top of stack
        static mut _stack_start: u32;
        // bottom of stack
        static mut _stack_end: u32;
    }

    let stack_top = unsafe { &mut _stack_start } as *mut _ as u32;
    let stack_bottom = unsafe { &mut _stack_end } as *mut _ as u32;

    // uncomment the functionality you want to test

    da.enable_sp_monitor(stack_bottom + 4096, stack_top);
    // da.enable_region0_monitor(stack_bottom, stack_bottom + 4096, true, true);
    // da.enable_region1_monitor(stack_bottom, stack_bottom + 4096, true, true);

    critical_section::with(|cs| DA.borrow_ref_mut(cs).replace(da));

    interrupt::enable(
        peripherals::Interrupt::ASSIST_DEBUG,
        interrupt::Priority::Priority3,
    )
    .unwrap();

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
