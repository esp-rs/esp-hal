//! This shows debug-assist

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32c2_hal::{
    assist_debug::DebugAssist,
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
};
use esp_backtrace as _;
use esp_println::println;

static DA: Mutex<RefCell<Option<DebugAssist>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut da = DebugAssist::new(
        peripherals.ASSIST_DEBUG,
        &mut system.peripheral_clock_control,
    );

    da.enable_sp_monitor(0x3fccee00, 0x3fcd0000);

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

        loop {}
    });
}
