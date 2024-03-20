#![no_main]
#![no_std]

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2

use core::{arch::asm, cell::RefCell};

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    interrupt::{self, CpuInterrupt, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::{SoftwareInterrupt, SoftwareInterruptControl},
};

static SWINT: Mutex<RefCell<Option<SoftwareInterruptControl>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c6", feature = "esp32h2"))] {
            let cpu_intr = &peripherals.INTPRI;
        } else {
            let cpu_intr = &peripherals.SYSTEM;
        }
    }
    let sw0_trigger_addr = cpu_intr.cpu_intr_from_cpu_0() as *const _ as u32;

    let system = peripherals.SYSTEM.split();
    let sw_int = system.software_interrupt_control;

    critical_section::with(|cs| SWINT.borrow_ref_mut(cs).replace(sw_int));
    interrupt::enable_direct(
        Interrupt::FROM_CPU_INTR0,
        Priority::Priority3,
        CpuInterrupt::Interrupt20,
    )
    .unwrap();
    unsafe {
        asm!(
            "
        csrrwi x0, 0x7e0, 1 #what to count, for cycles write 1 for instructions write 2
        csrrwi x0, 0x7e1, 0 #disable counter
        csrrwi x0, 0x7e2, 0 #reset counter
        "
        );
    }

    // interrupt is raised from assembly for max timer granularity.
    unsafe {
        asm!(
            "
        li {bit}, 1                   # Flip flag (bit 0)
        csrrwi x0, 0x7e1, 1           # enable timer
        sw {bit}, 0({addr})           # trigger FROM_CPU_INTR0
        ",
        options(nostack),
        addr = in(reg) sw0_trigger_addr,
        bit = out(reg) _,
        )
    }
    esp_println::println!("Returned");

    loop {}
}

#[no_mangle]
fn cpu_int_20_handler() {
    unsafe { asm!("csrrwi x0, 0x7e1, 0 #disable timer") }
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt0);
    });

    let mut perf_counter: u32 = 0;
    unsafe {
        asm!(
            "
            csrr {x}, 0x7e2
            ",
            options(nostack),
            x = inout(reg) perf_counter,
        )
    };
    esp_println::println!("Performance counter:{}", perf_counter);
}
