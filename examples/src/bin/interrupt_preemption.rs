//! Interrupt Preemption
//!
//! An example of how an interrupt can be preempted by another with higher
//! priority. Should show higher-numbered software interrupts happening during
//! the handling of lower-numbered ones.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2
//% FEATURES: interrupt-preemption

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::{SoftwareInterrupt, SoftwareInterruptControl},
};

static SWINT: Mutex<RefCell<Option<SoftwareInterruptControl>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let sw_int = system.software_interrupt_control;

    critical_section::with(|cs| SWINT.borrow_ref_mut(cs).replace(sw_int));

    interrupt::enable(Interrupt::FROM_CPU_INTR0, Priority::Priority1).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR1, Priority::Priority2).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR2, Priority::Priority2).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR3, Priority::Priority15).unwrap();

    // Raise mid priority interrupt.
    //
    // The handler raises one interrupt at lower priority, one at same and one at
    // higher. We expect to see the higher priority served immeiately before
    // exiting the handler Once the handler is exited we expect to see same
    // priority and low priority interrupts served in that order.
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .raise(SoftwareInterrupt::SoftwareInterrupt1);
    });

    loop {}
}

#[interrupt]
fn FROM_CPU_INTR0() {
    esp_println::println!("SW interrupt0");
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt0);
    });
}

#[interrupt]
fn FROM_CPU_INTR1() {
    esp_println::println!("SW interrupt1 entry");
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt1);
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .raise(SoftwareInterrupt::SoftwareInterrupt2); // raise interrupt at same priority
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .raise(SoftwareInterrupt::SoftwareInterrupt3); // raise interrupt at higher priority
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .raise(SoftwareInterrupt::SoftwareInterrupt0); // raise interrupt at
                                                           // lower priority
    });
    esp_println::println!("SW interrupt1 exit");
}

#[interrupt]
fn FROM_CPU_INTR2() {
    esp_println::println!("SW interrupt2");
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt2);
    });
}

#[interrupt]
fn FROM_CPU_INTR3() {
    esp_println::println!("SW interrupt3");
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt3);
    });
}
