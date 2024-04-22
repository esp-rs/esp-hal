//! Interrupt Preemption
//!
//! An example of how an interrupt can be preempted by another with higher
//! priority. Should show higher-numbered software interrupts happening during
//! the handling of lower-numbered ones.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    peripherals::Peripherals,
    prelude::*,
    system::{SoftwareInterrupt, SystemControl},
};

static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));
static SWINT1: Mutex<RefCell<Option<SoftwareInterrupt<1>>>> = Mutex::new(RefCell::new(None));
static SWINT2: Mutex<RefCell<Option<SoftwareInterrupt<2>>>> = Mutex::new(RefCell::new(None));
static SWINT3: Mutex<RefCell<Option<SoftwareInterrupt<3>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let mut sw_int = system.software_interrupt_control;

    critical_section::with(|cs| {
        sw_int
            .software_interrupt0
            .set_interrupt_handler(swint0_handler);
        SWINT0
            .borrow_ref_mut(cs)
            .replace(sw_int.software_interrupt0);

        sw_int
            .software_interrupt1
            .set_interrupt_handler(swint1_handler);
        SWINT1
            .borrow_ref_mut(cs)
            .replace(sw_int.software_interrupt1);

        sw_int
            .software_interrupt2
            .set_interrupt_handler(swint2_handler);
        SWINT2
            .borrow_ref_mut(cs)
            .replace(sw_int.software_interrupt2);

        sw_int
            .software_interrupt3
            .set_interrupt_handler(swint3_handler);
        SWINT3
            .borrow_ref_mut(cs)
            .replace(sw_int.software_interrupt3);
    });

    // Raise mid priority interrupt.
    //
    // The handler raises one interrupt at lower priority, one at same and one at
    // higher. We expect to see the higher priority served immeiately before
    // exiting the handler Once the handler is exited we expect to see same
    // priority and low priority interrupts served in that order.
    critical_section::with(|cs| {
        SWINT1.borrow_ref_mut(cs).as_mut().unwrap().raise();
    });

    loop {}
}

#[handler(priority = esp_hal::interrupt::Priority::Priority1)]
fn swint0_handler() {
    esp_println::println!("SW interrupt0");
    critical_section::with(|cs| {
        SWINT0.borrow_ref_mut(cs).as_mut().unwrap().reset();
    });
}

#[handler(priority = esp_hal::interrupt::Priority::Priority2)]
fn swint1_handler() {
    esp_println::println!("SW interrupt1 entry");
    critical_section::with(|cs| {
        SWINT1.borrow_ref_mut(cs).as_mut().unwrap().reset();
        SWINT2.borrow_ref_mut(cs).as_mut().unwrap().raise(); // raise interrupt at same priority
        SWINT3.borrow_ref_mut(cs).as_mut().unwrap().raise(); // raise interrupt at higher priority
        SWINT0.borrow_ref_mut(cs).as_mut().unwrap().raise(); // raise interrupt at lower priority
    });
    esp_println::println!("SW interrupt1 exit");
}

#[handler(priority = esp_hal::interrupt::Priority::Priority2)]
fn swint2_handler() {
    esp_println::println!("SW interrupt2");
    critical_section::with(|cs| {
        SWINT2.borrow_ref_mut(cs).as_mut().unwrap().reset();
    });
}

#[handler(priority = esp_hal::interrupt::Priority::Priority3)]
fn swint3_handler() {
    esp_println::println!("SW interrupt3");
    critical_section::with(|cs| {
        SWINT3.borrow_ref_mut(cs).as_mut().unwrap().reset();
    });
}
