//! Software Interrupts
//!
//! An example of how software interrupts can be raised and reset
//! Should rotate through all of the available interrupts printing their number
//! when raised.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::{SoftwareInterrupt, SoftwareInterruptControl},
    Delay,
};

static SWINT: Mutex<RefCell<Option<SoftwareInterruptControl>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let sw_int = system.software_interrupt_control;
    critical_section::with(|cs| SWINT.borrow_ref_mut(cs).replace(sw_int));

    interrupt::enable(Interrupt::FROM_CPU_INTR0, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR1, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR2, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::FROM_CPU_INTR3, Priority::Priority3).unwrap();

    let mut delay = Delay::new(&clocks);
    let mut counter = 0;

    loop {
        delay.delay_ms(500u32);
        match counter {
            0 => critical_section::with(|cs| {
                SWINT
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .unwrap()
                    .raise(SoftwareInterrupt::SoftwareInterrupt0);
            }),
            1 => critical_section::with(|cs| {
                SWINT
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .unwrap()
                    .raise(SoftwareInterrupt::SoftwareInterrupt1);
            }),
            2 => critical_section::with(|cs| {
                SWINT
                    .borrow_ref_mut(cs)
                    .as_mut()
                    .unwrap()
                    .raise(SoftwareInterrupt::SoftwareInterrupt2);
            }),
            3 => {
                critical_section::with(|cs| {
                    SWINT
                        .borrow_ref_mut(cs)
                        .as_mut()
                        .unwrap()
                        .raise(SoftwareInterrupt::SoftwareInterrupt3);
                });
                counter = -1
            }
            _ => {}
        }
        counter += 1;
    }
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
    esp_println::println!("SW interrupt1");
    critical_section::with(|cs| {
        SWINT
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .reset(SoftwareInterrupt::SoftwareInterrupt1);
    });
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
