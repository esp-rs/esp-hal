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
    delay::Delay,
    interrupt::interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl},
    prelude::*,
};

static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));
static SWINT1: Mutex<RefCell<Option<SoftwareInterrupt<1>>>> = Mutex::new(RefCell::new(None));
static SWINT2: Mutex<RefCell<Option<SoftwareInterrupt<2>>>> = Mutex::new(RefCell::new(None));
static SWINT3: Mutex<RefCell<Option<SoftwareInterrupt<3>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(CpuClock::boot_default());

    let mut sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

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

    let delay = Delay::new(&clocks);
    let mut counter = 0;

    loop {
        delay.delay_millis(500);
        match counter {
            0 => critical_section::with(|cs| {
                SWINT0.borrow_ref(cs).as_ref().unwrap().raise();
            }),
            1 => critical_section::with(|cs| {
                SWINT1.borrow_ref(cs).as_ref().unwrap().raise();
            }),
            2 => critical_section::with(|cs| {
                SWINT2.borrow_ref(cs).as_ref().unwrap().raise();
            }),
            3 => {
                critical_section::with(|cs| {
                    SWINT3.borrow_ref(cs).as_ref().unwrap().raise();
                });
                counter = -1
            }
            _ => {}
        }
        counter += 1;
    }
}

#[handler]
fn swint0_handler() {
    esp_println::println!("SW interrupt0");
    critical_section::with(|cs| {
        SWINT0.borrow_ref(cs).as_ref().unwrap().reset();
    });
}

#[handler]
fn swint1_handler() {
    esp_println::println!("SW interrupt1");
    critical_section::with(|cs| {
        SWINT1.borrow_ref(cs).as_ref().unwrap().reset();
    });
}

#[handler]
fn swint2_handler() {
    esp_println::println!("SW interrupt2");
    critical_section::with(|cs| {
        SWINT2.borrow_ref(cs).as_ref().unwrap().reset();
    });
}

#[handler]
fn swint3_handler() {
    esp_println::println!("SW interrupt3");
    critical_section::with(|cs| {
        SWINT3.borrow_ref(cs).as_ref().unwrap().reset();
    });
}
