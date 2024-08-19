//! GPIO interrupt
//!
//! This prints "Interrupt" when the boot button is pressed.
//! It also blinks an LED like the blinky example.
//!
//! The following wiring is assumed:
//! - LED => GPIO2

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{self, Event, Input, Io, Level, Output, Pull},
    macros::ram,
    prelude::*,
};

cfg_if::cfg_if! {
    if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
        static BUTTON: Mutex<RefCell<Option<Input<gpio::Gpio0>>>> = Mutex::new(RefCell::new(None));
    } else {
        static BUTTON: Mutex<RefCell<Option<Input<gpio::Gpio9>>>> = Mutex::new(RefCell::new(None));
    }
}

#[entry]
fn main() -> ! {
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    // Set GPIO2 as an output, and set its state high initially.
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(handler);
    let mut led = Output::new(io.pins.gpio2, Level::Low);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            let button = io.pins.gpio0;
        } else {
            let button = io.pins.gpio9;
        }
    }

    let mut button = Input::new(button, Pull::Up);

    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    led.set_high();

    let delay = Delay::new(&clocks);

    loop {
        led.toggle();
        delay.delay_millis(500);
    }
}

#[handler]
#[ram]
fn handler() {
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            esp_println::println!(
                "GPIO Interrupt with priority {}",
                esp_hal::xtensa_lx::interrupt::get_level()
            );
        } else {
            esp_println::println!("GPIO Interrupt");
        }
    }

    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        esp_println::println!("Button was the source of the interrupt");
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
