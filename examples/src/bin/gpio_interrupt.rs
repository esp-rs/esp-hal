//! GPIO interrupt
//!
//! This prints "Interrupt" when the boot button is pressed.
//! It also blinks an LED like the blinky example.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{self, Event, Input, PullDown, IO},
    interrupt::{self, Priority},
    macros::ram,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    Delay,
};

#[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
static BUTTON: Mutex<RefCell<Option<gpio::Gpio0<Input<PullDown>>>>> =
    Mutex::new(RefCell::new(None));
#[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
static BUTTON: Mutex<RefCell<Option<gpio::Gpio9<Input<PullDown>>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Set GPIO1 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio1.into_push_pull_output();

    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let mut button = io.pins.gpio0.into_pull_down_input();
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let mut button = io.pins.gpio9.into_pull_down_input();
    button.listen(Event::FallingEdge);

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));

    interrupt::enable(Interrupt::GPIO, Priority::Priority2).unwrap();

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[ram]
#[interrupt]
fn GPIO() {
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    esp_println::println!(
        "GPIO Interrupt with priority {}",
        esp_hal::xtensa_lx::interrupt::get_level()
    );
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    esp_println::println!("GPIO Interrupt");

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
