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
    delay::Delay,
    gpio::{self, Event, Input, PullDown, IO},
    macros::ram,
    peripherals::Peripherals,
    prelude::*,
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

    // Set GPIO2 as an output, and set its state high initially.
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(handler);
    let mut led = io.pins.gpio2.into_push_pull_output();

    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let mut button = io.pins.gpio0.into_pull_down_input();
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let mut button = io.pins.gpio9.into_pull_down_input();

    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    led.set_high();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let delay = Delay::new(&clocks);

    loop {
        led.toggle();
        delay.delay_millis(500);
    }
}

#[handler]
#[ram]
fn handler() {
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
