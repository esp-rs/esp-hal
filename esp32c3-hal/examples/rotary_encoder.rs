//! GPIO interrupt based rotary encoder input
//!
//! This prints "Clockwise" or "Anticlockwise" when detecting a state change
//! in a quadrature code style rotary encoder connected to GPIO18 + GPIO19

#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    gpio::{Gpio18, Gpio19, IO},
    gpio_types::{Event, Input, Pin, PullUp},
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use riscv_rt::entry;
use rotary_encoder_embedded::{Direction, RotaryEncoder};

static ROTARY_ENCODER: Mutex<
    RefCell<Option<RotaryEncoder<Gpio18<Input<PullUp>>, Gpio19<Input<PullUp>>>>>,
> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set GPIO18 + GPIO19 as inputs with pullup
    let mut gpio18 = io.pins.gpio18.into_pull_up_input();
    let mut gpio19 = io.pins.gpio19.into_pull_up_input();
    // Trigger an interrupt on both rising and falling edges of both inputs
    gpio18.listen(Event::AnyEdge);
    gpio19.listen(Event::AnyEdge);

    // Initialize the rotary encoder decoder and store it where the interrupt handler can reach it
    let rotary_encoder = RotaryEncoder::new(gpio18, gpio19);
    critical_section::with(|cs| ROTARY_ENCODER.borrow_ref_mut(cs).replace(rotary_encoder));

    interrupt::enable(pac::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();
    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        let mut rotary_encoder = ROTARY_ENCODER.borrow_ref_mut(cs);
        let rotary_encoder = rotary_encoder.as_mut().unwrap();

        // Clear the interrupt bit on both input pins
        let (gpio18, gpio19) = rotary_encoder.borrow_pins();
        gpio18.clear_interrupt();
        gpio19.clear_interrupt();

        // Update the rotary encoder state and print if a rotation was detected
        rotary_encoder.update();
        match rotary_encoder.direction() {
            Direction::Clockwise => {
                esp_println::println!("Clockwise");
            }
            Direction::Anticlockwise => {
                esp_println::println!("Anticlockwise");
            }
            Direction::None => {}
        }
    });
}
