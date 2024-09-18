//! This example shows how to configure a pin as a touch pad.
//!
//! The touch pad reading for GPIO pin 2 is manually read twice a second,
//! whereas GPIO pin 4 is configured to raise an interrupt upon touch.
//!
//! GPIO pins 2 and 4 must be connected to a touch pad (usually a larger copper
//! pad on a PCB).

//% CHIPS: esp32

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{GpioPin, Io},
    macros::ram,
    prelude::*,
    rtc_cntl::Rtc,
    touch::{Continuous, Touch, TouchConfig, TouchPad},
    Blocking,
};
use esp_println::println;

static TOUCH1: Mutex<RefCell<Option<TouchPad<GpioPin<4>, Continuous, Blocking>>>> =
    Mutex::new(RefCell::new(None));

#[handler]
#[ram]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut touch1 = TOUCH1.borrow_ref_mut(cs);
        let touch1 = touch1.as_mut().unwrap();
        if touch1.is_interrupt_set() {
            println!("touch 1 pin interrupt");
            touch1.clear_interrupt();
            // We disable the interrupt until the next loop iteration to avoid massive retriggering.
            touch1.disable_interrupt();
        }
    });
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.set_interrupt_handler(interrupt_handler);

    let touch_pin0 = io.pins.gpio2;
    let touch_pin1 = io.pins.gpio4;

    let touch_cfg = Some(TouchConfig {
        measurement_duration: Some(0x2000),
        ..Default::default()
    });

    let touch = Touch::continuous_mode(peripherals.TOUCH, touch_cfg);
    let mut touch0 = TouchPad::new(touch_pin0, &touch);
    let mut touch1 = TouchPad::new(touch_pin1, &touch);

    let delay = Delay::new();

    let touch1_baseline = touch1.read();

    critical_section::with(|cs| {
        // A good threshold is 2/3 of the reading when the pad is not touched.
        touch1.enable_interrupt(touch1_baseline * 2 / 3);
        TOUCH1.borrow_ref_mut(cs).replace(touch1)
    });

    loop {
        let touch_reading = touch0.read();
        println!("touch pad measurement: {touch_reading:?}");
        delay.delay_millis(500);

        critical_section::with(|cs| {
            TOUCH1
                .borrow_ref_mut(cs)
                .as_mut()
                .unwrap()
                .enable_interrupt(touch1_baseline * 2 / 3);
        });
    }
}
