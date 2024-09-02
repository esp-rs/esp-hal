//! This example shows how to use the touch pad in an embassy context.
//!
//! The touch pad reading for GPIO pin 2 is manually read twice a second,
//! whereas GPIO pin 4 is configured to raise an interrupt upon touch.
//!
//! GPIO pins 2 and 4 must be connected to a touch pad (usually a larger copper
//! pad on a PCB).

//% CHIPS: esp32
//% FEATURES: async embassy esp-hal-embassy/integrated-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::Io,
    rtc_cntl::Rtc,
    timer::timg::TimerGroup,
    touch::{Touch, TouchConfig, TouchPad},
};
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR);

    let touch_pin0 = io.pins.gpio2;
    let touch_pin1 = io.pins.gpio4;

    let touch_cfg = Some(TouchConfig {
        measurement_duration: Some(0x2000),
        ..Default::default()
    });

    let touch = Touch::async_mode(peripherals.TOUCH, &mut rtc, touch_cfg);
    let mut touch0 = TouchPad::new(touch_pin0, &touch);
    let mut touch1 = TouchPad::new(touch_pin1, &touch);

    // The touch peripheral needs some time for the first measurement
    Timer::after(Duration::from_millis(100)).await;
    let mut touch0_baseline = touch0.try_read();
    while touch0_baseline.is_none() {
        Timer::after(Duration::from_millis(100)).await;
        touch0_baseline = touch0.try_read();
    }
    let touch0_baseline = touch0.try_read().unwrap();
    let touch1_baseline = touch1.try_read().unwrap();

    println!("Waiting for touch pad interactions...");

    loop {
        match select(
            touch0.wait_for_touch(touch0_baseline * 2 / 3),
            touch1.wait_for_touch(touch1_baseline * 2 / 3),
        )
        .await
        {
            Either::First(_) => {
                println!("Touchpad 0 touched!");
            }
            Either::Second(_) => {
                println!("Touchpad 1 touched!");
            }
        }
        // Add some "dead-timer" to avoid command line spamming
        Timer::after(Duration::from_millis(500)).await;
    }
}
