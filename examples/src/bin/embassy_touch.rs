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
    clock::ClockControl,
    gpio::Io,
    peripherals::Peripherals,
    rtc_cntl::Rtc,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    touch::{Touch, TouchConfig, TouchPad},
};
use esp_println::println;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

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
