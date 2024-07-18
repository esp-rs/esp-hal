//! embassy wait
//!
//! This is an example of asynchronously `Wait`ing for a pin state (boot button) to change.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};

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
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    esp_hal_embassy::init(
        &clocks,
        mk_static!(
            [OneShotTimer<ErasedTimer>; 1],
            [OneShotTimer::new(timg0.timer0.into())]
        ),
    );

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let mut input = Input::new(io.pins.gpio0, Pull::Down);
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let mut input = Input::new(io.pins.gpio9, Pull::Down);

    loop {
        esp_println::println!("Waiting...");
        input.wait_for_rising_edge().await;
        esp_println::println!("Ping!");
        Timer::after(Duration::from_millis(100)).await;
    }
}
