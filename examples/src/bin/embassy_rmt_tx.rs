//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::Io,
    prelude::*,
    rmt::{asynch::TxChannelAsync, PulseCode, Rmt, TxChannelConfig, TxChannelCreatorAsync},
    timer::timg::TimerGroup,
};
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 32.MHz();
        } else {
            let freq = 80.MHz();
        }
    };

    let rmt = Rmt::new_async(peripherals.RMT, freq, &clocks).unwrap();

    let mut channel = rmt
        .channel0
        .configure(
            io.pins.gpio4,
            TxChannelConfig {
                clk_divider: 255,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    let mut data = [PulseCode {
        level1: true,
        length1: 200,
        level2: false,
        length2: 50,
    }; 20];

    data[data.len() - 2] = PulseCode {
        level1: true,
        length1: 3000,
        level2: false,
        length2: 500,
    };
    data[data.len() - 1] = PulseCode::default();

    loop {
        println!("transmit");
        channel.transmit(&data).await.unwrap();
        println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
