//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::Level,
    rmt::{PulseCode, Rmt, TxChannelAsync, TxChannelConfig, TxChannelCreatorAsync},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = Rate::from_mhz(32);
        } else {
            let freq = Rate::from_mhz(80);
        }
    };

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap().into_async();

    let mut channel = rmt
        .channel0
        .configure(
            peripherals.GPIO4,
            TxChannelConfig::default().with_clk_divider(255),
        )
        .unwrap();

    let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

    data[data.len() - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
    data[data.len() - 1] = PulseCode::empty();

    loop {
        println!("transmit");
        channel.transmit(&data).await.unwrap();
        println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
