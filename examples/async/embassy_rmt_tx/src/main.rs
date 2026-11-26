//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

//% CHIP_FILTER: rmt_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::Level,
    rmt::{PulseCode, Rmt, RmtConfig, TxConfig},
    time::Rate,
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_log::init_uart!(peripherals);

    log::info!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let rmt = Rmt::new(peripherals.RMT, RmtConfig::default())
        .unwrap()
        .into_async();

    let mut channel = rmt
        .async_tx_channel(
            peripherals.RMT_CH0,
            TxConfig::default().with_tick_rate(Rate::from_mhz(1)),
        )
        .unwrap()
        .with_pin(peripherals.GPIO4);

    let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

    data[data.len() - 1] = PulseCode::new(Level::High, 3000, Level::Low, 500);

    loop {
        log::info!("transmit");
        channel.transmit_async(&data).await.unwrap();
        log::info!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
