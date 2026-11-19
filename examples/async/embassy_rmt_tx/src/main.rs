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

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32" => peripherals.GPIO1,
        feature = "esp32c3" => peripherals.GPIO21,
        feature = "esp32c5" => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

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
