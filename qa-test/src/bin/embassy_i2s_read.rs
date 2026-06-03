//! This shows how to continuously receive data via I2S.
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN
//! to read 0 or 0xFF or connect DIN to WS to read two different values.
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer.
//!
//! The following wiring is assumed:
//! - MCLK =>  GPIO0
//! - BCLK =>  GPIO2
//! - WS   =>  GPIO4
//! - DIN  =>  GPIO5

//% CHIP_FILTER: i2s_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    dma_rx_stream_buffer,
    i2s::master::{Channels, Config, DataFormat, I2s},
    interrupt::software::SoftwareInterruptControl,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let dma_channel = cfg_select! {
        any(feature = "esp32", feature = "esp32s2") => peripherals.DMA_I2S0,
        _ => peripherals.DMA_CH0,
    };

    let buffer = dma_rx_stream_buffer!(4092 * 8, 2048);

    let i2s = I2s::new(
        peripherals.I2S0,
        dma_channel,
        Config::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(44100))
            .with_data_format(DataFormat::Data16Channel16)
            .with_channels(Channels::STEREO),
    )
    .unwrap()
    .with_mclk(peripherals.GPIO0)
    .into_async();

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(peripherals.GPIO2)
        .with_ws(peripherals.GPIO4)
        .with_din(peripherals.GPIO5)
        .build();

    println!("Start");

    let mut data = [0u8; 9000];
    let mut transaction = i2s_rx.read(buffer).ok().unwrap();
    loop {
        transaction.wait_for_available_async().await.unwrap();

        let avail = transaction.available_bytes();
        println!("available {}", avail);

        if avail > 0 {
            let count = transaction.pop(&mut data[..avail]);

            cfg_select! {
                feature = "esp32s2" => {
                    println!("got {} bytes", count,);
                }
                _ => {
                    println!(
                        "got {} bytes, {:x?}..{:x?}",
                        count,
                        &data[..10],
                        &data[count - 10..count]
                    );
                }
            }
        }
    }
}
