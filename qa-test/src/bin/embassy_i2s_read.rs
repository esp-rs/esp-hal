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

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    dma_buffers,
    i2s::master::{Channels, Config, DataFormat, I2s},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let dma_channel = peripherals.DMA_I2S0;
        } else {
            let dma_channel = peripherals.DMA_CH0;
        }
    }

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(4092 * 4, 0);

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
        .build(rx_descriptors);

    let buffer = rx_buffer;
    println!("Start");

    let mut data = [0u8; 5000];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let avail = transaction.available().await.unwrap();
        println!("available {}", avail);

        let count = transaction.pop(&mut data).await.unwrap();

        #[cfg(not(feature = "esp32s2"))]
        println!(
            "got {} bytes, {:x?}..{:x?}",
            count,
            &data[..10],
            &data[count - 10..count]
        );

        // esp-println is a bit slow on ESP32-S2 - don't run into DMA too late errors
        #[cfg(feature = "esp32s2")]
        println!("got {} bytes", count,);
    }
}
