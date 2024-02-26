//! This shows how to continously receive data via I2S.
//!
//! Pins used:
//! MCLK    GPIO0 (not ESP32)
//! BCLK    GPIO2
//! WS      GPIO4
//! DIN     GPIO5
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN
//! to read 0 or 0xFF or connect DIN to WS to read two different values.
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer.

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-executor-thread embassy-time-timg0 embassy-generic-timers

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    embassy::{self},
    i2s::{asynch::*, DataFormat, I2s, Standard},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    IO,
};
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.i2s0channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (_, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(0, 4092 * 4);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    #[cfg(esp32)]
    {
        i2s.with_mclk(io.pins.gpio0);
    }

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(io.pins.gpio2)
        .with_ws(io.pins.gpio4)
        .with_din(io.pins.gpio5)
        .build();

    let buffer = rx_buffer;
    println!("Start");

    let mut data = [0u8; 5000];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let avail = transaction.available().await;
        println!("available {}", avail);

        let count = transaction.pop(&mut data).await.unwrap();
        println!(
            "got {} bytes, {:x?}..{:x?}",
            count,
            &data[..10],
            &data[count - 10..count]
        );
    }
}
