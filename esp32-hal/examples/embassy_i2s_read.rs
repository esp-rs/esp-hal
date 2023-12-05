//! This shows how to continously receive data via I2S
//!
//! Pins used
//! BCLK    GPIO12
//! WS      GPIO13
//! DIN     GPIO14
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN to
//! read 0 or 0xFF or connect DIN to WS to read two different values
//!
//! You can also inspect the BCLK and WS with a logic analyzer

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_buffers,
    embassy::{self},
    i2s::{asynch::*, DataFormat, I2s, Standard},
    pdma::Dma,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    IO,
};
use esp_backtrace as _;
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger_from_env();
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(system.dma);
    let dma_channel = dma.i2s0channel;

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

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(io.pins.gpio12)
        .with_ws(io.pins.gpio13)
        .with_din(io.pins.gpio14)
        .build();

    // you need to manually enable the DMA channel's interrupt!
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::I2S0,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

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
