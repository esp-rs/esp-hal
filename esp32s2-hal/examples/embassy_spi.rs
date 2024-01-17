//! Embassy SPI
//!
//! Folowing pins are used:
//! SCLK    GPIO36
//! MISO    GPIO37
//! MOSI    GPIO35
//! CS      GPIO34
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! This is an example of running the embassy executor with SPI.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32s2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_descriptors,
    embassy::{self},
    pdma::*,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    IO,
};
use esp_backtrace as _;

#[main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32s2_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio36;
    let miso = io.pins.gpio37;
    let mosi = io.pins.gpio35;
    let cs = io.pins.gpio34;

    let dma = Dma::new(system.dma);
    let dma_channel = dma.spi2channel;

    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);

    let mut spi = Spi::new(peripherals.SPI2, 100u32.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
        .with_dma(dma_channel.configure(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));

    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes recieved: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
