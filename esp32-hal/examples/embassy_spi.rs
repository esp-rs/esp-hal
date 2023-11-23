//! Embassy SPI
//!
//! Folowing pins are used:
//! SCLK    GPIO19
//! MISO    GPIO25
//! MOSI    GPIO23
//! CS      GPIO22
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
use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_descriptors,
    pdma::*,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    timer::TimerGroup,
    IO,
};
use esp_backtrace as _;
use esp_hal_embassy_procmacros::main;

#[main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0.timer0);

    interrupt::enable(Interrupt::SPI2_DMA, Priority::Priority1).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio19;
    let miso = io.pins.gpio25;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio22;

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
