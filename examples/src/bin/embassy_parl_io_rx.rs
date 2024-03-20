//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.

//% CHIPS: esp32c6 esp32h2
//% FEATURES: async embassy embassy-executor-thread embassy-time-timg0 embassy-generic-timers

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    embassy,
    gpio::IO,
    parl_io::{BitPackOrder, NoClkPin, ParlIoRxOnly, RxFourBits},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
};
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let (_, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(0, 32000);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        1.MHz(),
        &clocks,
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(rx_pins, NoClkPin, BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    let buffer = rx_buffer;
    loop {
        parl_io_rx.read_dma_async(buffer).await.unwrap();
        println!("Received: {:02x?} ...", &buffer[..30]);

        Timer::after(Duration::from_millis(500)).await;
    }
}
