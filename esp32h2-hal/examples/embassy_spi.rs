//! Embassy SPI
//!
//! Folowing pins are used:
//! SCLK    GPIO1
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO11
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

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32h2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy,
    gdma::*,
    peripherals::Peripherals,
    prelude::*,
    spi::{dma::SpiDma, FullDuplexMode, Spi, SpiMode},
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

pub type SpiType<'d> =
    SpiDma<'d, esp32h2_hal::peripherals::SPI2, esp32h2_hal::gdma::Channel0, FullDuplexMode>;

#[embassy_executor::task]
async fn spi_task(spi: &'static mut SpiType<'static>) {
    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(spi, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes recieved: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32h2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32h2_hal::timer::TimerGroup::new(
            peripherals.TIMG0,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        embassy::init(&clocks, timer_group0.timer0);
    }

    esp32h2_hal::interrupt::enable(
        esp32h2_hal::peripherals::Interrupt::DMA_IN_CH0,
        esp32h2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    esp32h2_hal::interrupt::enable(
        esp32h2_hal::peripherals::Interrupt::DMA_OUT_CH0,
        esp32h2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio1;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let cs = io.pins.gpio11;

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    let descriptors = make_static!([0u32; 8 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);

    let spi = make_static!(Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        100u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .with_dma(dma_channel.configure(
        false,
        descriptors,
        rx_descriptors,
        DmaPriority::Priority0,
    )));

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(spi_task(spi)).ok();
    });
}
