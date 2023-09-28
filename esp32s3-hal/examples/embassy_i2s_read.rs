//! This shows how to continously receive data via I2S
//!
//! Pins used
//! MCLK    GPIO4
//! BCLK    GPIO1
//! WS      GPIO2
//! DIN     GPIO5
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN to
//! read 0 or 0xFF or connect DIN to WS to read two different values
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp32s3_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy::{self, executor::Executor},
    gdma::Gdma,
    gpio::GpioPin,
    i2s,
    i2s::{asynch::*, DataFormat, I2s, I2s0New, I2sRx, MclkPin, PinsBclkWsDin, Standard},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

#[embassy_executor::task]
async fn i2s_task(
    i2s_rx: I2sRx<
        'static,
        i2s::I2sPeripheral0,
        PinsBclkWsDin<
            'static,
            GpioPin<esp32s3_hal::gpio::Unknown, 1>,
            GpioPin<esp32s3_hal::gpio::Unknown, 2>,
            GpioPin<esp32s3_hal::gpio::Unknown, 3>,
        >,
        esp32s3_hal::gdma::Channel0,
    >,
) {
    let buffer = dma_buffer();
    esp_println::println!("Start");

    let mut data = [0u8; 5000];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let avail = transaction.available().await;
        esp_println::println!("available {}", avail);

        let count = transaction.pop(&mut data).await.unwrap();
        esp_println::println!(
            "got {} bytes, {:x?}..{:x?}",
            count,
            &data[..10],
            &data[count - 10..count]
        );
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(
        &clocks,
        esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks).timer0,
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let tx_descriptors = make_static!([0u32; 20 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);

    let i2s = I2s::new(
        peripherals.I2S0,
        MclkPin::new(io.pins.gpio4),
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        dma_channel.configure(
            false,
            tx_descriptors,
            rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let i2s_rx = i2s.i2s_rx.with_pins(PinsBclkWsDin::new(
        io.pins.gpio1,
        io.pins.gpio2,
        io.pins.gpio3,
    ));

    // you need to manually enable the DMA channel's interrupt!
    esp32s3_hal::interrupt::enable(
        esp32s3_hal::peripherals::Interrupt::DMA_IN_CH0,
        esp32s3_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(i2s_task(i2s_rx)).ok();
    });
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
