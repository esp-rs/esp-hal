//! This shows how to continuously receive data via I2S
//!
//! Pins used
//! BCLK    GPIO1
//! WS      GPIO2
//! DIN     GPIO3
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN to
//! read 0 or 0xFF or connect DIN to WS to read two different values
//!
//! You can also inspect the BCLK and WS with a logic analyzer

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp32s2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy::{self, executor::Executor},
    gpio::GpioPin,
    i2s,
    i2s::{asynch::*, DataFormat, I2s, I2s0New, I2sRx, NoMclk, PinsBclkWsDin, Standard},
    pdma::Dma,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    IO,
};
use esp_backtrace as _;
use esp_println::println;
use static_cell::make_static;

#[embassy_executor::task]
async fn i2s_task(
    i2s_rx: I2sRx<
        'static,
        i2s::I2sPeripheral0,
        PinsBclkWsDin<
            'static,
            GpioPin<esp32s2_hal::gpio::Unknown, 1>,
            GpioPin<esp32s2_hal::gpio::Unknown, 2>,
            GpioPin<esp32s2_hal::gpio::Unknown, 3>,
        >,
        esp32s2_hal::pdma::I2s0DmaChannel,
    >,
) {
    let buffer = dma_buffer();
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

#[entry]
fn main() -> ! {
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

    let tx_descriptors = make_static!([0u32; 20 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);

    let i2s = I2s::new(
        peripherals.I2S0,
        NoMclk {},
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

    let i2s_tx = i2s.i2s_rx.with_pins(PinsBclkWsDin::new(
        io.pins.gpio1,
        io.pins.gpio2,
        io.pins.gpio3,
    ));

    // you need to manually enable the DMA channel's interrupt!
    esp32s2_hal::interrupt::enable(
        esp32s2_hal::peripherals::Interrupt::I2S0,
        esp32s2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(i2s_task(i2s_tx)).ok();
    });
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
