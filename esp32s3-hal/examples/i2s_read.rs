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

use esp32s3_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    gdma::{Channel0, Gdma},
    gpio::Unknown,
    i2s::{DataFormat, I2s, I2s0New, I2sReadDma, MclkPin, PinsBclkWsDin, Standard},
    peripherals::{Peripherals, I2S0},
    prelude::*,
    IO,
};
use esp_backtrace as _;
use esp_hal_common::gpio::GpioPin;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut tx_descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    // Here we test that the type is
    // 1) reasonably simple (or at least this will flag changes that may make it
    // more complex)
    // 2) can be spelled out by the user
    let i2s: I2s<'_, I2S0, MclkPin<'_, GpioPin<Unknown, 4>>, Channel0> = I2s::new(
        peripherals.I2S0,
        MclkPin::new(io.pins.gpio4),
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

    let i2s_rx = i2s.i2s_rx.with_pins(PinsBclkWsDin::new(
        io.pins.gpio1,
        io.pins.gpio2,
        io.pins.gpio5,
    ));

    let buffer = dma_buffer();

    let mut transfer = i2s_rx.read_dma_circular(buffer).unwrap();
    println!("Started transfer");

    loop {
        let avail = transfer.available();

        if avail > 0 {
            let mut rcv = [0u8; 5000];
            transfer.pop(&mut rcv[..avail]).unwrap();
            println!("Received {:x?}...", &rcv[..30]);
        }
    }
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
