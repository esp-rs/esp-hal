//! This shows how to continously receive data via I2S.
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN
//! to read 0 or 0xFF or connect DIN to WS to read two different values.
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer.
//!
//! The following wiring is assumed:
//! - MCLK =>  GPIO0 (not supported on ESP32)
//! - BCLK =>  GPIO2
//! - WS   =>  GPIO4
//! - DIN  =>  GPIO5

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    i2s::{DataFormat, I2s, I2sReadDma, Standard},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.i2s0channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (_, tx_descriptors, mut rx_buffer, rx_descriptors) = dma_buffers!(0, 4 * 4092);

    // Here we test that the type is
    // 1) reasonably simple (or at least this will flag changes that may make it
    // more complex)
    // 2) can be spelled out by the user
    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100.Hz(),
        dma_channel.configure(false, DmaPriority::Priority0),
        tx_descriptors,
        rx_descriptors,
        &clocks,
    );

    #[cfg(not(feature = "esp32"))]
    let i2s = i2s.with_mclk(io.pins.gpio0);

    let mut i2s_rx = i2s
        .i2s_rx
        .with_bclk(io.pins.gpio2)
        .with_ws(io.pins.gpio4)
        .with_din(io.pins.gpio5)
        .build();

    let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();
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
