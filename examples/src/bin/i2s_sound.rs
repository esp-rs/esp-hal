//! This shows how to transmit data continously via I2S.
//!
//! Pins used:
//! BCLK    GPIO2
//! WS      GPIO4
//! DOUT    GPIO5
//!
//! Without an additional I2S sink device you can inspect the MCLK, BCLK, WS
//!  andDOUT with a logic analyzer.
//!
//! You can also connect e.g. a PCM510x to hear an annoying loud sine tone (full
//! scale), so turn down the volume before running this example.
//!
//! Wiring is like this:
//!
//! | Pin   | Connected to    |
//! |-------|-----------------|
//! | BCK   | GPIO0           |
//! | DIN   | GPIO2           |
//! | LRCK  | GPIO1           |
//! | SCK   | Gnd             |
//! | GND   | Gnd             |
//! | VIN   | +3V3            |
//! | FLT   | Gnd             |
//! | FMT   | Gnd             |
//! | DEMP  | Gnd             |
//! | XSMT  | +3V3            |

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    i2s::{DataFormat, I2s, I2sWriteDma, Standard},
    peripherals::Peripherals,
    prelude::*,
};

const SINE: [i16; 64] = [
    0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
    32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
    15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787, -23169,
    -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30272,
    -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511, -6392, -3211,
];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.i2s0channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(32000, 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(io.pins.gpio2)
        .with_ws(io.pins.gpio4)
        .with_dout(io.pins.gpio5)
        .build();

    let data =
        unsafe { core::slice::from_raw_parts(&SINE as *const _ as *const u8, SINE.len() * 2) };

    let mut idx = 0;
    for i in 0..usize::min(data.len(), tx_buffer.len()) {
        tx_buffer[i] = data[idx];

        idx += 1;

        if idx >= data.len() {
            idx = 0;
        }
    }

    let mut filler = [0u8; 10000];
    let mut transfer = i2s_tx.write_dma_circular(&tx_buffer).unwrap();

    loop {
        let avail = transfer.available();
        if avail > 0 {
            let avail = usize::min(10000, avail);
            for bidx in 0..avail {
                filler[bidx] = data[idx];
                idx += 1;

                if idx >= data.len() {
                    idx = 0;
                }
            }
            transfer.push(&filler[0..avail]).unwrap();
        }
    }
}
