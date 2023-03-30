//! This shows how to transmit data continously via I2S
//!
//! Pins used
//! MCLK    GPIO4
//! BCLK    GPIO1
//! WS      GPIO2
//! DOUT    GPIO3
//!
//! Without an additional I2S sink device you can inspect the MCLK, BCLK, WS and
//! DOUT with a logic analyzer
//!
//! You can also connect e.g. a PCM510x to hear an annoying loud sine tone (full
//! scale), so turn down the volume before running this example.
//!
//! Wiring is like this
//!
//! | Pin   | Connected to    |
//! |-------|-----------------|
//! | BCK   | GPIO1           |
//! | DIN   | GPIO3           |
//! | LRCK  | GPIO2           |
//! | SCK   | Gnd             |
//! | GND   | Gnd             |
//! | VIN   | +3V3            |
//! | FLT   | Gnd             |
//! | FMT   | Gnd             |
//! | DEMP  | Gnd             |
//! | XSMT  | +3V3            |

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    i2s::{DataFormat, I2s, I2s0New, I2sWriteDma, MclkPin, PinsBclkWsDout, Standard},
    pdma::Dma,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_backtrace as _;

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
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(system.dma, &mut system.peripheral_clock_control);
    let dma_channel = dma.i2s0channel;

    let mut tx_descriptors = [0u32; 20 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let i2s = I2s::new(
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
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2s_tx = i2s.i2s_tx.with_pins(PinsBclkWsDout::new(
        io.pins.gpio1,
        io.pins.gpio2,
        io.pins.gpio3,
    ));

    let data =
        unsafe { core::slice::from_raw_parts(&SINE as *const _ as *const u8, SINE.len() * 2) };

    let buffer = dma_buffer();
    let mut idx = 0;
    for i in 0..usize::min(data.len(), buffer.len()) {
        buffer[i] = data[idx];

        idx += 1;

        if idx >= data.len() {
            idx = 0;
        }
    }

    let mut filler = [0u8; 10000];

    let mut transfer = i2s_tx.write_dma_circular(buffer).unwrap();
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

fn dma_buffer() -> &'static mut [u8; 32000] {
    static mut BUFFER: [u8; 32000] = [0u8; 32000];
    unsafe { &mut BUFFER }
}
