//! This shows how to transmit data continuously via I2S
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
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use esp32h2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy,
    gdma::Gdma,
    gpio::GpioPin,
    i2s,
    i2s::{asynch::*, DataFormat, I2s, I2s0New, I2sTx, MclkPin, PinsBclkWsDout, Standard},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

const SINE: [i16; 64] = [
    0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
    32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
    15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787, -23169,
    -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30272,
    -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511, -6392, -3211,
];

#[embassy_executor::task]
async fn i2s_task(
    i2s_tx: I2sTx<
        'static,
        i2s::I2sPeripheral0,
        PinsBclkWsDout<
            'static,
            GpioPin<esp32h2_hal::gpio::Unknown, 1>,
            GpioPin<esp32h2_hal::gpio::Unknown, 2>,
            GpioPin<esp32h2_hal::gpio::Unknown, 3>,
        >,
        esp32h2_hal::gdma::Channel0,
    >,
) {
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
    let mut idx = 32000 % data.len();

    esp_println::println!("Start");
    let mut transaction = i2s_tx.write_dma_circular_async(buffer).unwrap();
    loop {
        for i in 0..filler.len() {
            filler[i] = data[(idx + i) % data.len()];
        }
        esp_println::println!("Next");

        let written = transaction.push(&filler).await.unwrap();
        idx = (idx + written) % data.len();
        esp_println::println!("written {}", written);
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
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
    embassy::init(
        &clocks,
        esp32h2_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks).timer0,
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

    let i2s_tx = i2s.i2s_tx.with_pins(PinsBclkWsDout::new(
        io.pins.gpio1,
        io.pins.gpio2,
        io.pins.gpio3,
    ));

    // you need to manually enable the DMA channel's interrupt!
    esp32h2_hal::interrupt::enable(
        esp32h2_hal::peripherals::Interrupt::DMA_OUT_CH0,
        esp32h2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(i2s_task(i2s_tx)).ok();
    });
}

fn dma_buffer() -> &'static mut [u8; 32000] {
    static mut BUFFER: [u8; 32000] = [0u8; 32000];
    unsafe { &mut BUFFER }
}
