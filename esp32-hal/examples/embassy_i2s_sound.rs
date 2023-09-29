//! This shows how to transmit data continously via I2S
//!
//! Pins used
//! BCLK    GPIO12
//! WS      GPIO13
//! DOUT    GPIO14
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
//! | BCK   | GPIO12          |
//! | DIN   | GPIO14          |
//! | LRCK  | GPIO13          |
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

use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy::{self, executor::Executor},
    gpio::GpioPin,
    i2s,
    i2s::{asynch::*, DataFormat, I2s, I2s0New, I2sTx, NoMclk, PinsBclkWsDout, Standard},
    pdma::Dma,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
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
            GpioPin<esp32_hal::gpio::Unknown, 12>,
            GpioPin<esp32_hal::gpio::Unknown, 13>,
            GpioPin<esp32_hal::gpio::Unknown, 14>,
        >,
        esp32_hal::pdma::I2s0DmaChannel,
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

    let i2s_tx = i2s.i2s_tx.with_pins(PinsBclkWsDout::new(
        io.pins.gpio12,
        io.pins.gpio13,
        io.pins.gpio14,
    ));

    // you need to manually enable the DMA channel's interrupt!
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::I2S0,
        esp32_hal::interrupt::Priority::Priority1,
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
