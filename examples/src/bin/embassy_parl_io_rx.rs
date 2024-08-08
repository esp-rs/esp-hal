//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO1, GPIO2, GPIO3, and GPIO4.

//% CHIPS: esp32c6 esp32h2
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    parl_io::{no_clk_pin, BitPackOrder, ParlIoRxOnly, RxFourBits},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{
        systimer::{SystemTimer, Target},
        ErasedTimer,
        OneShotTimer,
    },
};
use esp_println::println;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    let alarm0: ErasedTimer = systimer.alarm0.into();
    let timers = [OneShotTimer::new(alarm0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let (_, _, rx_buffer, rx_descriptors) = dma_buffers!(0, 32000);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure_for_async(false, DmaPriority::Priority0),
        rx_descriptors,
        1.MHz(),
        &clocks,
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(&mut rx_pins, no_clk_pin(), BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    let buffer = rx_buffer;
    loop {
        parl_io_rx.read_dma_async(buffer).await.unwrap();
        println!(
            "Received: {:02x?} ... {:02x?}",
            &buffer[..30],
            &buffer[(buffer.len() - 30)..]
        );

        Timer::after(Duration::from_millis(500)).await;
    }
}
