//! This shows using Parallel IO to output 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.
//! GPIO 5 as the "valid pin" (driven high during an active transfer) and GPIO
//! 6 as the clock signal output.
//!
//! You can use a logic analyzer to see how the pins are used.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32c6_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy,
    gdma::Gdma,
    gpio::IO,
    interrupt,
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIoTxOnly,
        SampleEdge,
        TxFourBits,
        TxPinConfigWithValidPin,
    },
    peripherals,
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_println::println;

#[main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c6_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32c6_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut tx_descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);

    let parl_io = ParlIoTxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        1u32.MHz(),
        &clocks,
    )
    .unwrap();

    let clock_pin = ClkOutPin::new(io.pins.gpio6);

    let mut parl_io_tx = parl_io
        .tx
        .with_config(
            pin_conf,
            clock_pin,
            0,
            SampleEdge::Normal,
            BitPackOrder::Msb,
        )
        .unwrap();

    // you need to manually enable the DMA channel's interrupt!
    interrupt::enable(
        peripherals::Interrupt::PARL_IO,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    let buffer = dma_buffer();
    for i in 0..buffer.len() {
        buffer[i] = (i % 255) as u8;
    }

    loop {
        parl_io_tx.write_dma_async(buffer).await.unwrap();
        println!("Transferred {} bytes", buffer.len());

        Timer::after(Duration::from_millis(500)).await;
    }
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
