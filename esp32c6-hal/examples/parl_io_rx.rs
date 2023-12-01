//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_buffers,
    gdma::Gdma,
    gpio::IO,
    parl_io::{BitPackOrder, NoClkPin, ParlIoRxOnly, RxFourBits},
    peripherals::Peripherals,
    prelude::*,
    Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let (_, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(0, 32000);

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
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

    let mut parl_io_rx = parl_io
        .rx
        .with_config(rx_pins, NoClkPin, BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    let mut buffer = rx_buffer;
    buffer.fill(0u8);

    let mut delay = Delay::new(&clocks);

    loop {
        let transfer = parl_io_rx.read_dma(buffer).unwrap();

        // the buffer and driver is moved into the transfer and we can get it back via
        // `wait`
        (buffer, parl_io_rx) = transfer.wait().unwrap();
        println!("Received: {:02x?} ...", &buffer[..30]);

        delay.delay_ms(500u32);
    }
}
