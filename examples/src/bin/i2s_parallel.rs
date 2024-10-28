//! This shows using Parallel IO to output 8 bit parallel data at 1MHz clock
//! rate with a delay of 10ms between each transfer.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO16, GPIO4, GPIO17, GPIO18, GPIO5, GPIO19, GPIO12, and GPIO14
//! - Clock output pin => GPIO25
//!
//! You can use a logic analyzer to see how the pins are used.

//% CHIPS: esp32

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority, DmaTxBuf},
    dma_buffers,
    gpio::Io,
    i2s_parallel::{I2sParallel, TxEightBits},
    prelude::*,
};
use log::info;

const BUFFER_SIZE: usize = 256;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Starting!");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let dma = Dma::new(peripherals.DMA);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let delay = Delay::new();

    let dma_channel = dma.i2s1channel;
    let i2s = peripherals.I2S1;
    let clock = io.pins.gpio25;

    let pins = TxEightBits::new(
        io.pins.gpio16,
        io.pins.gpio4,
        io.pins.gpio17,
        io.pins.gpio18,
        io.pins.gpio5,
        io.pins.gpio19,
        io.pins.gpio12,
        io.pins.gpio14,
    );

    let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, BUFFER_SIZE);
    let parallel = I2sParallel::new(
        i2s,
        dma_channel.configure(false, DmaPriority::Priority0),
        1.MHz(),
        pins,
        clock,
    );

    for (i, data) in tx_buffer.chunks_mut(4).enumerate() {
        let offset = i * 4;
        // i2s parallel driver expects the buffer to be interleaved
        data[0] = (offset + 2) as u8;
        data[1] = (offset + 3) as u8;
        data[2] = offset as u8;
        data[3] = (offset + 1) as u8;
    }

    let tx_buffer = unsafe {
        use esp_hal::dma::ReadBuffer;
        let (ptr, len) = tx_buffer.read_buffer();
        // SAFETY: tx_buffer is left inside of the TxBuf
        core::slice::from_raw_parts_mut(ptr as *mut u8, len)
    };

    let mut tx_buf: DmaTxBuf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("DmaTxBuf::new failed");

    let mut parallel = parallel;
    info!("Sending {} bytes!", BUFFER_SIZE);
    loop {
        let xfer = match parallel.send(tx_buf) {
            Ok(xfer) => xfer,
            Err(_) => {
                panic!("Failed to send buffer");
            }
        };
        (parallel, tx_buf) = xfer.wait();
        delay.delay_millis(10);
    }
}
