//! This shows using Parallel IO to output 16 bit parallel data at 1MHz clock
//! rate with a delay of 10ms between each transfer.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO16, GPIO4, GPIO17, GPIO18, GPIO5, GPIO19, GPIO12, GPIO14, GPIO27, GPIO26, GPIO13, GPIO33, GPIO32, GPIO21, GPIO22, and GPIO23
//! - Clock output pin => GPIO25
//!
//! You can use a logic analyzer to see how the pins are used.

//% CHIPS: esp32
#![no_std]
#![no_main]

use core::u16;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority, DmaTxBuf},
    dma_descriptors,
    gpio::Io,
    i2s_parallel::{I2sParallel, TxSixteenBits},
    prelude::*,
};
use log::{info, warn};

const BUFFER_SIZE: usize = 32768;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Starting!");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let dma = Dma::new(peripherals.DMA);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let delay = Delay::new();

    let dma_channel = dma.i2s0channel;
    let i2s = peripherals.I2S0;
    let clock = io.pins.gpio25;

    let pins = TxSixteenBits::new(
        io.pins.gpio16,
        io.pins.gpio4,
        io.pins.gpio17,
        io.pins.gpio18,
        io.pins.gpio5,
        io.pins.gpio19,
        io.pins.gpio12,
        io.pins.gpio14,
        io.pins.gpio27,
        io.pins.gpio26,
        io.pins.gpio13,
        io.pins.gpio33,
        io.pins.gpio32,
        io.pins.gpio21,
        io.pins.gpio22,
        io.pins.gpio23,
    );
    let tx_buffer = mk_static!([u16; BUFFER_SIZE], [0u16; BUFFER_SIZE]);
    let (_, tx_descriptors) = dma_descriptors!(0, BUFFER_SIZE * 2);
    // let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, BUFFER_SIZE);
    let parallel = I2sParallel::new(
        i2s,
        dma_channel.configure(false, DmaPriority::Priority0),
        1.MHz(),
        pins,
        clock,
    );

    for i in 0..BUFFER_SIZE {
        // the i2s parallel driver expects the buffer to be interleaved
        let index = i ^ 0x0001;
        if index >= BUFFER_SIZE {
            warn!("index out of bounds {} - skipping!", index);
            continue;
        }
        tx_buffer[index] = i as u16;
    }
    let tx_buffer = unsafe {
        use esp_hal::dma::ReadBuffer;
        let (ptr, len) = tx_buffer.read_buffer();
        // SAFETY: tx_buffer is left inside of the TxBuf
        core::slice::from_raw_parts_mut(ptr as *mut u8, len)
    };

    let mut tx_buf = Some(DmaTxBuf::new(tx_descriptors, tx_buffer).expect("DmaTxBuf::new failed"));

    // parallel.dump();
    let mut parallel = Some(parallel);
    info!("Sending {} bytes!", BUFFER_SIZE);
    loop {
        let p = parallel.take().unwrap();
        let xfer = match p.send(tx_buf.take().unwrap()) {
            Ok(xfer) => xfer,
            Err(_) => {
                panic!("Failed to send buffer");
            }
        };
        let (p, b) = xfer.wait();
        parallel = Some(p);
        tx_buf = Some(b);
        delay.delay_millis(10);
    }
}
