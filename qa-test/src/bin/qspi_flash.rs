//! SPI write and read a flash chip
//!
//! The following wiring is assumed:
//! - SCLK => GPIO0
//! - MISO => GPIO1
//! - MOSI => GPIO2
//! - IO2  => GPIO3
//! - IO3  => GPIO4
//! - CS   => GPIO5
//!
//! The following wiring is assumed for ESP32:
//! - SCLK => GPIO0
//! - MISO => GPIO2
//! - MOSI => GPIO4
//! - IO2  => GPIO5
//! - IO3  => GPIO13
//! - CS   => GPIO14
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect a flash chip (GD25Q64C was used) and make sure QE in the status
//! register is set.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    entry,
    spi::{
        master::{Address, Command, Config, Spi},
        DataMode,
        Mode,
    },
    time::RateExtU32,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let sclk = peripherals.GPIO0;
            let miso = peripherals.GPIO2;
            let mosi = peripherals.GPIO4;
            let sio2 = peripherals.GPIO5;
            let sio3 = peripherals.GPIO13;
            let cs = peripherals.GPIO14;
        } else {
            let sclk = peripherals.GPIO0;
            let miso = peripherals.GPIO1;
            let mosi = peripherals.GPIO2;
            let sio2 = peripherals.GPIO3;
            let sio3 = peripherals.GPIO4;
            let cs = peripherals.GPIO5;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let dma_channel = peripherals.DMA_SPI2;
        } else {
            let dma_channel = peripherals.DMA_CH0;
        }
    }

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(320, 256);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::Mode0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs)
    .with_dma(dma_channel);

    let delay = Delay::new();

    // write enable
    dma_tx_buf.set_length(0);
    let transfer = spi
        .half_duplex_write(
            DataMode::Single,
            Command::Command8(0x06, DataMode::Single),
            Address::None,
            0,
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();
    delay.delay_millis(250);

    // erase sector
    let transfer = spi
        .half_duplex_write(
            DataMode::Single,
            Command::Command8(0x20, DataMode::Single),
            Address::Address24(0x000000, DataMode::Single),
            0,
            dma_tx_buf.len(),
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();
    delay.delay_millis(250);

    // write enable
    let transfer = spi
        .half_duplex_write(
            DataMode::Single,
            Command::Command8(0x06, DataMode::Single),
            Address::None,
            0,
            dma_tx_buf.len(),
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();
    delay.delay_millis(250);

    // write data / program page
    dma_tx_buf.set_length(dma_tx_buf.capacity());
    dma_tx_buf.as_mut_slice().fill(b'!');
    dma_tx_buf.as_mut_slice()[0..][..5].copy_from_slice(&b"Hello"[..]);
    let transfer = spi
        .half_duplex_write(
            DataMode::Quad,
            Command::Command8(0x32, DataMode::Single),
            Address::Address24(0x000000, DataMode::Single),
            0,
            dma_tx_buf.len(),
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, _) = transfer.wait();
    delay.delay_millis(250);

    loop {
        // quad fast read
        let transfer = spi
            .half_duplex_read(
                DataMode::Quad,
                Command::Command8(0xeb, DataMode::Single),
                Address::Address32(0x000000 << 8, DataMode::Quad),
                4,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();

        // here we could do something else while DMA transfer is in progress
        // the buffers and spi is moved into the transfer and we can get it back via
        // `wait`
        (spi, dma_rx_buf) = transfer.wait();

        println!("{:x?}", dma_rx_buf.as_slice());
        for b in &mut dma_rx_buf.as_slice().iter() {
            if *b >= 32 && *b <= 127 {
                print!("{}", *b as char);
            } else {
                print!(".");
            }
        }
        println!();

        delay.delay_millis(250);
    }
}
