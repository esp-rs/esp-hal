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
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Address, Command, Spi},
        SpiDataMode,
        SpiMode,
    },
    system::SystemControl,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let sclk = io.pins.gpio0;
            let miso = io.pins.gpio2;
            let mosi = io.pins.gpio4;
            let sio2 = io.pins.gpio5;
            let sio3 = io.pins.gpio13;
            let cs = io.pins.gpio14;
        } else {
            let sclk = io.pins.gpio0;
            let miso = io.pins.gpio1;
            let mosi = io.pins.gpio2;
            let sio2 = io.pins.gpio3;
            let sio3 = io.pins.gpio4;
            let cs = io.pins.gpio5;
        }
    }

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.spi2channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(256, 320);
    let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(sio2),
            Some(sio3),
            Some(cs),
        )
        .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

    let delay = Delay::new(&clocks);

    // write enable
    dma_tx_buf.set_length(0);
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();
    delay.delay_millis(250);

    // erase sector
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x20, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();
    delay.delay_millis(250);

    // write enable
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
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
        .write(
            SpiDataMode::Quad,
            Command::Command8(0x32, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, _) = transfer.wait();
    delay.delay_millis(250);

    loop {
        // quad fast read
        let transfer = spi
            .read(
                SpiDataMode::Quad,
                Command::Command8(0xeb, SpiDataMode::Single),
                Address::Address32(0x000000 << 8, SpiDataMode::Quad),
                4,
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
