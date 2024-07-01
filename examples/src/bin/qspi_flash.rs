//! SPI write and read a flash chip
//!
//! Folowing pins are used:
//! SCLK            GPIO0
//! MISO/IO0        GPIO1
//! MOSI/IO1        GPIO2
//! IO2             GPIO3
//! IO3             GPIO4
//! CS              GPIO5
//!
//! Folowing pins are used for ESP32:
//! SCLK            GPIO0
//! MISO/IO0        GPIO2
//! MOSI/IO1        GPIO4
//! IO2             GPIO5
//! IO3             GPIO13
//! CS              GPIO14
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
    dma::{Dma, DmaPriority},
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

    let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(sio2),
            Some(sio3),
            Some(cs),
        )
        .with_dma(
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );

    let delay = Delay::new(&clocks);

    // DMA buffer require a static life-time
    let (zero_buf, _, _, _) = dma_buffers!(0);
    let send = tx_buffer;
    let mut receive = rx_buffer;

    // write enable
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
            &zero_buf,
        )
        .unwrap();
    transfer.wait().unwrap();
    delay.delay_millis(250);

    // erase sector
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x20, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            &zero_buf,
        )
        .unwrap();
    transfer.wait().unwrap();
    delay.delay_millis(250);

    // write enable
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
            &zero_buf,
        )
        .unwrap();
    transfer.wait().unwrap();
    delay.delay_millis(250);

    // write data / program page
    send.fill(b'!');
    send[0..][..5].copy_from_slice(&b"Hello"[..]);
    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(0x32, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            &send,
        )
        .unwrap();
    transfer.wait().unwrap();
    delay.delay_millis(250);

    loop {
        // quad fast read
        let transfer = spi
            .read(
                SpiDataMode::Quad,
                Command::Command8(0xeb, SpiDataMode::Single),
                Address::Address32(0x000000 << 8, SpiDataMode::Quad),
                4,
                &mut receive,
            )
            .unwrap();

        // here we could do something else while DMA transfer is in progress
        // the buffers and spi is moved into the transfer and we can get it back via
        // `wait`
        transfer.wait().unwrap();

        println!("{:x?}", &receive);
        for b in &mut receive.iter() {
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
