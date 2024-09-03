//! SPI slave loopback test using DMA
//!
//! The following wiring is assumed for the (bitbang) slave:
//!
//! - SCLK => GPIO0
//! - MISO => GPIO1
//! - MOSI => GPIO2
//! - CS   => GPIO3
//!
//! The following wiring is assumed for the (bitbang) master:
//! - SCLK => GPIO4
//! - MISO => GPIO5
//! - MOSI => GPIO8
//! - CS   => GPIO9
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//!
//! Connect corresponding master and slave pins to see the outgoing data is read
//! as incoming data. The master-side pins are chosen to make these connections
//! easy for the barebones chip; all are immediate neighbors of the slave-side
//! pins except SCLK. SCLK is between MOSI and VDD3P3_RTC on the barebones chip,
//! so no immediate neighbor is available.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::{Input, Io, Level, Output, Pull},
    prelude::*,
    spi::{
        slave::{prelude::*, Spi},
        SpiMode,
    },
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut master_sclk = Output::new(io.pins.gpio4, Level::Low);
    let master_miso = Input::new(io.pins.gpio5, Pull::None);
    let mut master_mosi = Output::new(io.pins.gpio8, Level::Low);
    let mut master_cs = Output::new(io.pins.gpio9, Level::High);

    let slave_sclk = io.pins.gpio0;
    let slave_miso = io.pins.gpio1;
    let slave_mosi = io.pins.gpio2;
    let slave_cs = io.pins.gpio3;

    let dma = Dma::new(peripherals.DMA);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32s2")] {
            let dma_channel = dma.spi2channel;
        } else {
            let dma_channel = dma.channel0;
        }
    }

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);

    let mut spi = Spi::new(
        peripherals.SPI2,
        slave_sclk,
        slave_mosi,
        slave_miso,
        slave_cs,
        SpiMode::Mode0,
    )
    .with_dma(
        dma_channel.configure(false, DmaPriority::Priority0),
        tx_descriptors,
        rx_descriptors,
    );

    let delay = Delay::new();

    // DMA buffer require a static life-time
    let master_send = &mut [0u8; 32000];
    let master_receive = &mut [0u8; 32000];
    let mut slave_send = tx_buffer;
    let mut slave_receive = rx_buffer;
    let mut i = 0;

    for (i, v) in master_send.iter_mut().enumerate() {
        *v = (i % 255) as u8;
    }
    for (i, v) in slave_send.iter_mut().enumerate() {
        *v = (254 - (i % 255)) as u8;
    }

    loop {
        master_send[0] = i;
        master_send[master_send.len() - 1] = i;
        slave_send[0] = i;
        slave_send[slave_send.len() - 1] = i;
        slave_receive.fill(0xff);
        i = i.wrapping_add(1);

        println!("Iteration {i}");

        println!("Do `dma_transfer`");

        let transfer = spi
            .dma_transfer(&mut slave_send, &mut slave_receive)
            .unwrap();

        bitbang_master(
            master_send,
            master_receive,
            &mut master_cs,
            &mut master_mosi,
            &mut master_sclk,
            &master_miso,
        );

        transfer.wait().unwrap();
        println!(
            "slave got {:x?} .. {:x?}, master got {:x?} .. {:x?}",
            &slave_receive[..10],
            &slave_receive[slave_receive.len() - 10..],
            &master_receive[..10],
            &master_receive[master_receive.len() - 10..]
        );

        delay.delay_millis(250);

        println!("Do `dma_read`");
        slave_receive.fill(0xff);
        let transfer = spi.dma_read(&mut slave_receive).unwrap();

        bitbang_master(
            master_send,
            master_receive,
            &mut master_cs,
            &mut master_mosi,
            &mut master_sclk,
            &master_miso,
        );

        transfer.wait().unwrap();
        println!(
            "slave got {:x?} .. {:x?}",
            &slave_receive[..10],
            &slave_receive[slave_receive.len() - 10..],
        );

        delay.delay_millis(250);

        println!("Do `dma_write`");
        let transfer = spi.dma_write(&mut slave_send).unwrap();

        master_receive.fill(0);

        bitbang_master(
            master_send,
            master_receive,
            &mut master_cs,
            &mut master_mosi,
            &mut master_sclk,
            &master_miso,
        );

        transfer.wait().unwrap();
        println!(
            "master got {:x?} .. {:x?}",
            &master_receive[..10],
            &master_receive[master_receive.len() - 10..],
        );

        delay.delay_millis(250);

        println!();
    }
}

fn bitbang_master(
    master_send: &[u8],
    master_receive: &mut [u8],
    master_cs: &mut Output,
    master_mosi: &mut Output,
    master_sclk: &mut Output,
    master_miso: &Input,
) {
    // Bit-bang out the contents of master_send and read into master_receive
    // as quickly as manageable. MSB first. Mode 0, so sampled on the rising
    // edge and set on the falling edge.
    master_cs.set_low();
    for (j, v) in master_send.iter().enumerate() {
        let mut b = *v;
        let mut rb = 0u8;
        for _ in 0..8 {
            if b & 128 != 0 {
                master_mosi.set_high();
            } else {
                master_mosi.set_low();
            }
            master_sclk.set_low();
            b <<= 1;
            rb <<= 1;
            // NB: adding about 24 NOPs here makes the clock's duty cycle
            // run at about 50% ... but we don't strictly need the delay,
            // either.
            master_sclk.set_high();
            if master_miso.is_high() {
                rb |= 1;
            }
        }
        master_receive[j] = rb;
    }
    master_sclk.set_low();

    master_cs.set_high();
    master_sclk.set_low();
}
