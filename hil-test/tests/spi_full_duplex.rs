//! SPI Full Duplex Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO8
//!
//! Connect MISO (GPIO2) and MOSI (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal::spi::SpiBus;
use esp_hal::{
    gpio::Io,
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
};
use hil_test as _;

struct Context {
    spi: Spi<'static, esp_hal::peripherals::SPI2, FullDuplexMode>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let System {
            peripherals,
            clocks,
            ..
        } = esp_hal::init(CpuClock::boot_default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let spi = Spi::new(peripherals.SPI2, 1000u32.kHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(cs),
        );

        Context { spi }
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        SpiBus::transfer(&mut ctx.spi, &mut read[..], &write[..])
            .expect("Symmetric transfer failed");
        assert_eq!(write, read);
    }

    #[test]
    #[timeout(3)]
    fn test_asymmetric_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00; 4];

        SpiBus::transfer(&mut ctx.spi, &mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_transfer_huge_buffer(mut ctx: Context) {
        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }
        let mut read = [0x00u8; 4096];

        SpiBus::transfer(&mut ctx.spi, &mut read[..], &write[..]).expect("Huge transfer failed");
        assert_eq!(write, read);
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_transfer_huge_buffer_no_alloc(mut ctx: Context) {
        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }

        ctx.spi
            .transfer_in_place(&mut write[..])
            .expect("Huge transfer failed");
        for byte in 0..write.len() {
            assert_eq!(write[byte], byte as u8);
        }
    }
}
