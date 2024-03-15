//! SPI Full Duplex Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS      GPIO5
//!
//! Connect MISO (GPIO2) and MOSI (GPIO4) pins.

#![no_std]
#![no_main]

use hil_test::esp_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
};

struct Context {
    spi: Spi<'static, esp_hal::peripherals::SPI2, FullDuplexMode>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio4;
        let cs = io.pins.gpio5;

        let spi = Spi::new(peripherals.SPI2, 1000u32.kHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(cs),
        );

        Context { spi }
    }
}

#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    fn test_symestric_transfer(mut ctx: Context) {
        let mut data = [0xde, 0xad, 0xbe, 0xef];
        let initial_data = data;

        ctx.spi
            .transfer(&mut data)
            .expect("Symmetric transfer failed");
        assert_eq!(initial_data, data);
    }

    #[test]
    fn test_asymestric_transfer(mut ctx: Context) {
        use embedded_hal_1::spi::SpiBus;

        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00; 4];
        SpiBus::transfer(&mut ctx.spi, &mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
    }

    #[test]
    fn test_symestric_transfer_huge_buffer(mut ctx: Context) {
        let mut data = [0x55u8; 4096];
        for byte in 0..data.len() {
            data[byte] = byte as u8;
        }
        let initial_data = data;

        ctx.spi.transfer(&mut data).expect("Huge transfer failed");
        assert_eq!(initial_data, data);
    }

    #[test]
    #[timeout(3)]
    fn test_symestric_transfer_huge_buffer_no_alloc(mut ctx: Context) {
        use embedded_hal_1::spi::SpiBus;

        let mut write = [0x55u8; 4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }

        SpiBus::transfer_in_place(&mut ctx.spi, &mut write[..]).expect("Huge transfer failed");
        for byte in 0..write.len() {
            assert_eq!(write[byte], byte as u8);
        }
    }
}
