//! SPI Full Duplex Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal::spi::SpiBus;
#[cfg(pcnt)]
use esp_hal::{
    gpio::interconnect::InputSignal,
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
};
use esp_hal::{
    gpio::Io,
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
};
use hil_test as _;

struct Context {
    spi: Spi<'static, SPI2, FullDuplexMode>,
    #[cfg(pcnt)]
    pcnt_source: InputSignal,
    #[cfg(pcnt)]
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let sclk = io.pins.gpio0;
        let (_, mosi) = hil_test::common_test_pins!(io);

        let mosi_loopback = mosi.peripheral_input();
        #[cfg(pcnt)]
        let mosi_loopback_pcnt = mosi.peripheral_input();
        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_miso(mosi_loopback);

        #[cfg(pcnt)]
        let pcnt = Pcnt::new(peripherals.PCNT);

        Context {
            spi,
            #[cfg(pcnt)]
            pcnt_source: mosi_loopback_pcnt,
            #[cfg(pcnt)]
            pcnt_unit: pcnt.unit0,
        }
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
    #[cfg(pcnt)]
    fn test_asymmetric_write(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        SpiBus::write(&mut ctx.spi, &write[..]).expect("Asymmetric write failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(unit.get_value(), 9);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_asymmetric_write_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        SpiBus::transfer(&mut ctx.spi, &mut [], &write[..]).expect("Asymmetric transfer failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(unit.get_value(), 9);
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
