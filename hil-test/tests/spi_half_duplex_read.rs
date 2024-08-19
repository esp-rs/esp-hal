//! SPI Half Duplex Read Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//!
//! GPIO    GPIO3
//!
//! Connect MISO (GPIO2) and GPIO (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{
        clock::ClockControl,
        dma::{Dma, DmaPriority},
        dma_buffers,
        gpio::{Io, Level, Output},
        peripherals::Peripherals,
        prelude::_fugit_RateExtU32,
        spi::{
            master::{prelude::*, Address, Command, Spi},
            SpiDataMode,
            SpiMode,
        },
        system::SystemControl,
    };

    #[init]
    fn init() {}

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;

        let mut miso_mirror = Output::new(io.pins.gpio3, Level::High);

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (_, tx_descriptors, mut rx_buffer, rx_descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);

        let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_sck(sclk)
            .with_miso(miso)
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        // Fill with neither 0x00 nor 0xFF.
        rx_buffer.fill(5);

        // SPI should read '0's from the MISO pin
        miso_mirror.set_low();

        let transfer = spi
            .read(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                &mut rx_buffer,
            )
            .unwrap();
        transfer.wait().unwrap();

        assert_eq!(rx_buffer, &[0x00; DMA_BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        miso_mirror.set_high();

        let transfer = spi
            .read(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                &mut rx_buffer,
            )
            .unwrap();
        transfer.wait().unwrap();

        assert_eq!(rx_buffer, &[0xFF; DMA_BUFFER_SIZE]);
    }
}
