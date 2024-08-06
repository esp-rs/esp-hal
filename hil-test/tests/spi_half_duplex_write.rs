//! SPI Half Duplex Write Test
//!
//! Following pins are used:
//! SCLK    GPIO0
//! MOSI    GPIO2
//!
//! PCNT    GPIO3
//!
//! Connect MOSI (GPIO2) and PCNT (GPIO3) pins.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{
        clock::ClockControl,
        dma::{Dma, DmaPriority, DmaTxBuf},
        dma_buffers,
        gpio::{Io, Pull},
        pcnt::{
            channel::{EdgeMode, PcntInputConfig, PcntSource},
            Pcnt,
        },
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
    fn test_spi_writes_are_correctly_by_pcnt() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let dma = Dma::new(peripherals.DMA);

        let sclk = io.pins.gpio0;
        let mosi = io.pins.gpio2;
        let mosi_mirror = io.pins.gpio3;

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (buffer, descriptors, _, _) = dma_buffers!(DMA_BUFFER_SIZE, 0);
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte 3 pos edges.
        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);

        let transfer = spi
            .write(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_tx_buf) = transfer.wait();

        assert_eq!(unit.get_value(), (3 * DMA_BUFFER_SIZE) as _);

        let transfer = spi
            .write(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait();

        assert_eq!(unit.get_value(), (6 * DMA_BUFFER_SIZE) as _);
    }
}
