//! SPI Full Duplex DMA ASYNC Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO8
//!
//! Only for test_dma_read_dma_write_pcnt and test_dma_read_dma_transfer_pcnt
//! tests:
//! PCNT    GPIO2
//! OUTPUT  GPIO5 (helper to keep MISO LOW)
//!
//! The idea of using PCNT (input) here is to connect MOSI to it and count the
//! edges of whatever SPI writes (in this test case 3 pos edges).
//!
//! Connect MISO (GPIO2) and MOSI (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    system::SystemControl,
};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        spi::master::dma::SpiDmaBus,
    };

    use super::*;

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    // S3 is disabled due to https://github.com/esp-rs/esp-hal/issues/1524#issuecomment-2255306292
    #[cfg(not(feature = "esp32s3"))]
    fn test_asymmetric_dma_transfer() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4, 2);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice()[0..1], dma_rx_buf.as_slice()[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer_huge_buffer() {
        const DMA_BUFFER_SIZE: usize = 4096;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        for (i, d) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *d = i as _;
        }

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    #[cfg(any(
        feature = "esp32",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32s3"
    ))]
    fn test_dma_read_dma_write_pcnt() {
        use esp_hal::{
            gpio::{Level, Output, Pull},
            pcnt::{
                channel::{EdgeMode, PcntInputConfig, PcntSource},
                Pcnt,
            },
        };

        const DMA_BUFFER_SIZE: usize = 5;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;
        let mosi_mirror = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio6;
        let cs = io.pins.gpio8;

        let mut out_pin = Output::new(io.pins.gpio5, Level::High);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_tx_buf) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    #[cfg(any(
        feature = "esp32",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32s3"
    ))]
    fn test_dma_read_dma_transfer_pcnt() {
        use esp_hal::{
            gpio::{Level, Output, Pull},
            pcnt::{
                channel::{EdgeMode, PcntInputConfig, PcntSource},
                Pcnt,
            },
        };

        const DMA_BUFFER_SIZE: usize = 5;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;
        let mosi_mirror = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio6;
        let cs = io.pins.gpio8;

        let mut out_pin = Output::new(io.pins.gpio5, Level::High);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi
                .dma_transfer(dma_tx_buf, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_asymmetric_transfer() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(&tx_buf[0..1], &rx_buf[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer_huge_buffer() {
        const DMA_BUFFER_SIZE: usize = 4096;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(40);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = core::array::from_fn(|i| i as _);
        let mut rx_buf = [0; DMA_BUFFER_SIZE];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }
}
