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

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
};

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

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

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send, receive);
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

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send[0..1], receive[0..1]);
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

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0x55u8; 4096]);
        for byte in 0..send.len() {
            send[byte] = byte as u8;
        }

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send, receive);
    }

    #[test]
    #[timeout(3)]
    fn test_try_using_non_dma_memory_tx_buffer() {
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

        let (_, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let tx_buffer = {
            // using `static`, not `static mut`, places the array in .rodata
            static TX_BUFFER: [u8; DMA_BUFFER_SIZE] = [42u8; DMA_BUFFER_SIZE];
            unsafe {
                core::slice::from_raw_parts(
                    &mut *(core::ptr::addr_of!(TX_BUFFER) as *mut u8),
                    DMA_BUFFER_SIZE,
                )
            }
        };

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let mut receive = rx_buffer;

        assert!(matches!(
            spi.dma_transfer(&tx_buffer, &mut receive),
            Err(esp_hal::spi::Error::DmaError(
                esp_hal::dma::DmaError::UnsupportedMemoryRegion
            ))
        ));
    }

    #[test]
    #[timeout(3)]
    fn test_try_using_non_dma_memory_rx_buffer() {
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

        let (tx_buffer, tx_descriptors, _, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let rx_buffer = {
            // using `static`, not `static mut`, places the array in .rodata
            static RX_BUFFER: [u8; DMA_BUFFER_SIZE] = [42u8; DMA_BUFFER_SIZE];
            unsafe {
                core::slice::from_raw_parts_mut(
                    &mut *(core::ptr::addr_of!(RX_BUFFER) as *mut u8),
                    DMA_BUFFER_SIZE,
                )
            }
        };

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let mut receive = rx_buffer;
        assert!(matches!(
            spi.dma_transfer(&tx_buffer, &mut receive),
            Err(esp_hal::spi::Error::DmaError(
                esp_hal::dma::DmaError::UnsupportedMemoryRegion
            ))
        ));
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer_owned() {
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

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let send = tx_buffer;
        let receive = rx_buffer;

        send.copy_from_slice(&[0x55u8; 4096]);
        for byte in 0..send.len() {
            send[byte] = byte as u8;
        }

        let transfer = spi.dma_transfer_owned(send, receive).unwrap();
        let (_, send, receive) = transfer.wait().unwrap();
        assert_eq!(send, receive);
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

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = rx_buffer;

        // Fill the buffer where each byte has 3 pos edges.
        tx_buffer.fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(&mut receive).unwrap();
            transfer.wait().unwrap();
            assert_eq!(receive, &[0, 0, 0, 0, 0]);

            let transfer = spi.dma_write(&tx_buffer).unwrap();
            transfer.wait().unwrap();
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

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = rx_buffer;

        // Fill the buffer where each byte has 3 pos edges.
        tx_buffer.fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(&mut receive).unwrap();
            transfer.wait().unwrap();
            assert_eq!(receive, &[0, 0, 0, 0, 0]);

            let transfer = spi.dma_transfer(&tx_buffer, &mut receive).unwrap();
            transfer.wait().unwrap();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }
}
