//! SPI Full Duplex DMA Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO8
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

        // DMA buffer require a static life-time
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

        // DMA buffer require a static life-time
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

        // DMA buffer require a static life-time
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

        // DMA buffer require a static life-time
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
}
