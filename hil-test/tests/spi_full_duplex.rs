//! SPI Full Duplex test suite.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

// FIXME: add async test cases that don't rely on PCNT

#![no_std]
#![no_main]

use embedded_hal::spi::SpiBus;
#[cfg(pcnt)]
use embedded_hal_async::spi::SpiBus as SpiBusAsync;
use esp_hal::{
    dma::{Dma, DmaDescriptor, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Io, Level, NoPin},
    peripherals::SPI2,
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
};
#[cfg(pcnt)]
use esp_hal::{
    gpio::interconnect::InputSignal,
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannelCreator = esp_hal::dma::Spi2DmaChannelCreator;
    } else {
        type DmaChannelCreator = esp_hal::dma::ChannelCreator<0>;
    }
}

struct Context {
    spi: Spi<'static, SPI2, FullDuplexMode>,
    dma_channel: DmaChannelCreator,
    // Reuse the really large buffer so we don't run out of DRAM with many tests
    rx_buffer: &'static mut [u8],
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_buffer: &'static mut [u8],
    tx_descriptors: &'static mut [DmaDescriptor],
    #[cfg(pcnt)]
    pcnt_source: InputSignal,
    #[cfg(pcnt)]
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let (_, mosi) = hil_test::common_test_pins!(io);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        #[cfg(pcnt)]
        let mosi_loopback_pcnt = mosi.peripheral_input();
        let mosi_loopback = mosi.peripheral_input();
        let spi = Spi::new(peripherals.SPI2, 10000.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_miso(mosi_loopback);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);

        #[cfg(pcnt)]
        let pcnt = Pcnt::new(peripherals.PCNT);
        Context {
            spi,
            dma_channel,
            rx_buffer,
            rx_descriptors,
            tx_buffer,
            tx_descriptors,
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

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

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
    #[cfg(pcnt)]
    fn test_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi
                .dma_transfer(dma_rx_buf, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    fn test_symmetric_dma_transfer(ctx: Context) {
        // This test case sends a large amount of data, multiple times to verify that
        // https://github.com/esp-rs/esp-hal/issues/2151 is and remains fixed.
        let mut dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *v = (i % 255) as u8;
        }

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        for i in 0..4 {
            dma_tx_buf.as_mut_slice()[0] = i as u8;
            *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i as u8;
            let transfer = spi
                .dma_transfer(dma_rx_buf, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();

            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
            if dma_tx_buf.as_slice() != dma_rx_buf.as_slice() {
                defmt::info!("dma_tx_buf: {:?}", dma_tx_buf.as_slice()[0..100]);
                defmt::info!("dma_rx_buf: {:?}", dma_rx_buf.as_slice()[0..100]);
                panic!("Mismatch at iteration {}", i);
            }
        }
    }

    #[test]
    #[timeout(3)]
    fn test_asymmetric_dma_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(2, 4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));
        let transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, (dma_rx_buf, mut dma_tx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice()[0..2], dma_rx_buf.as_slice()[0..2]);

        // Try transfer again to make sure DMA isn't in a broken state.

        dma_tx_buf.fill(&[0xaa, 0xdd, 0xef, 0xbe]);

        let transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice()[0..2], dma_rx_buf.as_slice()[0..2]);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_asymmetric_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(&tx_buf[0..1], &rx_buf[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer_huge_buffer(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4096;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(40);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0))
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = core::array::from_fn(|i| i as _);
        let mut rx_buf = [0; DMA_BUFFER_SIZE];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    async fn test_async_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(
                ctx.dma_channel
                    .configure_for_async(false, DmaPriority::Priority0),
            )
            .with_buffers(dma_rx_buf, dma_tx_buf);

        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; DMA_BUFFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; DMA_BUFFER_SIZE];

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBusAsync::read(&mut spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0, 0, 0, 0, 0]);

            SpiBusAsync::write(&mut spi, &transmit).await.unwrap();
            assert_eq!(ctx.pcnt_unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    async fn test_async_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(
                ctx.dma_channel
                    .configure_for_async(false, DmaPriority::Priority0),
            )
            .with_buffers(dma_rx_buf, dma_tx_buf);

        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; DMA_BUFFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; DMA_BUFFER_SIZE];

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBusAsync::read(&mut spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0, 0, 0, 0, 0]);

            SpiBusAsync::transfer(&mut spi, &mut receive, &transmit)
                .await
                .unwrap();
            assert_eq!(ctx.pcnt_unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    fn test_write_read(ctx: Context) {
        let spi = ctx
            .spi
            .with_mosi(NoPin)
            .with_miso(Level::High)
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
        let (spi, dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
        let (spi, mut dma_rx_buf) = transfer.wait();

        let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
        let (spi, _dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
        let (_, dma_rx_buf) = transfer.wait();

        assert_eq!(&[0xff, 0xff, 0xff, 0xff], dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    fn cancel_stops_transaction(mut ctx: Context) {
        // Slow down
        ctx.spi.change_bus_frequency(100.Hz());

        // Set up a large buffer that would trigger a timeout
        let dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        let mut transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        transfer.wait();
    }

    #[test]
    #[timeout(3)]
    fn can_transmit_after_cancel(mut ctx: Context) {
        // Slow down
        ctx.spi.change_bus_frequency(100.Hz());

        // Set up a large buffer that would trigger a timeout
        let mut dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel.configure(false, DmaPriority::Priority0));

        let mut transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

        spi.change_bus_frequency(10000.kHz());

        let transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        let (_, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        if dma_tx_buf.as_slice() != dma_rx_buf.as_slice() {
            defmt::info!("dma_tx_buf: {:?}", dma_tx_buf.as_slice()[0..100]);
            defmt::info!("dma_rx_buf: {:?}", dma_rx_buf.as_slice()[0..100]);
            panic!("Failed to transmit after cancel");
        }
    }
}
