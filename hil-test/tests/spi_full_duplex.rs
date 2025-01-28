//! SPI Full Duplex test suite.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES(unstable): unstable
//% FEATURES(stable):

// FIXME: add async test cases that don't rely on PCNT

#![no_std]
#![no_main]

use embedded_hal::spi::SpiBus;
use embedded_hal_async::spi::SpiBus as SpiBusAsync;
use esp_hal::{
    spi::master::{Config, Spi},
    Blocking,
};
use fugit::RateExtU32;
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(feature = "unstable")] {
        use esp_hal::{
            dma::{DmaDescriptor, DmaRxBuf, DmaTxBuf},
            dma_buffers,
            gpio::{Level, NoPin},
        };
        #[cfg(pcnt)]
        use esp_hal::{
            gpio::interconnect::InputSignal,
            pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
        };
    }
}

#[cfg(feature = "unstable")]
cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannel = esp_hal::dma::Spi2DmaChannel;
    } else {
        type DmaChannel = esp_hal::dma::DmaChannel0;
    }
}

struct Context {
    spi: Spi<'static, Blocking>,
    #[cfg(feature = "unstable")]
    dma_channel: DmaChannel,
    // Reuse the really large buffer so we don't run out of DRAM with many tests
    rx_buffer: &'static mut [u8],
    #[cfg(feature = "unstable")]
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_buffer: &'static mut [u8],
    #[cfg(feature = "unstable")]
    tx_descriptors: &'static mut [DmaDescriptor],
    #[cfg(all(pcnt, feature = "unstable"))]
    pcnt_source: InputSignal,
    #[cfg(all(pcnt, feature = "unstable"))]
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let (_, mosi) = hil_test::common_test_pins!(peripherals);

        #[cfg(feature = "unstable")]
        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "unstable")] {
                let (miso, mosi) = mosi.split();

                #[cfg(pcnt)]
                let mosi_loopback_pcnt = miso.clone();

                let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
            } else {
                use esp_hal::peripheral::Peripheral;
                let miso = unsafe { mosi.clone_unchecked() };

                static mut TX_BUFFER: [u8; 4096] = [0; 4096];
                static mut RX_BUFFER: [u8; 4096] = [0; 4096];

                let tx_buffer = unsafe { (&raw mut TX_BUFFER).as_mut().unwrap() };
                let rx_buffer = unsafe { (&raw mut RX_BUFFER).as_mut().unwrap() };
            }
        }

        // Need to set miso first so that mosi can overwrite the
        // output connection (because we are using the same pin to loop back)
        let spi = Spi::new(peripherals.SPI2, Config::default().with_frequency(10.MHz()))
            .unwrap()
            .with_sck(peripherals.GPIO0)
            .with_miso(miso)
            .with_mosi(mosi);

        cfg_if::cfg_if! {
            if #[cfg(feature = "unstable")] {
                #[cfg(pcnt)]
                let pcnt = Pcnt::new(peripherals.PCNT);

                Context {
                    spi,
                    rx_buffer,
                    tx_buffer,
                    dma_channel,
                    rx_descriptors,
                    tx_descriptors,
                    #[cfg(pcnt)]
                    pcnt_source: mosi_loopback_pcnt,
                    #[cfg(pcnt)]
                    pcnt_unit: pcnt.unit0,
                }
            } else {
                Context {
                    spi,
                    rx_buffer,
                    tx_buffer,
                }
            }
        }
    }

    #[test]
    fn test_symmetric_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        SpiBus::transfer(&mut ctx.spi, &mut read[..], &write[..])
            .expect("Symmetric transfer failed");
        assert_eq!(write, read);
    }

    #[test]
    async fn test_async_symmetric_transfer(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00u8; 4];

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut read[..], &write[..])
            .await
            .expect("Symmetric transfer failed");
        assert_eq!(write, read);
    }

    #[test]
    fn test_asymmetric_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00; 4];

        SpiBus::transfer(&mut ctx.spi, &mut read[0..2], &write[..])
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
    }

    #[test]
    async fn test_async_asymmetric_transfer(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read: [u8; 4] = [0x00; 4];

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut read[0..2], &write[..])
            .await
            .expect("Asymmetric transfer failed");
        assert_eq!(write[0], read[0]);
        assert_eq!(read[2], 0x00u8);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    fn test_asymmetric_write(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        SpiBus::write(&mut ctx.spi, &write[..]).expect("Asymmetric write failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(unit.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    async fn test_async_asymmetric_write(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::write(&mut spi, &write[..])
            .await
            .expect("Asymmetric write failed");

        assert_eq!(unit.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    async fn async_write_after_sync_write_waits_for_flush(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut spi = ctx.spi.into_async();

        // Slow down SCLK so that transferring the buffer takes a while.
        spi.apply_config(&Config::default().with_frequency(80.kHz()))
            .expect("Apply config failed");

        SpiBus::write(&mut spi, &write[..]).expect("Sync write failed");
        SpiBusAsync::write(&mut spi, &write[..])
            .await
            .expect("Async write failed");

        assert_eq!(unit.value(), 34);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    fn test_asymmetric_write_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        SpiBus::transfer(&mut ctx.spi, &mut [], &write[..]).expect("Asymmetric transfer failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(unit.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    async fn test_async_asymmetric_write_transfer(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let unit = ctx.pcnt_unit;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut [], &write[..])
            .await
            .expect("Asymmetric transfer failed");

        assert_eq!(unit.value(), 9);
    }

    #[test]
    fn test_symmetric_transfer_huge_buffer(mut ctx: Context) {
        let write = &mut ctx.tx_buffer[0..4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }
        let read = &mut ctx.rx_buffer[0..4096];

        SpiBus::transfer(&mut ctx.spi, &mut read[..], &write[..]).expect("Huge transfer failed");
        assert_eq!(write, read);
    }

    #[test]
    async fn test_async_symmetric_transfer_huge_buffer(ctx: Context) {
        let write = &mut ctx.tx_buffer[0..4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }
        let read = &mut ctx.rx_buffer[0..4096];

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut read[..], &write[..])
            .await
            .expect("Huge transfer failed");
        for idx in 0..write.len() {
            assert_eq!(write[idx], read[idx], "Mismatch at index {}", idx);
        }
    }

    #[test]
    fn test_symmetric_transfer_huge_buffer_in_place(mut ctx: Context) {
        let write = &mut ctx.tx_buffer[0..4096];
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
    async fn test_async_symmetric_transfer_huge_buffer_in_place(ctx: Context) {
        let write = &mut ctx.tx_buffer[0..4096];
        for byte in 0..write.len() {
            write[byte] = byte as u8;
        }

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer_in_place(&mut spi, &mut write[..])
            .await
            .expect("Huge transfer failed");
        for byte in 0..write.len() {
            assert_eq!(write[byte], byte as u8);
        }
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    fn test_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        dma_rx_buf.set_length(TRANSFER_SIZE);
        dma_tx_buf.set_length(TRANSFER_SIZE);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice()[..TRANSFER_SIZE].copy_from_slice(&[5; TRANSFER_SIZE]);
            let transfer = spi
                .read(TRANSFER_SIZE, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(&dma_rx_buf.as_slice()[..TRANSFER_SIZE], &[0; TRANSFER_SIZE]);

            let transfer = spi
                .write(TRANSFER_SIZE, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_tx_buf) = transfer.wait();
            assert_eq!(unit.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    fn test_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        dma_rx_buf.set_length(TRANSFER_SIZE);
        dma_tx_buf.set_length(TRANSFER_SIZE);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice()[..TRANSFER_SIZE].copy_from_slice(&[5; TRANSFER_SIZE]);
            let transfer = spi
                .read(TRANSFER_SIZE, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(&dma_rx_buf.as_slice()[..TRANSFER_SIZE], &[0; TRANSFER_SIZE]);

            let transfer = spi
                .transfer(TRANSFER_SIZE, dma_rx_buf, TRANSFER_SIZE, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
            assert_eq!(unit.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn test_symmetric_dma_transfer(ctx: Context) {
        // This test case sends a large amount of data, multiple times to verify that
        // https://github.com/esp-rs/esp-hal/issues/2151 is and remains fixed.
        let mut dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *v = (i % 255) as u8;
        }

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        for i in 0..4 {
            dma_tx_buf.as_mut_slice()[0] = i as u8;
            *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i as u8;
            let transfer = spi
                .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
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
    #[cfg(feature = "unstable")]
    fn test_asymmetric_dma_transfer(ctx: Context) {
        const WRITE_SIZE: usize = 4;
        const READ_SIZE: usize = 2;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, 4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let spi = ctx.spi.with_dma(ctx.dma_channel);
        let transfer = spi
            .transfer(READ_SIZE, dma_rx_buf, WRITE_SIZE, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, (dma_rx_buf, mut dma_tx_buf)) = transfer.wait();
        assert_eq!(
            dma_tx_buf.as_slice()[0..READ_SIZE],
            dma_rx_buf.as_slice()[0..READ_SIZE]
        );

        // Try transfer again to make sure DMA isn't in a broken state.

        dma_tx_buf.fill(&[0xaa, 0xdd, 0xef, 0xbe]);

        let transfer = spi
            .transfer(READ_SIZE, dma_rx_buf, WRITE_SIZE, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        assert_eq!(
            dma_tx_buf.as_slice()[0..READ_SIZE],
            dma_rx_buf.as_slice()[0..READ_SIZE]
        );
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    fn test_dma_bus_read_write_pcnt(ctx: Context) {
        const TRANSFER_SIZE: usize = 4;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        // Fill the buffer where each byte has 3 pos edges.
        let tx_buf = [0b0110_1010; TRANSFER_SIZE];
        let mut rx_buf = [0; TRANSFER_SIZE];

        for i in 1..4 {
            // Preset as 5, expect 0 repeated receive
            rx_buf.copy_from_slice(&[5; TRANSFER_SIZE]);
            spi.read(&mut rx_buf).unwrap();
            assert_eq!(rx_buf, [0; TRANSFER_SIZE]);

            spi.write(&tx_buf).unwrap();
            assert_eq!(ctx.pcnt_unit.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn test_dma_bus_symmetric_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn test_dma_bus_asymmetric_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(&tx_buf[0..1], &rx_buf[0..1]);
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn test_dma_bus_symmetric_transfer_huge_buffer(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4096;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(40);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = core::array::from_fn(|i| i as _);
        let mut rx_buf = [0; DMA_BUFFER_SIZE];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    async fn test_async_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; TRANSFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; TRANSFER_SIZE];

        for i in 1..4 {
            receive.copy_from_slice(&[5; TRANSFER_SIZE]);
            SpiBusAsync::read(&mut spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0; TRANSFER_SIZE]);

            SpiBusAsync::write(&mut spi, &transmit).await.unwrap();
            assert_eq!(ctx.pcnt_unit.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(all(pcnt, feature = "unstable"))]
    async fn test_async_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; TRANSFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; TRANSFER_SIZE];

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBusAsync::read(&mut spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0, 0, 0, 0, 0]);

            SpiBusAsync::transfer(&mut spi, &mut receive, &transmit)
                .await
                .unwrap();
            assert_eq!(ctx.pcnt_unit.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn test_write_read(ctx: Context) {
        let spi = ctx
            .spi
            .with_mosi(NoPin)
            .with_miso(Level::High)
            .with_dma(ctx.dma_channel);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi
            .read(dma_rx_buf.len(), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, mut dma_rx_buf) = transfer.wait();

        let transfer = spi
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, _dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi
            .read(dma_rx_buf.len(), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, dma_rx_buf) = transfer.wait();

        assert_eq!(&[0xff, 0xff, 0xff, 0xff], dma_rx_buf.as_slice());
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn cancel_stops_transaction(mut ctx: Context) {
        // Slow down. At 80kHz, the transfer is supposed to take a bit over 3 seconds.
        // This means that without working cancellation, the test case should
        // fail.
        ctx.spi
            .apply_config(&Config::default().with_frequency(80.kHz()))
            .unwrap();

        // Set up a large buffer that would trigger a timeout
        let dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel);

        let mut transfer = spi
            .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        transfer.wait();
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn can_transmit_after_cancel(mut ctx: Context) {
        // Slow down. At 80kHz, the transfer is supposed to take a bit over 3 seconds.
        ctx.spi
            .apply_config(&Config::default().with_frequency(80.kHz()))
            .unwrap();

        // Set up a large buffer that would trigger a timeout
        let mut dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let mut transfer = spi
            .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

        spi.apply_config(&Config::default().with_frequency(10.MHz()))
            .unwrap();

        let transfer = spi
            .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        let (_, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        if dma_tx_buf.as_slice() != dma_rx_buf.as_slice() {
            defmt::info!("dma_tx_buf: {:?}", dma_tx_buf.as_slice()[0..100]);
            defmt::info!("dma_rx_buf: {:?}", dma_rx_buf.as_slice()[0..100]);
            panic!("Failed to transmit after cancel");
        }
    }

    #[test]
    #[cfg(feature = "unstable")]
    async fn cancelling_an_awaited_transfer_does_nothing(ctx: Context) {
        // Set up a large buffer that would trigger a timeout
        let dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel).into_async();

        let mut transfer = spi
            .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.wait_for_done().await;
        transfer.cancel();

        transfer.wait_for_done().await;
        transfer.cancel();
        _ = transfer.wait();
    }
}
