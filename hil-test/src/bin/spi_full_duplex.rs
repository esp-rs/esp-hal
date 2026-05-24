//! SPI Full Duplex test suite.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32h2 esp32s2 esp32s3
//% FEATURES(unstable): esp-alloc unstable
//% FEATURES(stable):

// FIXME: add async test cases that don't rely on PCNT

#![no_std]
#![no_main]

use embedded_hal::spi::SpiBus;
use embedded_hal_async::spi::SpiBus as SpiBusAsync;
use esp_hal::{
    Blocking,
    gpio::Input,
    spi::master::{Config, Spi},
    time::Rate,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(feature = "unstable")] {
        use esp_hal::peripherals::SPI2;
        use esp_hal::spi::master::{Address, Command, DataMode};

        #[cfg(spi_master_supports_dma)]
        use esp_hal::{
            gpio::{Level, NoPin},
            dma::{DmaDescriptor, DmaRxBuf, DmaTxBuf},
            dma_buffers,
            spi::master::SpiDma,
        };

        #[cfg(all(spi_master_supports_dma, dma_can_access_psram))]
        use allocator_api2::vec::Vec;

        #[cfg(pcnt_driver_supported)]
        use esp_hal::pcnt::{channel::EdgeMode, unit::Unit, Pcnt};

        #[cfg(all(spi_master_supports_dma, pcnt_driver_supported))]
        use esp_hal::Async;
    }
}

#[cfg(all(spi_master_supports_dma, feature = "unstable"))]
cfg_if::cfg_if! {
    if #[cfg(dma_kind = "pdma")] {
        type DmaChannel<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
    } else {
        type DmaChannel<'d> = esp_hal::peripherals::DMA_CH0<'d>;
    }
}

struct Context {
    spi: Spi<'static, Blocking>,

    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    dma_channel: DmaChannel<'static>,

    // Reuse the really large buffer so we don't run out of DRAM with many tests
    rx_buffer: &'static mut [u8],

    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_buffer: &'static mut [u8],

    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    tx_descriptors: &'static mut [DmaDescriptor],

    miso_input: Input<'static>,
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    sclk_input: Input<'static>,

    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(all(pcnt_driver_supported, feature = "unstable"))]
macro_rules! set_up_pcnt {
    ($ctx:expr, $input:ident) => {{
        $ctx.pcnt_unit
            .channel0
            .set_edge_signal($ctx.$input.peripheral_input());
        $ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        $ctx.pcnt_unit
    }};
}

/// Helper function to run a test case with various memory regions for the buffers, including
/// various alignments in external RAM if available.
#[cfg(feature = "unstable")]
fn run_test_in_all_memory_regions<const BUFFER_SIZE: usize>(
    mut test_fn: impl FnMut(&mut [u8], &mut [u8]),
) {
    defmt::info!("Testing buffer size {}", BUFFER_SIZE);
    // TODO: add cases that mix internal and external RAM.

    // Test buffers in internal RAM
    let mut tx_buf = [0; BUFFER_SIZE];
    let mut rx_buf = [0; BUFFER_SIZE];
    test_fn(&mut tx_buf, &mut rx_buf);

    #[cfg(dma_can_access_psram)]
    {
        // Test buffers in external RAM using various alignment offsets.

        const MAX_SHIFT: usize = 15;
        let buf_len = BUFFER_SIZE + MAX_SHIFT;

        let mut external_rx_memory = Vec::with_capacity_in(buf_len, esp_alloc::ExternalMemory);
        let mut external_tx_memory = Vec::with_capacity_in(buf_len, esp_alloc::ExternalMemory);

        external_rx_memory.resize(buf_len, 0);
        external_tx_memory.resize(buf_len, 0);

        for shift in 0..=MAX_SHIFT {
            external_rx_memory.fill(0);

            defmt::info!("Testing PSRAM with shift {}", shift);
            let rx_buf = &mut external_rx_memory[shift..][..BUFFER_SIZE];
            let tx_buf = &mut external_tx_memory[shift..][..BUFFER_SIZE];

            test_fn(tx_buf, rx_buf);
            assert!(external_rx_memory[..shift].iter().all(|&b| b == 0));
            assert!(
                external_rx_memory[shift + BUFFER_SIZE..]
                    .iter()
                    .all(|&b| b == 0)
            );
        }
    }
}

/// Async version of [`run_test_in_all_memory_regions`].
#[cfg(all(feature = "unstable", pcnt_driver_supported, spi_master_supports_dma))]
async fn run_async_test_in_all_memory_regions<const BUFFER_SIZE: usize>(
    mut test_fn: impl AsyncFnMut(&mut [u8], &mut [u8]),
) {
    defmt::info!("Testing buffer size {}", BUFFER_SIZE);
    let mut tx_buf = [0; BUFFER_SIZE];
    let mut rx_buf = [0; BUFFER_SIZE];
    test_fn(&mut tx_buf, &mut rx_buf).await;

    #[cfg(dma_can_access_psram)]
    {
        const MAX_SHIFT: usize = 15;
        let buf_len = BUFFER_SIZE + MAX_SHIFT;

        let mut external_rx_memory = Vec::with_capacity_in(buf_len, esp_alloc::ExternalMemory);
        let mut external_tx_memory = Vec::with_capacity_in(buf_len, esp_alloc::ExternalMemory);

        external_rx_memory.resize(buf_len, 0);
        external_tx_memory.resize(buf_len, 0);

        for shift in 0..=MAX_SHIFT {
            external_rx_memory.fill(0);

            defmt::info!("Testing PSRAM with shift {}", shift);
            let rx_buf = &mut external_rx_memory[shift..][..BUFFER_SIZE];
            let tx_buf = &mut external_tx_memory[shift..][..BUFFER_SIZE];

            test_fn(tx_buf, rx_buf).await;
            assert!(external_rx_memory[..shift].iter().all(|&b| b == 0));
            assert!(
                external_rx_memory[shift + BUFFER_SIZE..]
                    .iter()
                    .all(|&b| b == 0)
            );
        }
    }
}

#[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
fn count_edges(buf: &[u8]) -> i16 {
    let mut count = 0;
    let mut prev_bit = 0;
    for byte in buf {
        for i in 0..8 {
            let bit = (byte >> i) & 1;
            if bit == 1 && prev_bit == 0 {
                count += 1;
            }
            prev_bit = bit;
        }
    }
    count
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use hil_test::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        #[cfg(all(dma_can_access_psram, feature = "unstable"))]
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

        let (_, miso) = hil_test::common_test_pins!(peripherals);
        let sclk = hil_test::unconnected_pin!(peripherals);

        // A bit ugly but the peripheral interconnect APIs aren't yet stable.
        let mosi = unsafe { miso.clone_unchecked() };
        let miso_input = unsafe { miso.clone_unchecked() };
        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_input = unsafe { sclk.clone_unchecked() };

        // Will be used later to detect edges directly or through PCNT.
        let miso_input = Input::new(miso_input, Default::default());

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_input = Input::new(sclk_input, Default::default());

        #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
        cfg_if::cfg_if! {
            if #[cfg(dma_kind = "pdma")] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(all(spi_master_supports_dma, feature = "unstable"))] {
                let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
            } else {
                static mut TX_BUFFER: [u8; 4096] = [0; 4096];
                static mut RX_BUFFER: [u8; 4096] = [0; 4096];

                let tx_buffer = unsafe { (&raw mut TX_BUFFER).as_mut().unwrap() };
                let rx_buffer = unsafe { (&raw mut RX_BUFFER).as_mut().unwrap() };
            }
        }

        // Need to set miso first so that mosi can overwrite the
        // output connection (because we are using the same pin to loop back)
        let spi = Spi::new(
            peripherals.SPI2,
            Config::default().with_frequency(Rate::from_mhz(10)),
        )
        .unwrap()
        .with_sck(sclk)
        .with_miso(miso)
        .with_mosi(mosi);

        cfg_if::cfg_if! {
            if #[cfg(feature = "unstable")] {
                #[cfg(pcnt_driver_supported)]
                let pcnt = Pcnt::new(peripherals.PCNT);

                Context {
                    spi,
                    rx_buffer,
                    tx_buffer,
                    miso_input,
                    #[cfg(pcnt_driver_supported)]
                    sclk_input,
                    #[cfg(spi_master_supports_dma)]
                    dma_channel,
                    #[cfg(spi_master_supports_dma)]
                    rx_descriptors,
                    #[cfg(spi_master_supports_dma)]
                    tx_descriptors,
                    #[cfg(pcnt_driver_supported)]
                    pcnt_unit: pcnt.unit0,
                }
            } else {
                Context {
                    spi,
                    rx_buffer,
                    tx_buffer,
                    miso_input,
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
        let mut read = [0x00; 2];

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        SpiBus::transfer(&mut ctx.spi, &mut read, &write).expect("Asymmetric transfer failed");
        assert_eq!(read[0], write[0]);
        assert_eq!(read[1], write[1]);

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        assert_eq!(sclk_counter.value(), 4 * 8);
    }

    #[test]
    fn test_asymmetric_transfer_read_more(mut ctx: Context) {
        let write = [0xde, 0xad];
        let mut read = [0x00; 4];

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        SpiBus::transfer(&mut ctx.spi, &mut read, &write).expect("Asymmetric transfer failed");
        assert_eq!(read[0], write[0]);
        assert_eq!(read[1], write[1]);
        assert_eq!(read[2], 0x00);
        assert_eq!(read[3], 0x00);

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        assert_eq!(sclk_counter.value(), 4 * 8);
    }

    #[test]
    async fn test_async_asymmetric_transfer(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];
        let mut read = [0x00; 2];

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut read, &write)
            .await
            .expect("Asymmetric transfer failed");
        assert_eq!(read[0], write[0]);
        assert_eq!(read[1], write[1]);

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        assert_eq!(sclk_counter.value(), 4 * 8);
    }

    #[test]
    async fn test_async_asymmetric_transfer_read_more(ctx: Context) {
        let write = [0xde, 0xad];
        let mut read = [0x00; 4];

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut read, &write)
            .await
            .expect("Asymmetric transfer failed");
        assert_eq!(read[0], write[0]);
        assert_eq!(read[1], write[1]);
        assert_eq!(read[2], 0x00);
        assert_eq!(read[3], 0x00);

        #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
        assert_eq!(sclk_counter.value(), 4 * 8);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    fn test_asymmetric_write(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        SpiBus::write(&mut ctx.spi, &write[..]).expect("Asymmetric write failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(miso_pulse_counter.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    async fn test_async_asymmetric_write(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::write(&mut spi, &write[..])
            .await
            .expect("Asymmetric write failed");

        assert_eq!(miso_pulse_counter.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    async fn async_write_after_sync_write_waits_for_flush(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef];

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        let mut spi = ctx.spi.into_async();

        // Slow down SCLK so that transferring the buffer takes a while.
        spi.apply_config(&Config::default().with_frequency(Rate::from_khz(80)))
            .expect("Apply config failed");

        SpiBus::write(&mut spi, &write[..]).expect("Sync write failed");
        SpiBusAsync::write(&mut spi, &write[..])
            .await
            .expect("Async write failed");

        assert_eq!(miso_pulse_counter.value(), 34);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    fn test_asymmetric_write_transfer(mut ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        SpiBus::transfer(&mut ctx.spi, &mut [], &write[..]).expect("Asymmetric transfer failed");
        // Flush because we're not reading, so the write may happen in the background
        ctx.spi.flush().expect("Flush failed");

        assert_eq!(miso_pulse_counter.value(), 9);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    async fn test_async_asymmetric_write_transfer(ctx: Context) {
        let write = [0xde, 0xad, 0xbe, 0xef];

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        let mut spi = ctx.spi.into_async();
        SpiBusAsync::transfer(&mut spi, &mut [], &write[..])
            .await
            .expect("Asymmetric transfer failed");

        assert_eq!(miso_pulse_counter.value(), 9);
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
    #[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
    fn test_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice()[..TRANSFER_SIZE].copy_from_slice(&[5; TRANSFER_SIZE]);
            let transfer = spi
                .read_buffer(TRANSFER_SIZE, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(&dma_rx_buf.as_slice()[..TRANSFER_SIZE], &[0; TRANSFER_SIZE]);

            let transfer = spi
                .write_buffer(TRANSFER_SIZE, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_tx_buf) = transfer.wait();
            assert_eq!(miso_pulse_counter.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
    fn test_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8;
        const TRANSFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice()[..TRANSFER_SIZE].copy_from_slice(&[5; TRANSFER_SIZE]);
            let transfer = spi
                .read_buffer(TRANSFER_SIZE, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(&dma_rx_buf.as_slice()[..TRANSFER_SIZE], &[0; TRANSFER_SIZE]);

            let transfer = spi
                .transfer_buffers(TRANSFER_SIZE, dma_rx_buf, TRANSFER_SIZE, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
            assert_eq!(miso_pulse_counter.value(), (i * 3 * TRANSFER_SIZE) as _);
        }
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
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
                .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
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
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    fn test_asymmetric_dma_transfer(ctx: Context) {
        const WRITE_SIZE: usize = 4;
        const READ_SIZE: usize = 2;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, 4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let spi = ctx.spi.with_dma(ctx.dma_channel);
        let transfer = spi
            .transfer_buffers(READ_SIZE, dma_rx_buf, WRITE_SIZE, dma_tx_buf)
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
            .transfer_buffers(READ_SIZE, dma_rx_buf, WRITE_SIZE, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        assert_eq!(
            dma_tx_buf.as_slice()[0..READ_SIZE],
            dma_rx_buf.as_slice()[0..READ_SIZE]
        );
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
    fn test_dma_bus_read_write_pcnt(ctx: Context) {
        // Test logic
        fn test_write(
            spi: &mut SpiDma<'_, Blocking>,
            miso_pulse_counter: &Unit<'_, 0>,
            tx_buf: &mut [u8],
            rx_buf: &mut [u8],
        ) {
            // Fill the buffer with data where each byte has 3 pos edges.
            tx_buf.fill(0b0110_1010);

            for _ in 1..4 {
                miso_pulse_counter.clear();

                // Preset as 5, expect 0 repeated receive
                rx_buf.fill(5);
                spi.read(rx_buf).unwrap();
                assert!(rx_buf.iter().all(|&b| b == 0));

                spi.write(tx_buf).unwrap();
                assert_eq!(miso_pulse_counter.value(), count_edges(tx_buf));
            }
        }

        // Set up SPI
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        // Run test with various buffer sizes
        run_test_in_all_memory_regions::<4>(|tx_buf, rx_buf| {
            test_write(&mut spi, &miso_pulse_counter, tx_buf, rx_buf);
        });
        run_test_in_all_memory_regions::<128>(|tx_buf, rx_buf| {
            test_write(&mut spi, &miso_pulse_counter, tx_buf, rx_buf);
        });
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    fn test_dma_bus_symmetric_transfer(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(128);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        fn test(spi: &mut SpiDma<'_, Blocking>, tx_buf: &mut [u8], rx_buf: &mut [u8]) {
            // Fill TX buffer with data
            let mut i = 0u8;
            tx_buf.fill_with(|| {
                let byte = i;
                i = i.wrapping_add(1);
                byte
            });

            defmt::debug!("Symmetric transfer");

            spi.transfer(rx_buf, tx_buf).unwrap();
            assert_eq!(tx_buf, rx_buf);

            rx_buf.fill(0);

            defmt::debug!("Asymmetric transfer - read more than write");

            let asymmetric_rx_len = rx_buf.len();
            let asymmetric_tx_len = tx_buf.len() / 2;
            let smaller_len = asymmetric_rx_len.min(asymmetric_tx_len);
            spi.transfer(
                &mut rx_buf[0..asymmetric_rx_len],
                &tx_buf[0..asymmetric_tx_len],
            )
            .unwrap();
            assert_eq!(tx_buf[0..smaller_len], rx_buf[0..smaller_len]);

            rx_buf.fill(0);

            defmt::debug!("Asymmetric transfer - write more than read");

            let asymmetric_rx_len = rx_buf.len() / 2;
            let asymmetric_tx_len = tx_buf.len();
            let smaller_len = asymmetric_rx_len.min(asymmetric_tx_len);
            spi.transfer(
                &mut rx_buf[0..asymmetric_rx_len],
                &tx_buf[0..asymmetric_tx_len],
            )
            .unwrap();
            assert_eq!(tx_buf[0..smaller_len], rx_buf[0..smaller_len]);
        }

        run_test_in_all_memory_regions::<4>(|tx_buf, rx_buf| test(&mut spi, tx_buf, rx_buf));
        run_test_in_all_memory_regions::<48>(|tx_buf, rx_buf| test(&mut spi, tx_buf, rx_buf));
        run_test_in_all_memory_regions::<63>(|tx_buf, rx_buf| test(&mut spi, tx_buf, rx_buf));
        // This test case exists because it (63-68 bytes) timed out on S2 when originally added.
        run_test_in_all_memory_regions::<64>(|tx_buf, rx_buf| test(&mut spi, tx_buf, rx_buf));
        run_test_in_all_memory_regions::<4096>(|tx_buf, rx_buf| test(&mut spi, tx_buf, rx_buf));
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
    async fn test_async_dma_read_dma_write_pcnt(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        async fn test_write(
            spi: &mut SpiDma<'_, Async>,
            miso_pulse_counter: &Unit<'_, 0>,
            tx_buf: &mut [u8],
            rx_buf: &mut [u8],
        ) {
            tx_buf.fill(0b0110_1010);

            for _ in 1..4 {
                miso_pulse_counter.clear();

                rx_buf.fill(5);
                SpiBusAsync::read(spi, rx_buf).await.unwrap();
                assert!(rx_buf.iter().all(|&b| b == 0));

                SpiBusAsync::write(spi, tx_buf).await.unwrap();
                assert_eq!(miso_pulse_counter.value(), count_edges(tx_buf));
            }
        }

        run_async_test_in_all_memory_regions::<4>(async |tx_buf, rx_buf| {
            test_write(&mut spi, &miso_pulse_counter, tx_buf, rx_buf).await;
        })
        .await;
        run_async_test_in_all_memory_regions::<128>(async |tx_buf, rx_buf| {
            test_write(&mut spi, &miso_pulse_counter, tx_buf, rx_buf).await;
        })
        .await;
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, spi_master_supports_dma, feature = "unstable"))]
    async fn test_async_dma_read_dma_transfer_pcnt(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        let miso_pulse_counter = set_up_pcnt!(ctx, miso_input);

        async fn test_transfer(
            spi: &mut SpiDma<'_, Async>,
            miso_pulse_counter: &Unit<'_, 0>,
            tx_buf: &mut [u8],
            rx_buf: &mut [u8],
        ) {
            tx_buf.fill(0b0110_1010);

            for _ in 1..4 {
                miso_pulse_counter.clear();

                rx_buf.fill(5);
                SpiBusAsync::read(spi, rx_buf).await.unwrap();
                assert!(rx_buf.iter().all(|&b| b == 0));

                SpiBusAsync::transfer(spi, rx_buf, tx_buf).await.unwrap();
                assert_eq!(miso_pulse_counter.value(), count_edges(tx_buf));
                assert_eq!(tx_buf, rx_buf);
            }
        }

        run_async_test_in_all_memory_regions::<4>(async |tx_buf, rx_buf| {
            test_transfer(&mut spi, &miso_pulse_counter, tx_buf, rx_buf).await;
        })
        .await;
        run_async_test_in_all_memory_regions::<128>(async |tx_buf, rx_buf| {
            test_transfer(&mut spi, &miso_pulse_counter, tx_buf, rx_buf).await;
        })
        .await;
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
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
            .write_buffer(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi
            .read_buffer(dma_rx_buf.len(), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, mut dma_rx_buf) = transfer.wait();

        let transfer = spi
            .write_buffer(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (spi, _dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi
            .read_buffer(dma_rx_buf.len(), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, dma_rx_buf) = transfer.wait();

        assert_eq!(&[0xff, 0xff, 0xff, 0xff], dma_rx_buf.as_slice());
    }

    #[test]
    async fn cancel_stops_basic_async_spi_transfer(mut ctx: Context) {
        // Slow down. We don't rely on the transfer speed much, just that it's slow
        // enough that we can detect pulses if cancelling the future leaves the
        // transfer running.
        ctx.spi
            .apply_config(&Config::default().with_frequency(Rate::from_khz(800)))
            .unwrap();

        let mut spi = ctx.spi.into_async();

        for i in 0..ctx.tx_buffer.len() {
            ctx.tx_buffer[i] = (i % 256) as u8;
        }

        let transfer = spi.transfer_in_place_async(ctx.tx_buffer);

        // Wait for a bit before cancelling
        let cancel = async {
            for _ in 0..100 {
                embassy_futures::yield_now().await;
            }
        };

        embassy_futures::select::select(transfer, cancel).await;

        // Listen for a while to see if the SPI peripheral correctly stopped.
        let detect_edge = ctx.miso_input.wait_for_any_edge();
        let wait = async {
            for _ in 0..10000 {
                embassy_futures::yield_now().await;
            }
        };

        let result = embassy_futures::select::select(detect_edge, wait).await;

        // Assert that we timed out - we should not have detected any edges
        assert!(
            matches!(result, embassy_futures::select::Either::Second(_)),
            "Detected edge after cancellation"
        );
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    fn cancel_stops_dma_transaction(mut ctx: Context) {
        // Slow down. At 80kHz, the transfer is supposed to take a bit over 3 seconds.
        // This means that without working cancellation, the test case should
        // fail.
        ctx.spi
            .apply_config(&Config::default().with_frequency(Rate::from_khz(80)))
            .unwrap();

        // Set up a large buffer that would trigger a timeout
        let dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel);

        let mut transfer = spi
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        transfer.wait();
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    fn can_transmit_after_cancel(mut ctx: Context) {
        // Slow down. At 80kHz, the transfer is supposed to take a bit over 3 seconds.
        ctx.spi
            .apply_config(&Config::default().with_frequency(Rate::from_khz(80)))
            .unwrap();

        // Set up a large buffer that would trigger a timeout
        let mut dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let mut transfer = spi
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.cancel();
        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

        spi.apply_config(&Config::default().with_frequency(Rate::from_mhz(10)))
            .unwrap();

        let transfer = spi
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
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
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    async fn cancelling_an_awaited_transfer_does_nothing(ctx: Context) {
        // Set up a large buffer that would trigger a timeout
        let dma_rx_buf = DmaRxBuf::new(ctx.rx_descriptors, ctx.rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(ctx.tx_descriptors, ctx.tx_buffer).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel).into_async();

        let mut transfer = spi
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        transfer.wait_for_done().await;
        transfer.cancel();

        transfer.wait_for_done().await;
        transfer.cancel();
        _ = transfer.wait();
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn transfer_works_after_half_duplex_operation(ctx: Context) {
        let mut spi = ctx.spi;

        let mut buffer = [0u8; 4];
        spi.half_duplex_read(
            DataMode::Dual,
            Command::_8Bit(0x92, DataMode::SingleTwoDataLines),
            Address::_32Bit(0x000000_00, DataMode::Dual),
            0,
            &mut buffer,
        )
        .unwrap();

        const DATA: &[u8] = &[0xde, 0xad, 0xbe, 0xef];
        let mut buffer: [u8; 4] = [0x00u8; 4];
        buffer.copy_from_slice(DATA);

        spi.transfer(&mut buffer)
            .expect("Symmetric transfer failed");
        assert_eq!(buffer, DATA);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    fn half_duplex_operation_works_after_nonblocking_write(mut ctx: Context) {
        // SpiBus::write returns before the transfer is complete. This test verifies that
        // half_duplex functions flush before they reconfigure the peripheral.
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        ctx.spi
            .apply_config(&Config::default().with_frequency(Rate::from_khz(80)))
            .unwrap();

        // 320 clock cycles
        let buffer = [0x00; 40];
        SpiBus::write(&mut ctx.spi, &buffer).expect("Write failed");

        // 160 clock cycles
        ctx.spi
            .half_duplex_write(DataMode::Dual, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(sclk_counter.value(), 480);

        sclk_counter.clear();

        // 320 clock cycles
        let mut buffer = [0x00; 40];
        SpiBus::write(&mut ctx.spi, &buffer).expect("Write failed");

        // 160 clock cycles
        ctx.spi
            .half_duplex_read(DataMode::Dual, Command::None, Address::None, 0, &mut buffer)
            .unwrap();

        assert_eq!(sclk_counter.value(), 480);
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, pcnt_driver_supported, feature = "unstable"))]
    fn dma_transfer_works_with_nothing_to_read(ctx: Context) {
        ctx.pcnt_unit
            .channel0
            .set_edge_signal(ctx.miso_input.peripheral_input());
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0x02, 0x02, 0x02, 0x02];
        spi.transfer(&mut [], &tx_buf).unwrap();

        assert_eq!(ctx.pcnt_unit.value(), 4);
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, pcnt_driver_supported, feature = "unstable"))]
    async fn async_dma_transfer_works_with_nothing_to_read(ctx: Context) {
        ctx.pcnt_unit
            .channel0
            .set_edge_signal(ctx.miso_input.peripheral_input());
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        let tx_buf = [0x02, 0x02, 0x02, 0x02];
        spi.transfer_async(&mut [], &tx_buf).await.unwrap();

        assert_eq!(ctx.pcnt_unit.value(), 4);
    }

    #[test]
    #[cfg(all(pcnt_driver_supported, feature = "unstable"))]
    fn half_duplex_operation_resets_size_before_empty_write(mut ctx: Context) {
        // SpiBus::write returns before the transfer is complete. This test verifies that
        // half_duplex functions flush before they reconfigure the peripheral.
        let sclk_counter = set_up_pcnt!(ctx, sclk_input);

        ctx.spi
            .apply_config(&Config::default().with_frequency(Rate::from_khz(80)))
            .unwrap();

        // 32 clock cycles
        let buffer = [0x00; 4];
        ctx.spi
            .half_duplex_write(DataMode::Single, Command::None, Address::None, 0, &buffer)
            .unwrap();

        // 8 clock cycles
        ctx.spi
            .half_duplex_write(
                DataMode::Dual,
                Command::_8Bit(0, DataMode::Single),
                Address::None,
                0,
                &[],
            )
            .unwrap();

        assert_eq!(sclk_counter.value(), 40);
    }

    #[test]
    #[cfg(all(spi_master_supports_dma, feature = "unstable"))]
    fn dma_transfer_works_after_half_duplex_operation(ctx: Context) {
        let mut spi = ctx.spi;

        let mut buffer = [0u8; 4];
        spi.half_duplex_read(
            DataMode::Dual,
            Command::_8Bit(0x92, DataMode::SingleTwoDataLines),
            Address::_32Bit(0x000000_00, DataMode::Dual),
            0,
            &mut buffer,
        )
        .unwrap();

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mut spi = spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();
        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[cfg(feature = "unstable")] // Needed for register access
    fn test_clock_calculation_accuracy(mut ctx: Context) {
        let lowest = if cfg!(esp32h2) { 78048 } else { 78125 };

        let f_mst = if cfg!(esp32c2) {
            40_000_000
        } else if cfg!(esp32h2) {
            48_000_000
        } else if cfg!(esp32c5) {
            80_000_000 // pre-divided by 2
        } else {
            80_000_000
        };
        let inputs = [lowest, 100_000, 1_000_000, f_mst];
        let expected_outputs = [lowest, 100_000, 1_000_000, f_mst];

        for (input, expectation) in inputs.into_iter().zip(expected_outputs.into_iter()) {
            ctx.spi
                .apply_config(&Config::default().with_frequency(Rate::from_hz(input)))
                .unwrap();

            // Read back effective SCLK
            let spi2 = unsafe { SPI2::steal() };

            let clock = spi2.register_block().clock().read();

            let n = clock.clkcnt_n().bits() as u32;
            let pre = clock.clkdiv_pre().bits() as u32;

            let actual = f_mst / ((n + 1) * (pre + 1));

            assert_eq!(actual, expectation);
        }
    }
}
