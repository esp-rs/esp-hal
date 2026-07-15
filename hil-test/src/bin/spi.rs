//! SPI test suite.

//% CHIP_FILTER: spi_master_driver_supported
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

cfg_select! {
    feature = "unstable" => {
        use esp_hal::peripherals::SPI2;
        use esp_hal::spi::master::{Address, Command, DataMode};

        #[cfg(spi_master_supports_dma)]
        use esp_hal::{
            gpio::{Level, NoPin},
            dma::{DmaDescriptor, DmaRxBuf, DmaTxBuf, aligned::DmaAlignedMut},
            dma_buffers,
            dma_rx_buffer,
            dma_tx_buffer,
            spi::master::SpiDma,
        };

        #[cfg(all(spi_master_supports_dma, dma_can_access_psram))]
        use allocator_api2::vec::Vec;

        #[cfg(pcnt_driver_supported)]
        use esp_hal::pcnt::{channel::EdgeMode, unit::Unit, Pcnt};

        #[cfg(all(spi_master_supports_dma, pcnt_driver_supported))]
        use esp_hal::Async;
    }
    _ => {}
}

#[cfg(all(spi_master_supports_dma, feature = "unstable"))]
type DmaChannel<'a> = cfg_select! {
    spi_master_dma_engine = "SPI_DMA" => {
        esp_hal::peripherals::DMA_SPI2<'a>
    },
    spi_master_dma_engine = "AHB_GDMA" => {
        esp_hal::peripherals::DMA_CH0<'a>
    },
    spi_master_dma_engine = "AXI_GDMA" => {
        esp_hal::peripherals::DMA_AXI_CH0<'a>
    },
};

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

// Needed by run_loopback_test! and dma_alloc_tx_buffer! macros below.
#[cfg(all(dma_can_access_psram, feature = "unstable"))]
extern crate alloc;

// $align must be a compile-time const: dma_descriptors_impl! emits a static
// whose array size depends on it, and statics cannot depend on runtime values.
#[cfg(all(dma_can_access_psram, feature = "unstable"))]
macro_rules! run_loopback_test {
    ($spi:expr, $align:expr) => {{
        use defmt::error;
        const PSRAM_DMA_BUFFER_SIZE: usize = 8192;
        let layout =
            core::alloc::Layout::from_size_align(PSRAM_DMA_BUFFER_SIZE, $align as usize).unwrap();
        let buffer = unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, PSRAM_DMA_BUFFER_SIZE)
        };

        const DMA_CHUNK_SIZE: usize = 4096 - $align as usize;
        let descriptors = esp_hal::dma_descriptors_impl!(PSRAM_DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
        let dma_tx_buf = esp_hal::dma::DmaTxBuf::new_with_config(
            descriptors,
            unsafe { esp_hal::dma::aligned::DmaAlignedMut::new_unchecked(buffer) },
            $align,
        )
        .unwrap();
        let dma_rx_buf = esp_hal::dma_rx_buffer!(PSRAM_DMA_BUFFER_SIZE).unwrap();
        psram_run_loopback($spi, dma_tx_buf, dma_rx_buf);
    }};
}

#[cfg(all(dma_can_access_psram, feature = "unstable"))]
fn psram_run_loopback(
    spi: SpiDma<'static, Blocking>,
    mut dma_tx_buf: esp_hal::dma::DmaTxBuf,
    mut dma_rx_buf: esp_hal::dma::DmaRxBuf,
) {
    for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
        *v = (i % 256) as u8;
    }

    let mut spi = spi;

    for i in 0..4u8 {
        dma_tx_buf.as_mut_slice()[0] = i;
        *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i;

        let transfer = spi
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

        for j in 0..dma_rx_buf.as_slice().len() {
            if dma_rx_buf.as_slice()[j] != dma_tx_buf.as_slice()[j] {
                defmt::panic!(
                    "Mismatch at iteration {}, index {}: expected {=u8:#04x}, got {=u8:#04x}",
                    i,
                    j,
                    dma_tx_buf.as_slice()[j],
                    dma_rx_buf.as_slice()[j],
                );
            }
        }
    }
}

// $align must be a compile-time const for the same reason as run_loopback_test!.
#[cfg(all(esp32s3, feature = "unstable"))]
macro_rules! dma_alloc_tx_buffer {
    ($size:expr, $align:expr) => {{
        use defmt::error;
        let layout = core::alloc::Layout::from_size_align($size, $align as usize).unwrap();
        let buffer = unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        };

        const DMA_CHUNK_SIZE: usize = 4096 - $align as usize;
        let descriptors = esp_hal::dma_descriptors_impl!($size, DMA_CHUNK_SIZE);
        esp_hal::dma::DmaTxBuf::new_with_config(
            descriptors,
            unsafe { esp_hal::dma::aligned::DmaAlignedMut::new_unchecked(buffer) },
            $align,
        )
    }};
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
        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => {
                peripherals.DMA_SPI2
            },
            spi_master_dma_engine = "AHB_GDMA" => {
                peripherals.DMA_CH0
            },
            spi_master_dma_engine = "AXI_GDMA" => {
                peripherals.DMA_AXI_CH0
            },
        };

        cfg_select! {
            all(spi_master_supports_dma, feature = "unstable") => {
                let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
            }
            _ => {
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

        cfg_select! {
            feature = "unstable" => {
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
                    pcnt_unit: Pcnt::new(peripherals.PCNT).unit0,
                }
            }
            _ => {
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
        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();

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
        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();

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
        let mut dma_rx_buf = DmaRxBuf::new(
            DmaAlignedMut::new(ctx.rx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.rx_buffer).unwrap(),
        )
        .unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(
            DmaAlignedMut::new(ctx.tx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.tx_buffer).unwrap(),
        )
        .unwrap();

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
        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(4).unwrap();

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
        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();

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
        let dma_rx_buf = dma_rx_buffer!(128).unwrap();
        let dma_tx_buf = dma_tx_buffer!(128).unwrap();

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
        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();
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
        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();
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

        let mut dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(4).unwrap();

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
        let dma_rx_buf = DmaRxBuf::new(
            DmaAlignedMut::new(ctx.rx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.rx_buffer).unwrap(),
        )
        .unwrap();
        let dma_tx_buf = DmaTxBuf::new(
            DmaAlignedMut::new(ctx.tx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.tx_buffer).unwrap(),
        )
        .unwrap();

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
        let mut dma_rx_buf = DmaRxBuf::new(
            DmaAlignedMut::new(ctx.rx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.rx_buffer).unwrap(),
        )
        .unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(
            DmaAlignedMut::new(ctx.tx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.tx_buffer).unwrap(),
        )
        .unwrap();

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
        let dma_rx_buf = DmaRxBuf::new(
            DmaAlignedMut::new(ctx.rx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.rx_buffer).unwrap(),
        )
        .unwrap();
        let dma_tx_buf = DmaTxBuf::new(
            DmaAlignedMut::new(ctx.tx_descriptors).unwrap(),
            DmaAlignedMut::new(ctx.tx_buffer).unwrap(),
        )
        .unwrap();

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

        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();

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

        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();

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

        let dma_rx_buf = dma_rx_buffer!(4).unwrap();
        let dma_tx_buf = dma_tx_buffer!(4).unwrap();

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
        #[track_caller]
        fn assert_frequency_roundtrips(
            ctx: &mut Context,
            source: SpiFunctionClockConfig,
            input: u32,
        ) {
            let f_mst = SpiInstance::function_clock_source_frequency(source);
            ctx.spi
                .apply_config(
                    &Config::default()
                        .with_frequency(Rate::from_hz(input))
                        .with_clock_source(source),
                )
                .unwrap();

            // Read back effective SCLK
            let spi2 = unsafe { SPI2::steal() };

            let clock = spi2.register_block().clock().read();

            let n = clock.clkcnt_n().bits() as u32 + 1;
            let pre = clock.clkdiv_pre().bits() as u32 + 1;

            let actual = f_mst / (n * pre);

            assert_eq!(
                actual, input,
                "source: {:?} ({}), n={}, pre={}",
                source, f_mst, n, pre
            );
        }

        fn check_typical_values(ctx: &mut Context, source: SpiFunctionClockConfig) {
            esp_hal::clock::ll::ClockTree::with(|clocks| {
                SpiInstance::Spi2.request_function_clock(clocks)
            });
            let f_min = SpiInstance::function_clock_source_frequency(source) / 1024;
            assert_frequency_roundtrips(ctx, source, f_min);
            assert_frequency_roundtrips(
                ctx,
                source,
                SpiInstance::function_clock_source_frequency(source),
            );

            if f_min < 100_000 {
                assert_frequency_roundtrips(ctx, source, 100_000);
            }
            assert_frequency_roundtrips(ctx, source, 1_000_000);
            assert_frequency_roundtrips(ctx, source, 2_000_000);
        }

        use esp_hal::clock::ll::{SpiFunctionClockConfig, SpiInstance};

        #[cfg(any(esp32, esp32s2, esp32s3))]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::Apb);
        #[cfg(not(any(esp32, esp32s2)))]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::Xtal);
        #[cfg(esp32c2)]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::Pll40m);
        #[cfg(esp32h2)]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::PllF48m);
        #[cfg(esp32c3)]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::Pll80m);
        #[cfg(esp32c6)]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::PllF80m);
        #[cfg(esp32c5)]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::PllF120m);
        #[cfg(any(esp32c5, esp32c61))]
        check_typical_values(&mut ctx, SpiFunctionClockConfig::PllF160m);
    }
}

#[cfg(all(dma_can_access_psram, feature = "unstable"))]
#[embedded_test::tests(default_timeout = 10)]
mod psram_dma {
    use esp_hal::{
        Blocking,
        dma::ExternalBurstConfig,
        spi::{
            Mode,
            master::{Config, Spi, SpiDma},
        },
        time::Rate,
    };
    use hil_test as _;

    use super::*;

    struct Context {
        spi: SpiDma<'static, Blocking>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

        let (_, miso) = hil_test::common_test_pins!(peripherals);
        let sclk = hil_test::unconnected_pin!(peripherals);
        let mosi = unsafe { miso.clone_unchecked() };

        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => peripherals.DMA_SPI2,
            spi_master_dma_engine = "AHB_GDMA" => peripherals.DMA_CH0,
            spi_master_dma_engine = "AXI_GDMA" => peripherals.DMA_AXI_CH0,
        };

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_miso(miso)
        .with_mosi(mosi)
        .with_dma(dma_channel);

        Context { spi }
    }

    #[test]
    fn test_psram_tx_loopback_size16(ctx: Context) {
        run_loopback_test!(ctx.spi, ExternalBurstConfig::Size16);
    }

    #[test]
    fn test_psram_tx_loopback_size32(ctx: Context) {
        run_loopback_test!(ctx.spi, ExternalBurstConfig::Size32);
    }

    #[test]
    #[cfg(not(esp32s2))]
    fn test_psram_tx_loopback_size64(ctx: Context) {
        run_loopback_test!(ctx.spi, ExternalBurstConfig::Size64);
    }
}

// FIXME: Only validated on the ESP32-S3 HIL bench; other PSRAM chips pick up spurious PCNT pulses
// there.
#[cfg(all(esp32s3, feature = "unstable"))]
#[embedded_test::tests(default_timeout = 3)]
mod half_duplex_write_psram {
    use esp_hal::{
        Blocking,
        dma::ExternalBurstConfig,
        dma_rx_buffer,
        gpio::{Flex, interconnect::InputSignal},
        pcnt::{Pcnt, channel::EdgeMode, unit::Unit},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi, SpiDma},
        },
        time::Rate,
    };

    use super::*;

    struct Context {
        spi: SpiDma<'static, Blocking>,
        pcnt_unit: Unit<'static, 0>,
        pcnt_source: InputSignal<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

        let sclk = peripherals.GPIO0;
        let (mosi, _) = hil_test::common_test_pins!(peripherals);

        let pcnt = Pcnt::new(peripherals.PCNT);

        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => peripherals.DMA_SPI2,
            spi_master_dma_engine = "AHB_GDMA" => peripherals.DMA_CH0,
            spi_master_dma_engine = "AXI_GDMA" => peripherals.DMA_AXI_CH0,
        };

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_sio0(mosi)
        .with_dma(dma_channel);

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            pcnt_source: mosi_loopback,
        }
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;
        const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size32;
        let mut dma_tx_buf = dma_alloc_tx_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);
        let transfer = spi
            .half_duplex_write_buffer(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_tx_buf) = transfer.wait();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        let transfer = spi
            .half_duplex_write_buffer(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[test]
    fn test_spidmabus_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;
        const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size32;

        let dma_tx_buf = dma_alloc_tx_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT).unwrap();
        let dma_rx_buf = dma_rx_buffer!(1).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_buffers(dma_rx_buf, dma_tx_buf);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
        spi.half_duplex_write(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &buffer,
        )
        .unwrap();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        spi.half_duplex_write(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &buffer,
        )
        .unwrap();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }
}

#[cfg(spi_slave_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod read {
    use esp_hal::{
        Blocking,
        gpio::{Level, Output, OutputConfig},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi},
        },
        time::Rate,
    };
    #[cfg(spi_master_supports_dma)]
    use esp_hal::{dma_rx_buffer, dma_tx_buffer};

    #[cfg(spi_master_supports_dma)]
    type DmaChannel<'a> = cfg_select! {
        spi_master_dma_engine = "SPI_DMA" => {
            esp_hal::peripherals::DMA_SPI2<'a>
        },
        spi_master_dma_engine = "AHB_GDMA" => {
            esp_hal::peripherals::DMA_CH0<'a>
        },
        spi_master_dma_engine = "AXI_GDMA" => {
            esp_hal::peripherals::DMA_AXI_CH0<'a>
        },
    };

    struct Context {
        spi: Spi<'static, Blocking>,
        miso_mirror: Output<'static>,
        #[cfg(spi_master_supports_dma)]
        dma_channel: DmaChannel<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sclk = peripherals.GPIO0;
        let (miso, miso_mirror) = hil_test::common_test_pins!(peripherals);

        let miso_mirror = Output::new(miso_mirror, Level::High, OutputConfig::default());

        #[cfg(spi_master_supports_dma)]
        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => {
                peripherals.DMA_SPI2
            },
            spi_master_dma_engine = "AHB_GDMA" => {
                peripherals.DMA_CH0
            },
            spi_master_dma_engine = "AXI_GDMA" => {
                peripherals.DMA_AXI_CH0
            },
        };

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_miso(miso);

        Context {
            spi,
            miso_mirror,
            #[cfg(spi_master_supports_dma)]
            dma_channel,
        }
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const BUFFER_SIZE: usize = 145;

        ctx.miso_mirror.set_low();

        let mut rx_buf = [0u8; BUFFER_SIZE];
        let mut spi = ctx.spi;

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut rx_buf,
        )
        .unwrap();

        assert_eq!(rx_buf.as_slice(), &[0x00; BUFFER_SIZE]);

        ctx.miso_mirror.set_high();

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut rx_buf,
        )
        .unwrap();

        assert_eq!(rx_buf.as_slice(), &[0xFF; BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();

        ctx.miso_mirror.set_low();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let transfer = spi
            .half_duplex_read_buffer(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        ctx.miso_mirror.set_high();

        let transfer = spi
            .half_duplex_read_buffer(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();

        (_, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let dma_tx_buf = dma_tx_buffer!(1).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        ctx.miso_mirror.set_low();

        let mut buffer = [0xAA; DMA_BUFFER_SIZE];
        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        ctx.miso_mirror.set_high();

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn data_mode_combinations_are_not_rejected(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();

        let mut buffer = [0xAA; DMA_BUFFER_SIZE];
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let modes = [
            (Command::None, Address::None, DataMode::SingleTwoDataLines),
            (Command::None, Address::None, DataMode::Single),
            (Command::None, Address::None, DataMode::Dual),
            (Command::None, Address::None, DataMode::Quad),
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Single,
            ),
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Dual,
            ),
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Quad,
            ),
            (
                Command::_8Bit(0x32, DataMode::SingleTwoDataLines),
                Address::_24Bit(0x2C << 8, DataMode::SingleTwoDataLines),
                DataMode::Quad,
            ),
        ];

        for (command, address, data) in modes {
            if let Err(e) = spi.half_duplex_read(data, command, address, 0, &mut buffer) {
                panic!(
                    "Failed to read with command {:?}, address {:?}, data mode {:?}: {:?}",
                    command, address, data, e
                );
            }
        }
    }
}

#[cfg(pcnt_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod write {
    use esp_hal::{
        Blocking,
        gpio::{Flex, interconnect::InputSignal},
        pcnt::{Pcnt, channel::EdgeMode, unit::Unit},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi},
        },
        time::Rate,
    };
    #[cfg(spi_master_supports_dma)]
    use esp_hal::{dma_rx_buffer, dma_tx_buffer};

    #[cfg(spi_master_supports_dma)]
    type DmaChannel<'a> = cfg_select! {
        spi_master_dma_engine = "SPI_DMA" => {
            esp_hal::peripherals::DMA_SPI2<'a>
        },
        spi_master_dma_engine = "AHB_GDMA" => {
            esp_hal::peripherals::DMA_CH0<'a>
        },
        spi_master_dma_engine = "AXI_GDMA" => {
            esp_hal::peripherals::DMA_AXI_CH0<'a>
        },
    };

    struct Context {
        spi: Spi<'static, Blocking>,
        pcnt_unit: Unit<'static, 0>,
        pcnt_source: InputSignal<'static>,
        cs_unit: Unit<'static, 1>,
        cs_source: InputSignal<'static>,

        #[cfg(spi_master_supports_dma)]
        dma_channel: DmaChannel<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mosi, _) = hil_test::common_test_pins!(peripherals);
        let (sclk, cs) = hil_test::i2c_pins!(peripherals);

        let pcnt = Pcnt::new(peripherals.PCNT);

        #[cfg(spi_master_supports_dma)]
        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => {
                peripherals.DMA_SPI2
            },
            spi_master_dma_engine = "AHB_GDMA" => {
                peripherals.DMA_CH0
            },
            spi_master_dma_engine = "AXI_GDMA" => {
                peripherals.DMA_AXI_CH0
            },
        };

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        let mut cs = Flex::new(cs);
        cs.set_input_enable(true);
        cs.set_output_enable(true);
        let cs_loopback = cs.peripheral_input();

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_sio0(mosi)
        .with_cs(cs);

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            pcnt_source: mosi_loopback,
            cs_unit: pcnt.unit1,
            cs_source: cs_loopback,
            #[cfg(spi_master_supports_dma)]
            dma_channel,
        }
    }

    fn perform_spi_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const BUFFER_SIZE: usize = 145;

        let tx_buf = &mut [0; BUFFER_SIZE];
        let unit = ctx.pcnt_unit;
        let cs_unit = ctx.cs_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        cs_unit.channel0.set_edge_signal(ctx.cs_source);
        cs_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        tx_buf.fill(0b0110_1010);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, tx_buf)
            .unwrap();

        assert_eq!(unit.value(), (3 * BUFFER_SIZE) as _);
        assert_eq!(cs_unit.value(), 1);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, tx_buf)
            .unwrap();

        assert_eq!(unit.value(), (6 * BUFFER_SIZE) as _);
        assert_eq!(cs_unit.value(), 2);
    }

    #[cfg(spi_master_supports_dma)]
    fn perform_spidma_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const DMA_BUFFER_SIZE: usize = 4;

        let mut dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);

        let transfer = spi
            .half_duplex_write_buffer(
                mode,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_tx_buf) = transfer.wait();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        let transfer = spi
            .half_duplex_write_buffer(
                mode,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        let (_spi, _) = transfer.wait();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[cfg(spi_master_supports_dma)]
    fn perform_spidmabus_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const DMA_BUFFER_SIZE: usize = 4;

        let dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let unit = ctx.pcnt_unit;
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
        spi.half_duplex_write(mode, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }
}

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
#[cfg(any(feature = "unstable", spi_slave_driver_supported))]
#[cfg(spi_slave_supports_dma)]
mod spi_slave {
    use esp_hal::{
        Blocking,
        gpio::{Flex, Input, InputConfig, Level, OutputConfig, Pull},
        spi::{Mode, slave::Spi},
    };
    #[cfg(spi_slave_supports_dma)]
    use esp_hal::{dma_rx_buffer, dma_tx_buffer};

    #[cfg(spi_slave_supports_dma)]
    type DmaChannel<'a> = cfg_select! {
        spi_slave_dma_engine = "SPI_DMA" => {
            esp_hal::peripherals::DMA_SPI2<'a>
        },
        spi_slave_dma_engine = "AHB_GDMA" => {
            esp_hal::peripherals::DMA_CH0<'a>
        },
    };

    struct Context {
        spi: Spi<'static, Blocking>,
        #[cfg(spi_slave_supports_dma)]
        dma_channel: DmaChannel<'static>,
        bitbang_spi: BitbangSpi,
    }

    struct BitbangSpi {
        sclk: Flex<'static>,
        mosi: Flex<'static>,
        miso: Input<'static>,
        cs: Flex<'static>,
    }

    impl BitbangSpi {
        fn new(
            sclk: Flex<'static>,
            mosi: Flex<'static>,
            miso: Input<'static>,
            cs: Flex<'static>,
        ) -> Self {
            Self {
                sclk,
                mosi,
                miso,
                cs,
            }
        }

        fn assert_cs(&mut self) {
            self.sclk.set_level(Level::Low);
            self.cs.set_level(Level::Low);
        }

        fn deassert_cs(&mut self) {
            self.sclk.set_level(Level::Low);
            self.cs.set_level(Level::High);
        }

        fn shift_bit(&mut self, bit: bool) -> bool {
            self.mosi.set_level(Level::from(bit));
            self.sclk.set_level(Level::High);
            let miso = self.miso.level().into();
            self.sclk.set_level(Level::Low);
            miso
        }

        fn shift_byte(&mut self, byte: u8) -> u8 {
            let mut out = 0;
            for i in 0..8 {
                let shift = 7 - i;
                out |= (self.shift_bit((byte >> shift) & 1 != 0) as u8) << shift;
            }
            out
        }

        fn transfer_buf(&mut self, rx: &mut [u8], tx: &[u8]) {
            self.assert_cs();
            for (tx, rx) in tx.iter().zip(rx.iter_mut()) {
                *rx = self.shift_byte(*tx);
            }
            self.deassert_cs();
        }
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mosi_pin, miso_pin) = hil_test::i2c_pins!(peripherals);
        let (sclk_pin, _) = hil_test::common_test_pins!(peripherals);
        let cs_pin = hil_test::unconnected_pin!(peripherals);

        #[cfg(spi_slave_supports_dma)]
        let dma_channel = cfg_select! {
            spi_slave_dma_engine = "SPI_DMA" => {
                peripherals.DMA_SPI2
            },
            spi_slave_dma_engine = "AHB_GDMA" => {
                peripherals.DMA_CH0
            },
        };

        let mut mosi_gpio = Flex::new(mosi_pin);
        mosi_gpio.apply_output_config(&OutputConfig::default());
        mosi_gpio.set_level(Level::Low);
        mosi_gpio.set_output_enable(true);

        let mut cs_gpio = Flex::new(cs_pin);
        cs_gpio.apply_output_config(&OutputConfig::default());
        cs_gpio.set_level(Level::High);
        cs_gpio.set_output_enable(true);

        let mut sclk_gpio = Flex::new(sclk_pin);
        sclk_gpio.apply_output_config(&OutputConfig::default());
        sclk_gpio.set_level(Level::Low);
        sclk_gpio.set_output_enable(true);

        let mut miso_gpio = Flex::new(miso_pin);
        miso_gpio.set_input_enable(true);
        miso_gpio.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        miso_gpio.apply_output_config(&OutputConfig::default());

        let cs = cs_gpio.peripheral_input();
        let sclk = sclk_gpio.peripheral_input();
        let mosi = mosi_gpio.peripheral_input();
        let (miso_in, miso_out) = unsafe { miso_gpio.split_into_drivers() };

        Context {
            spi: Spi::new(peripherals.SPI2, Mode::_1)
                .with_sck(sclk)
                .with_mosi(mosi)
                .with_miso(miso_out)
                .with_cs(cs),
            bitbang_spi: BitbangSpi::new(sclk_gpio, mosi_gpio, miso_in, cs_gpio),
            #[cfg(spi_slave_supports_dma)]
            dma_channel,
        }
    }

    #[test]
    fn test_basic_dma(mut ctx: Context) {
        const DMA_SIZE: usize = 32;

        let mut slave_receive = dma_rx_buffer!(DMA_SIZE).unwrap();
        let mut slave_send = dma_tx_buffer!(DMA_SIZE).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel);

        let master_send = &mut [0u8; DMA_SIZE];
        let master_receive = &mut [0xFFu8; DMA_SIZE];

        for (i, v) in master_send.iter_mut().enumerate() {
            *v = (i % 255) as u8;
        }
        for (i, v) in slave_send.as_mut_slice().iter_mut().enumerate() {
            *v = (254 - (i % 255)) as u8;
        }
        slave_receive.as_mut_slice().fill(0xFF);

        let transfer = spi
            .transfer(DMA_SIZE, slave_receive, DMA_SIZE, slave_send)
            .unwrap();

        ctx.bitbang_spi.transfer_buf(master_receive, master_send);

        (_, (slave_receive, slave_send)) = transfer.wait();

        assert_eq!(slave_receive.as_slice(), master_send);
        assert_eq!(master_receive, slave_send.as_slice());
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
#[cfg(spi_master_supports_dma)]
mod qspi_dma {
    #[cfg(pcnt_driver_supported)]
    use esp_hal::gpio::Flex;
    use esp_hal::{
        Blocking,
        dma::{DmaRxBuf, DmaTxBuf},
        dma_rx_buffer,
        dma_tx_buffer,
        gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pull},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi, SpiDma},
        },
        time::Rate,
    };
    #[cfg(pcnt_driver_supported)]
    use esp_hal::{
        pcnt::{Pcnt, channel::EdgeMode, unit::Unit},
        peripherals::PCNT,
    };

    type DmaChannel0<'a> = cfg_select! {
        spi_master_dma_engine = "SPI_DMA" => {
            esp_hal::peripherals::DMA_SPI2<'a>
        },
        spi_master_dma_engine = "AHB_GDMA" => {
            esp_hal::peripherals::DMA_CH0<'a>
        },
        spi_master_dma_engine = "AXI_GDMA" => {
            esp_hal::peripherals::DMA_AXI_CH0<'a>
        },
    };

    cfg_select! {
        esp32 => {
            const COMMAND_DATA_MODES: [DataMode; 1] = [DataMode::SingleTwoDataLines];
        }
        _ => {
            const COMMAND_DATA_MODES: [DataMode; 2] = [DataMode::SingleTwoDataLines, DataMode::Quad];
        }
    }

    type SpiUnderTest<'a> = SpiDma<'a, Blocking>;

    struct Context {
        spi: Spi<'static, Blocking>,
        #[cfg(pcnt_driver_supported)]
        pcnt: PCNT<'static>,
        dma_channel: DmaChannel0<'static>,
        gpios: [AnyPin<'static>; 3],
    }

    fn transfer_read(
        spi: SpiUnderTest,
        dma_rx_buf: DmaRxBuf,
        command: Command,
    ) -> (SpiUnderTest, DmaRxBuf) {
        let transfer = spi
            .half_duplex_read_buffer(
                DataMode::Quad,
                command,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait()
    }

    fn transfer_write(
        spi: SpiUnderTest,
        dma_tx_buf: DmaTxBuf,
        write: u8,
        command_data_mode: DataMode,
    ) -> (SpiUnderTest, DmaTxBuf) {
        let transfer = spi
            .half_duplex_write_buffer(
                DataMode::Quad,
                Command::_8Bit(write as u16, command_data_mode),
                Address::_24Bit(
                    write as u32 | (write as u32) << 8 | (write as u32) << 16,
                    DataMode::Quad,
                ),
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait()
    }

    #[cfg(pcnt_driver_supported)]
    fn transfer_write_cpu<'s>(
        mut spi: Spi<'s, Blocking>,
        tx_buf: &[u8],
        write: u8,
        command_data_mode: DataMode,
    ) -> Spi<'s, Blocking> {
        spi.half_duplex_write(
            DataMode::Quad,
            Command::_8Bit(write as u16, command_data_mode),
            Address::_24Bit(
                write as u32 | (write as u32) << 8 | (write as u32) << 16,
                DataMode::Quad,
            ),
            0,
            tx_buf,
        )
        .unwrap();
        spi
    }

    fn execute_read(mut spi: SpiUnderTest, mut miso_mirror: Output<'static>, expected: u8) {
        const DMA_BUFFER_SIZE: usize = 4;

        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();

        miso_mirror.set_low();
        (spi, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
        assert_eq!(dma_rx_buf.as_slice(), &[0; DMA_BUFFER_SIZE]);

        miso_mirror.set_high();
        (_, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
        assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
    }

    fn execute_write_read(mut spi: SpiUnderTest, mut mosi_mirror: Output<'static>, expected: u8) {
        const DMA_BUFFER_SIZE: usize = 4;

        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();

        dma_tx_buf.fill(&[0x00; DMA_BUFFER_SIZE]);

        for command_data_mode in COMMAND_DATA_MODES {
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, expected, command_data_mode);

            mosi_mirror.set_high();

            (spi, dma_rx_buf) = transfer_read(
                spi,
                dma_rx_buf,
                Command::_8Bit(expected as u16, command_data_mode),
            );
            assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
        }
    }

    #[cfg(pcnt_driver_supported)]
    fn execute_write(
        unit0: Unit<'_, 0>,
        unit1: Unit<'_, 1>,
        mut spi: SpiUnderTest<'_>,
        write: u8,
        data_on_multiple_pins: bool,
    ) {
        const DMA_BUFFER_SIZE: usize = 4;

        let mut dma_tx_buf = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();

        for command_data_mode in COMMAND_DATA_MODES {
            dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

            unit0.clear();
            unit1.clear();
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), 8);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 7);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 8);
                }
            }

            dma_tx_buf.set_length(0);
            unit0.clear();
            unit1.clear();
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), 4);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 3);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 4);
                }
            }
        }
    }

    #[cfg(pcnt_driver_supported)]
    async fn execute_write_async(
        unit0: Unit<'_, 0>,
        unit1: Unit<'_, 1>,
        mut spi: SpiDma<'_, esp_hal::Async>,
        write: u8,
        data_on_multiple_pins: bool,
    ) {
        const BUFFER_SIZE: usize = 4;

        let tx_buf = [write; BUFFER_SIZE];

        for command_data_mode in COMMAND_DATA_MODES {
            unit0.clear();
            unit1.clear();
            spi.half_duplex_write_async(
                DataMode::Quad,
                Command::_8Bit(write as u16, command_data_mode),
                Address::_24Bit(
                    write as u32 | (write as u32) << 8 | (write as u32) << 16,
                    DataMode::Quad,
                ),
                0,
                &tx_buf,
            )
            .await
            .unwrap();
            assert_eq!(unit0.value() + unit1.value(), 8);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 7);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 8);
                }
            }

            unit0.clear();
            unit1.clear();
            spi.half_duplex_write_async(
                DataMode::Quad,
                Command::_8Bit(write as u16, command_data_mode),
                Address::_24Bit(
                    write as u32 | (write as u32) << 8 | (write as u32) << 16,
                    DataMode::Quad,
                ),
                0,
                &[],
            )
            .await
            .unwrap();
            assert_eq!(unit0.value() + unit1.value(), 4);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 3);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 4);
                }
            }
        }
    }

    #[cfg(pcnt_driver_supported)]
    fn execute_write_with_cpu(
        unit0: Unit<'_, 0>,
        unit1: Unit<'_, 1>,
        mut spi: Spi<'_, Blocking>,
        write: u8,
        data_on_multiple_pins: bool,
    ) {
        const BUFFER_SIZE: usize = 145;

        let tx_buf = [write; BUFFER_SIZE];

        for command_data_mode in COMMAND_DATA_MODES {
            unit0.clear();
            unit1.clear();
            spi = transfer_write_cpu(spi, &tx_buf, write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), (BUFFER_SIZE + 4) as _);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), (BUFFER_SIZE + 3) as _);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), (BUFFER_SIZE + 4) as _);
                }
            }

            unit0.clear();
            unit1.clear();
            spi = transfer_write_cpu(spi, &[], write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), 4);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 3);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 4);
                }
            }
        }
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mut pin, mut pin_mirror) = hil_test::common_test_pins!(peripherals);
        let mut unconnected_pin = hil_test::unconnected_pin!(peripherals);

        let config = InputConfig::default().with_pull(Pull::Down);
        let _ = Input::new(pin.reborrow(), config);
        let _ = Input::new(pin_mirror.reborrow(), config);
        let _ = Input::new(unconnected_pin.reborrow(), config);

        let dma_channel = cfg_select! {
            spi_master_dma_engine = "SPI_DMA" => {
                peripherals.DMA_SPI2
            },
            spi_master_dma_engine = "AHB_GDMA" => {
                peripherals.DMA_CH0
            },
            spi_master_dma_engine = "AXI_GDMA" => {
                peripherals.DMA_AXI_CH0
            },
        };

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap();

        Context {
            spi,
            #[cfg(pcnt_driver_supported)]
            pcnt: peripherals.PCNT,
            dma_channel,
            gpios: [pin.into(), pin_mirror.into(), unconnected_pin.into()],
        }
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio0(pin).with_dma(ctx.dma_channel);
        execute_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_0_with_cpu(ctx: Context) {
        const BUFFER_SIZE: usize = 145;

        let [pin, pin_mirror, _] = ctx.gpios;
        let _pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let mut spi = ctx.spi.with_sio0(pin);
        let mut buffer = [0; BUFFER_SIZE];

        spi.half_duplex_read(DataMode::Quad, Command::None, Address::None, 0, &mut buffer)
            .unwrap();

        assert_eq!(buffer, [0b0001_0001; BUFFER_SIZE]);
    }

    #[test]
    async fn test_spi_reads_correctly_from_gpio_pin_0_async(ctx: Context) {
        const BUFFER_SIZE: usize = 4;

        let [pin, pin_mirror, _] = ctx.gpios;
        let mut pin_mirror = Output::new(pin_mirror, Level::Low, OutputConfig::default());

        let dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();
        let mut spi = ctx
            .spi
            .with_sio0(pin)
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        let mut buffer = [0; BUFFER_SIZE];
        spi.half_duplex_read_async(DataMode::Quad, Command::None, Address::None, 0, &mut buffer)
            .await
            .unwrap();
        assert_eq!(buffer, [0; BUFFER_SIZE]);

        pin_mirror.set_high();
        spi.half_duplex_read_async(DataMode::Quad, Command::None, Address::None, 0, &mut buffer)
            .await
            .unwrap();
        assert_eq!(buffer, [0b0001_0001; BUFFER_SIZE]);
    }

    #[test]
    async fn test_spi_reads_correctly_from_gpio_pin_0_async_with_cpu_fallback(mut ctx: Context) {
        const BUFFER_SIZE: usize = 145;

        let [pin, pin_mirror, _] = ctx.gpios;
        let _pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        ctx.spi
            .apply_config(
                &Config::default().with_min_async_transfer_size(BUFFER_SIZE.saturating_add(1)),
            )
            .unwrap();
        let mut spi = ctx
            .spi
            .with_sio0(pin)
            .with_dma(ctx.dma_channel)
            .into_async();
        let mut buffer = [0; BUFFER_SIZE];

        spi.half_duplex_read_async(DataMode::Quad, Command::None, Address::None, 0, &mut buffer)
            .await
            .unwrap();

        assert_eq!(buffer, [0b0001_0001; BUFFER_SIZE]);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio1(pin).with_dma(ctx.dma_channel);
        execute_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);
        execute_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);
        execute_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio0(pin).with_dma(ctx.dma_channel);
        execute_write_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio1(pin).with_dma(ctx.dma_channel);
        execute_write_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);
        execute_write_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());
        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);
        execute_write_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        let [_, _, mosi] = ctx.gpios;
        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx.spi.with_sio0(mosi).with_dma(ctx.dma_channel);
        execute_write(unit0, unit1, spi, 0b0000_0001, false);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    async fn test_spi_writes_correctly_to_pin_0_async(ctx: Context) {
        const BUFFER_SIZE: usize = 4;

        let [_, _, mosi] = ctx.gpios;
        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();
        let spi = ctx
            .spi
            .with_sio0(mosi)
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async();

        execute_write_async(unit0, unit1, spi, 0b0000_0001, false).await;
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x(
            ctx.spi,
            ctx.dma_channel,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio1(sio_n),
            0b0000_0010,
        );
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x(
            ctx.spi,
            ctx.dma_channel,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio2(sio_n),
            0b0000_0100,
        );
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x(
            ctx.spi,
            ctx.dma_channel,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio3(sio_n),
            0b0000_1000,
        );
    }

    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_x(
        spi: Spi<'_, Blocking>,
        dma_channel: DmaChannel0<'_>,
        pcnt: PCNT<'_>,
        gpio: AnyPin<'_>,
        mosi: AnyPin<'_>,
        set_pin: impl for<'a> FnOnce(Spi<'a, Blocking>, Flex<'a>) -> Spi<'a, Blocking>,
        write: u8,
    ) {
        let pcnt = Pcnt::new(pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        let mut sio_n = Flex::new(gpio);
        sio_n.set_input_enable(true);
        sio_n.set_output_enable(true);
        let sio3_loopback = sio_n.peripheral_input();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(sio3_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = set_pin(spi, sio_n).with_sio0(mosi).with_dma(dma_channel);
        execute_write(unit0, unit1, spi, write, true);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_0_with_cpu(ctx: Context) {
        let [_, _, mosi] = ctx.gpios;
        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx.spi.with_sio0(mosi);
        execute_write_with_cpu(unit0, unit1, spi, 0b0000_0001, false);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_1_with_cpu(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x_with_cpu(
            ctx.spi,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio1(sio_n),
            0b0000_0010,
        );
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_2_with_cpu(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x_with_cpu(
            ctx.spi,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio2(sio_n),
            0b0000_0100,
        );
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_3_with_cpu(ctx: Context) {
        let [gpio, _, mosi] = ctx.gpios;
        test_spi_writes_correctly_to_pin_x_with_cpu(
            ctx.spi,
            ctx.pcnt,
            gpio,
            mosi,
            |spi, sio_n| spi.with_sio3(sio_n),
            0b0000_1000,
        );
    }

    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_x_with_cpu(
        spi: Spi<'_, Blocking>,
        pcnt: PCNT<'_>,
        gpio: AnyPin<'_>,
        mosi: AnyPin<'_>,
        set_pin: impl for<'a> FnOnce(Spi<'a, Blocking>, Flex<'a>) -> Spi<'a, Blocking>,
        write: u8,
    ) {
        let pcnt = Pcnt::new(pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let mut mosi = Flex::new(mosi);
        mosi.set_input_enable(true);
        mosi.set_output_enable(true);
        let mosi_loopback = mosi.peripheral_input();

        let mut sio_n = Flex::new(gpio);
        sio_n.set_input_enable(true);
        sio_n.set_output_enable(true);
        let sio3_loopback = sio_n.peripheral_input();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(sio3_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = set_pin(spi, sio_n).with_sio0(mosi);
        execute_write_with_cpu(unit0, unit1, spi, write, true);
    }
}
