//! SPI DMA loopback test with PSRAM TX buffer
//!
//! Sends data from PSRAM via SPI DMA and receives it back into internal RAM
//! (MOSI looped back to MISO). Tests that DMA descriptor burst alignment and
//! cache writeback for PSRAM buffers produce no data corruption.

//% CHIP_FILTER: dma_can_access_psram
//% FEATURES: unstable esp-alloc

#![no_std]
#![no_main]

extern crate alloc;

use defmt::error;
use esp_alloc as _;
use esp_hal::{
    Blocking,
    dma::ExternalBurstConfig,
    dma_rx_buffer,
    spi::{
        Mode,
        master::{Config, Spi, SpiDma},
    },
    time::Rate,
};
use hil_test as _;

macro_rules! dma_alloc_tx_buffer {
    ($size:expr, $align:expr) => {{
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

struct Context {
    spi: SpiDma<'static, Blocking>,
}

#[embedded_test::tests(default_timeout = 10)]
mod tests {
    use super::*;

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

    /// SPI DMA loopback with TX from PSRAM, ExternalBurstConfig::Size64.
    #[test]
    fn test_psram_tx_loopback_size64(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8192;
        const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;

        let mut dma_tx_buf = dma_alloc_tx_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT).unwrap();
        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();

        for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *v = (i % 256) as u8;
        }

        let mut spi = ctx.spi;

        for i in 0..4u8 {
            dma_tx_buf.as_mut_slice()[0] = i;
            *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i;

            let transfer = spi
                .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();

            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

            for j in 0..DMA_BUFFER_SIZE {
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

    /// SPI DMA loopback with TX from PSRAM, ExternalBurstConfig::Size32.
    #[test]
    fn test_psram_tx_loopback_size32(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 8192;
        const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size32;

        let mut dma_tx_buf = dma_alloc_tx_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT).unwrap();
        let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();

        for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *v = (i % 256) as u8;
        }

        let mut spi = ctx.spi;

        for i in 0..4u8 {
            dma_tx_buf.as_mut_slice()[0] = i;
            *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i;

            let transfer = spi
                .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();

            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();

            for j in 0..DMA_BUFFER_SIZE {
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
}
