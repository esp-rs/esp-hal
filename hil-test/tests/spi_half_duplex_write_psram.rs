//! SPI Half Duplex Write Test

//% CHIPS: esp32s3
//% FEATURES: unstable octal-psram

#![no_std]
#![no_main]
use defmt::error;
use esp_alloc as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf, ExternalBurstConfig},
    dma_buffers,
    dma_descriptors_chunk_size,
    gpio::interconnect::InputSignal,
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
    spi::{
        master::{Address, Command, Config, Spi, SpiDma},
        DataMode,
        Mode,
    },
    time::RateExtU32,
    Blocking,
};
use hil_test as _;
extern crate alloc;

macro_rules! dma_alloc_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        }
    }};
}

struct Context {
    spi: SpiDma<'static, Blocking>,
    pcnt_unit: Unit<'static, 0>,
    pcnt_source: InputSignal,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

        let sclk = peripherals.GPIO0;
        let (mosi, _) = hil_test::common_test_pins!(peripherals);

        let pcnt = Pcnt::new(peripherals.PCNT);

        let dma_channel = peripherals.DMA_CH0;

        let (mosi_loopback, mosi) = mosi.split();

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(100.kHz())
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
        const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;

        let (_, descriptors) = dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
        let buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
        let mut dma_tx_buf = DmaTxBuf::new_with_config(descriptors, buffer, DMA_ALIGNMENT).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);
        let transfer = spi
            .half_duplex_write(
                DataMode::FourWire,
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
            .half_duplex_write(
                DataMode::FourWire,
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
        const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size32; // matches dcache line size
        const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize; // 64 byte aligned

        let (_, descriptors) = dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
        let buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
        let dma_tx_buf = DmaTxBuf::new_with_config(descriptors, buffer, DMA_ALIGNMENT).unwrap();

        let (rx, rxd, _, _) = dma_buffers!(1, 0);
        let dma_rx_buf = DmaRxBuf::new(rxd, rx).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_buffers(dma_rx_buf, dma_tx_buf);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
        // Write the buffer where each byte has 3 pos edges.
        spi.half_duplex_write(DataMode::FourWire, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        spi.half_duplex_write(DataMode::FourWire, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }
}
