//! SPI Half Duplex Write Test
//% FEATURES: psram
//% CHIPS: esp32s3

#![no_std]
#![no_main]
use esp_alloc as _;
use esp_hal::{
    dma::{Dma, DmaBufBlkSize, DmaPriority, DmaTxBuf},
    dma_descriptors_chunk_size,
    gpio::{interconnect::InputSignal, Io},
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{Address, Command, Spi, SpiDma},
        HalfDuplexMode,
        SpiDataMode,
        SpiMode,
    },
    Blocking,
};
use hil_test as _;
extern crate alloc;

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "esp32",
        feature = "esp32s2",
    ))] {
        use esp_hal::dma::Spi2DmaChannel as DmaChannel0;
    } else {
        use esp_hal::dma::DmaChannel0;
    }
}

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
    spi: SpiDma<'static, SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    pcnt_unit: Unit<'static, 0>,
    pcnt_source: InputSignal,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    // defmt::* is load-bearing, it ensures that the assert in dma_buffers! is not
    // using defmt's non-const assert. Doing so would result in a compile error.
    #[allow(unused_imports)]
    use defmt::{assert_eq, *};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let (mosi, _) = hil_test::common_test_pins!(io);

        let pcnt = Pcnt::new(peripherals.PCNT);
        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let mosi_loopback = mosi.peripheral_input();

        let spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            pcnt_source: mosi_loopback,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 64;
        const DMA_CHUNK_SIZE: usize = 4032; // 64 byte aligned
        const DMA_ALIGNMENT: DmaBufBlkSize = DmaBufBlkSize::Size32; // matches dcache line size

        let (_, descriptors) = dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
        let buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
        let mut dma_tx_buf =
            DmaTxBuf::new_with_chunk_size(descriptors, buffer, DMA_CHUNK_SIZE, Some(DMA_ALIGNMENT))
                .unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
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

    // TODO: when the first test is working, we can add a second one!
    // #[test]
    // #[timeout(3)]
    // fn test_spidmabus_writes_are_correctly_by_pcnt(ctx: Context) {
    //     const DMA_BUFFER_SIZE: usize = 64;

    //     let (rx, rxd, buffer, descriptors) = dma_buffers!(1,
    // DMA_BUFFER_SIZE);     let dma_rx_buf = DmaRxBuf::new(rxd,
    // rx).unwrap();     let dma_tx_buf = DmaTxBuf::new(descriptors,
    // buffer).unwrap();

    //     let unit = ctx.pcnt_unit;
    //     let mut spi = ctx.spi.with_buffers(dma_rx_buf, dma_tx_buf);

    //     unit.channel0.set_edge_signal(ctx.pcnt_source);
    //     unit.channel0
    //         .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

    //     let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
    //     // Write the buffer where each byte has 3 pos edges.
    //     spi.write(
    //         SpiDataMode::Single,
    //         Command::None,
    //         Address::None,
    //         0,
    //         &buffer,
    //     )
    //     .unwrap();

    //     assert_eq!(unit.get_value(), (3 * DMA_BUFFER_SIZE) as _);

    //     spi.write(
    //         SpiDataMode::Single,
    //         Command::None,
    //         Address::None,
    //         0,
    //         &buffer,
    //     )
    //     .unwrap();

    //     assert_eq!(unit.get_value(), (6 * DMA_BUFFER_SIZE) as _);
    // }
}
