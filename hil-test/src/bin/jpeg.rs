//! JPEG encoding formats, subsampling, and DMA buffer handling.

//% CHIP_FILTER: jpeg_driver_supported&&psram_driver_supported
//% FEATURES: unstable psram

#![no_std]
#![no_main]

use allocator_api2::boxed::Box;
use esp_alloc::ExternalMemory;
use esp_hal::jpeg::{Config, Encoder, Error, PixelFormat, Subsampling};
use hil_test as _;

const WIDTH: u16 = 33;
const HEIGHT: u16 = 35;
const BYTES: usize = WIDTH as usize * HEIGHT as usize * 2;
// The smallest output alignment that the encoder accepts in each memory region.
const SRAM_ALIGNMENT: usize = if cfg!(soc_internal_memory_cached) {
    64
} else {
    4
};
const PSRAM_ALIGNMENT: usize = if cfg!(soc_internal_memory_cached) {
    128
} else {
    64
};

#[repr(align(128))]
struct Buffer<const N: usize>([u8; N]);

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
    #[cfg(pixel_accelerator_driver_supported)]
    fn dma2d_stays_enabled_while_a_driver_owns_channels() {
        use esp_hal::{
            peripherals::{DMA2D, HP_SYS_CLKRST},
            ppa::srm::Srm,
        };

        let clock_enabled = || {
            let clock = HP_SYS_CLKRST::regs();
            cfg_select! {
                esp32s31 => clock.dma2d_ctrl0().read().sys_clk_en().bit(),
                _ => clock.soc_clk_ctrl1().read().dma2d_sys_clk_en().bit(),
            }
        };

        let mut p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        #[cfg(esp32s31)]
        let initial = esp_hal::peripherals::HP_SYS::regs()
            .dma2d_mem_lp_ctrl()
            .read();
        let jpeg = Encoder::new(p.JPEG, p.DMA2D_TX_CH0, p.DMA2D_RX_CH0);
        assert!(clock_enabled());
        #[cfg(esp32s31)]
        {
            let powered = esp_hal::peripherals::HP_SYS::regs()
                .dma2d_mem_lp_ctrl()
                .read();
            assert_eq!(
                powered._2ddma_mem_force_ctrl().bit(),
                initial._2ddma_mem_force_ctrl().bit()
            );
            assert!(!powered._2ddma_mem_lp_en().bit());
        }
        let dma = DMA2D::regs();
        let jpeg_link = dma.out_link_addr_ch(0);
        jpeg_link.write(|w| unsafe { w.outlink_addr_ch().bits(0x100) });
        let srm = Srm::new(
            p.PPA.reborrow(),
            p.DMA2D_TX_CH1.reborrow(),
            p.DMA2D_RX_CH2.reborrow(),
        );
        assert_eq!(jpeg_link.read().outlink_addr_ch().bits(), 0x100);
        drop(srm);
        assert!(clock_enabled());
        assert_eq!(jpeg_link.read().outlink_addr_ch().bits(), 0x100);

        let srm = Srm::new(p.PPA, p.DMA2D_TX_CH1, p.DMA2D_RX_CH2);
        let srm_link = dma.out_link_addr_ch(1);
        srm_link.write(|w| unsafe { w.outlink_addr_ch().bits(0x200) });
        drop(jpeg);
        assert!(clock_enabled());
        assert_eq!(srm_link.read().outlink_addr_ch().bits(), 0x200);
        drop(srm);
        assert!(!clock_enabled());
    }

    #[test]
    fn psram_encode_after_output_exhaustion() {
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
        let mut jpeg = Encoder::new(p.JPEG, p.DMA2D_TX_CH0, p.DMA2D_RX_CH1);

        let mut input = Buffer([0; BYTES]);
        let mut state = 1_u32;
        for pixel in input.0.as_chunks_mut::<2>().0 {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            *pixel = ((state >> 16) as u16).to_le_bytes();
        }
        let config = Config {
            width: WIDTH,
            height: HEIGHT,
            quality: 90,
            input_format: PixelFormat::Rgb565Le,
            subsampling: Subsampling::Yuv420,
        };
        // Exercise SRAM output with the smallest alignment.
        let mut reference = Buffer([0xa5; 4096]);
        let (guard, sram) = reference.0.split_at_mut(SRAM_ALIGNMENT);
        let size = jpeg.encode(&input.0, sram, config).unwrap();
        assert!(guard.iter().all(|&byte| byte == 0xa5));
        let jpeg_bytes = &sram[..size];

        let mut source = Box::new_in(Buffer([0; BYTES + 2]), ExternalMemory);
        let mut output = Box::new_in(Buffer([0; 4096]), ExternalMemory);
        let psram_input = &mut source.0[2..];
        psram_input.copy_from_slice(&input.0);
        let capacity = size.next_multiple_of(PSRAM_ALIGNMENT);
        let mut encode = |output: &mut [u8]| jpeg.encode(psram_input, output, config);
        assert_eq!(
            encode(&mut output.0[..capacity - PSRAM_ALIGNMENT]),
            Err(Error::OutputTooSmall)
        );
        assert_eq!(encode(&mut output.0[..capacity]), Ok(size));
        assert!(jpeg_bytes == &output.0[..size]);
        assert!(
            jpeg_bytes
                .windows(12)
                .any(|bytes| bytes == [0xff, 0xc0, 0, 17, 8, 0, 35, 0, 33, 3, 1, 0x22])
        );
        assert!(jpeg_bytes.starts_with(&[0xff, 0xd8]));
        assert!(jpeg_bytes.ends_with(&[0xff, 0xd9]));
    }

    #[test]
    fn encode_subsampling() {
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let mut jpeg = Encoder::new(p.JPEG, p.DMA2D_TX_CH0, p.DMA2D_RX_CH0);
        let input = Buffer([0x81; BYTES]);
        let mut output = Buffer([0; 4096]);
        let config = Config {
            width: WIDTH,
            height: HEIGHT,
            quality: 90,
            input_format: PixelFormat::Gray8,
            subsampling: Subsampling::Yuv444,
        };
        assert_eq!(
            jpeg.encode(&input.0, &mut output.0, config),
            Err(Error::UnsupportedFormat)
        );
        for (format, subsampling, components, factor) in [
            (PixelFormat::Rgb565Le, Subsampling::Yuv444, 3, 0x11),
            (PixelFormat::Gray8, Subsampling::Grayscale, 1, 0x11),
            (PixelFormat::Rgb565Le, Subsampling::Yuv422, 3, 0x21),
        ] {
            let config = Config {
                input_format: format,
                subsampling,
                ..config
            };
            let size = jpeg.encode(&input.0, &mut output.0, config).unwrap();
            let encoded = &output.0[..size];
            let sof = [
                0xff,
                0xc0,
                0,
                8 + components * 3,
                8,
                0,
                HEIGHT as u8,
                0,
                WIDTH as u8,
                components,
                1,
                factor,
            ];
            assert!(
                encoded.windows(sof.len()).any(|bytes| bytes == sof),
                "{format:?} {subsampling:?}"
            );
            assert!(encoded.ends_with(&[0xff, 0xd9]));
        }
    }

    #[test]
    fn encode_565_byte_orders() {
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let mut jpeg = Encoder::new(p.JPEG, p.DMA2D_TX_CH0, p.DMA2D_RX_CH0);
        let mut little_endian = Buffer([0; BYTES]);
        let mut big_endian = Buffer([0; BYTES]);
        for (index, (little, big)) in little_endian
            .0
            .as_chunks_mut::<2>()
            .0
            .iter_mut()
            .zip(big_endian.0.as_chunks_mut::<2>().0)
            .enumerate()
        {
            let pixel = (index as u16).wrapping_mul(73);
            *little = pixel.to_le_bytes();
            *big = pixel.to_be_bytes();
        }

        let mut reference = Buffer([0; 4096]);
        let mut output = Buffer([0; 4096]);
        for (little, big) in [
            (PixelFormat::Rgb565Le, PixelFormat::Rgb565Be),
            (PixelFormat::Bgr565Le, PixelFormat::Bgr565Be),
        ] {
            let config = Config {
                width: WIDTH,
                height: HEIGHT,
                quality: 90,
                input_format: little,
                subsampling: Subsampling::Yuv444,
            };
            let size = jpeg
                .encode(&little_endian.0, &mut reference.0, config)
                .unwrap();
            for (format, input) in [(big, &big_endian.0), (little, &little_endian.0)] {
                let config = Config {
                    input_format: format,
                    ..config
                };
                assert_eq!(
                    jpeg.encode(input, &mut output.0, config),
                    Ok(size),
                    "{format:?}"
                );
                assert!(output.0[..size] == reference.0[..size], "{format:?}");
            }
        }
    }

    #[test]
    fn packed_ycbcr422_downsamples_to_ycbcr420() {
        const SIDE: u16 = 16;
        const PIXELS: usize = SIDE as usize * SIDE as usize;
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let mut jpeg = Encoder::new(p.JPEG, p.DMA2D_TX_CH0, p.DMA2D_RX_CH0);
        let config = Config {
            width: SIDE,
            height: SIDE,
            quality: 90,
            input_format: PixelFormat::Yuv420Packed,
            subsampling: Subsampling::Yuv420,
        };

        let mut uyvy = Buffer([0; PIXELS * 2]);
        let mut yuv420 = Buffer([0; PIXELS * 3 / 2]);
        for (pair, (uyvy, yuv420)) in uyvy
            .0
            .as_chunks_mut::<4>()
            .0
            .iter_mut()
            .zip(yuv420.0.as_chunks_mut::<3>().0)
            .enumerate()
        {
            let y0 = 32 + pair as u8;
            let y1 = y0 + 16;
            let row = pair / (SIDE as usize / 2);
            *uyvy = [90, y0, 175, y1];
            *yuv420 = [if row.is_multiple_of(2) { 90 } else { 175 }, y0, y1];
        }

        let mut reference = Buffer([0; 1024]);
        let size = jpeg.encode(&yuv420.0, &mut reference.0, config).unwrap();

        let mut output = Buffer([0; 1024]);
        let result = jpeg.encode(
            &uyvy.0,
            &mut output.0,
            Config {
                input_format: PixelFormat::Uyvy422,
                ..config
            },
        );
        assert_eq!(result, Ok(size));
        assert!(output.0[..size] == reference.0[..size]);
    }
}
