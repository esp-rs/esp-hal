use super::Error;

/// Pixel layout of the uncompressed input image.
///
/// YCbCr inputs are interpreted as full-range samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum PixelFormat {
    /// Little-endian RGB565.
    Rgb565Le,
    /// Big-endian RGB565.
    Rgb565Be,
    /// Little-endian BGR565.
    Bgr565Le,
    /// Big-endian BGR565.
    Bgr565Be,
    /// `R, G, B` bytes per pixel.
    Rgb888,
    /// `B, G, R` bytes per pixel.
    Bgr888,
    /// 8-bit grayscale.
    Gray8,
    /// `Y, Cb, Cr` bytes per pixel.
    Yuv444,
    /// `Cr, Cb, Y` bytes per pixel.
    Vuy444,
    /// `Y0, Cb, Y1, Cr` bytes per pair of pixels.
    ///
    /// The image width must be even.
    Yuyv422,
    /// `Cb, Y0, Cr, Y1` bytes per pair of pixels.
    ///
    /// The image width must be even.
    Uyvy422,
    /// Packed YCbCr 4:2:0.
    ///
    /// Each 2x2 block stores `Cb, Y00, Y01` in the first row and `Cr, Y10, Y11` in the second.
    /// The image width and height must be even.
    Yuv420Packed,
}

/// Chroma subsampling of the encoded JPEG.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Subsampling {
    /// YCbCr 4:4:4.
    Yuv444,
    /// YCbCr 4:2:2.
    Yuv422,
    /// YCbCr 4:2:0.
    Yuv420,
    /// Grayscale.
    Grayscale,
}

pub(super) struct Sampling {
    luma_blocks: (u8, u8),
    pub(super) has_chroma: bool,
    pub(super) sample_sel: u8,
    pub(super) dma_macro_block_size: u8,
}

impl Sampling {
    pub(super) fn new(subsampling: Subsampling) -> Self {
        let (luma_blocks, sample_sel, dma_macro_block_size) = match subsampling {
            Subsampling::Yuv444 | Subsampling::Grayscale => ((1, 1), 0, 0),
            Subsampling::Yuv422 => ((2, 1), 1, 1),
            Subsampling::Yuv420 => ((2, 2), 2, 2),
        };
        Self {
            luma_blocks,
            has_chroma: subsampling != Subsampling::Grayscale,
            sample_sel,
            dma_macro_block_size,
        }
    }

    pub(super) fn mcu_size(&self) -> (u32, u32) {
        let (horizontal, vertical) = self.luma_blocks;
        (8 * u32::from(horizontal), 8 * u32::from(vertical))
    }

    pub(super) fn blocks_per_mcu(&self) -> usize {
        let (horizontal, vertical) = self.luma_blocks;
        usize::from(horizontal) * usize::from(vertical) + 2 * usize::from(self.has_chroma)
    }

    pub(super) fn luma_sampling_factor(&self) -> u8 {
        let (horizontal, vertical) = self.luma_blocks;
        horizontal << 4 | vertical
    }
}

#[derive(Clone, Copy)]
pub(super) enum ColorSpace {
    Rgb888   = 0,
    Ycbcr422 = 1,
    Rgb565   = 2,
    Gray     = 3,
    Ycbcr444 = 4,
    Ycbcr420 = 5,
}

pub(super) struct InputLayout {
    pub(super) color_space: ColorSpace,
    pub(super) pixel_reverse: bool,
    pub(super) dma_swap_bytes: bool,
    pub(super) dma_pbyte: u32,
    pub(super) dma_block_width: u32,
    pub(super) len: usize,
    pub(super) alignment: usize,
}

impl InputLayout {
    pub(super) fn new(
        format: PixelFormat,
        subsampling: Subsampling,
        width: usize,
        height: usize,
    ) -> Result<Self, Error> {
        let (color_space, pixel_reverse) = match format {
            PixelFormat::Rgb565Le | PixelFormat::Rgb565Be => (ColorSpace::Rgb565, false),
            PixelFormat::Bgr565Le | PixelFormat::Bgr565Be => (ColorSpace::Rgb565, true),
            PixelFormat::Rgb888 => (ColorSpace::Rgb888, true),
            PixelFormat::Bgr888 => (ColorSpace::Rgb888, false),
            PixelFormat::Gray8 => (ColorSpace::Gray, false),
            PixelFormat::Yuv444 => (ColorSpace::Ycbcr444, true),
            PixelFormat::Vuy444 => (ColorSpace::Ycbcr444, false),
            PixelFormat::Yuyv422 => (ColorSpace::Ycbcr422, true),
            PixelFormat::Uyvy422 => (ColorSpace::Ycbcr422, false),
            PixelFormat::Yuv420Packed => (ColorSpace::Ycbcr420, false),
        };

        use ColorSpace::*;
        use Subsampling::*;
        // TX block widths are in pixels.
        // Based on ESP32-S31 TRM v0.2 Table 4.4-4 (p. 245)
        // and ESP32-P4 TRM v1.0 Table 8.4-4 (p. 838).
        let dma_block_width = match (color_space, subsampling) {
            (Rgb888 | Ycbcr444, Yuv444) => 40,
            (Rgb888 | Ycbcr444, Yuv422 | Yuv420) => 32,
            (Rgb565, Yuv444) | (Rgb565 | Ycbcr422, Yuv422) => 64,
            // Packed 4:2:0 uses the same TX block width as RGB565 with 4:2:0 output.
            // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_driver_jpeg/jpeg_param.c#L62>.
            (Rgb565 | Ycbcr422 | Ycbcr420, Yuv420) => 48,
            (Gray, Grayscale) => 128,
            _ => return Err(Error::UnsupportedFormat),
        };

        let pixels = width * height;
        let (dma_pbyte, len, alignment) = match color_space {
            Ycbcr422 if !width.is_multiple_of(2) => return Err(Error::InvalidInput),
            Ycbcr420 if !width.is_multiple_of(2) || !height.is_multiple_of(2) => {
                return Err(Error::InvalidInput);
            }
            Gray => (1, pixels, 1),
            Ycbcr420 => (2, pixels * 3 / 2, 1),
            Rgb565 | Ycbcr422 => (3, pixels * 2, 2),
            Rgb888 | Ycbcr444 => (4, pixels * 3, 1),
        };
        Ok(Self {
            color_space,
            pixel_reverse,
            dma_swap_bytes: matches!(format, PixelFormat::Rgb565Be | PixelFormat::Bgr565Be),
            dma_pbyte,
            dma_block_width,
            alignment,
            len,
        })
    }
}
