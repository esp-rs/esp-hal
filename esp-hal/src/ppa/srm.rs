#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Scaling, Rotation, and Mirroring (SRM)
//!
//! ## Overview
//!
//! The SRM engine scales, rotates, mirrors, and converts images using the pixel
//! processing accelerator (PPA) and 2D-DMA.
//!
//! ## Configuration
//!
//! [`Config`] selects the transformation. [`Input`] and [`Output`] describe the images.
//!
//! Image buffers must meet the following requirements:
//!
//! - Each buffer must contain at least `width * height * format.bytes_per_pixel()` bytes for its
//!   image size.
//! - Buffers must be entirely within internal RAM or initialized PSRAM.
//! - PSRAM destination addresses and buffer lengths must be multiples of 64 bytes.
//!
//! PSRAM encryption and ECC configurations requiring strictly aligned memory transactions are
//! not supported.
//!
//! ## Examples
//!
//! ### Rotating an RGB565 image
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::ppa::srm::{Config, Input, Offset, Output, PixelFormat, Rotation, Size, Srm};
//!
//! let source = [0; 17 * 4 * 2];
//! let mut destination = [0; 17 * 4 * 2];
//! let size = Size {
//!     width: 17,
//!     height: 4,
//! };
//! let config = Config::default().with_rotation(Rotation::Degrees90);
//! let mut srm = Srm::new(
//!     peripherals.PPA,
//!     peripherals.DMA2D_TX_CH0,
//!     peripherals.DMA2D_RX_CH0,
//! );
//! srm.run(
//!     Input {
//!         data: &source,
//!         size,
//!         format: PixelFormat::Rgb565,
//!     },
//!     Output {
//!         data: &mut destination,
//!         size: config.output_size(size)?,
//!         format: PixelFormat::Rgb565,
//!         offset: Offset::default(),
//!     },
//!     config,
//! )?;
//! # {after_snippet}
//! ```

use core::marker::PhantomData;

use crate::{
    Blocking,
    DriverMode,
    dma2d::{
        AnyDma2dRxChannel,
        AnyDma2dTxChannel,
        BURST_128_BYTES,
        CACHE_LINE_SIZE,
        Channels,
        MACRO_BLOCK_NONE,
        writeback,
    },
    peripherals::{DMA2D, HP_SYS, PPA},
    soc::{is_slice_in_dram, is_slice_in_psram},
    system::{GenericPeripheralGuard, Peripheral, PeripheralClockControl},
};

const MAX_DIMENSION: usize = 0x3fff;
// SRM output dimensions are limited to 13 bits.
const MAX_SRM_DIMENSION: usize = 0x1fff;
const SRM_TRIGGER: u8 = 1;
const INPUT_BLOCK_HEIGHT: u16 = 34;
const OUTPUT_BLOCK_SIZE: Size = Size {
    width: 2,
    height: 2,
};

/// Image dimensions in pixels.
///
/// Both dimensions must be nonzero and no greater than `0x3fff`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Size {
    /// Width in pixels.
    pub width: usize,
    /// Height in pixels.
    pub height: usize,
}

impl Size {
    fn is_valid(self, max_dimension: usize) -> bool {
        self.width != 0
            && self.height != 0
            && self.width <= max_dimension
            && self.height <= max_dimension
    }
}

/// Position within a destination image, in pixels.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Offset {
    /// Horizontal offset in pixels.
    pub x: usize,
    /// Vertical offset in pixels.
    pub y: usize,
}

/// Pixel formats supported by the PPA SRM operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum PixelFormat {
    /// Packed UYVY 4:2:2, using limited-range BT.601 color conversion.
    ///
    /// Input widths must be even. Output widths, horizontal offsets, and transformed region
    /// widths must be even.
    Uyvy422,
    /// RGB565 in native byte order.
    ///
    /// Conversion to 8-bit channels fills the least significant bits with zeros.
    Rgb565,
    /// RGB888, stored as B, G, R bytes.
    Rgb888,
    /// ARGB8888 in native byte order, stored as B, G, R, A bytes.
    ///
    /// Input alpha is preserved. Inputs without alpha are treated as opaque.
    Argb8888,
    /// One byte per pixel, using `(85 * R + 86 * G + 85 * B) >> 8` for RGB input.
    Gray8,
}

impl PixelFormat {
    /// Returns the number of bytes per pixel.
    #[instability::unstable]
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Gray8 => 1,
            Self::Rgb565 | Self::Uyvy422 => 2,
            Self::Rgb888 => 3,
            Self::Argb8888 => 4,
        }
    }

    fn dma_pixel_size(self) -> u32 {
        match self {
            Self::Gray8 => 1,
            Self::Rgb565 | Self::Uyvy422 => 3,
            Self::Rgb888 => 4,
            Self::Argb8888 => 5,
        }
    }

    fn color_mode(self) -> u8 {
        match self {
            Self::Uyvy422 => 9,
            Self::Rgb565 => 2,
            Self::Rgb888 => 1,
            Self::Argb8888 => 0,
            Self::Gray8 => 12,
        }
    }

    fn input_block_width(self) -> u16 {
        // A 32-pixel SRM block uses an input block width of 34 pixels,
        // or 36 for packed YUV422.
        // See <https://github.com/espressif/esp-idf/blob/ce2100de51a4d9a372f26578fd2ee763ca4b1279/components/esp_hal_ppa/esp32s31/include/hal/ppa_ll.h#L586-L605>.
        match self {
            Self::Uyvy422 => 36,
            Self::Rgb565 | Self::Rgb888 | Self::Argb8888 | Self::Gray8 => 34,
        }
    }
}

/// Counterclockwise image rotation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Rotation {
    /// No rotation.
    #[default]
    Degrees0,
    /// 90 degrees.
    Degrees90,
    /// 180 degrees.
    Degrees180,
    /// 270 degrees.
    Degrees270,
}

impl Rotation {
    fn angle(self) -> u8 {
        match self {
            Self::Degrees0 => 0,
            Self::Degrees90 => 1,
            Self::Degrees180 => 2,
            Self::Degrees270 => 3,
        }
    }

    fn output_size(self, size: Size) -> Size {
        match self {
            Self::Degrees0 | Self::Degrees180 => size,
            Self::Degrees90 | Self::Degrees270 => Size {
                width: size.height,
                height: size.width,
            },
        }
    }
}

/// Configuration for a PPA SRM operation.
///
/// Scaling uses the source axes and bilinear interpolation. Factors must be at least `1/16`
/// and less than `256`, and are truncated to multiples of `1/16`.
/// Rotation follows scaling. Mirroring uses the rotated image's axes.
/// Each transformed dimension must be no greater than `0x1fff`.
#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct Config {
    /// Rotation applied to the source image.
    rotation: Rotation,
    /// Horizontal scaling factor. See [`Config`] for the range and precision.
    scale_x: f32,
    /// Vertical scaling factor. See [`Config`] for the range and precision.
    scale_y: f32,
    /// Horizontal mirroring.
    mirror_x: bool,
    /// Vertical mirroring.
    mirror_y: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rotation: Rotation::default(),
            scale_x: 1.0,
            scale_y: 1.0,
            mirror_x: false,
            mirror_y: false,
        }
    }
}

impl Config {
    /// Returns the transformed image dimensions.
    ///
    /// Scaled dimensions are rounded down to whole pixels and exchanged for 90-degree and
    /// 270-degree rotations.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidScale`] when a scaling factor is invalid.
    /// - [`Error::InvalidSize`] when the source or transformed dimensions are invalid.
    #[instability::unstable]
    pub fn output_size(&self, size: Size) -> Result<Size, Error> {
        if !size.is_valid(MAX_DIMENSION) {
            return Err(Error::InvalidSize);
        }
        if !(1.0 / 16.0..256.0).contains(&self.scale_x)
            || !(1.0 / 16.0..256.0).contains(&self.scale_y)
        {
            return Err(Error::InvalidScale);
        }
        let size = self.rotation.output_size(Size {
            width: size.width * (self.scale_x * 16.0) as usize / 16,
            height: size.height * (self.scale_y * 16.0) as usize / 16,
        });
        if !size.is_valid(MAX_SRM_DIMENSION) {
            return Err(Error::InvalidSize);
        }
        Ok(size)
    }
}

/// Source image for an SRM operation.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Input<'a> {
    /// Source pixels. See the [buffer requirements][self].
    pub data: &'a [u8],
    /// Source image dimensions.
    pub size: Size,
    /// Pixel format.
    pub format: PixelFormat,
}

/// Destination image for an SRM operation.
///
/// The transformed image must fit within [`Self::size`] at [`Self::offset`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Output<'a> {
    /// Destination pixels. See the [buffer requirements][self].
    pub data: &'a mut [u8],
    /// Full destination image dimensions.
    pub size: Size,
    /// Pixel format.
    pub format: PixelFormat,
    /// Position of the converted image within the destination, in pixels.
    pub offset: Offset,
}

/// Errors returned by the PPA SRM operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Error {
    /// Invalid image dimensions, destination offset, or buffer length.
    InvalidSize,
    /// A scaling factor is outside the supported range or is NaN.
    InvalidScale,
    /// A PSRAM destination address or length is not 64-byte aligned.
    UnalignedBuffer,
    /// A buffer is outside internal RAM or initialized PSRAM.
    UnsupportedMemoryRegion,
    /// A 2D-DMA or PPA operation error.
    Hardware,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidSize => f.write_str("invalid image dimensions, offset, or buffer length"),
            Self::InvalidScale => f.write_str("invalid scaling factor"),
            Self::UnalignedBuffer => {
                f.write_str("PSRAM destination address or length is not 64-byte aligned")
            }
            Self::UnsupportedMemoryRegion => {
                f.write_str("image buffer is outside internal RAM or initialized PSRAM")
            }
            Self::Hardware => f.write_str("SRM hardware error"),
        }
    }
}

/// PPA SRM driver.
///
/// The driver owns the PPA and one DMA2D channel in each direction.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Srm<'d, Dm: DriverMode> {
    _ppa: PPA<'d>,
    channels: Channels<'d>,
    ppa_mem_lp: (bool, bool),
    _ppa_guard: GenericPeripheralGuard<{ Peripheral::Ppa as u8 }>,
    _mode: PhantomData<Dm>,
}

impl<'d> Srm<'d, Blocking> {
    /// Creates a blocking SRM driver with a transmit and receive DMA2D channel.
    #[instability::unstable]
    pub fn new(
        ppa: PPA<'d>,
        tx: impl Into<AnyDma2dTxChannel<'d>>,
        rx: impl Into<AnyDma2dRxChannel<'d>>,
    ) -> Self {
        let channels = Channels::new(tx, rx);
        let ppa_guard = GenericPeripheralGuard::new();
        let memory = HP_SYS::regs().ppa_mem_lp_ctrl();
        let previous = memory.read();
        memory.modify(|_, w| {
            w.ppa_sprf_mem_force_ctrl().set_bit();
            w.ppa_sprf_mem_lp_en().clear_bit()
        });
        Self {
            _ppa: ppa,
            channels,
            ppa_mem_lp: (
                previous.ppa_sprf_mem_force_ctrl().bit(),
                previous.ppa_sprf_mem_lp_en().bit(),
            ),
            _ppa_guard: ppa_guard,
            _mode: PhantomData,
        }
    }
}

impl<Dm: DriverMode> Srm<'_, Dm> {
    /// Scales, rotates, mirrors, and converts a source image into a destination region.
    ///
    /// The requirements for images and transformations are in [`Input`], [`Output`], and
    /// [`Config`]. Buffers must also meet the [module-level requirements][self].
    ///
    /// On success, pixels outside the destination region are unchanged.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidSize`] when dimensions, offsets, or buffer lengths are invalid.
    /// - [`Error::InvalidScale`] when a scaling factor is invalid.
    /// - [`Error::UnalignedBuffer`] when a PSRAM destination address or length is not 64-byte
    ///   aligned.
    /// - [`Error::UnsupportedMemoryRegion`] when a buffer is outside the supported memory regions.
    /// - [`Error::Hardware`] on a peripheral error.
    ///
    /// The destination contents are unspecified after a hardware error.
    #[instability::unstable]
    pub fn run(
        &mut self,
        input: Input<'_>,
        output: Output<'_>,
        config: Config,
    ) -> Result<(), Error> {
        let transformed = config.output_size(input.size)?;
        if !output.size.is_valid(MAX_DIMENSION)
            || input.data.len()
                < input.size.width * input.size.height * input.format.bytes_per_pixel()
            || output.data.len()
                < output.size.width * output.size.height * output.format.bytes_per_pixel()
        {
            return Err(Error::InvalidSize);
        }
        if input.format == PixelFormat::Uyvy422 && !input.size.width.is_multiple_of(2) {
            return Err(Error::InvalidSize);
        }
        if output.format == PixelFormat::Uyvy422
            && (!output.size.width.is_multiple_of(2)
                || !output.offset.x.is_multiple_of(2)
                || !transformed.width.is_multiple_of(2))
        {
            return Err(Error::InvalidSize);
        }
        if output.offset.x > output.size.width
            || output.offset.y > output.size.height
            || transformed.width > output.size.width - output.offset.x
            || transformed.height > output.size.height - output.offset.y
        {
            return Err(Error::InvalidSize);
        }
        if !(is_slice_in_dram(input.data) || is_slice_in_psram(input.data))
            || !(is_slice_in_dram(output.data) || is_slice_in_psram(output.data))
        {
            return Err(Error::UnsupportedMemoryRegion);
        }
        if is_slice_in_psram(output.data)
            && (!(output.data.as_ptr() as usize).is_multiple_of(CACHE_LINE_SIZE)
                || !output.data.len().is_multiple_of(CACHE_LINE_SIZE))
        {
            return Err(Error::UnalignedBuffer);
        }

        let mut tx = Descriptor::new(
            input.data.as_ptr(),
            input.size,
            input.size,
            Offset::default(),
            input.format,
        );
        let mut rx = Descriptor::new(
            output.data.as_mut_ptr(),
            output.size,
            OUTPUT_BLOCK_SIZE,
            output.offset,
            output.format,
        );
        let dma = DMA2D::regs();
        let (tx_channel, rx_channel) = self.channels.indices();
        let ppa = PPA::regs();

        self.reset_for_transfer();

        self.channels.connect(SRM_TRIGGER);
        dma.out_conf0_ch(tx_channel).modify(|_, w| unsafe {
            w.out_dscr_port_en_ch().set_bit();
            w.out_page_bound_en_ch().set_bit();
            w.out_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.out_macro_block_size_ch().bits(MACRO_BLOCK_NONE);
            w.outdscr_burst_en_ch().set_bit()
        });
        dma.in_conf0_ch(rx_channel).modify(|_, w| unsafe {
            w.in_dscr_port_en_ch().set_bit();
            w.in_page_bound_en_ch().set_bit();
            w.in_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.in_macro_block_size_ch().bits(MACRO_BLOCK_NONE);
            w.indscr_burst_en_ch().set_bit()
        });
        dma.out_dscr_port_blk_ch(tx_channel).write(|w| unsafe {
            w.out_dscr_port_blk_h_ch()
                .bits(input.format.input_block_width());
            w.out_dscr_port_blk_v_ch().bits(INPUT_BLOCK_HEIGHT)
        });
        ppa.srm_byte_order()
            .write(|w| w.srm_bk_size_sel().clear_bit());
        ppa.srm_color_mode().write(|w| unsafe {
            w.srm_rx_cm().bits(input.format.color_mode());
            w.srm_tx_cm().bits(output.format.color_mode());
            w.yuv_rx_range().clear_bit();
            w.yuv2rgb_protocal().clear_bit();
            w.yuv_tx_range().clear_bit();
            w.rgb2yuv_protocal().clear_bit();
            w.yuv422_rx_byte_order().bits(0)
        });
        ppa.srm_fix_alpha().reset();
        ppa.rgb2gray().reset();
        let scale_x = (config.scale_x * 16.0) as u16;
        let scale_y = (config.scale_y * 16.0) as u16;
        ppa.srm_scal_rotate().write(|w| unsafe {
            w.srm_scal_x_int().bits((scale_x >> 4) as u8);
            w.srm_scal_x_frag().bits((scale_x & 15) as u8);
            w.srm_scal_y_int().bits((scale_y >> 4) as u8);
            w.srm_scal_y_frag().bits((scale_y & 15) as u8);
            w.srm_rotate_angle().bits(config.rotation.angle());
            w.srm_mirror_x().bit(config.mirror_x);
            w.srm_mirror_y().bit(config.mirror_y)
        });

        writeback(input.data);
        writeback(output.data);
        writeback(core::slice::from_ref(&tx));
        writeback(core::slice::from_ref(&rx));
        // SAFETY: The descriptors and buffers stay live through channel cleanup.
        unsafe {
            self.channels
                .start((&raw mut tx) as u32, (&raw mut rx) as u32)
        };
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_start().set_bit());

        let result = self.wait_for_completion();
        self.channels.reset();
        if result.is_err() {
            PeripheralClockControl::reset(Peripheral::Ppa);
        }
        result?;
        if is_slice_in_psram(output.data) {
            unsafe {
                crate::soc::cache_invalidate_addr(
                    output.data.as_ptr() as u32,
                    output.data.len() as u32,
                );
            }
        }
        Ok(())
    }

    fn reset_for_transfer(&mut self) {
        self.channels.reset();
        let ppa = PPA::regs();
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().set_bit());
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().clear_bit());
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        let dma = DMA2D::regs();
        let (tx_channel, rx_channel) = self.channels.indices();
        let ppa = PPA::regs();
        loop {
            let rx_status = dma.in_int_raw_ch(rx_channel).read();
            let tx_status = dma.out_int_raw_ch(tx_channel).read();
            let parameter_status = ppa.srm_param_err_st().read().bits();
            if rx_status.in_dscr_err_ch_int_raw().bit()
                || rx_status.in_err_eof_ch_int_raw().bit()
                || rx_status.in_dscr_empty_ch_int_raw().bit()
                || tx_status.out_dscr_err_ch_int_raw().bit()
                || parameter_status != 0
            {
                debug!(
                    "SRM failed: DMA RX={:#x}, DMA TX={:#x}, PPA={:#x}, parameter={:#x}",
                    rx_status.bits(),
                    tx_status.bits(),
                    ppa.srm_status().read().bits(),
                    parameter_status,
                );
                return Err(Error::Hardware);
            }
            if rx_status.in_suc_eof_ch_int_raw().bit() {
                return Ok(());
            }
        }
    }
}

impl<Dm: DriverMode> Drop for Srm<'_, Dm> {
    fn drop(&mut self) {
        HP_SYS::regs().ppa_mem_lp_ctrl().modify(|_, w| {
            w.ppa_sprf_mem_force_ctrl().bit(self.ppa_mem_lp.0);
            w.ppa_sprf_mem_lp_en().bit(self.ppa_mem_lp.1)
        });
    }
}

// DMA2D uses 24-byte descriptors aligned to 8 bytes. Word 1 encodes the pixel size in bits 28–31.
// See <https://github.com/espressif/esp-idf/blob/ce2100de51a4d9a372f26578fd2ee763ca4b1279/components/esp_hal_dma/include/hal/dma2d_types.h#L22-L76>.
#[repr(C, align(8))]
struct Descriptor([u32; 6]);

impl Descriptor {
    fn new(
        buffer: *const u8,
        picture: Size,
        block: Size,
        offset: Offset,
        format: PixelFormat,
    ) -> Self {
        const DMA2D: u32 = 1 << 29;
        const EOF: u32 = 1 << 30;
        const OWNER: u32 = 1 << 31;
        Self([
            block.height as u32 | ((block.width as u32) << 14) | DMA2D | EOF | OWNER,
            picture.height as u32
                | ((picture.width as u32) << 14)
                | (format.dma_pixel_size() << 28),
            offset.y as u32 | ((offset.x as u32) << 14),
            buffer as u32,
            0,
            0,
        ])
    }
}
