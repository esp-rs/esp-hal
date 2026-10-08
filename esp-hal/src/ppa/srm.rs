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
//! - Each buffer must contain exactly `width * height * format.bytes_per_pixel()` bytes for its
//!   image size.
//! - Buffers must be entirely within internal RAM or initialized PSRAM.
//! - Buffer addresses must be 64-byte aligned, and lengths must be multiples of 64 bytes.
//!
//! ## Examples
//!
//! ### Rotating an RGB565 image
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::ppa::srm::{Config, Input, Offset, Output, PixelFormat, Rotation, Size, Srm};
//!
//! #[repr(C, align(64))]
//! struct Image([u8; 128]);
//!
//! let source = Image([0; 128]);
//! let mut destination = Image([0; 128]);
//! let size = Size {
//!     width: 16,
//!     height: 4,
//! };
//! let config = Config::default().with_rotation(Rotation::Degrees90);
//! let mut srm = Srm::new(peripherals.PPA, peripherals.DMA2D);
//! srm.run(
//!     Input {
//!         data: &source.0,
//!         size,
//!         format: PixelFormat::Rgb565,
//!     },
//!     Output {
//!         data: &mut destination.0,
//!         size: config.output_size(size)?,
//!         format: PixelFormat::Rgb565,
//!         offset: Offset::default(),
//!     },
//!     config,
//! )?;
//! # {after_snippet}
//! ```
//!
//! ## Implementation State
//!
//! Asynchronous transfers are not implemented.
//! PSRAM configurations requiring strictly aligned memory transactions, such as encryption or
//! ECC, are not supported.

use core::{
    marker::PhantomData,
    sync::atomic::{Ordering, compiler_fence},
};

use crate::{
    Blocking,
    DriverMode,
    peripherals::{DMA2D, HP_SYS, PPA},
    soc::{is_slice_in_dram, is_slice_in_psram},
    system::{GenericPeripheralGuard, Peripheral, PeripheralClockControl},
    time::{Duration, Instant},
};

const DMA_CHANNEL: usize = 0;
const ALIGNMENT: usize = 64;
const MAX_DIMENSION: usize = 0x3fff;
// DMA register encodings:
// https://github.com/espressif/esp-idf/blob/ce2100de51a4d9a372f26578fd2ee763ca4b1279/components/esp_hal_dma/esp32s31/include/hal/dma2d_ll.h#L283-L379
const DMA_BURST_128_BYTES: u8 = 4;
const DMA_MACRO_BLOCK_NONE: u8 = 3;
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
    fn is_valid(self) -> bool {
        self.width != 0
            && self.height != 0
            && self.width <= MAX_DIMENSION
            && self.height <= MAX_DIMENSION
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
        // SRM input padding for 32x32 blocks:
        // https://github.com/espressif/esp-idf/blob/ce2100de51a4d9a372f26578fd2ee763ca4b1279/components/esp_hal_ppa/esp32s31/include/hal/ppa_ll.h#L586-L604
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
        if !size.is_valid() {
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
        if !size.is_valid() {
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
    /// A buffer address or length does not meet the DMA alignment requirement.
    UnalignedBuffer,
    /// A buffer is outside internal RAM or initialized PSRAM.
    UnsupportedMemoryRegion,
    /// A 2D-DMA or PPA operation error.
    Hardware,
    /// A channel reset or transfer timeout.
    Timeout,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidSize => f.write_str("invalid image dimensions, offset, or buffer length"),
            Self::InvalidScale => f.write_str("invalid scaling factor"),
            Self::UnalignedBuffer => {
                f.write_str("image buffer address or length is not 64-byte aligned")
            }
            Self::UnsupportedMemoryRegion => {
                f.write_str("image buffer is outside internal RAM or initialized PSRAM")
            }
            Self::Hardware => f.write_str("SRM hardware error"),
            Self::Timeout => f.write_str("SRM timed out"),
        }
    }
}

/// PPA SRM driver.
///
/// The driver owns the PPA and the entire 2D-DMA controller.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Srm<'d, Dm: DriverMode> {
    _ppa: PPA<'d>,
    _dma: DMA2D<'d>,
    ppa_mem_lp: (bool, bool),
    dma_mem_lp: (bool, bool),
    _ppa_guard: GenericPeripheralGuard<{ Peripheral::Ppa as u8 }>,
    _dma_guard: GenericPeripheralGuard<{ Peripheral::Dma2d as u8 }>,
    _mode: PhantomData<Dm>,
}

impl<'d> Srm<'d, Blocking> {
    /// Creates a new SRM driver in [`Blocking`] mode.
    #[instability::unstable]
    pub fn new(ppa: PPA<'d>, dma: DMA2D<'d>) -> Self {
        let ppa_guard = GenericPeripheralGuard::new();
        let dma_guard = GenericPeripheralGuard::new();
        let system = HP_SYS::regs();
        let ppa_mem_lp = system.ppa_mem_lp_ctrl().read();
        let ppa_mem_lp = (
            ppa_mem_lp.ppa_sprf_mem_force_ctrl().bit(),
            ppa_mem_lp.ppa_sprf_mem_lp_en().bit(),
        );
        let dma_mem_lp = system.dma2d_mem_lp_ctrl().read();
        let dma_mem_lp = (
            dma_mem_lp._2ddma_mem_force_ctrl().bit(),
            dma_mem_lp._2ddma_mem_lp_en().bit(),
        );
        system.ppa_mem_lp_ctrl().modify(|_, w| {
            w.ppa_sprf_mem_force_ctrl()
                .set_bit()
                .ppa_sprf_mem_lp_en()
                .clear_bit()
        });
        system.dma2d_mem_lp_ctrl().modify(|_, w| {
            w._2ddma_mem_force_ctrl()
                .set_bit()
                ._2ddma_mem_lp_en()
                .clear_bit()
        });
        let dma_regs = DMA2D::regs();
        dma_regs.rst_conf().modify(|_, w| w.clk_en().set_bit());
        dma_regs.rst_conf().modify(|_, w| w.axim_rd_rst().set_bit());
        dma_regs
            .rst_conf()
            .modify(|_, w| w.axim_rd_rst().clear_bit());
        dma_regs.rst_conf().modify(|_, w| w.axim_wr_rst().set_bit());
        dma_regs
            .rst_conf()
            .modify(|_, w| w.axim_wr_rst().clear_bit());
        Self {
            _ppa: ppa,
            _dma: dma,
            ppa_mem_lp,
            dma_mem_lp,
            _ppa_guard: ppa_guard,
            _dma_guard: dma_guard,
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
    /// - [`Error::UnalignedBuffer`] when an address or length is not 64-byte aligned.
    /// - [`Error::UnsupportedMemoryRegion`] when a buffer is outside the supported memory regions.
    /// - [`Error::Hardware`] on a peripheral error.
    /// - [`Error::Timeout`] when channel reset takes more than 100 ms or the transfer takes more
    ///   than 500 ms.
    ///
    /// The destination contents are unspecified after a hardware error or timeout.
    #[instability::unstable]
    pub fn run(
        &mut self,
        input: Input<'_>,
        output: Output<'_>,
        config: Config,
    ) -> Result<(), Error> {
        let Input {
            data: source,
            size: source_size,
            format: source_format,
        } = input;
        let Output {
            data: destination,
            size: destination_size,
            format: destination_format,
            offset,
        } = output;
        let Size { width, height } = source_size;
        let Size {
            width: output_width,
            height: output_height,
        } = config.output_size(source_size)?;
        let Size {
            width: destination_width,
            height: destination_height,
        } = destination_size;
        let Offset {
            x: offset_x,
            y: offset_y,
        } = offset;
        if !destination_size.is_valid()
            || (source_format == PixelFormat::Uyvy422 && !width.is_multiple_of(2))
            || (destination_format == PixelFormat::Uyvy422
                && (!destination_width.is_multiple_of(2)
                    || !offset_x.is_multiple_of(2)
                    || !output_width.is_multiple_of(2)))
            || offset_x
                .checked_add(output_width)
                .is_none_or(|end| end > destination_width)
            || offset_y
                .checked_add(output_height)
                .is_none_or(|end| end > destination_height)
            || source.len() != width * height * source_format.bytes_per_pixel()
            || destination.len()
                != destination_width * destination_height * destination_format.bytes_per_pixel()
        {
            return Err(Error::InvalidSize);
        }
        if !(is_slice_in_dram(source) || is_slice_in_psram(source))
            || !(is_slice_in_dram(destination) || is_slice_in_psram(destination))
        {
            return Err(Error::UnsupportedMemoryRegion);
        }
        if !(source.as_ptr() as usize).is_multiple_of(ALIGNMENT)
            || !source.len().is_multiple_of(ALIGNMENT)
            || !(destination.as_ptr() as usize).is_multiple_of(ALIGNMENT)
            || !destination.len().is_multiple_of(ALIGNMENT)
        {
            return Err(Error::UnalignedBuffer);
        }

        let mut tx = Descriptor::new(
            source.as_ptr(),
            source_size,
            source_size,
            Offset { x: 0, y: 0 },
            source_format,
        );
        let mut rx = Descriptor::new(
            destination.as_mut_ptr(),
            destination_size,
            OUTPUT_BLOCK_SIZE,
            offset,
            destination_format,
        );
        let tx_address = (&raw mut tx) as u32;
        let rx_address = (&raw mut rx) as u32;
        let dma = DMA2D::regs();
        let ppa = PPA::regs();

        self.reset_channels()?;

        dma.out_peri_sel_ch(DMA_CHANNEL)
            .modify(|_, w| unsafe { w.out_peri_sel_ch().bits(1) });
        dma.in_peri_sel_ch(DMA_CHANNEL)
            .modify(|_, w| unsafe { w.in_peri_sel_ch().bits(1) });
        dma.out_conf0_ch(DMA_CHANNEL).modify(|_, w| unsafe {
            w.out_dscr_port_en_ch()
                .set_bit()
                .out_page_bound_en_ch()
                .set_bit()
                .out_mem_burst_length_ch()
                .bits(DMA_BURST_128_BYTES)
                .out_macro_block_size_ch()
                .bits(DMA_MACRO_BLOCK_NONE)
                .outdscr_burst_en_ch()
                .set_bit()
        });
        dma.in_conf0_ch(DMA_CHANNEL).modify(|_, w| unsafe {
            w.in_dscr_port_en_ch()
                .set_bit()
                .in_page_bound_en_ch()
                .set_bit()
                .in_mem_burst_length_ch()
                .bits(DMA_BURST_128_BYTES)
                .in_macro_block_size_ch()
                .bits(DMA_MACRO_BLOCK_NONE)
                .indscr_burst_en_ch()
                .set_bit()
        });
        dma.out_dscr_port_blk_ch(DMA_CHANNEL).write(|w| unsafe {
            w.out_dscr_port_blk_h_ch()
                .bits(source_format.input_block_width())
                .out_dscr_port_blk_v_ch()
                .bits(INPUT_BLOCK_HEIGHT)
        });
        dma.out_int_clr_ch(DMA_CHANNEL)
            .write(|w| unsafe { w.bits(u32::MAX) });
        dma.in_int_clr_ch(DMA_CHANNEL)
            .write(|w| unsafe { w.bits(u32::MAX) });

        ppa.srm_byte_order()
            .write(|w| w.srm_bk_size_sel().clear_bit());
        ppa.srm_color_mode().write(|w| unsafe {
            w.srm_rx_cm()
                .bits(source_format.color_mode())
                .srm_tx_cm()
                .bits(destination_format.color_mode())
                .yuv_rx_range()
                .clear_bit()
                .yuv2rgb_protocal()
                .clear_bit()
                .yuv_tx_range()
                .clear_bit()
                .rgb2yuv_protocal()
                .clear_bit()
                .yuv422_rx_byte_order()
                .bits(0)
        });
        ppa.srm_fix_alpha().reset();
        ppa.rgb2gray().reset();
        let scale_x = (config.scale_x * 16.0) as u16;
        let scale_y = (config.scale_y * 16.0) as u16;
        ppa.srm_scal_rotate().write(|w| unsafe {
            w.srm_scal_x_int()
                .bits((scale_x >> 4) as u8)
                .srm_scal_x_frag()
                .bits((scale_x & 15) as u8)
                .srm_scal_y_int()
                .bits((scale_y >> 4) as u8)
                .srm_scal_y_frag()
                .bits((scale_y & 15) as u8)
                .srm_rotate_angle()
                .bits(config.rotation.angle())
                .srm_mirror_x()
                .bit(config.mirror_x)
                .srm_mirror_y()
                .bit(config.mirror_y)
        });

        unsafe {
            crate::soc::cache_writeback_addr(source.as_ptr() as u32, source.len() as u32);
            crate::soc::cache_writeback_addr(destination.as_ptr() as u32, destination.len() as u32);
            crate::soc::cache_writeback_addr(tx_address, size_of::<Descriptor>() as u32);
            crate::soc::cache_writeback_addr(rx_address, size_of::<Descriptor>() as u32);
        }
        compiler_fence(Ordering::SeqCst);
        dma.out_link_addr_ch(DMA_CHANNEL)
            .write(|w| unsafe { w.outlink_addr_ch().bits(tx_address) });
        dma.in_link_addr_ch(DMA_CHANNEL)
            .write(|w| unsafe { w.inlink_addr_ch().bits(rx_address) });
        dma.out_link_conf_ch(DMA_CHANNEL)
            .modify(|_, w| w.outlink_start_ch().set_bit());
        dma.in_link_conf_ch(DMA_CHANNEL)
            .modify(|_, w| w.inlink_start_ch().set_bit());
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_start().set_bit());

        self.wait_for_completion()?;
        unsafe {
            crate::soc::cache_invalidate_addr(
                destination.as_ptr() as u32,
                destination.len() as u32,
            );
        }
        Ok(())
    }

    fn reset_channels(&mut self) -> Result<(), Error> {
        let dma = DMA2D::regs();
        let ppa = PPA::regs();
        dma.out_link_conf_ch(DMA_CHANNEL)
            .modify(|_, w| w.outlink_stop_ch().set_bit());
        dma.in_link_conf_ch(DMA_CHANNEL)
            .modify(|_, w| w.inlink_stop_ch().set_bit());
        dma.out_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.out_cmd_disable_ch().set_bit());
        dma.in_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.in_cmd_disable_ch().set_bit());
        let started = Instant::now();
        while !dma
            .out_state_ch(DMA_CHANNEL)
            .read()
            .out_reset_avail_ch()
            .bit()
            || !dma
                .in_state_ch(DMA_CHANNEL)
                .read()
                .in_reset_avail_ch()
                .bit()
        {
            if started.elapsed() > Duration::from_millis(100) {
                return Err(self.abort_with_error(Error::Timeout));
            }
        }
        dma.out_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.out_rst_ch().set_bit());
        dma.out_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.out_rst_ch().clear_bit());
        dma.in_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.in_rst_ch().set_bit());
        dma.in_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.in_rst_ch().clear_bit());
        dma.out_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.out_cmd_disable_ch().clear_bit());
        dma.in_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.in_cmd_disable_ch().clear_bit());
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().set_bit());
        ppa.srm_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().clear_bit());
        Ok(())
    }

    fn wait_for_completion(&mut self) -> Result<(), Error> {
        let dma = DMA2D::regs();
        let ppa = PPA::regs();
        let started = Instant::now();
        loop {
            let input_status = dma.in_int_raw_ch(DMA_CHANNEL).read();
            let output_status = dma.out_int_raw_ch(DMA_CHANNEL).read();
            let parameter_status = ppa.srm_param_err_st().read().bits();
            if input_status.in_dscr_err_ch_int_raw().bit()
                || input_status.in_err_eof_ch_int_raw().bit()
                || input_status.in_dscr_empty_ch_int_raw().bit()
                || output_status.out_dscr_err_ch_int_raw().bit()
                || parameter_status != 0
            {
                return Err(self.abort_with_error(Error::Hardware));
            }
            if input_status.in_suc_eof_ch_int_raw().bit() {
                return Ok(());
            }
            if started.elapsed() > Duration::from_millis(500) {
                return Err(self.abort_with_error(Error::Timeout));
            }
        }
    }

    fn abort_with_error(&mut self, error: Error) -> Error {
        let dma = DMA2D::regs();
        let ppa = PPA::regs();
        debug!(
            "SRM failed ({:?}): DMA input={:#x}, DMA output={:#x}, PPA={:#x}, parameter={:#x}",
            error,
            dma.in_int_raw_ch(DMA_CHANNEL).read().bits(),
            dma.out_int_raw_ch(DMA_CHANNEL).read().bits(),
            ppa.srm_status().read().bits(),
            ppa.srm_param_err_st().read().bits(),
        );
        self.abort();
        error
    }

    fn abort(&mut self) {
        let dma = DMA2D::regs();
        dma.out_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.out_cmd_disable_ch().set_bit());
        dma.in_conf0_ch(DMA_CHANNEL)
            .modify(|_, w| w.in_cmd_disable_ch().set_bit());
        PeripheralClockControl::reset(Peripheral::Ppa);
        PeripheralClockControl::reset(Peripheral::Dma2d);
        dma.rst_conf().modify(|_, w| w.clk_en().set_bit());
    }
}

impl<Dm: DriverMode> Drop for Srm<'_, Dm> {
    fn drop(&mut self) {
        self.abort();
        let system = HP_SYS::regs();
        system.ppa_mem_lp_ctrl().modify(|_, w| {
            w.ppa_sprf_mem_force_ctrl()
                .bit(self.ppa_mem_lp.0)
                .ppa_sprf_mem_lp_en()
                .bit(self.ppa_mem_lp.1)
        });
        system.dma2d_mem_lp_ctrl().modify(|_, w| {
            w._2ddma_mem_force_ctrl()
                .bit(self.dma_mem_lp.0)
                ._2ddma_mem_lp_en()
                .bit(self.dma_mem_lp.1)
        });
    }
}

// Descriptor layout and pixel-size encoding:
// https://github.com/espressif/esp-idf/blob/ce2100de51a4d9a372f26578fd2ee763ca4b1279/components/esp_hal_dma/include/hal/dma2d_types.h#L22-L76
#[repr(C, align(64))]
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
