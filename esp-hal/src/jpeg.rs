//! Hardware baseline JPEG encoding.
//!
//! ## Supported formats
//!
//! | Input layout | Output subsampling |
//! | --- | --- |
//! | RGB565 and BGR565 (either byte order), RGB888, BGR888 | 4:4:4, 4:2:2, 4:2:0 |
//! | YCbCr 4:4:4 (YUV, VUY) | 4:4:4, 4:2:2, 4:2:0 |
//! | YCbCr 4:2:2 (YUYV, UYVY) | 4:2:2, 4:2:0 |
//! | Packed YCbCr 4:2:0 | 4:2:0 |
//! | Grayscale | Grayscale |

mod format;
mod header;

use core::{
    ptr,
    sync::atomic::{Ordering, compiler_fence},
};

use self::format::{InputLayout, Sampling};
pub use self::format::{PixelFormat, Subsampling};
use crate::{
    dma::aligned::{PSRAM_CACHE_LINE, region_dma_alignment},
    dma2d::{AnyDma2dRxChannel, BURST_128_BYTES, Channels, invalidate, writeback},
    pac::jpeg::int_raw::R as InterruptStatus,
    peripherals::{DMA2D, DMA2D_TX_CH0, JPEG},
    soc::{is_slice_in_dram, is_slice_in_psram},
    system::{GenericPeripheralGuard, Peripheral, PeripheralClockControl},
    time::{Duration, Instant},
};

const RX_BUFFER_ALIGNMENT: usize = 4;
const JPEG_TRIGGER: u8 = 0;
const DMA_FIELD_BITS: u32 = 14;
const DMA_FIELD_MASK: u32 = (1 << DMA_FIELD_BITS) - 1;
const MAX_DIMENSION: usize = DMA_FIELD_MASK as usize;
const MAX_PAYLOAD: usize = (1 << (2 * DMA_FIELD_BITS)) - 1;

const _: () = core::assert!(header::LEN.is_multiple_of(PSRAM_CACHE_LINE));

/// A JPEG encoder error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Error {
    /// Invalid dimensions, quality, or input size.
    InvalidInput,
    /// Unsupported buffer memory region, size, or alignment.
    InvalidBuffer,
    /// Incompatible input pixel format and output subsampling.
    UnsupportedFormat,
    /// Insufficient space for the compressed image.
    OutputTooSmall,
    /// The configured completion wait expired.
    Timeout,
    /// JPEG or DMA2D hardware error.
    Hardware,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::InvalidInput => "invalid JPEG input",
            Self::InvalidBuffer => "invalid JPEG buffer",
            Self::UnsupportedFormat => "incompatible input format and subsampling",
            Self::OutputTooSmall => "JPEG output buffer is too small",
            Self::Timeout => "JPEG encoding timed out",
            Self::Hardware => "JPEG hardware error",
        })
    }
}

/// JPEG encoding settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Config {
    /// Image width in pixels, from 1 to 16383.
    pub width: u16,
    /// Image height in pixels, from 1 to 16383.
    pub height: u16,
    /// Compression quality, from 1 to 100.
    pub quality: u8,
    /// Pixel layout of the input image.
    pub input_format: PixelFormat,
    /// Chroma subsampling of the encoded JPEG.
    pub subsampling: Subsampling,
}

/// A blocking baseline JPEG encoder.
#[derive(Debug)]
#[instability::unstable]
pub struct Encoder<'d> {
    _peripheral: JPEG<'d>,
    _memory: MemoryPower,
    channels: Channels<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::Jpeg as u8 }>,
    timeout: Option<Duration>,
}

impl<'d> Encoder<'d> {
    /// Creates a new JPEG encoder using DMA2D TX0 for macroblock reordering.
    #[instability::unstable]
    pub fn new(
        peripheral: JPEG<'d>,
        tx: DMA2D_TX_CH0<'d>,
        rx: impl Into<AnyDma2dRxChannel<'d>>,
    ) -> Self {
        Self {
            _peripheral: peripheral,
            channels: Channels::new(tx, rx),
            _guard: GenericPeripheralGuard::new(),
            _memory: MemoryPower::force_on(),
            timeout: None,
        }
    }

    /// Sets the software timeout for JPEG completion.
    ///
    /// The timeout is disabled by default (`None`).
    /// DMA cleanup can extend past this limit.
    #[instability::unstable]
    pub fn set_timeout(&mut self, timeout: Option<Duration>) {
        self.timeout = timeout;
    }

    /// Encodes a tightly packed image and returns the JPEG length in bytes.
    ///
    /// See [supported formats](crate::jpeg#supported-formats) for valid format combinations
    /// and [`PixelFormat`] for input layout requirements.
    ///
    /// The input must be in SRAM or initialized PSRAM.
    /// RGB565, BGR565, and YCbCr 4:2:2 input must be aligned to two bytes.
    ///
    /// The output address and length must be multiples of
    #[doc = cfg_select! {
        esp32p4 => "64 bytes in cached SRAM, 4 bytes in uncached SRAM, or 128 bytes in PSRAM.",
        esp32s31 => "4 bytes in SRAM or 64 bytes in PSRAM.",
    }]
    /// PSRAM configurations that require strictly aligned memory transactions, such as encryption
    /// or ECC, are unsupported.
    /// Output contents are unspecified on error.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when dimensions or quality are invalid, or the input is too short.
    /// - [`Error::UnsupportedFormat`] when the input format and subsampling are incompatible.
    /// - [`Error::InvalidBuffer`] when a buffer is inaccessible or unaligned, or the output payload
    ///   capacity exceeds the DMA descriptor limit.
    /// - [`Error::OutputTooSmall`] when the JPEG does not fit.
    /// - [`Error::Timeout`] when the configured completion wait expires.
    /// - [`Error::Hardware`] on a JPEG or DMA2D error.
    #[instability::unstable]
    pub fn encode(
        &mut self,
        input: &[u8],
        output: &mut [u8],
        config: Config,
    ) -> Result<usize, Error> {
        let Config {
            width,
            height,
            quality,
            input_format,
            subsampling,
        } = config;
        let width_usize = usize::from(width);
        let height_usize = usize::from(height);
        if !(1..=MAX_DIMENSION).contains(&width_usize)
            || !(1..=MAX_DIMENSION).contains(&height_usize)
            || !(1..=100).contains(&quality)
        {
            return Err(Error::InvalidInput);
        }
        let layout = InputLayout::new(input_format, subsampling, width_usize, height_usize)?;
        let input = input.get(..layout.len).ok_or(Error::InvalidInput)?;
        if output.len() <= header::LEN {
            return Err(Error::OutputTooSmall);
        }
        let output_alignment = region_dma_alignment(output.as_ptr() as usize)
            .ok_or(Error::InvalidBuffer)?
            .max(RX_BUFFER_ALIGNMENT);
        if !(is_slice_in_dram(input) || is_slice_in_psram(input))
            || !(is_slice_in_dram(output) || is_slice_in_psram(output))
            || !(input.as_ptr() as usize).is_multiple_of(layout.alignment)
            || !(output.as_ptr() as usize).is_multiple_of(output_alignment)
            || !output.len().is_multiple_of(output_alignment)
            || output.len() - header::LEN > MAX_PAYLOAD
        {
            return Err(Error::InvalidBuffer);
        }

        let sampling = Sampling::new(subsampling);
        let (luminance, chrominance) = header::quantization_tables(quality, &sampling);
        let chrominance = chrominance.as_ref();
        let (header, payload) = output.split_first_chunk_mut::<{ header::LEN }>().unwrap();
        // Allow 64 symbols per block, each up to 16 Huffman bits and 16 coefficient bits.
        // Byte stuffing can double the scan size, and EOI adds two bytes.
        const MAX_BLOCK_BYTES: usize = 64 * (16 + 16) / 8 * 2;
        let (mcu_width, mcu_height) = sampling.mcu_size();
        let mcu_count =
            width_usize.div_ceil(mcu_width as usize) * height_usize.div_ceil(mcu_height as usize);
        let max_scan_length = mcu_count
            .saturating_mul(sampling.blocks_per_mcu() * MAX_BLOCK_BYTES)
            .saturating_add(2);
        let capacity = payload
            .len()
            .min(max_scan_length)
            .next_multiple_of(output_alignment);
        let payload = &mut payload[..capacity];
        header::write(header, width, height, &sampling, &luminance, chrominance);

        const DMA_OWNED: u32 = 1 << 31;
        const MODE_2D: u32 = 1 << 29;
        const MULTIPLE_BLOCKS: u32 = 1 << 28;
        // Use the visible dimensions for HA and VA so DMA2D pads partial macroblocks.
        let mut tx = Descriptor([
            DMA_OWNED | MODE_2D | (layout.dma_block_width << DMA_FIELD_BITS) | mcu_height,
            (layout.dma_pbyte << 28) | (u32::from(width) << DMA_FIELD_BITS) | u32::from(height),
            MULTIPLE_BLOCKS,
            input.as_ptr() as u32,
            0,
            0,
        ]);
        let capacity = capacity as u32;
        let mut rx = Descriptor([
            DMA_OWNED | (capacity & DMA_FIELD_MASK),
            capacity >> DMA_FIELD_BITS,
            0,
            payload.as_mut_ptr() as u32,
            0,
            0,
        ]);

        self.channels.reset();
        let jpeg = JPEG::regs();
        let color_space_bits = layout.color_space as u8;
        #[cfg(esp32p4)]
        let (color_space_bits, extd_select) = match layout.color_space {
            format::ColorSpace::Ycbcr444 => (0, Some(false)),
            format::ColorSpace::Ycbcr420 => (0, Some(true)),
            _ => (color_space_bits, None),
        };
        jpeg.config().modify(|_, w| w.soft_rst().set_bit());
        jpeg.config().modify(|_, w| unsafe {
            w.soft_rst().clear_bit();
            w.mode().clear_bit();
            w.color_space().bits(color_space_bits);
            w.sample_sel().bits(sampling.sample_sel);
            w.pixel_rev().bit(layout.pixel_reverse);
            w.tailer_en().set_bit();
            w.ff_check_en().set_bit();
            w.qnr_precision().clear_bit();
            w.qnr_fifo_en().set_bit();
            w.lqnr_tbl_sel().bits(0);
            w.cqnr_tbl_sel().bits(1)
        });
        #[cfg(esp32p4)]
        jpeg.extd_config().write(|w| {
            w.extd_color_space_en().bit(extd_select.is_some());
            w.extd_color_space().bit(extd_select.unwrap_or(false))
        });
        jpeg.pic_size().write(|w| unsafe {
            w.ha().bits(width);
            w.va().bits(height)
        });
        jpeg.int_clr().write(|w| unsafe { w.bits(u32::MAX) });

        for &value in &luminance {
            jpeg.t0qnr()
                .write(|w| unsafe { w.t0_qnr_val().bits(u32::from(value)) });
        }
        if let Some(chrominance) = chrominance {
            for &value in chrominance {
                jpeg.t1qnr()
                    .write(|w| unsafe { w.t1_qnr_val().bits(u32::from(value)) });
            }
        }

        let dma = DMA2D::regs();
        let (tx_channel, rx_channel) = self.channels.indices();
        self.channels.connect(JPEG_TRIGGER);
        if layout.dma_swap_bytes {
            self.channels.swap_tx_pixel_bytes();
        }
        dma.out_conf0_ch(tx_channel).modify(|_, w| unsafe {
            w.out_reorder_en_ch().set_bit();
            w.out_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.out_page_bound_en_ch().set_bit();
            w.out_macro_block_size_ch()
                .bits(sampling.dma_macro_block_size);
            w.outdscr_burst_en_ch().set_bit()
        });
        dma.in_conf0_ch(rx_channel).modify(|_, w| unsafe {
            w.in_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.in_page_bound_en_ch().set_bit();
            w.indscr_burst_en_ch().set_bit()
        });

        writeback(input);
        writeback(payload);
        writeback(core::slice::from_ref(&tx));
        writeback(core::slice::from_ref(&rx));
        // SAFETY: Reset left the channels idle. Cache writeback is complete, and the descriptors
        // and buffers stay live until the channels stop.
        unsafe {
            self.channels
                .start((&raw mut tx) as u32, (&raw mut rx) as u32)
        };
        jpeg.config().modify(|_, w| w.jpeg_start().set_bit());

        let result = self.wait_for_completion();
        self.channels.reset();
        if result.is_err() {
            PeripheralClockControl::reset(Peripheral::Jpeg);
        }
        // RX can write the payload and descriptor before reporting an error.
        // SAFETY: The payload and RX descriptor each cover whole cache lines.
        unsafe {
            invalidate(payload);
            invalidate(core::slice::from_ref(&rx));
        }
        result?;

        compiler_fence(Ordering::SeqCst);
        let word0 = unsafe { ptr::read_volatile(&raw const rx.0[0]) };
        let word1 = unsafe { ptr::read_volatile(&raw const rx.0[1]) };
        let length_low = (word0 >> DMA_FIELD_BITS) & DMA_FIELD_MASK;
        let length_high = (word1 >> DMA_FIELD_BITS) & DMA_FIELD_MASK;
        let length = length_low | (length_high << DMA_FIELD_BITS);
        if length < 2 || length > capacity {
            return Err(Error::Hardware);
        }
        Ok(header::LEN + length as usize)
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        let dma = DMA2D::regs();
        let (tx_channel, rx_channel) = self.channels.indices();
        let timeout = self.timeout.map(|timeout| (Instant::now(), timeout));
        loop {
            let rx = dma.in_int_raw_ch(rx_channel).read();
            let tx = dma.out_int_raw_ch(tx_channel).read();
            let jpeg = JPEG::regs().int_raw().read();
            if rx.in_dscr_err_ch_int_raw().bit()
                || rx.in_err_eof_ch_int_raw().bit()
                || tx.out_dscr_err_ch_int_raw().bit()
                || tx.outfifo_ro_ovf_ch_int_raw().bit()
                || tx.outfifo_ro_udf_ch_int_raw().bit()
                || jpeg.rle_parallel_err().bit()
                || jpeg.en_frame_eof_err().bit()
                || encode_timeout(&jpeg)
            {
                return Err(Error::Hardware);
            }
            if rx.in_suc_eof_ch_int_raw().bit() {
                return Ok(());
            }
            if rx.in_dscr_empty_ch_int_raw().bit() {
                return Err(Error::OutputTooSmall);
            }
            if timeout.is_some_and(|(start, timeout)| start.elapsed() >= timeout) {
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
    }
}

cfg_select! {
    esp32s31 => {
        use crate::peripherals::HP_SYS;

        /// Forces the JPEG memory on, and restores its low-power control when dropped.
        #[derive(Debug)]
        struct MemoryPower {
            force_ctrl: bool,
            low_power: bool,
        }

        impl MemoryPower {
            fn force_on() -> Self {
                let memory = HP_SYS::regs().jpeg_mem_lp_ctrl();
                let previous = memory.read();
                memory.modify(|_, w| {
                    w.jpeg_mem_force_ctrl().set_bit();
                    w.jpeg_mem_lp_en().clear_bit()
                });
                Self {
                    force_ctrl: previous.jpeg_mem_force_ctrl().bit(),
                    low_power: previous.jpeg_mem_lp_en().bit(),
                }
            }
        }

        impl Drop for MemoryPower {
            fn drop(&mut self) {
                HP_SYS::regs().jpeg_mem_lp_ctrl().modify(|_, w| {
                    w.jpeg_mem_force_ctrl().bit(self.force_ctrl);
                    w.jpeg_mem_lp_en().bit(self.low_power)
                });
            }
        }

        fn encode_timeout(status: &InterruptStatus) -> bool {
            status.encode_timeout().bit()
        }
    }
    _ => {
        // The JPEG memory has no software low-power control.
        #[derive(Debug)]
        struct MemoryPower;

        impl MemoryPower {
            fn force_on() -> Self {
                Self
            }
        }

        // ESP32-P4 has no JPEG encode timeout interrupt.
        fn encode_timeout(_status: &InterruptStatus) -> bool {
            false
        }
    }
}

// RX writes back the descriptor. Keep it on its own cache line on a cached stack.
#[cfg_attr(soc_internal_memory_cached, repr(C, align(128)))]
#[cfg_attr(not(soc_internal_memory_cached), repr(C, align(64)))]
struct Descriptor([u32; 6]);
