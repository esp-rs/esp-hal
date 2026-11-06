//! Hardware baseline JPEG encoding.

mod header;

use core::{
    ptr,
    sync::atomic::{Ordering, compiler_fence},
};

use crate::{
    dma2d::{
        AnyDma2dRxChannel,
        BURST_128_BYTES,
        CACHE_LINE_SIZE,
        Channels,
        MACRO_BLOCK_NONE,
        writeback,
    },
    peripherals::{DMA2D, DMA2D_TX_CH0, HP_SYS, JPEG},
    soc::{cache_invalidate_addr, is_slice_in_dram, is_slice_in_psram},
    system::{GenericPeripheralGuard, Peripheral, PeripheralClockControl},
    time::{Duration, Instant},
};

const RX_BUFFER_ALIGNMENT: usize = 4;
const JPEG_TRIGGER: u8 = 0;
const MCU_SIZE: u32 = 16;
const MACRO_BLOCK_16X16: u8 = 2;
const DMA_FIELD_BITS: u32 = 14;
const DMA_FIELD_MASK: u32 = (1 << DMA_FIELD_BITS) - 1;
const MAX_DIMENSION: usize = DMA_FIELD_MASK as usize;
const MAX_PAYLOAD: usize = (1 << (2 * DMA_FIELD_BITS)) - 1;

const _: () = core::assert!(header::LEN.is_multiple_of(CACHE_LINE_SIZE));

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
            Self::OutputTooSmall => "JPEG output buffer is too small",
            Self::Timeout => "JPEG encoding timed out",
            Self::Hardware => "JPEG hardware error",
        })
    }
}

/// A blocking RGB565 to baseline JPEG encoder.
#[derive(Debug)]
#[instability::unstable]
pub struct Encoder<'d> {
    _peripheral: JPEG<'d>,
    channels: Channels<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::Jpeg as u8 }>,
    memory_low_power: (bool, bool),
    timeout: Option<Duration>,
}

impl<'d> Encoder<'d> {
    /// Creates a JPEG encoder with DMA2D TX0 and any receive channel.
    ///
    /// TX0 is the only transmit channel that supports the macroblock reordering
    /// required for JPEG encoding.
    /// See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_dma/esp32s31/include/hal/dma2d_ll.h#L1002-L1011>.
    #[instability::unstable]
    pub fn new(
        peripheral: JPEG<'d>,
        tx: DMA2D_TX_CH0<'d>,
        rx: impl Into<AnyDma2dRxChannel<'d>>,
    ) -> Self {
        let channels = Channels::new(tx, rx);
        let guard = GenericPeripheralGuard::new();
        let memory = HP_SYS::regs().jpeg_mem_lp_ctrl();
        let previous = memory.read();
        memory.modify(|_, w| {
            w.jpeg_mem_force_ctrl().set_bit();
            w.jpeg_mem_lp_en().clear_bit()
        });
        Self {
            _peripheral: peripheral,
            channels,
            _guard: guard,
            timeout: None,
            memory_low_power: (
                previous.jpeg_mem_force_ctrl().bit(),
                previous.jpeg_mem_lp_en().bit(),
            ),
        }
    }

    /// Sets the time limit for waiting for JPEG completion.
    ///
    /// Pass `None` to wait without a software timeout. This is the default.
    /// DMA cleanup can extend past this limit.
    #[instability::unstable]
    pub fn set_timeout(&mut self, timeout: Option<Duration>) {
        self.timeout = timeout;
    }

    /// Encodes a tightly packed little-endian RGB565 image as a YCbCr 4:2:0 JPEG.
    ///
    /// Returns the JPEG length in bytes. Width and height must be in the range 1 to
    /// 16383 pixels, and quality must be in the range 1 to 100. The input must be in
    /// SRAM or initialized PSRAM and aligned to two bytes. Partial 16x16 blocks
    /// require no input padding.
    ///
    /// The output address and length must be aligned to four bytes in SRAM or
    /// 64 bytes in PSRAM. PSRAM configurations that require strictly aligned
    /// memory transactions, such as encryption or ECC, are unsupported.
    /// Output contents are unspecified on error.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when dimensions or quality are invalid, or the input is too short.
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
        width: u16,
        height: u16,
        quality: u8,
    ) -> Result<usize, Error> {
        let width_usize = usize::from(width);
        let height_usize = usize::from(height);
        if !(1..=MAX_DIMENSION).contains(&width_usize)
            || !(1..=MAX_DIMENSION).contains(&height_usize)
            || !(1..=100).contains(&quality)
        {
            return Err(Error::InvalidInput);
        }
        let input = input
            .get(..width_usize * height_usize * 2)
            .ok_or(Error::InvalidInput)?;
        if output.len() <= header::LEN {
            return Err(Error::OutputTooSmall);
        }
        let output_in_psram = is_slice_in_psram(output);
        let output_alignment = if output_in_psram {
            CACHE_LINE_SIZE
        } else {
            RX_BUFFER_ALIGNMENT
        };
        if !(is_slice_in_dram(input) || is_slice_in_psram(input))
            || !(output_in_psram || is_slice_in_dram(output))
            || !(input.as_ptr() as usize).is_multiple_of(2)
            || !(output.as_ptr() as usize).is_multiple_of(output_alignment)
            || !output.len().is_multiple_of(output_alignment)
            || output.len() - header::LEN > MAX_PAYLOAD
        {
            return Err(Error::InvalidBuffer);
        }

        let (luminance, chrominance) = header::quantization_tables(quality);
        let (header, payload) = output.split_first_chunk_mut::<{ header::LEN }>().unwrap();
        // Bound the DMA and cache maintenance range. A 4:2:0 MCU has six blocks,
        // each with at most 64 symbols of 16 Huffman bits and 16 coefficient bits.
        // Byte stuffing can double the scan size, and EOI adds two bytes.
        const MAX_MCU_BYTES: usize = 6 * 64 * (16 + 16) / 8 * 2;
        let mcu_count =
            width_usize.div_ceil(MCU_SIZE as usize) * height_usize.div_ceil(MCU_SIZE as usize);
        let max_scan_length = (mcu_count * MAX_MCU_BYTES + 2).next_multiple_of(output_alignment);
        let capacity = payload.len().min(max_scan_length);
        let payload = &mut payload[..capacity];
        header::write(header, width, height, &luminance, &chrominance);

        const DMA_OWNED: u32 = 1 << 31;
        const MODE_2D: u32 = 1 << 29;
        const MULTIPLE_BLOCKS: u32 = 1 << 28;
        const RGB565_PBYTE: u32 = 3 << 28;
        // RGB565 to 4:2:0 uses a 48x16 DMA block. Use the visible dimensions
        // for HA and VA so DMA2D pads partial macroblocks.
        let mut tx = Descriptor([
            DMA_OWNED | MODE_2D | (48 << DMA_FIELD_BITS) | MCU_SIZE,
            RGB565_PBYTE | (u32::from(width) << DMA_FIELD_BITS) | u32::from(height),
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
        jpeg.config().modify(|_, w| w.soft_rst().set_bit());
        jpeg.config().modify(|_, w| unsafe {
            w.soft_rst().clear_bit();
            w.mode().clear_bit();
            w.color_space().bits(2);
            w.sample_sel().bits(2);
            w.pixel_rev().clear_bit();
            w.tailer_en().set_bit();
            w.ff_check_en().set_bit();
            w.qnr_precision().clear_bit();
            w.qnr_fifo_en().set_bit();
            w.lqnr_tbl_sel().bits(0);
            w.cqnr_tbl_sel().bits(1)
        });
        jpeg.pic_size().write(|w| unsafe {
            w.ha().bits(width);
            w.va().bits(height)
        });
        jpeg.int_clr().write(|w| unsafe { w.bits(u32::MAX) });
        jpeg.dqt_info().write(|w| unsafe {
            w.t0_dqt_info().bits(0);
            w.t1_dqt_info().bits(1);
            w.t2_dqt_info().bits(2);
            w.t3_dqt_info().bits(3)
        });
        // Write coefficients through raw pointers because the PAC marks the FIFO ports read-only.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_jpeg/esp32s31/include/hal/jpeg_ll.h#L274-L322>.
        for (port, values) in [
            (jpeg.t0qnr().as_ptr(), &luminance),
            (jpeg.t1qnr().as_ptr(), &chrominance),
            (jpeg.t2qnr().as_ptr(), &luminance),
            (jpeg.t3qnr().as_ptr(), &chrominance),
        ] {
            for &value in values {
                unsafe { port.write_volatile(u32::from(value)) };
            }
        }

        let dma = DMA2D::regs();
        let (tx_channel, rx_channel) = self.channels.indices();
        self.channels.connect(JPEG_TRIGGER);
        dma.out_conf0_ch(tx_channel).modify(|_, w| unsafe {
            w.out_reorder_en_ch().set_bit();
            w.out_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.out_page_bound_en_ch().set_bit();
            w.out_macro_block_size_ch().bits(MACRO_BLOCK_16X16);
            w.outdscr_burst_en_ch().set_bit()
        });
        dma.in_conf0_ch(rx_channel).modify(|_, w| unsafe {
            w.in_mem_burst_length_ch().bits(BURST_128_BYTES);
            w.in_page_bound_en_ch().set_bit();
            w.in_macro_block_size_ch().bits(MACRO_BLOCK_NONE);
            w.indscr_burst_en_ch().set_bit()
        });

        writeback(input);
        writeback(payload);
        writeback(core::slice::from_ref(&tx));
        writeback(core::slice::from_ref(&rx));
        // SAFETY: The descriptors and buffers stay live through channel cleanup.
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
        invalidate(payload);
        invalidate(core::slice::from_ref(&rx));
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
                || jpeg.encode_timeout().bit()
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

impl Drop for Encoder<'_> {
    fn drop(&mut self) {
        HP_SYS::regs().jpeg_mem_lp_ctrl().modify(|_, w| {
            w.jpeg_mem_force_ctrl().bit(self.memory_low_power.0);
            w.jpeg_mem_lp_en().bit(self.memory_low_power.1)
        });
    }
}

// RX writes back the descriptor. Keep it on its own cache line on a PSRAM stack.
#[repr(C, align(64))]
struct Descriptor([u32; 6]);

fn invalidate<T>(buffer: &[T]) {
    if is_slice_in_psram(buffer) {
        unsafe {
            cache_invalidate_addr(
                buffer.as_ptr() as u32,
                core::mem::size_of_val(buffer) as u32,
            )
        };
    }
}
