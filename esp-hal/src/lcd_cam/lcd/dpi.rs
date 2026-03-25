#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # LCD - RGB/Digital Parallel Interface Mode
//!
//! ## Overview
//!
//! The LCD_CAM peripheral Dpi driver provides support for the DPI (commonly
//! know as RGB) format/timing. The driver mandates DMA (Direct Memory Access)
//! for efficient data transfer.
//!
//! ## Examples
//!
//! ### A display
//!
//! The following example shows how to setup and send a solid frame to a DPI
//! display.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::gpio::Level;
//! # use esp_hal::lcd_cam::{
//! #     LcdCam,
//! #     lcd::{
//! #         ClockMode, Polarity, Phase,
//! #         dpi::{Config, Dpi, Format, FrameTiming, self}
//! #     }
//! # };
//! # use esp_hal::dma_loop_buffer;
//!
//! # let channel = peripherals.DMA_CH0;
//! # let mut dma_buf = dma_loop_buffer!(32);
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//!
//! let config = dpi::Config::default()
//!     .with_frequency(Rate::from_mhz(1))
//!     .with_clock_mode(ClockMode {
//!         polarity: Polarity::IdleLow,
//!         phase: Phase::ShiftLow,
//!     })
//!     .with_format(Format {
//!         enable_2byte_mode: true,
//!         ..Default::default()
//!     })
//!     .with_timing(FrameTiming {
//!         horizontal_active_width: 480,
//!         horizontal_total_width: 520,
//!         horizontal_blank_front_porch: 10,
//!
//!         vertical_active_height: 480,
//!         vertical_total_height: 510,
//!         vertical_blank_front_porch: 10,
//!
//!         hsync_width: 10,
//!         vsync_width: 10,
//!
//!         hsync_position: 0,
//!     })
//!     .with_vsync_idle_level(Level::High)
//!     .with_hsync_idle_level(Level::High)
//!     .with_de_idle_level(Level::Low)
//!     .with_disable_black_region(false);
//!
//! let mut dpi = Dpi::new(lcd_cam.lcd, channel, config)?
//!     .with_vsync(peripherals.GPIO3)
//!     .with_hsync(peripherals.GPIO46)
//!     .with_de(peripherals.GPIO17)
//!     .with_pclk(peripherals.GPIO9)
//!     // Blue
//!     .with_data0(peripherals.GPIO10)
//!     .with_data1(peripherals.GPIO11)
//!     .with_data2(peripherals.GPIO12)
//!     .with_data3(peripherals.GPIO13)
//!     .with_data4(peripherals.GPIO14)
//!     // Green
//!     .with_data5(peripherals.GPIO21)
//!     .with_data6(peripherals.GPIO8)
//!     .with_data7(peripherals.GPIO18)
//!     .with_data8(peripherals.GPIO45)
//!     .with_data9(peripherals.GPIO38)
//!     .with_data10(peripherals.GPIO39)
//!     // Red
//!     .with_data11(peripherals.GPIO40)
//!     .with_data12(peripherals.GPIO41)
//!     .with_data13(peripherals.GPIO42)
//!     .with_data14(peripherals.GPIO2)
//!     .with_data15(peripherals.GPIO1);
//!
//! let color: u16 = 0b11111_000000_00000; // RED
//! for chunk in dma_buf.chunks_mut(2) {
//!     chunk.copy_from_slice(&color.to_le_bytes());
//! }
//!
//! let transfer = dpi.send(false, dma_buf).map_err(|e| e.0)?;
//! transfer.wait();
//! # {after_snippet}
//! ```

use core::{
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

#[cfg(feature = "unstable")]
use critical_section::Mutex;

use crate::{
    Blocking,
    DriverMode,
    dma::{
        ChannelTx,
        DmaDescriptor,
        DmaError,
        DmaPeripheral,
        DmaTxBuffer,
        DmaTxInterrupt,
        Owner,
        PeripheralTxChannel,
        TxChannelFor,
    },
    gpio::{Level, OutputConfig, OutputSignal, interconnect::PeripheralOutput},
    lcd_cam::{
        BitOrder,
        ByteOrder,
        ClockError,
        calculate_clkm,
        lcd::{ClockMode, DelayMode, Lcd, Phase, Polarity},
    },
    pac,
    peripherals::LCD_CAM,
    soc::clocks::ClockTree,
    system::{self, GenericPeripheralGuard},
    time::Rate,
};

#[cfg(feature = "unstable")]
static BOUNCE_STATE: Mutex<core::cell::RefCell<Option<DmaBounceBuffer>>> =
    Mutex::new(core::cell::RefCell::new(None));

/// Errors that can occur when configuring the DPI peripheral.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Clock configuration error.
    Clock(ClockError),
    /// Framebuffer size is not divisible by bounce buffer size, or bounce
    /// buffers are not equal in size.
    InvalidBounceBufferSize,
}

#[cfg(feature = "unstable")]
/// Bounce buffer state for continuous RGB DPI output from a PSRAM framebuffer.
///
/// Holds two SRAM bounce buffers and their circular DMA descriptor chain.
/// The GDMA EOF interrupt (or [`DpiBounceTransfer::poll`]) refills the completed
/// buffer from the PSRAM framebuffer to maintain continuous display output.
///
/// Ported from ESP-IDF's RGB panel bounce buffer architecture:
/// <https://github.com/espressif/esp-idf/blob/662a3be354759d9487bf4b1a629fadb766cb1800/components/esp_lcd/rgb/esp_lcd_panel_rgb.c#L217-L225>
///
/// # Note
///
/// The framebuffer must be a multiple of the bounce buffer size.
#[instability::unstable]
#[allow(dead_code)]
pub struct DmaBounceBuffer {
    descriptors: *mut [DmaDescriptor; 2],
    bounce_bufs: [&'static mut [u8]; 2],
    framebuffer: *const u8,
    fb_len: usize,
    pub(crate) bounce_pos: usize,
    pub(crate) expect_eof_count: usize,
    pub(crate) eof_count: usize,
}

#[cfg(feature = "unstable")]
#[allow(dead_code)]
impl DmaBounceBuffer {
    /// Create a new bounce buffer state.
    ///
    /// # Arguments
    /// - `descriptors`: `'static` storage for the two DMA descriptors
    /// - `framebuffer`: the PSRAM framebuffer to stream (must be a multiple of `bounce_buf0.len()`)
    /// - `bounce_buf0` / `bounce_buf1`: SRAM buffers for the ping-pong DMA pipeline (must be equal
    ///   size)
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::InvalidBounceBufferSize`] if the framebuffer length is not divisible
    /// by the bounce buffer size.
    #[instability::unstable]
    pub fn new(
        descriptors: &'static mut [DmaDescriptor; 2],
        framebuffer: &'static mut [u8],
        bounce_buf0: &'static mut [u8],
        bounce_buf1: &'static mut [u8],
    ) -> Result<Self, ConfigError> {
        let bb_size = bounce_buf0.len();

        if bb_size == 0
            || bb_size != bounce_buf1.len()
            || !framebuffer.len().is_multiple_of(bb_size)
        {
            return Err(ConfigError::InvalidBounceBufferSize);
        }

        let expect_eof_count = framebuffer.len() / bb_size;
        let fb_len = framebuffer.len();
        let fb_ptr = framebuffer.as_ptr();

        let mut this = Self {
            descriptors: descriptors as *mut [DmaDescriptor; 2],
            bounce_bufs: [bounce_buf0, bounce_buf1],
            framebuffer: fb_ptr,
            fb_len,
            bounce_pos: 0,
            expect_eof_count,
            eof_count: 0,
        };

        this.setup_descriptors();

        Ok(this)
    }

    fn setup_descriptors(&mut self) {
        let bb_size = self.bounce_bufs[0].len();
        // SAFETY: `self.descriptors` is a raw pointer derived from a `&'static mut [DmaDescriptor;
        // 2]` passed by the caller to `DmaBounceBuffer::new()`. The static lifetime
        // guarantees the memory is valid for the entire program. Exclusive access is
        // maintained because `DmaBounceBuffer::new()` takes ownership of the `&'static mut`
        // reference and we hold `&mut self`.
        let descriptors = unsafe { &mut *self.descriptors };

        descriptors[0] = DmaDescriptor::EMPTY;
        descriptors[0].set_size(bb_size);
        descriptors[0].set_length(bb_size);
        descriptors[0].set_suc_eof(true);
        descriptors[0].set_owner(Owner::Dma);
        descriptors[0].buffer = self.bounce_bufs[0].as_mut_ptr();

        descriptors[1] = DmaDescriptor::EMPTY;
        descriptors[1].set_size(bb_size);
        descriptors[1].set_length(bb_size);
        descriptors[1].set_suc_eof(true);
        descriptors[1].set_owner(Owner::Dma);
        descriptors[1].buffer = self.bounce_bufs[1].as_mut_ptr();

        descriptors[0].next = &mut descriptors[1] as *mut DmaDescriptor;
        descriptors[1].next = &mut descriptors[0] as *mut DmaDescriptor;
    }

    pub(crate) fn first_descriptor_ptr(&mut self) -> *mut DmaDescriptor {
        // SAFETY: Same as `setup_descriptors()` — `self.descriptors` always points to
        // valid static memory that lives for `'static`. `as_mut_ptr` on a mutable slice is safe.
        unsafe { (*self.descriptors).as_mut_ptr() }
    }

    pub(crate) fn initial_fill(&mut self) {
        self.fill_bounce_buf(0);
        self.fill_bounce_buf(1);
    }

    pub(crate) fn refill(&mut self, completed_buf_idx: usize) -> bool {
        self.fill_bounce_buf(completed_buf_idx);

        self.eof_count += 1;
        if self.eof_count >= self.expect_eof_count {
            self.eof_count = 0;
            true
        } else {
            false
        }
    }

    pub(crate) fn arm_interrupt(
        &self,
        channel: &mut ChannelTx<Blocking, PeripheralTxChannel<LCD_CAM<'_>>>,
    ) {
        channel.listen_out(DmaTxInterrupt::Eof);
    }

    pub(crate) fn install(self) {
        critical_section::with(|cs| {
            *BOUNCE_STATE.borrow_ref_mut(cs) = Some(self);
        });
    }

    /// Swap the active framebuffer pointer and reset position to start of frame.
    pub(crate) fn set_framebuffer(&mut self, fb: *const u8) {
        self.framebuffer = fb;
        self.bounce_pos = 0;
        self.eof_count = 0;
    }

    fn fill_bounce_buf(&mut self, bounce_buf_idx: usize) {
        let bb_size = self.bounce_bufs[0].len();

        if self.bounce_pos >= self.fb_len {
            self.bounce_pos = 0;
        }

        let end = self.bounce_pos + bb_size;

        // SAFETY: self.framebuffer points to a valid &'static mut [u8] of length
        // self.fb_len, set in new() or swapped via set_framebuffer(). bounce_pos
        // is always < fb_len and fb_len is a multiple of bb_size, so the range
        // [bounce_pos..bounce_pos+bb_size] is always within bounds.
        let src =
            unsafe { core::slice::from_raw_parts(self.framebuffer.add(self.bounce_pos), bb_size) };

        self.bounce_bufs[bounce_buf_idx].copy_from_slice(src);
        self.bounce_pos = end;
    }
}

#[cfg(feature = "unstable")]
// SAFETY: `DmaBounceBuffer` contains a raw pointer (`*mut [DmaDescriptor; 2]`) which prevents
// auto-derived `Send`/`Sync`. The pointer is constructed from a `&'static mut [DmaDescriptor; 2]`
// and is valid for the `'static` lifetime. Access to the pointed-to data is controlled
// exclusively through `DmaBounceBuffer` (via `&mut self` methods and `install()` which moves
// ownership into a `critical_section::Mutex`). It is therefore safe to send across threads
// and access behind a shared reference when protected by a critical section.
unsafe impl Send for DmaBounceBuffer {}
// SAFETY: See the safety rationale above for `Send`.
#[cfg(feature = "unstable")]
unsafe impl Sync for DmaBounceBuffer {}

#[cfg(feature = "unstable")]
/// Refill bounce buffers from the PSRAM framebuffer.
///
/// Called by [`DpiBounceTransfer::poll`] to check for completed DMA
/// transfers and copy the next framebuffer chunk into the finished bounce
/// buffer. Must be called with the DMA channel reference so the EOF
/// interrupt can be checked and cleared properly.
pub(crate) fn bounce_buffer_refill(
    channel: &mut ChannelTx<Blocking, PeripheralTxChannel<LCD_CAM<'_>>>,
) -> bool {
    if channel
        .pending_out_interrupts()
        .contains(DmaTxInterrupt::Eof)
    {
        channel.clear_out(DmaTxInterrupt::Eof);

        critical_section::with(|cs| {
            if let Some(state) = BOUNCE_STATE.borrow_ref_mut(cs).as_mut() {
                let buf_idx = state.eof_count % 2;
                return state.refill(buf_idx);
            }
            false
        })
    } else {
        false
    }
}

/// Represents the RGB LCD interface.
pub struct Dpi<'d, Dm: DriverMode> {
    lcd_cam: LCD_CAM<'d>,
    tx_channel: ChannelTx<Blocking, PeripheralTxChannel<LCD_CAM<'d>>>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,
    _mode: PhantomData<Dm>,
}

impl<'d, Dm> Dpi<'d, Dm>
where
    Dm: DriverMode,
{
    /// Create a new instance of the RGB/DPI driver.
    pub fn new(
        lcd: Lcd<'d, Dm>,
        channel: impl TxChannelFor<LCD_CAM<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_channel = ChannelTx::new(channel.degrade());
        // TODO: set eof_till_data_popped = false via PAC when field is accessible

        let mut this = Self {
            lcd_cam: lcd.lcd_cam,
            tx_channel,
            _guard: lcd._guard,
            _mode: PhantomData,
        };

        this.apply_config(&config)?;

        Ok(this)
    }

    fn regs(&self) -> &pac::lcd_cam::RegisterBlock {
        self.lcd_cam.register_block()
    }

    /// Applies the configuration to the peripheral.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Clock`] variant will be returned if the frequency passed
    /// in `Config` is too low.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Due to https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
        // the LCD_PCLK divider must be at least 2. To make up for this the user
        // provided frequency is doubled to match.
        let (i, divider) = ClockTree::with(|clocks| {
            calculate_clkm(
                (config.frequency.as_hz() * 2) as _,
                &[
                    crate::soc::clocks::xtal_clk_frequency(clocks) as usize,
                    crate::soc::clocks::pll_d2_frequency(clocks) as usize,
                    crate::soc::clocks::crypto_pwm_clk_frequency(clocks) as usize,
                ],
            )
        })
        .map_err(ConfigError::Clock)?;

        self.regs().lcd_clock().write(|w| unsafe {
            // Force enable the clock for all configuration registers.
            w.clk_en().set_bit();
            w.lcd_clk_sel().bits((i + 1) as _);
            w.lcd_clkm_div_num().bits(divider.div_num as _);
            w.lcd_clkm_div_b().bits(divider.div_b as _);
            w.lcd_clkm_div_a().bits(divider.div_a as _); // LCD_PCLK = LCD_CLK / 2
            w.lcd_clk_equ_sysclk().clear_bit();
            w.lcd_clkcnt_n().bits(2 - 1); // Must not be 0.
            w.lcd_ck_idle_edge()
                .bit(config.clock_mode.polarity == Polarity::IdleHigh);
            w.lcd_ck_out_edge()
                .bit(config.clock_mode.phase == Phase::ShiftHigh)
        });
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());

        self.regs()
            .lcd_rgb_yuv()
            .write(|w| w.lcd_conv_bypass().clear_bit());

        self.regs().lcd_user().modify(|_, w| {
            if config.format.enable_2byte_mode {
                w.lcd_8bits_order().bit(false);
                w.lcd_byte_order()
                    .bit(config.format.byte_order == ByteOrder::Inverted);
            } else {
                w.lcd_8bits_order()
                    .bit(config.format.byte_order == ByteOrder::Inverted);
                w.lcd_byte_order().bit(false);
            }
            w.lcd_bit_order()
                .bit(config.format.bit_order == BitOrder::Inverted);
            w.lcd_2byte_en().bit(config.format.enable_2byte_mode);

            // Only valid in Intel8080 mode.
            w.lcd_cmd().clear_bit();
            w.lcd_dummy().clear_bit();

            // This needs to be explicitly set for RGB mode.
            w.lcd_always_out_en().set_bit();
            w.lcd_dout().set_bit()
        });

        let timing = &config.timing;
        self.regs().lcd_ctrl().modify(|_, w| unsafe {
            // Enable RGB mode, and input VSYNC, HSYNC, and DE signals.
            w.lcd_rgb_mode_en().set_bit();

            w.lcd_hb_front()
                .bits((timing.horizontal_blank_front_porch as u16).saturating_sub(1));
            w.lcd_va_height()
                .bits((timing.vertical_active_height as u16).saturating_sub(1));
            w.lcd_vt_height()
                .bits((timing.vertical_total_height as u16).saturating_sub(1))
        });
        self.regs().lcd_ctrl1().modify(|_, w| unsafe {
            w.lcd_vb_front()
                .bits((timing.vertical_blank_front_porch as u8).saturating_sub(1));
            w.lcd_ha_width()
                .bits((timing.horizontal_active_width as u16).saturating_sub(1));
            w.lcd_ht_width()
                .bits((timing.horizontal_total_width as u16).saturating_sub(1))
        });
        self.regs().lcd_ctrl2().modify(|_, w| unsafe {
            w.lcd_vsync_width()
                .bits((timing.vsync_width as u8).saturating_sub(1));
            w.lcd_vsync_idle_pol().bit(config.vsync_idle_level.into());
            w.lcd_de_idle_pol().bit(config.de_idle_level.into());
            w.lcd_hs_blank_en().bit(config.hs_blank_en);
            w.lcd_hsync_width()
                .bits((timing.hsync_width as u8).saturating_sub(1));
            w.lcd_hsync_idle_pol().bit(config.hsync_idle_level.into());
            w.lcd_hsync_position().bits(timing.hsync_position as u8)
        });

        self.regs().lcd_misc().modify(|_, w| unsafe {
            // TODO: Find out what this field actually does.
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num().bits((1 << 5) - 1);

            // Doesn't matter for RGB mode.
            w.lcd_vfk_cyclelen().bits(0);
            w.lcd_vbk_cyclelen().bits(0);

            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().clear_bit();

            // Enable blank region when LCD sends data out.
            w.lcd_bk_en().bit(!config.disable_black_region)
        });
        self.regs().lcd_dly_mode().modify(|_, w| unsafe {
            w.lcd_de_mode().bits(config.de_mode as u8);
            w.lcd_hsync_mode().bits(config.hsync_mode as u8);
            w.lcd_vsync_mode().bits(config.vsync_mode as u8);
            w
        });
        self.regs().lcd_data_dout_mode().modify(|_, w| unsafe {
            w.dout0_mode().bits(config.output_bit_mode as u8);
            w.dout1_mode().bits(config.output_bit_mode as u8);
            w.dout2_mode().bits(config.output_bit_mode as u8);
            w.dout3_mode().bits(config.output_bit_mode as u8);
            w.dout4_mode().bits(config.output_bit_mode as u8);
            w.dout5_mode().bits(config.output_bit_mode as u8);
            w.dout6_mode().bits(config.output_bit_mode as u8);
            w.dout7_mode().bits(config.output_bit_mode as u8);
            w.dout8_mode().bits(config.output_bit_mode as u8);
            w.dout9_mode().bits(config.output_bit_mode as u8);
            w.dout10_mode().bits(config.output_bit_mode as u8);
            w.dout11_mode().bits(config.output_bit_mode as u8);
            w.dout12_mode().bits(config.output_bit_mode as u8);
            w.dout13_mode().bits(config.output_bit_mode as u8);
            w.dout14_mode().bits(config.output_bit_mode as u8);
            w.dout15_mode().bits(config.output_bit_mode as u8)
        });

        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit());

        Ok(())
    }

    /// Assign the VSYNC pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the VSYNC
    /// signal.
    pub fn with_vsync(self, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        OutputSignal::LCD_V_SYNC.connect_to(&pin);

        self
    }

    /// Assign the HSYNC pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the HSYNC
    /// signal.
    pub fn with_hsync(self, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        OutputSignal::LCD_H_SYNC.connect_to(&pin);

        self
    }

    /// Assign the DE pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DE
    /// signal.
    pub fn with_de(self, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        OutputSignal::LCD_H_ENABLE.connect_to(&pin);

        self
    }

    /// Assign the PCLK pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the PCLK
    /// signal.
    pub fn with_pclk(self, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        OutputSignal::LCD_PCLK.connect_to(&pin);

        self
    }

    fn with_data_pin(self, signal: OutputSignal, pin: impl PeripheralOutput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        signal.connect_to(&pin);

        self
    }

    /// Assign the DATA_0 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_0
    /// signal.
    pub fn with_data0(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_0, pin)
    }

    /// Assign the DATA_1 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_1
    /// signal.
    pub fn with_data1(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_1, pin)
    }

    /// Assign the DATA_2 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_2
    /// signal.
    pub fn with_data2(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_2, pin)
    }

    /// Assign the DATA_3 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_3
    /// signal.
    pub fn with_data3(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_3, pin)
    }

    /// Assign the DATA_4 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_4
    /// signal.
    pub fn with_data4(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_4, pin)
    }

    /// Assign the DATA_5 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_5
    /// signal.
    pub fn with_data5(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_5, pin)
    }

    /// Assign the DATA_6 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_6
    /// signal.
    pub fn with_data6(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_6, pin)
    }

    /// Assign the DATA_7 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_7
    /// signal.
    pub fn with_data7(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_7, pin)
    }

    /// Assign the DATA_8 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_8
    /// signal.
    pub fn with_data8(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_8, pin)
    }

    /// Assign the DATA_9 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the DATA_9
    /// signal.
    pub fn with_data9(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_9, pin)
    }

    /// Assign the DATA_10 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_10 signal.
    pub fn with_data10(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_10, pin)
    }

    /// Assign the DATA_11 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_11 signal.
    pub fn with_data11(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_11, pin)
    }

    /// Assign the DATA_12 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_12 signal.
    pub fn with_data12(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_12, pin)
    }

    /// Assign the DATA_13 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_13 signal.
    pub fn with_data13(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_13, pin)
    }

    /// Assign the DATA_14 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_14 signal.
    pub fn with_data14(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_14, pin)
    }

    /// Assign the DATA_15 pin for the LCD_CAM.
    ///
    /// Sets the specified pin to push-pull output and connects it to the
    /// DATA_15 signal.
    pub fn with_data15(self, pin: impl PeripheralOutput<'d>) -> Self {
        self.with_data_pin(OutputSignal::LCD_DATA_15, pin)
    }

    /// Start a continuous bounce-buffered RGB DPI transfer from a PSRAM framebuffer.
    ///
    /// Sets up a circular two-descriptor DMA chain that continuously streams data from
    /// `bounce_state`'s framebuffer to the LCD_CAM peripheral. Bounce buffers are
    /// refilled in software as part of the transfer's polling API.
    ///
    /// After calling this method, call [`DpiBounceTransfer::poll`] periodically to
    /// refill bounce buffers for continuous operation.
    ///
    /// # Errors
    ///
    /// Returns `(DmaError, Dpi, DmaBounceBuffer)` on DMA setup failure.
    #[cfg(feature = "unstable")]
    #[instability::unstable]
    pub fn send_bounce_buffered(
        mut self,
        mut bounce_state: DmaBounceBuffer,
    ) -> Result<DpiBounceTransfer<'d, Dm>, (DmaError, Dpi<'d, Dm>, DmaBounceBuffer)> {
        bounce_state.initial_fill();

        // SAFETY: `DmaTxBuffer::prepare()` for `DmaBounceBuffer` returns a `Preparation` whose
        // `start` pointer addresses the first element of a `'static` descriptor array. The
        // descriptors remain valid and at stable addresses for the entire duration of the
        // transfer.
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(DmaPeripheral::LcdCam, &mut bounce_state)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, bounce_state));
        }

        bounce_state.install();

        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.regs()
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        self.regs()
            .lcd_misc()
            .modify(|_, w| w.lcd_next_frame_en().set_bit());

        self.regs().lcd_user().modify(|_, w| {
            w.lcd_update().set_bit();
            w.lcd_start().set_bit()
        });

        Ok(DpiBounceTransfer {
            dpi: ManuallyDrop::new(self),
            back_buffer: None,
        })
    }

    /// Sending out the [DmaTxBuffer] to the RGB/DPI interface.
    ///
    /// - `next_frame_en`: Automatically send the next frame data when the current frame is sent
    ///   out.
    pub fn send<TX: DmaTxBuffer>(
        mut self,
        next_frame_en: bool,
        mut buf: TX,
    ) -> Result<DpiTransfer<'d, TX, Dm>, (DmaError, Self, TX)> {
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(DmaPeripheral::LcdCam, &mut buf)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, buf));
        }

        // Reset LCD control unit and Async Tx FIFO
        self.regs()
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.regs()
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        self.regs().lcd_misc().modify(|_, w| {
            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().bit(next_frame_en)
        });

        // Start the transfer.
        self.regs().lcd_user().modify(|_, w| {
            w.lcd_update().set_bit();
            w.lcd_start().set_bit()
        });

        Ok(DpiTransfer {
            dpi: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

#[cfg(feature = "unstable")]
unsafe impl DmaTxBuffer for DmaBounceBuffer {
    type View = ();
    type Final = ();

    fn prepare(&mut self) -> crate::dma::Preparation {
        crate::dma::Preparation {
            start: self.first_descriptor_ptr(),
            #[cfg(psram_dma)]
            accesses_psram: false,
            direction: crate::dma::TransferDirection::Out,
            burst_transfer: crate::dma::BurstConfig::default(),
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> Self::View {}

    fn from_view(_view: Self::View) -> Self::Final {}
}

#[cfg(feature = "unstable")]
#[instability::unstable]
/// Represents an ongoing continuous (bounce-buffered) DPI RGB transfer.
///
/// Stop with [`DpiBounceTransfer::stop`], or call [`DpiBounceTransfer::poll`]
/// periodically to refill bounce buffers.
pub struct DpiBounceTransfer<'d, Dm: DriverMode> {
    dpi: ManuallyDrop<Dpi<'d, Dm>>,
    back_buffer: Option<(*mut u8, usize)>,
}

#[cfg(feature = "unstable")]
impl<'d, Dm: DriverMode> DpiBounceTransfer<'d, Dm> {
    /// Returns true if the transfer is no longer active.
    #[instability::unstable]
    pub fn is_done(&self) -> bool {
        self.dpi.regs().lcd_user().read().lcd_start().bit_is_clear()
    }

    /// Stops the transfer and returns ownership of the DPI peripheral.
    #[instability::unstable]
    pub fn stop(mut self) -> Dpi<'d, Dm> {
        self.stop_peripheral();
        // SAFETY: This is the only path that extracts `self.dpi` from `ManuallyDrop`.
        // `core::mem::forget(self)` below prevents `Drop` from running and taking it again.
        let dpi = unsafe { ManuallyDrop::take(&mut self.dpi) };
        core::mem::forget(self);
        dpi
    }

    /// Poll for completed DMA transfers and refill bounce buffers.
    ///
    /// Call this in a tight loop to keep the display refreshed. Each call
    /// checks for a GDMA EOF event and, if one occurred, copies the next
    /// framebuffer chunk into the completed bounce buffer.
    #[instability::unstable]
    pub fn poll(&mut self) -> bool {
        bounce_buffer_refill(&mut self.dpi.tx_channel)
    }

    /// Register a back buffer for double-buffered rendering.
    ///
    /// The back buffer must be the same size as the front buffer passed to
    /// [`DmaBounceBuffer::new`]. After calling this, use [`Self::back_buffer`]
    /// to draw into it and [`Self::swap_buffers`] to swap at the next frame
    /// boundary.
    #[instability::unstable]
    pub fn set_back_buffer(&mut self, buf: &'static mut [u8]) {
        let valid = critical_section::with(|cs| {
            BOUNCE_STATE
                .borrow_ref(cs)
                .as_ref()
                .is_some_and(|state| buf.len() == state.fb_len)
        });
        assert!(valid, "back buffer length must match front buffer length");
        self.back_buffer = Some((buf.as_mut_ptr(), buf.len()));
    }

    /// Get a mutable reference to the back buffer for drawing.
    ///
    /// Returns `None` if no back buffer was registered via [`Self::set_back_buffer`].
    #[instability::unstable]
    pub fn back_buffer(&mut self) -> Option<&mut [u8]> {
        self.back_buffer.map(|(ptr, len)| {
            // SAFETY: ptr comes from a &'static mut [u8] passed to set_back_buffer().
            // We have exclusive access via &mut self.
            unsafe { core::slice::from_raw_parts_mut(ptr, len) }
        })
    }

    /// Swap front and back buffers at the next frame boundary.
    ///
    /// Polls until the current frame finishes, then atomically swaps which
    /// framebuffer the bounce buffer pipeline reads from. After this call,
    /// the old front buffer becomes the new back buffer (available via
    /// [`Self::back_buffer`]) and the old back buffer becomes the new front
    /// (displayed on screen).
    ///
    /// Does nothing if no back buffer was registered.
    #[instability::unstable]
    pub fn swap_buffers(&mut self) {
        let Some((back_ptr, back_len)) = self.back_buffer else {
            return;
        };

        loop {
            if self.poll() || self.is_done() {
                break;
            }
        }

        critical_section::with(|cs| {
            if let Some(state) = BOUNCE_STATE.borrow_ref_mut(cs).as_mut() {
                let old_fb = state.framebuffer;
                state.set_framebuffer(back_ptr as *const u8);
                self.back_buffer = Some((old_fb as *mut u8, back_len));
            }
        });
    }

    fn stop_peripheral(&mut self) {
        self.dpi.tx_channel.unlisten_out(DmaTxInterrupt::Eof);
        self.dpi.tx_channel.clear_out(DmaTxInterrupt::Eof);

        critical_section::with(|cs| {
            *BOUNCE_STATE.borrow_ref_mut(cs) = None;
        });

        self.dpi
            .regs()
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());
        self.dpi.tx_channel.stop_transfer();
    }
}

#[cfg(feature = "unstable")]
impl<Dm: DriverMode> Drop for DpiBounceTransfer<'_, Dm> {
    fn drop(&mut self) {
        self.stop_peripheral();
        // SAFETY: `drop()` is only called once, so `self.dpi` has not been taken.
        // This mirrors the pattern used in `DpiTransfer::drop()`.
        unsafe { ManuallyDrop::drop(&mut self.dpi) };
    }
}

/// Represents an ongoing (or potentially finished) transfer using the RGB LCD
/// interface
pub struct DpiTransfer<'d, BUF: DmaTxBuffer, Dm: DriverMode> {
    dpi: ManuallyDrop<Dpi<'d, Dm>>,
    buffer_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF: DmaTxBuffer, Dm: DriverMode> DpiTransfer<'d, BUF, Dm> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.dpi.regs().lcd_user().read().lcd_start().bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(mut self) -> (Dpi<'d, Dm>, BUF::Final) {
        self.stop_peripherals();
        let (dpi, view) = self.release();
        (dpi, BUF::from_view(view))
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: If you specified `next_frame_en` as true in [Dpi::send], you're
    /// just waiting for a DMA error when you call this.
    pub fn wait(mut self) -> (Result<(), DmaError>, Dpi<'d, Dm>, BUF::Final) {
        while !self.is_done() {
            core::hint::spin_loop();
        }

        // Stop the DMA.
        //
        // If the user sends more data to the DMA than the LCD_CAM needs for a single
        // frame, the DMA will still be running after the LCD_CAM stops.
        self.dpi.tx_channel.stop_transfer();

        // Note: There is no "done" interrupt to clear.

        let (dpi, view) = self.release();
        let result = if dpi.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, dpi, BUF::from_view(view))
    }

    fn release(mut self) -> (Dpi<'d, Dm>, BUF::View) {
        // SAFETY: Since forget is called on self, we know that self.dpi and
        // self.buffer_view won't be touched again.
        let result = unsafe {
            let dpi = ManuallyDrop::take(&mut self.dpi);
            let view = ManuallyDrop::take(&mut self.buffer_view);
            (dpi, view)
        };
        core::mem::forget(self);
        result
    }

    fn stop_peripherals(&mut self) {
        // Stop the LCD_CAM peripheral.
        self.dpi
            .regs()
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        // Stop the DMA
        self.dpi.tx_channel.stop_transfer();
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Deref for DpiTransfer<'_, BUF, Dm> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> DerefMut for DpiTransfer<'_, BUF, Dm> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Drop for DpiTransfer<'_, BUF, Dm> {
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.dpi and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.dpi);
            ManuallyDrop::take(&mut self.buffer_view)
        };
        let _ = BUF::from_view(view);
    }
}

/// Configuration settings for the RGB/DPI interface.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Specifies the clock mode, including polarity and phase settings.
    clock_mode: ClockMode,

    /// The frequency of the pixel clock.
    frequency: Rate,

    /// Format of the byte data sent out.
    format: Format,

    /// Timing settings for the peripheral.
    timing: FrameTiming,

    /// The vsync signal level in IDLE state.
    vsync_idle_level: Level,

    /// The hsync signal level in IDLE state.
    hsync_idle_level: Level,

    /// The de signal level in IDLE state.
    de_idle_level: Level,

    /// If enabled, the hsync pulse will be sent out in vertical blanking lines.
    /// i.e. When no valid data is actually sent out. Otherwise, hysnc
    /// pulses will only be sent out in active region lines.
    hs_blank_en: bool,

    /// Disables blank region when LCD sends data out.
    disable_black_region: bool,

    /// The output LCD_DE is delayed by module clock LCD_CLK.
    de_mode: DelayMode,
    /// The output LCD_HSYNC is delayed by module clock LCD_CLK.
    hsync_mode: DelayMode,
    /// The output LCD_VSYNC is delayed by module clock LCD_CLK.
    vsync_mode: DelayMode,
    /// The output data bits are delayed by module clock LCD_CLK.
    output_bit_mode: DelayMode,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            clock_mode: Default::default(),
            frequency: Rate::from_mhz(1),
            format: Default::default(),
            timing: Default::default(),
            vsync_idle_level: Level::Low,
            hsync_idle_level: Level::Low,
            de_idle_level: Level::Low,
            hs_blank_en: true,
            disable_black_region: false,
            de_mode: Default::default(),
            hsync_mode: Default::default(),
            vsync_mode: Default::default(),
            output_bit_mode: Default::default(),
        }
    }
}

/// Controls how the peripheral should treat data received from the DMA.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Format {
    /// Configures the bit order for data transmission.
    pub bit_order: BitOrder,

    /// Configures the byte order for data transmission.
    ///
    /// - In 8-bit mode, [ByteOrder::Inverted] means every two bytes are swapped.
    /// - In 16-bit mode, this controls the byte order (endianness).
    pub byte_order: ByteOrder,

    /// If true, the width of the output is 16 bits.
    /// Otherwise, the width of the output is 8 bits.
    pub enable_2byte_mode: bool,
}

/// The timing numbers for the driver to follow.
///
/// Note: The names of the fields in this struct don't match what you
/// would typically find in an LCD's datasheet. Carefully read the doc on each
/// field to understand what to set it to.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameTiming {
    /// The horizontal total width of a frame (in units of PCLK).
    ///
    /// This should be greater than `horizontal_blank_front_porch` +
    /// `horizontal_active_width`.
    ///
    /// Max is 4096 (12 bits).
    pub horizontal_total_width: usize,

    /// The horizontal blank front porch of a frame (in units of PCLK).
    ///
    /// This is the number of PCLKs between the start of the line and the start
    /// of active data in the line.
    ///
    /// Note: This includes `hsync_width`.
    ///
    /// Max is 2048 (11 bits).
    pub horizontal_blank_front_porch: usize,

    /// The horizontal active width of a frame. i.e. The number of pixels in a
    /// line. This is typically the horizontal resolution of the screen.
    ///
    /// Max is 4096 (12 bits).
    pub horizontal_active_width: usize,

    /// The vertical total height of a frame (in units of lines).
    ///
    /// This should be greater than `vertical_blank_front_porch` +
    /// `vertical_active_height`.
    ///
    /// Max is 1024 (10 bits).
    pub vertical_total_height: usize,

    /// The vertical blank front porch height of a frame (in units of lines).
    ///
    /// This is the number of (blank/invalid) lines before the start of the
    /// frame.
    ///
    /// Note: This includes `vsync_width`.
    ///
    /// Max is 256 (8 bits).
    pub vertical_blank_front_porch: usize,

    /// The vertical active height of a frame. i.e. The number of lines in a
    /// frame. This is typically the vertical resolution of the screen.
    ///
    /// Max is 1024 (10 bits).
    pub vertical_active_height: usize,

    /// It is the width of LCD_VSYNC active pulse in a line (in units of lines).
    ///
    /// Max is 128 (7 bits).
    pub vsync_width: usize,

    /// The width of LCD_HSYNC active pulse in a line (in units of PCLK).
    ///
    /// This should be less than vertical_blank_front_porch, otherwise the hsync
    /// pulse will overlap with valid pixel data.
    ///
    /// Max is 128 (7 bits).
    pub hsync_width: usize,

    /// It is the position of LCD_HSYNC active pulse in a line (in units of
    /// PCLK).
    ///
    /// This should be less than horizontal_total_width.
    ///
    /// Max is 128 (7 bits).
    pub hsync_position: usize,
}
