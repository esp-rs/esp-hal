//! # LCD - RGB/Digital Parallel Interface Mode
//!
//! ## Overview
//!
//! ## Example
//!
//! ### A display.

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{ChannelTx, DmaChannelConvert, DmaEligible, DmaError, DmaPeripheral, DmaTxBuffer, Tx},
    gpio::{interconnect::PeripheralOutput, Level, OutputSignal},
    lcd_cam::{
        lcd::{ClockMode, DelayMode, Lcd, Phase, Polarity},
        private::calculate_clkm,
        BitOrder,
        ByteOrder,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
    Mode,
};

/// Represents the RGB LCD interface.
pub struct Dpi<'d> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: ChannelTx<'d, <LCD_CAM as DmaEligible>::Dma>,
}

impl<'d> Dpi<'d> {
    /// Create a new instance of the RGB/DPI driver.
    pub fn new<DM: Mode, CH>(
        lcd: Lcd<'d, DM>,
        channel: ChannelTx<'d, CH>,
        frequency: HertzU32,
        config: Config,
    ) -> Self
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
    {
        let lcd_cam = lcd.lcd_cam;

        let clocks = Clocks::get();
        // Due to https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
        // the LCD_PCLK divider must be at least 2. To make up for this the user
        // provided frequency is doubled to match.
        let (i, divider) = calculate_clkm(
            (frequency.to_Hz() * 2) as _,
            &[
                clocks.xtal_clock.to_Hz() as _,
                clocks.cpu_clock.to_Hz() as _,
                clocks.crypto_pwm_clock.to_Hz() as _,
            ],
        );

        lcd_cam.lcd_clock().write(|w| unsafe {
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
        lcd_cam.lcd_user().modify(|_, w| w.lcd_reset().set_bit());

        lcd_cam
            .lcd_rgb_yuv()
            .write(|w| w.lcd_conv_bypass().clear_bit());

        lcd_cam.lcd_user().modify(|_, w| {
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
            w.lcd_dout().set_bit()
        });

        let timing = &config.timing;
        lcd_cam.lcd_ctrl().modify(|_, w| unsafe {
            // Enable RGB mode, and input VSYNC, HSYNC, and DE signals.
            w.lcd_rgb_mode_en().set_bit();

            w.lcd_hb_front()
                .bits((timing.horizontal_blank_front_porch as u16).saturating_sub(1));
            w.lcd_va_height()
                .bits((timing.vertical_active_height as u16).saturating_sub(1));
            w.lcd_vt_height()
                .bits((timing.vertical_total_height as u16).saturating_sub(1))
        });
        lcd_cam.lcd_ctrl1().modify(|_, w| unsafe {
            w.lcd_vb_front()
                .bits((timing.vertical_blank_front_porch as u8).saturating_sub(1));
            w.lcd_ha_width()
                .bits((timing.horizontal_active_width as u16).saturating_sub(1));
            w.lcd_ht_width()
                .bits((timing.horizontal_total_width as u16).saturating_sub(1))
        });
        lcd_cam.lcd_ctrl2().modify(|_, w| unsafe {
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

        lcd_cam.lcd_misc().modify(|_, w| unsafe {
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
        lcd_cam.lcd_dly_mode().modify(|_, w| unsafe {
            w.lcd_de_mode().bits(config.de_mode as u8);
            w.lcd_hsync_mode().bits(config.hsync_mode as u8);
            w.lcd_vsync_mode().bits(config.vsync_mode as u8);
            w
        });
        lcd_cam.lcd_data_dout_mode().modify(|_, w| unsafe {
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

        lcd_cam.lcd_user().modify(|_, w| w.lcd_update().set_bit());

        Self {
            lcd_cam,
            tx_channel: channel.degrade(),
        }
    }

    /// Configures the control pins for the RGB/DPI interface.
    pub fn with_ctrl_pins<
        VSYNC: PeripheralOutput,
        HSYNC: PeripheralOutput,
        DE: PeripheralOutput,
        PCLK: PeripheralOutput,
    >(
        self,
        vsync: impl Peripheral<P = VSYNC> + 'd,
        hsync: impl Peripheral<P = HSYNC> + 'd,
        de: impl Peripheral<P = DE> + 'd,
        pclk: impl Peripheral<P = PCLK> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(vsync);
        crate::into_mapped_ref!(hsync);
        crate::into_mapped_ref!(de);
        crate::into_mapped_ref!(pclk);

        vsync.set_to_push_pull_output(crate::private::Internal);
        hsync.set_to_push_pull_output(crate::private::Internal);
        de.set_to_push_pull_output(crate::private::Internal);
        pclk.set_to_push_pull_output(crate::private::Internal);

        vsync.connect_peripheral_to_output(OutputSignal::LCD_V_SYNC, crate::private::Internal);
        hsync.connect_peripheral_to_output(OutputSignal::LCD_H_SYNC, crate::private::Internal);
        de.connect_peripheral_to_output(OutputSignal::LCD_H_ENABLE, crate::private::Internal);
        pclk.connect_peripheral_to_output(OutputSignal::LCD_PCLK, crate::private::Internal);

        self
    }

    /// Configures the data pins for the RGB/DPI interface.
    #[allow(clippy::too_many_arguments)]
    pub fn with_data_pins<
        D0: PeripheralOutput,
        D1: PeripheralOutput,
        D2: PeripheralOutput,
        D3: PeripheralOutput,
        D4: PeripheralOutput,
        D5: PeripheralOutput,
        D6: PeripheralOutput,
        D7: PeripheralOutput,
        D8: PeripheralOutput,
        D9: PeripheralOutput,
        D10: PeripheralOutput,
        D11: PeripheralOutput,
        D12: PeripheralOutput,
        D13: PeripheralOutput,
        D14: PeripheralOutput,
        D15: PeripheralOutput,
    >(
        self,
        d0: impl Peripheral<P = D0> + 'd,
        d1: impl Peripheral<P = D1> + 'd,
        d2: impl Peripheral<P = D2> + 'd,
        d3: impl Peripheral<P = D3> + 'd,
        d4: impl Peripheral<P = D4> + 'd,
        d5: impl Peripheral<P = D5> + 'd,
        d6: impl Peripheral<P = D6> + 'd,
        d7: impl Peripheral<P = D7> + 'd,
        d8: impl Peripheral<P = D8> + 'd,
        d9: impl Peripheral<P = D9> + 'd,
        d10: impl Peripheral<P = D10> + 'd,
        d11: impl Peripheral<P = D11> + 'd,
        d12: impl Peripheral<P = D12> + 'd,
        d13: impl Peripheral<P = D13> + 'd,
        d14: impl Peripheral<P = D14> + 'd,
        d15: impl Peripheral<P = D15> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(d0);
        crate::into_mapped_ref!(d1);
        crate::into_mapped_ref!(d2);
        crate::into_mapped_ref!(d3);
        crate::into_mapped_ref!(d4);
        crate::into_mapped_ref!(d5);
        crate::into_mapped_ref!(d6);
        crate::into_mapped_ref!(d7);
        crate::into_mapped_ref!(d8);
        crate::into_mapped_ref!(d9);
        crate::into_mapped_ref!(d10);
        crate::into_mapped_ref!(d11);
        crate::into_mapped_ref!(d12);
        crate::into_mapped_ref!(d13);
        crate::into_mapped_ref!(d14);
        crate::into_mapped_ref!(d15);

        d0.set_to_push_pull_output(crate::private::Internal);
        d1.set_to_push_pull_output(crate::private::Internal);
        d2.set_to_push_pull_output(crate::private::Internal);
        d3.set_to_push_pull_output(crate::private::Internal);
        d4.set_to_push_pull_output(crate::private::Internal);
        d5.set_to_push_pull_output(crate::private::Internal);
        d6.set_to_push_pull_output(crate::private::Internal);
        d7.set_to_push_pull_output(crate::private::Internal);
        d8.set_to_push_pull_output(crate::private::Internal);
        d9.set_to_push_pull_output(crate::private::Internal);
        d10.set_to_push_pull_output(crate::private::Internal);
        d11.set_to_push_pull_output(crate::private::Internal);
        d12.set_to_push_pull_output(crate::private::Internal);
        d13.set_to_push_pull_output(crate::private::Internal);
        d14.set_to_push_pull_output(crate::private::Internal);
        d15.set_to_push_pull_output(crate::private::Internal);

        d0.connect_peripheral_to_output(OutputSignal::LCD_DATA_0, crate::private::Internal);
        d1.connect_peripheral_to_output(OutputSignal::LCD_DATA_1, crate::private::Internal);
        d2.connect_peripheral_to_output(OutputSignal::LCD_DATA_2, crate::private::Internal);
        d3.connect_peripheral_to_output(OutputSignal::LCD_DATA_3, crate::private::Internal);
        d4.connect_peripheral_to_output(OutputSignal::LCD_DATA_4, crate::private::Internal);
        d5.connect_peripheral_to_output(OutputSignal::LCD_DATA_5, crate::private::Internal);
        d6.connect_peripheral_to_output(OutputSignal::LCD_DATA_6, crate::private::Internal);
        d7.connect_peripheral_to_output(OutputSignal::LCD_DATA_7, crate::private::Internal);
        d8.connect_peripheral_to_output(OutputSignal::LCD_DATA_8, crate::private::Internal);
        d9.connect_peripheral_to_output(OutputSignal::LCD_DATA_9, crate::private::Internal);
        d10.connect_peripheral_to_output(OutputSignal::LCD_DATA_10, crate::private::Internal);
        d11.connect_peripheral_to_output(OutputSignal::LCD_DATA_11, crate::private::Internal);
        d12.connect_peripheral_to_output(OutputSignal::LCD_DATA_12, crate::private::Internal);
        d13.connect_peripheral_to_output(OutputSignal::LCD_DATA_13, crate::private::Internal);
        d14.connect_peripheral_to_output(OutputSignal::LCD_DATA_14, crate::private::Internal);
        d15.connect_peripheral_to_output(OutputSignal::LCD_DATA_15, crate::private::Internal);

        self
    }

    /// Same as [Self::with_data_pins] but specifies which pin likely
    /// corresponds to which color.
    #[allow(clippy::too_many_arguments)]
    pub fn with_color_pins<
        D0: PeripheralOutput,
        D1: PeripheralOutput,
        D2: PeripheralOutput,
        D3: PeripheralOutput,
        D4: PeripheralOutput,
        D5: PeripheralOutput,
        D6: PeripheralOutput,
        D7: PeripheralOutput,
        D8: PeripheralOutput,
        D9: PeripheralOutput,
        D10: PeripheralOutput,
        D11: PeripheralOutput,
        D12: PeripheralOutput,
        D13: PeripheralOutput,
        D14: PeripheralOutput,
        D15: PeripheralOutput,
    >(
        self,
        b0: impl Peripheral<P = D0> + 'd,
        b1: impl Peripheral<P = D1> + 'd,
        b2: impl Peripheral<P = D2> + 'd,
        b3: impl Peripheral<P = D3> + 'd,
        b4: impl Peripheral<P = D4> + 'd,
        g0: impl Peripheral<P = D5> + 'd,
        g1: impl Peripheral<P = D6> + 'd,
        g2: impl Peripheral<P = D7> + 'd,
        g3: impl Peripheral<P = D8> + 'd,
        g4: impl Peripheral<P = D9> + 'd,
        g5: impl Peripheral<P = D10> + 'd,
        r0: impl Peripheral<P = D11> + 'd,
        r1: impl Peripheral<P = D12> + 'd,
        r2: impl Peripheral<P = D13> + 'd,
        r3: impl Peripheral<P = D14> + 'd,
        r4: impl Peripheral<P = D15> + 'd,
    ) -> Self {
        self.with_data_pins(
            b0, b1, b2, b3, b4, g0, g1, g2, g3, g4, g5, r0, r1, r2, r3, r4,
        )
    }

    /// Sending out the [DmaTxBuffer] to the RGB/DPI interface.
    ///
    /// - `next_frame_en`: Automatically send the next frame data when the
    ///   current frame is sent out.
    pub fn send<TX: DmaTxBuffer>(
        mut self,
        next_frame_en: bool,
        mut buf: TX,
    ) -> Result<DpiTransfer<'d, TX>, (DmaError, Self, TX)> {
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(DmaPeripheral::LcdCam, &mut buf)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, buf));
        }

        // Reset LCD control unit and Async Tx FIFO
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        self.lcd_cam.lcd_misc().modify(|_, w| {
            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().bit(next_frame_en)
        });

        // Start the transfer.
        self.lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_update().set_bit();
            w.lcd_start().set_bit()
        });

        Ok(DpiTransfer {
            dpi: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially finished) transfer using the RGB LCD
/// interface
pub struct DpiTransfer<'d, BUF: DmaTxBuffer> {
    dpi: ManuallyDrop<Dpi<'d>>,
    buffer_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF: DmaTxBuffer> DpiTransfer<'d, BUF> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.dpi
            .lcd_cam
            .lcd_user()
            .read()
            .lcd_start()
            .bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(mut self) -> (Dpi<'d>, BUF) {
        self.stop_peripherals();
        let (dpi, view) = self.release();
        (dpi, BUF::from_view(view))
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: If you specified `next_frame_en` as true in [Dpi::send], you're
    /// just waiting for a DMA error when you call this.
    pub fn wait(mut self) -> (Result<(), DmaError>, Dpi<'d>, BUF) {
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

    fn release(mut self) -> (Dpi<'d>, BUF::View) {
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
            .lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        // Stop the DMA
        self.dpi.tx_channel.stop_transfer();
    }
}

impl<'d, BUF: DmaTxBuffer> Deref for DpiTransfer<'d, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<'d, BUF: DmaTxBuffer> DerefMut for DpiTransfer<'d, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<'d, BUF: DmaTxBuffer> Drop for DpiTransfer<'d, BUF> {
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

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration settings for the RGB/DPI interface.
pub struct Config {
    /// Specifies the clock mode, including polarity and phase settings.
    pub clock_mode: ClockMode,

    /// Format of the byte data sent out.
    pub format: Format,

    /// Timing settings for the peripheral.
    pub timing: FrameTiming,

    /// The vsync signal level in IDLE state.
    pub vsync_idle_level: Level,

    /// The hsync signal level in IDLE state.
    pub hsync_idle_level: Level,

    /// The de signal level in IDLE state.
    pub de_idle_level: Level,

    /// If enabled, the hsync pulse will be sent out in vertical blanking lines.
    /// i.e. When no valid data is actually sent out. Otherwise, hysnc
    /// pulses will only be sent out in active region lines.
    pub hs_blank_en: bool,

    /// Disables blank region when LCD sends data out.
    pub disable_black_region: bool,

    /// The output LCD_DE is delayed by module clock LCD_CLK.
    pub de_mode: DelayMode,
    /// The output LCD_HSYNC is delayed by module clock LCD_CLK.
    pub hsync_mode: DelayMode,
    /// The output LCD_VSYNC is delayed by module clock LCD_CLK.
    pub vsync_mode: DelayMode,
    /// The output data bits are delayed by module clock LCD_CLK.
    pub output_bit_mode: DelayMode,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            clock_mode: Default::default(),
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
    /// - In 8-bit mode, [ByteOrder::Inverted] means every two bytes are
    ///   swapped.
    /// - In 16-bit mode, this controls the byte order (endianness).
    pub byte_order: ByteOrder,

    /// If true, the width of the output is 16 bits.
    /// Otherwise, the width of the output is 8 bits.
    pub enable_2byte_mode: bool,
}

/// The timing numbers for the driver to follow.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameTiming {
    /// The horizontal total width of a frame.
    ///
    /// This should be greater than `horizontal_blank_front_porch` +
    /// `horizontal_active_width`.
    ///
    /// Max is 4096 (12 bits).
    pub horizontal_total_width: usize,

    /// The horizontal blank front porch of a frame.
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

    /// The vertical total height of a frame.
    ///
    /// This should be greater than `vertical_blank_front_porch` +
    /// `vertical_active_height`.
    ///
    /// Max is 1024 (10 bits).
    pub vertical_total_height: usize,

    /// The vertical blank front porch height of a frame.
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

    /// It is the width of LCD_VSYNC active pulse in a line.
    ///
    /// Max is 128 (7 bits).
    pub vsync_width: usize,

    /// The width of LCD_HSYNC active pulse in a line.
    ///
    /// Max is 128 (7 bits).
    pub hsync_width: usize,

    /// It is the position of LCD_HSYNC active pulse in a line.
    ///
    /// Max is 128 (7 bits).
    pub hsync_position: usize,
}
