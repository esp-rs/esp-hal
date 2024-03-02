//! LCD
//!
//! This module is capable of operating in either RGB (not yet implemented),
//! MOTO6800 or I8080 mode. For more information on these modes, please refer
//! to the documentation in their respective modules.

use super::{BitOrder, ByteOrder, LcdClockSource};
use crate::{peripheral::PeripheralRef, peripherals::LCD_CAM};
use bitflags::bitflags;

pub mod i8080;

/// LCD_CLK = LCD_CLK_S / (N + b/a), the N register is 8 bit-width
const LCD_CLK_FRAC_DIV_N_MAX: u32 = 256;
/// LCD_PCLK = LCD_CLK / MO, the MO register is 6 bit-width
const LCD_PCLK_DIV_MAX: u32 = 64;

/// LCD peripheral
pub struct Lcd<'d> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
}
impl<'d> Lcd<'d> {
    /// Enable the LCD peripheral's clockgating.
    /// This basically wakes up the peripheral and makes it consume energy.
    #[inline(always)]
    pub fn enable_clock(&self, enabled: bool) {
        self.lcd_cam
            .lcd_clock()
            .modify(|_, w| w.clk_en().bit(enabled));
    }

    /// Select the clock source to drive the LCD peripheral with.
    ///
    /// # Arguments
    /// * `clock_src` - The clock to drive the LCD peripheral
    #[inline(always)]
    pub fn select_clock_src(&self, clock_src: LcdClockSource) {
        self.lcd_cam
            .lcd_clock()
            .modify(|_, w| w.lcd_clk_sel().variant(clock_src as _));
    }

    /// Set clock divider of LCD peripheral.
    /// Most often, the source clock is not used 1:1 but instead modified by a
    /// divider in order to construct lower frequencies from it to drive
    /// peripherals with. This divider is configured here, consisting of the
    /// three components: N b/a
    ///
    /// # Arguments
    /// * `div_num` - (N) Integer part of the clock divider
    /// * `div_a` - (a) Denominator part of the divider
    /// * `div_b` - (b) Numerator part of the clock divider
    #[inline(always)]
    pub fn set_group_clock_coeff(&self, mut div_num: u32, div_a: u32, div_b: u32) {
        // lcd_clk = module_clock_src / (N + b/a)
        debug_assert!(div_num >= 2 && div_num <= LCD_CLK_FRAC_DIV_N_MAX);
        // div_full == 0 means LCD_LL_CLK_FRAC_DIV_N_MAX divider in hardware
        if div_num >= LCD_CLK_FRAC_DIV_N_MAX {
            div_num = 0;
        }
        self.lcd_cam
            .lcd_clock()
            .modify(|_, w|
                w.lcd_clkm_div_num().variant(div_num as _)
                    .lcd_clkm_div_a().variant(div_a as _)
                    .lcd_clkm_div_b().variant(div_b as _)
            );
    }

    /// Configure the PCLK clock level that should be used while no transaction
    /// is active.
    ///
    /// # Arguments
    /// * `level` - PCLK clock level during idle, where high(`true`) and
    /// low(`false`).
    #[inline(always)]
    pub fn set_clock_idle_level(&self, level: bool) {
        self.lcd_cam
            .lcd_clock()
            .modify(|_, w| w.lcd_ck_idle_edge().bit(level));
    }

    /// Set the PCLK sample edge
    ///
    /// # Arguments
    /// * `phase` - Edge phase of the pixel clock at which to sample
    #[inline(always)]
    pub fn set_pixel_clock_edge(&self, phase: Phase) {
        self.lcd_cam
            .lcd_clock()
            .modify(|_, w| w.lcd_ck_out_edge().bit(phase == Phase::ShiftLow));
    }

    /// Set pixel clock prescaler.
    /// This configures the ratio between the configured LCD peripheral clock
    /// and the pixel clock, which is used to push out pixels with.
    /// Essentially, this is a second clock divider: PCLK = LCD_CLK /
    /// prescale
    #[inline(always)]
    pub fn set_pixel_clock_prescale(&self, prescale: u32) {
        debug_assert!(prescale > 0);
        debug_assert!(prescale <= LCD_PCLK_DIV_MAX);
        // Formula: pixel_clk = lcd_clk / (1 + clkcnt_n)
        // clkcnt_n can't be zero
        let clkcnt_n = (prescale - 1).max(0);
        // signals that pclk == lcd_clk
        let lcd_clk_equ_sysclk = prescale == 1;
        self.lcd_cam.lcd_clock().modify(|_, w| {
            w.lcd_clk_equ_sysclk()
                .bit(lcd_clk_equ_sysclk)
                .lcd_clkcnt_n()
                .variant(clkcnt_n as _)
        });
    }

    /// Enable/Disable the LCD peripheral's internal RGB-YUV color format
    /// converter.
    pub fn enable_rgb_yuv_converter(&self, enabled: bool) {
        self.lcd_cam
            .lcd_rgb_yuv()
            .modify(|_, w| w.lcd_conv_bypass().bit(enabled));
    }

    // TODO: rest of the yuv-rgb convert stuff

    /// Configure the clock cycles to spend in each of the three transaction
    /// phases. For RGB-Mode, all phases except the data phase are disabled.
    ///
    /// # Arguments
    /// * `cmd_cycles` - Clock cycles to spend in the CMD phase
    /// * `dummy_cycles` - Clock cycles to spend in the CMD phase
    /// * `data_cycles` - Clock cycles to spend in the DATA phase.
    #[inline(always)]
    pub fn set_phase_cycles(
        &self,
        cmd_cycles: Option<u32>,
        dummy_cycles: Option<u32>,
        data_cycles: Option<u32>,
    ) {
        self.lcd_cam.lcd_user().modify(|_, w| {
            if let Some(cmd_cycles) = cmd_cycles {
                debug_assert!(cmd_cycles <= 2);
                w.lcd_cmd()
                    .bit(cmd_cycles > 0)
                    .lcd_cmd_2_cycle_en()
                    .bit(cmd_cycles > 1);
            }
            if let Some(dummy_cycles) = dummy_cycles {
                w.lcd_dummy()
                    .bit(dummy_cycles > 0)
                    .lcd_dummy_cyclelen()
                    .variant(dummy_cycles.wrapping_sub(1) as _);
            }
            if let Some(data_cycles) = data_cycles {
                w.lcd_dout()
                    .bit(data_cycles > 0)
                    .lcd_dout_cyclelen()
                    .variant(data_cycles.wrapping_sub(1) as _);
            }
            w
        });
    }

    /// Configure the amount of cycles for blank phases
    ///
    /// # Arguments
    /// * `fk_cycles` - Clock cycles to spend for front blank
    /// * `bk_cycles` - Clock cycles to spend for back blank
    #[inline(always)]
    pub fn set_blank_cycles(&self, fk_cycles: u32, bk_cycles: u32) {
        self.lcd_cam.lcd_misc().modify(|_, w| {
            w.lcd_bk_en()
                .bit(fk_cycles > 0 || bk_cycles > 0)
                .lcd_vfk_cyclelen()
                .variant(fk_cycles.wrapping_sub(1) as _)
                .lcd_vbk_cyclelen()
                .variant(bk_cycles.wrapping_sub(1) as _)
        });
    }

    /// Configure the data line width
    /// This is essentially the amount of bits per pixel and thus either 8 or
    /// 16.
    #[inline(always)]
    pub fn set_data_width(&self, width: BusWidth) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_2byte_en().bit(width == BusWidth::Bit16));
    }

    /// Configures whether the LCD peripheral continues the data phase as long
    /// as the DMA has content to send.
    ///
    /// # Arguments
    /// * `en` - `true`: The number of data cycles will be controlled by DMA
    /// buffer size, instead of lcd_dout_cyclelen        `false`: The number
    /// of data cycles will be controlled by lcd_dout_cyclelen
    #[inline(always)]
    pub fn enable_output_always_on(&self, enabled: bool) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_always_out_en().bit(enabled));
    }

    /// Start the LCD transaction
    /// This will cause this LCD peripheral to start pushing data from the FIFO.
    ///
    /// # Warning
    /// Make sure data in the FIFO is ready *before* calling this method,
    /// otherwise garbage data will be pushed into the display.
    #[inline(always)]
    pub fn start(&self) {
        // update parameters before actually starting the transaction
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit());
        // start transaction
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().set_bit());
    }

    /// Stop the LCD transaction
    #[inline(always)]
    pub fn stop(&self) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit());
    }

    /// Reset LCD TX controller and RGB/YUV converter
    ///
    /// # Warning
    /// This does not reset the data currently contained in the TX FIFO buffer.
    #[inline(always)]
    pub fn reset(&self) {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
    }

    /// Reset the LCD peripheral's asynchronous TX FIFO buffer.
    #[inline(always)]
    pub fn fifo_reset(&self) {
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());
    }

    /// (I8080-Mode) Set the level of the DC line for different transaction phases.
    ///
    /// # Arguments
    /// * `idle_phase` - Level of the DC line during IDLE phase
    /// * `cmd_phase` - Level of the DC line during CMD phase
    /// * `dummy_phase` - Level of the DC line during DUMMY phase
    /// * `data_phase` - Level of the DC line during DATA phase
    #[inline(always)]
    pub fn set_dc_level(
        &self,
        idle_phase: Polarity,
        cmd_phase: Polarity,
        dummy_phase: Polarity,
        data_phase: Polarity,
    ) {
        self.lcd_cam.lcd_misc().modify(|_, w| {
            w.lcd_cd_idle_edge()
                .bit(idle_phase == Polarity::IdleHigh)
                .lcd_cd_cmd_set()
                .bit(cmd_phase != idle_phase)
                .lcd_cd_dummy_set()
                .bit(dummy_phase != idle_phase)
                .lcd_cd_data_set()
                .bit(data_phase != idle_phase)
        });
    }

    /// (I8080-Mode) Set the DC line delay as number of cycles.
    #[inline(always)]
    pub fn set_dc_delay_ticks(&self, delay: DelayMode) {
        self.lcd_cam
            .lcd_dly_mode()
            .modify(|_, w| w.lcd_cd_mode().variant(delay as _));
    }

    /// (I8080-Mode) Set the LCD command (data for the CMD phase).
    /// This automatically changes the value format of `command` for the
    /// register and thus expects it to either contain 32bit of data (32bit
    /// data_width), or 16bit (8bit data_width) in the lower half.
    ///
    /// # Arguments
    /// * `data_width` - Width of a data line
    /// * `command` - Command value
    #[inline(always)]
    pub fn set_command(&self, data_width: BusWidth, mut command: u32) {
        // if command phase has two cycles, in the first cycle, command[15:0] is sent
        // out via lcd_data_out[15:0] in the second cycle, command[31:16] is
        // sent out via lcd_data_out[15:0]
        if data_width == BusWidth::Bit8 {
            // move second command 8 bit upwards
            command = (command & 0xFF) | (command & 0xFF00) << 8;
        }
        self.set_command_raw(command);
    }

    /// Set the raw LCD command register (data for the CMD phase).
    #[inline(always)]
    pub fn set_command_raw(&self, raw_command: u32) {
        self.lcd_cam
            .lcd_cmd_val()
            .write(|w| unsafe { w.bits(raw_command) });
    }

    /// Set bit-order for TX data.
    /// This controls whether the LCD controller applies bit-order reversal for
    /// the data in the FIFO before sending it out to a connected LCD
    /// (data[0:N] -> data[N:0]).
    #[inline(always)]
    pub fn set_bit_order(&self, order: BitOrder) {
        let enable_reversal = order != BitOrder::default();
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_bit_order().bit(enable_reversal));
    }

    /// Set bus-width and byte-order for TX data.
    /// This controls whether the LCD controller applies byte-order reversal for
    /// the data in the FIFO before sending it out to a connected LCD.
    #[inline(always)]
    pub fn set_width_and_byte_order(&self, bus_width: BusWidth, order: ByteOrder) {
        let enable_reversal = order != ByteOrder::default();
        match bus_width {
            // {B0}{B1}{B2}{B3} => {B1}{B0}{B3}{B2}
            BusWidth::Bit8 => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_8bits_order().bit(enable_reversal));
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_byte_order().clear_bit());
            }
            // {B1,B0},{B3,B2} => {B0,B1}{B2,B3}
            BusWidth::Bit16 => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_byte_order().bit(enable_reversal));
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_8bits_order().clear_bit());
            }
        }
    }

    /// Configures the LCD peripheral's mode of operation (selected Protocol).
    #[inline(always)]
    pub fn set_bus_mode(&self, mode: BusMode) {
        self.lcd_cam
            .lcd_ctrl()
            .modify(|_, w| w.lcd_rgb_mode_en().bit(mode == BusMode::RGB));
    }

    /// Configures whether the LCD peripheral automatically starts the next
    /// frame after the previous finished sending. If the peripheral is
    /// running in RGB mode, this basically means that it continues sending
    /// data.
    #[inline(always)]
    pub fn set_auto_next_frame(&self, enabled: bool) {
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_next_frame_en().bit(enabled));
    }

    /// (RGB-Mode) Configures whether to output the HSYNC signal in the porch
    /// region.
    #[inline(always)]
    pub fn set_output_hsync_in_porch_region(&self, enabled: bool) {
        self.lcd_cam
            .lcd_ctrl2()
            .modify(|_, w| w.lcd_hs_blank_en().bit(enabled));
    }

    /// (RGB-Mode) Configure the HSYNC signal's offset within a line of pixels.
    #[inline(always)]
    pub fn set_hsync_position(&self, offset_in_line: u32) {
        self.lcd_cam
            .lcd_ctrl2()
            .modify(|_, w| w.lcd_hsync_position().variant(offset_in_line as _));
    }

    /// (RGB-Mode) Configures the LCD peripheral's horizontal timing
    ///
    /// # Arguments
    /// * `hsw` - Horizontal sync width
    /// * `hbp` - Horizontal back porch
    /// * `active_width` - Horizontal active width
    /// * `hfp` - Horizontal front porch
    #[inline(always)]
    pub fn set_horizontal_timing(&self, hsw: u32, hbp: u32, active_width: u32, hfp: u32) {
        self.lcd_cam
            .lcd_ctrl2()
            .modify(|_, w| w.lcd_hsync_width().variant((hsw - 1) as _));
        self.lcd_cam
            .lcd_ctrl()
            .modify(|_, w| w.lcd_hb_front().variant((hbp + hsw - 1) as _));
        self.lcd_cam.lcd_ctrl1().modify(|_, w| {
            w.lcd_ha_width()
                .variant((active_width - 1) as _)
                .lcd_ht_width()
                .variant((hsw + hbp + active_width + hfp - 1) as _)
        });
    }

    /// (RGB-Mode) Configures the LCD peripheral's vertical timing
    ///
    /// # Arguments
    /// * `vsw` - Vertical sync width
    /// * `vbp` - Vertical back porch
    /// * `active_height` - Vertical active height
    /// * `vfp` - Vertical front porch
    #[inline(always)]
    pub fn set_vertical_timing(&self, vsw: u32, vbp: u32, active_height: u32, vfp: u32) {
        self.lcd_cam
            .lcd_ctrl2()
            .modify(|_, w| w.lcd_vsync_width().variant((vsw - 1) as _));
        self.lcd_cam
            .lcd_ctrl1()
            .modify(|_, w| w.lcd_vb_front().variant((vbp + vsw - 1) as _));
        self.lcd_cam.lcd_ctrl().modify(|_, w| {
            w.lcd_va_height()
                .variant((active_height - 1) as _)
                .lcd_vt_height()
                .variant((vsw + vbp + active_height + vfp - 1) as _)
        });
    }

    /// (RGB-Mode) Set level state for HSYNC, VSYNC, DE at IDLE phase.
    /// For each of the three control pins HSYNC, VSYNC and DE, these settings
    /// control whether their level during idle should be high(`true`) or
    /// low(`false`).
    ///
    /// # Arguments
    /// * `hsync_idle_level` - HSYNC level on IDLE phase
    /// * `vsync_idle_level` - VSYNC level on IDLE phase
    /// * `de_idle_level` - DE level on IDLE phase
    #[inline(always)]
    pub fn set_idle_level(
        &self,
        hsync_idle_level: bool,
        vsync_idle_level: bool,
        de_idle_level: bool,
    ) {
        self.lcd_cam.lcd_ctrl2().modify(|_, w| {
            w.lcd_hsync_idle_pol()
                .bit(hsync_idle_level)
                .lcd_vsync_idle_pol()
                .bit(vsync_idle_level)
                .lcd_de_idle_pol()
                .bit(de_idle_level)
        });
    }

    /// (RGB-Mode) Set extra delay for the HSYNC, VSYNC, and DE sync signals.
    #[inline(always)]
    pub fn set_sync_delay_ticks(&self, hsync_delay: u32, vsync_delay: u32, de_delay: u32) {
        self.lcd_cam.lcd_dly_mode().modify(|_, w| {
            w.lcd_hsync_mode()
                .variant(hsync_delay as _)
                .lcd_vsync_mode()
                .variant(vsync_delay as _)
                .lcd_de_mode()
                .variant(de_delay as _)
        });
    }

    /// (RGB-Mode) Set extra delay on the data lines.
    #[inline(always)]
    pub fn set_data_delay_ticks(&self, delay: u32) {
        let mut reg_val = 0;
        for i in 0..16 {
            reg_val |= (delay & 0x03) << (2 * i);
        }
        self.lcd_cam
            .lcd_data_dout_mode()
            .write(|w| unsafe { w.bits(reg_val) });
    }

    /// Enable/Disable the status of all interrupts selected by the `change_mask`.
    ///
    /// # Arguments
    /// * `change_mask` - Mask that selects the interrupts whose status should be changed
    /// * `enabled` - New status to set for the interrupts selected by the `change_mask`
    #[inline(always)]
    pub fn enable_interrupts(&self, change_mask: LcdInterrupt, enabled: bool) {
        self.lcd_cam.lc_dma_int_ena().modify(|_, w| {
            if change_mask.contains(LcdInterrupt::VSync) {
                w.lcd_vsync_int_ena().bit(enabled);
            }
            if change_mask.contains(LcdInterrupt::TransmissionDone) {
                w.lcd_trans_done_int_ena().bit(enabled);
            }
            w
        });
    }

    /// Set the enable/disable status of all LCD peripheral interrupts.
    ///
    /// # Arguments
    /// * `interrupts` - The interrupt status to set. All LCD peripheral interrupts in
    ///                 in this bitfield will be anbled, all others will be disabled.
    #[inline(always)]
    pub fn set_interrupts(&self, interrupts: LcdInterrupt) {
        self.lcd_cam.lc_dma_int_ena().modify(|_, w|
            w.lcd_vsync_int_ena().bit(interrupts.contains(LcdInterrupt::VSync))
                .lcd_trans_done_int_ena().bit(interrupts.contains(LcdInterrupt::TransmissionDone))
        );
    }

    /// Get the enable/disable status of all LCD peripheral interrupts.
    pub fn get_interrupt_status(&self) -> LcdInterrupt {
        LcdInterrupt::from_bits(self.lcd_cam.lc_dma_int_st().read().bits())
            .unwrap_or(LcdInterrupt::None)
    }

    /// Clear interrupt status of the LCD peripheral interrupts selected by
    /// the given `change_mask`. This clears pending interrupts.
    pub fn clear_interrupt_status(&self, change_mask: LcdInterrupt) {
        self.lcd_cam.lc_dma_int_clr().write(|w| unsafe { w.bits(change_mask.bits()) });
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusMode {
    /// Configures the LCD peripheral to talk Intel's I8080 display protocol.
    #[default]
    I8080,
    /// Configures the LCD peripheral to talk the RGB display protocol.
    RGB,
}

bitflags!{
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct LcdInterrupt : u32 {
        const None = 0;
        /// (RGB-Mode) Interrupt that signals that one complete frame was sent
        /// to the LCD and that the hardware is currently in the VSYNC phase.
        ///
        /// # Warning
        /// In stream mode, transmission continues while this interrupt's ISR
        /// is executed. So don't take too long!
        const VSync = (1 << 0);
        /// (I8080-Mode) Interrupt that signals that one transmission finished
        /// transmission to the wire.
        const TransmissionDone = (1 << 1);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusWidth {
    Bit8,
    Bit16,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockMode {
    pub polarity: Polarity,
    pub phase: Phase,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    #[default]
    IdleLow,
    IdleHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Phase {
    /// Negative edge:  v
    ///           ------|     |-------
    ///                 |     |
    ///                 |-----|
    #[default]
    ShiftLow,
    /// Positive edge:  v
    ///                 |-----|
    ///                 |     |
    ///           ------|     |-------
    ShiftHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DelayMode {
    /// Output without delay.
    #[default]
    None        = 0,
    /// Delayed by the rising edge of LCD_CLK.
    RaisingEdge = 1,
    /// Delayed by the falling edge of LCD_CLK.
    FallingEdge = 2,
}
