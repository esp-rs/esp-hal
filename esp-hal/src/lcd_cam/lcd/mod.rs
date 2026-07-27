//! LCD
//!
//! ## Overview
//! The LCD module is designed to send parallel video data signals, and its bus
//! supports RGB, MOTO6800, and I8080 interface timing.
//!
//! For more information on these modes, please refer to the documentation in
//! their respective modules.

use core::marker::PhantomData;

use super::GenericPeripheralGuard;
use crate::{
    Async,
    Blocking,
    DriverMode,
    clock::ll::{ClockTree, LcdCamInstance},
    lcd_cam::{ClockError, calculate_clkm},
    peripherals::LCD_CAM,
    system,
    time::Rate,
};

pub mod dpi;
pub mod i8080;

pub(super) struct Inner<'d> {
    /// The `LCD_CAM` peripheral reference for managing the LCD functionality.
    pub lcd_cam: LCD_CAM<'d>,

    pub _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,

    pub clock_requested: bool,
}

impl<'a> Drop for Inner<'a> {
    fn drop(&mut self) {
        if self.clock_requested {
            ClockTree::with(|clocks| LcdCamInstance::LcdCam.release_lcd_clock(clocks));
        }
    }
}

/// Represents an LCD interface.
pub struct Lcd<'d, Dm: DriverMode> {
    pub(super) inner: Inner<'d>,

    /// A marker for the mode of operation (blocking or asynchronous).
    pub(super) _mode: PhantomData<Dm>,
}

struct ClockConfig {
    /// Specifies the clock mode, including polarity and phase settings.
    pub clock_mode: ClockMode,

    /// The frequency of the pixel clock.
    pub frequency: Rate,
}

impl<'d, Dm: DriverMode> Lcd<'d, Dm> {
    fn regs(&self) -> &crate::pac::lcd_cam::RegisterBlock {
        self.inner.lcd_cam.register_block()
    }

    fn configure_clocks(&mut self, config: &ClockConfig) -> Result<(), ClockError> {
        let sources = property!("clock_tree.lcd_cam.lcd_clock");
        let (i, divider) = calculate_clkm(
            config.frequency.as_hz() as _,
            &sources.map(|source| LcdCamInstance::lcd_clock_source_frequency(source) as usize),
        )?;

        self.regs().lcd_clock().write(|w| unsafe {
            // Force enable the clock for all configuration registers.
            w.clk_en().set_bit();
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
        ClockTree::with(|clocks| {
            LcdCamInstance::LcdCam.configure_lcd_clock(clocks, sources[i]);
            if !self.inner.clock_requested {
                LcdCamInstance::LcdCam.request_lcd_clock(clocks);
                self.inner.clock_requested = true;
            }
        });

        Ok(())
    }

    pub(super) fn into_async(self) -> Lcd<'d, Async> {
        Lcd {
            inner: self.inner,
            _mode: PhantomData,
        }
    }
}

impl<'d> Lcd<'d, Async> {
    pub(super) fn into_blocking(self) -> Lcd<'d, Blocking> {
        Lcd {
            inner: self.inner,
            _mode: PhantomData,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the clock mode configuration for the LCD interface.
pub struct ClockMode {
    /// The polarity of the clock signal (idle high or low).
    pub polarity: Polarity,

    /// The phase of the clock signal (shift on the rising or falling edge).
    pub phase: Phase,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the polarity of the clock signal for the LCD interface.
pub enum Polarity {
    /// The clock signal is low when idle.
    #[default]
    IdleLow,

    /// The clock signal is high when idle.
    IdleHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the phase of the clock signal for the LCD interface.
pub enum Phase {
    /// Data is shifted on the low (falling) edge of the clock signal.
    #[default]
    ShiftLow,

    /// Data is shifted on the high (rising) edge of the clock signal.
    ShiftHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the delay mode for the LCD signal output.
pub enum DelayMode {
    /// Output without delay.
    #[default]
    None        = 0,
    /// Delayed by the rising edge of LCD_CLK.
    RaisingEdge = 1,
    /// Delayed by the falling edge of LCD_CLK.
    FallingEdge = 2,
}
