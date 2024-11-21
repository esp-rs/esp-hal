//! LCD
//!
//! ## Overview
//! The LCD module is designed to send parallel video data signals, and its bus
//! supports RGB, MOTO6800, and I8080 interface timing.
//!
//! For more information on these modes, please refer to the documentation in
//! their respective modules.
//!
//! ## Implementation State
//! - RGB is not supported yet

use super::GenericPeripheralGuard;
use crate::{
    peripheral::PeripheralRef,
    peripherals::LCD_CAM,
    system::{self},
};

pub mod dpi;
pub mod i8080;

/// Represents an LCD interface.
pub struct Lcd<'d, DM: crate::Mode> {
    /// The `LCD_CAM` peripheral reference for managing the LCD functionality.
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,

    /// A marker for the mode of operation (blocking or asynchronous).
    pub(crate) _mode: core::marker::PhantomData<DM>,

    pub(super) _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,
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
