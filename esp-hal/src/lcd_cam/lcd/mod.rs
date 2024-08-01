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

use crate::{peripheral::PeripheralRef, peripherals::LCD_CAM};

pub mod i8080;

pub struct Lcd<'d, DM: crate::Mode> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
    pub(crate) _mode: core::marker::PhantomData<DM>,
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
    #[default]
    ShiftLow,
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
