use crate::{peripheral::PeripheralRef, peripherals::LCD_CAM};

pub mod i8080;

pub struct Lcd<'d> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
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
