//! Placeholder pins/signals.
//!
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.
// This module also contains peripheral signal impls for `Level` to avoid
// polluting the main module.

use super::*;
use crate::gpio::interconnect::connect_input_signal;

impl crate::peripheral::Peripheral for Level {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        *self
    }
}

impl Level {
    pub(crate) fn pull_direction(&self, _pull: Pull) {}

    pub(crate) fn input_signals(
        &self,
        _: private::Internal,
    ) -> &'static [(AlternateFunction, InputSignal)] {
        &[]
    }

    pub(crate) fn init_input(&self, _pull: Pull) {}

    pub(crate) fn enable_input(&self, _on: bool) {}

    pub(crate) fn is_input_high(&self) -> bool {
        *self == Level::High
    }

    pub(crate) fn connect_input_to_peripheral(&self, signal: InputSignal) {
        let value = match self {
            Level::High => ONE_INPUT,
            Level::Low => ZERO_INPUT,
        };

        connect_input_signal(signal, value, false, true);
    }

    pub(crate) fn set_to_open_drain_output(&self) {}
    pub(crate) fn set_to_push_pull_output(&self) {}
    pub(crate) fn enable_output(&self, _on: bool) {}
    pub(crate) fn set_output_high(&self, _on: bool) {}
    pub(crate) fn set_drive_strength(&self, _strength: DriveStrength) {}
    pub(crate) fn enable_open_drain(&self, _on: bool) {}

    pub(crate) fn is_set_high(&self) -> bool {
        false
    }

    pub(crate) fn output_signals(
        &self,
        _: private::Internal,
    ) -> &'static [(AlternateFunction, OutputSignal)] {
        &[]
    }

    pub(crate) fn connect_peripheral_to_output(&self, _signal: OutputSignal) {}

    pub(crate) fn disconnect_from_peripheral_output(&self, _signal: OutputSignal) {}
}

/// Placeholder pin, used when no pin is required when using a peripheral.
///
/// When used as a peripheral signal, `NoPin` is equivalent to [`Level::Low`].
#[derive(Default, Clone, Copy)]
pub struct NoPin;

impl crate::peripheral::Peripheral for NoPin {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self
    }
}

impl private::Sealed for NoPin {}

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for NoPin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}
