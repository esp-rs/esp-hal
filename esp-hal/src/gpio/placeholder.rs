//! Placeholder pins/signals.
//!
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.
// This module also contains peripheral signal impls for `Level` to avoid
// polluting the main module.

use super::*;

impl Level {
    pub(crate) fn pull_direction(&self, _pull: Pull) {}

    pub(crate) fn input_signals(
        &self,
        _: private::Internal,
    ) -> &'static [(AlternateFunction, InputSignal)] {
        &[]
    }

    pub(crate) fn init_input(&self, _pull: Pull) {}

    pub(crate) fn set_input_enable(&self, _on: bool) {}

    pub(crate) fn is_input_high(&self) -> bool {
        *self == Level::High
    }

    pub(crate) fn set_to_open_drain_output(&self) {}
    pub(crate) fn set_to_push_pull_output(&self) {}
    pub(crate) fn set_output_enable(&self, _on: bool) {}
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
}

/// Placeholder pin, used when no pin is required when using a peripheral.
///
/// When used as a peripheral signal, `NoPin` is equivalent to [`Level::Low`].
#[derive(Default, Clone, Copy)]
pub struct NoPin;

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
