//! Placeholder pins/signals.
//!
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.
// This module also contains peripheral signal impls for `Level` to avoid
// polluting the main module.

use super::*;

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
