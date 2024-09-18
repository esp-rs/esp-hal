//! Placeholder pins/signals.
//!
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.
// This module also contains peripheral signal impls for `Level` to avoid
// polluting the main module.

use super::*;

impl crate::peripheral::Peripheral for Level {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        *self
    }
}

impl PeripheralSignal for Level {
    fn pull_direction(&self, _pull: Pull, _internal: private::Internal) {}
}

impl PeripheralInput for Level {
    fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6] {
        [None; 6]
    }

    fn init_input(&self, _pull: Pull, _: private::Internal) {}

    fn enable_input(&mut self, _on: bool, _: private::Internal) {}

    fn enable_input_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn is_input_high(&self, _: private::Internal) -> bool {
        *self == Level::High
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        let value = match self {
            Level::High => ONE_INPUT,
            Level::Low => ZERO_INPUT,
        };

        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(value)
            });
    }

    fn disconnect_input_from_peripheral(&mut self, _signal: InputSignal, _: private::Internal) {}
}

impl PeripheralOutput for Level {
    fn set_to_open_drain_output(&mut self, _: private::Internal) {}

    fn set_to_push_pull_output(&mut self, _: private::Internal) {}

    fn enable_output(&mut self, _on: bool, _: private::Internal) {}

    fn set_output_high(&mut self, _on: bool, _: private::Internal) {}

    fn set_drive_strength(&mut self, _strength: DriveStrength, _: private::Internal) {}

    fn enable_open_drain(&mut self, _on: bool, _: private::Internal) {}

    fn enable_output_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_up_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_down_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn is_set_high(&self, _: private::Internal) -> bool {
        false
    }

    fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6] {
        [None; 6]
    }

    fn connect_peripheral_to_output(&mut self, _signal: OutputSignal, _: private::Internal) {}

    fn disconnect_from_peripheral_output(&mut self, _signal: OutputSignal, _: private::Internal) {}
}

/// Placeholder pin, used when no pin is required when using a peripheral.
///
/// When used as a peripheral signal, `NoPin` is equivalent to [`Level::Low`].
#[derive(Default, Clone, Copy)]
pub struct NoPin;

impl crate::peripheral::Peripheral for NoPin {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self
    }
}

impl private::Sealed for NoPin {}

impl PeripheralSignal for NoPin {
    fn pull_direction(&self, _pull: Pull, _internal: private::Internal) {}
}

impl PeripheralInput for NoPin {
    delegate::delegate! {
        to Level::Low {
            fn init_input(&self, _pull: Pull, _internal: private::Internal);
            fn enable_input(&mut self, _on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, _on: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn connect_input_to_peripheral(&mut self, _signal: InputSignal, _internal: private::Internal);
            fn disconnect_input_from_peripheral(&mut self, _signal: InputSignal, _internal: private::Internal);
            fn input_signals(&self, _internal: private::Internal) -> [Option<InputSignal>; 6];
        }
    }
}

impl PeripheralOutput for NoPin {
    delegate::delegate! {
        to Level::Low {
            fn set_to_open_drain_output(&mut self, _internal: private::Internal);
            fn set_to_push_pull_output(&mut self, _internal: private::Internal);
            fn enable_output(&mut self, _on: bool, _internal: private::Internal);
            fn set_output_high(&mut self, _on: bool, _internal: private::Internal);
            fn set_drive_strength(&mut self, _strength: DriveStrength, _internal: private::Internal);
            fn enable_open_drain(&mut self, _on: bool, _internal: private::Internal);
            fn enable_output_in_sleep_mode(&mut self, _on: bool, _internal: private::Internal);
            fn internal_pull_up_in_sleep_mode(&mut self, _on: bool, _internal: private::Internal);
            fn internal_pull_down_in_sleep_mode(&mut self, _on: bool, _internal: private::Internal);
            fn is_set_high(&self, _internal: private::Internal) -> bool;
            fn output_signals(&self, _internal: private::Internal) -> [Option<OutputSignal>; 6];
            fn connect_peripheral_to_output(&mut self, _signal: OutputSignal, _internal: private::Internal);
            fn disconnect_from_peripheral_output(&mut self, _signal: OutputSignal, _internal: private::Internal);
        }
    }
}

impl embedded_hal_02::digital::v2::OutputPin for NoPin {
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl embedded_hal_02::digital::v2::StatefulOutputPin for NoPin {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(false)
    }
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}

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
