//! Placeholder pins.
//!
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.

use super::*;

/// DummyPin, not useful everywhere as it panics if number() is called
#[derive(Default, Clone)]
pub struct DummyPin {
    value: bool,
}

impl DummyPin {
    /// Create a dummy pin.
    pub fn new() -> Self {
        Self { value: false }
    }
}

impl crate::peripheral::Peripheral for DummyPin {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self { value: self.value }
    }
}

impl private::Sealed for DummyPin {}

impl PeripheralInput for DummyPin {
    fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6] {
        [None; 6]
    }

    fn init_input(&self, _pull_down: bool, _pull_up: bool, _: private::Internal) {}

    fn enable_input(&mut self, _on: bool, _: private::Internal) {}

    fn enable_input_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn is_input_high(&self, _: private::Internal) -> bool {
        self.value
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        let value = if self.value { ONE_INPUT } else { ZERO_INPUT };

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

impl PeripheralInput for Level {
    delegate::delegate! {
        to match self {
            Level::High => DummyPin { value: true },
            Level::Low => DummyPin { value: false },
        }  {
            fn input_signals(&self, _internal: private::Internal) -> [Option<InputSignal>; 6];
            fn init_input(&self, _pd: bool, _pu: bool, _internal: private::Internal);
            fn enable_input(&mut self, _on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, _on: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn connect_input_to_peripheral(&mut self, _signal: InputSignal, _internal: private::Internal);
            fn disconnect_input_from_peripheral(&mut self, _signal: InputSignal, _internal: private::Internal);
        }
    }
}

impl PeripheralOutput for DummyPin {
    fn set_to_open_drain_output(&mut self, _: private::Internal) {}

    fn set_to_push_pull_output(&mut self, _: private::Internal) {}

    fn enable_output(&mut self, _on: bool, _: private::Internal) {}

    fn set_output_high(&mut self, on: bool, _: private::Internal) {
        self.value = on;
    }

    fn set_drive_strength(&mut self, _strength: DriveStrength, _: private::Internal) {}

    fn enable_open_drain(&mut self, _on: bool, _: private::Internal) {}

    fn enable_output_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_up_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_down_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_up(&mut self, _on: bool, _: private::Internal) {}

    fn internal_pull_down(&mut self, _on: bool, _: private::Internal) {}

    fn is_set_high(&self, _: private::Internal) -> bool {
        self.value
    }

    fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6] {
        [None; 6]
    }

    fn connect_peripheral_to_output(&mut self, _signal: OutputSignal, _: private::Internal) {}

    fn disconnect_from_peripheral_output(&mut self, _signal: OutputSignal, _: private::Internal) {}
}

impl embedded_hal_02::digital::v2::OutputPin for DummyPin {
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output_high(true, private::Internal);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output_high(false, private::Internal);
        Ok(())
    }
}
impl embedded_hal_02::digital::v2::StatefulOutputPin for DummyPin {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(PeripheralOutput::is_set_high(self, private::Internal))
    }
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!PeripheralOutput::is_set_high(self, private::Internal))
    }
}

impl embedded_hal::digital::ErrorType for DummyPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for DummyPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output_high(true, private::Internal);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output_high(false, private::Internal);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for DummyPin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(PeripheralOutput::is_set_high(self, private::Internal))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!PeripheralOutput::is_set_high(self, private::Internal))
    }
}
