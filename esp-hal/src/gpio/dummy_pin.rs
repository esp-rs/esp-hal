//! "Dummy" pins".
//! These are useful to pass them into peripheral drivers where you don't want
//! an actual pin but one is required.

use super::*;

/// DummyPin, not useful everywhere as it panics if number() is called
#[derive(Default)]
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

impl Pin for DummyPin {
    fn number(&self, _: private::Internal) -> u8 {
        panic!("DummyPin not supported here!");
    }

    fn sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn set_alternate_function(&mut self, _alternate: AlternateFunction, _: private::Internal) {}

    fn is_listening(&self, _: private::Internal) -> bool {
        false
    }

    fn listen_with_options(
        &mut self,
        _event: Event,
        _int_enable: bool,
        _nmi_enable: bool,
        _wake_up_from_light_sleep: bool,
        _: private::Internal,
    ) {
    }

    fn unlisten(&mut self, _: private::Internal) {}

    fn is_interrupt_set(&self, _: private::Internal) -> bool {
        false
    }

    fn clear_interrupt(&mut self, _: private::Internal) {}

    fn wakeup_enable(&mut self, _enable: bool, _event: WakeEvent, _: private::Internal) {}
}

impl OutputPin for DummyPin {
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

    fn connect_peripheral_to_output(&mut self, _signal: OutputSignal, _: private::Internal) {}

    fn connect_peripheral_to_output_with_options(
        &mut self,
        _signal: OutputSignal,
        _invert: bool,
        _invert_enable: bool,
        _enable_from_gpio: bool,
        _force_via_gpio_mux: bool,
        _: private::Internal,
    ) {
    }

    fn disconnect_peripheral_from_output(&mut self, _: private::Internal) {}

    fn is_set_high(&self, _: private::Internal) -> bool {
        self.value
    }
}

impl InputPin for DummyPin {
    fn init_input(&self, _pull_down: bool, _pull_up: bool, _: private::Internal) {}

    fn set_to_input(&mut self, _: private::Internal) {}

    fn enable_input(&mut self, _on: bool, _: private::Internal) {}

    fn enable_input_in_sleep_mode(&mut self, _on: bool, _: private::Internal) {}

    fn is_input_high(&self, _: private::Internal) -> bool {
        self.value
    }

    fn connect_input_to_peripheral(&mut self, _signal: InputSignal, _: private::Internal) {}

    fn connect_input_to_peripheral_with_options(
        &mut self,
        _signal: InputSignal,
        _invert: bool,
        _force_via_gpio_mux: bool,
        _: private::Internal,
    ) {
    }

    fn disconnect_input_from_peripheral(&mut self, _signal: InputSignal, _: private::Internal) {}
}
