//! Type-erased wrappers for GPIO pins.
//! These are useful to pass them into peripheral drivers.
//!
//! If you want a generic pin for GPIO input/output look into
//! [Output],[OutputOpenDrain] and [Input]

use super::*;

#[derive(Clone, Copy)]
enum Inverted {
    NonInverted,
    Inverted,
}

impl Inverted {
    fn is_inverted(&self) -> bool {
        match self {
            Inverted::NonInverted => false,
            Inverted::Inverted => true,
        }
    }
}

/// Generic pin wrapper for pins which can be Output or Input.
pub struct AnyPin<'d> {
    pin: ErasedPin,
    inverted: Inverted,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyPin<'d> {
    /// Create wrapper for the given pin.
    #[inline]
    pub fn new<P: OutputPin + InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
    ) -> Self {
        crate::into_ref!(pin);
        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            inverted: Inverted::NonInverted,
            _phantom: PhantomData,
        }
    }

    /// Create wrapper for the given pin. The peripheral signal will be
    /// inverted.
    #[inline]
    pub fn new_inverted<P: OutputPin + InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
    ) -> Self {
        crate::into_ref!(pin);
        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            inverted: Inverted::Inverted,
            _phantom: PhantomData,
        }
    }
}

impl<'d> crate::peripheral::Peripheral for AnyPin<'d> {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            inverted: self.inverted,
            _phantom: PhantomData,
        }
    }
}

impl<'d> private::Sealed for AnyPin<'d> {}

impl<'d> Pin for AnyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn number(&self, _internal: private::Internal) -> u8;
            fn sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn set_alternate_function(&mut self, alternate: AlternateFunction, _internal: private::Internal);
            fn is_listening(&self, _internal: private::Internal) -> bool;
            fn listen_with_options(
                &mut self,
                event: Event,
                int_enable: bool,
                nmi_enable: bool,
                wake_up_from_light_sleep: bool,
                _internal: private::Internal,
            );
            fn unlisten(&mut self, _internal: private::Internal);
            fn is_interrupt_set(&self, _internal: private::Internal) -> bool;
            fn clear_interrupt(&mut self, _internal: private::Internal);
        }
    }
}

impl<'d> OutputPin for AnyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn set_to_open_drain_output(&mut self, _internal: private::Internal);
            fn set_to_push_pull_output(&mut self, _internal: private::Internal);
            fn enable_output(&mut self, on: bool, _internal: private::Internal);
            fn set_output_high(&mut self, on: bool, _internal: private::Internal);
            fn set_drive_strength(&mut self, strength: DriveStrength, _internal: private::Internal);
            fn enable_open_drain(&mut self, on: bool, _internal: private::Internal);
            fn enable_output_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_up(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_down(&mut self, on: bool, _internal: private::Internal);
            fn disconnect_peripheral_from_output(&mut self, _internal: private::Internal);
            fn is_set_high(&self, _internal: private::Internal) -> bool;
        }
    }

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _internal: private::Internal) {
        self.pin.connect_peripheral_to_output_with_options(
            signal,
            self.inverted.is_inverted(),
            false,
            false,
            self.inverted.is_inverted(),
            private::Internal,
        );
    }

    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
        _internal: private::Internal,
    ) {
        if self.inverted.is_inverted() {
            self.pin.connect_peripheral_to_output_with_options(
                signal,
                true,
                false,
                false,
                true,
                private::Internal,
            );
        } else {
            self.pin.connect_peripheral_to_output_with_options(
                signal,
                invert,
                invert_enable,
                enable_from_gpio,
                force_via_gpio_mux,
                private::Internal,
            );
        }
    }
}

impl<'d> InputPin for AnyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn init_input(&self, pull_down: bool, pull_up: bool, _internal: private::Internal);
            fn set_to_input(&mut self, _internal: private::Internal);
            fn enable_input(&mut self, on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _internal: private::Internal);
        }
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _internal: private::Internal) {
        self.pin.connect_input_to_peripheral_with_options(
            signal,
            self.inverted.is_inverted(),
            self.inverted.is_inverted(),
            private::Internal,
        );
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
        _internal: private::Internal,
    ) {
        if self.inverted.is_inverted() {
            self.pin.connect_input_to_peripheral_with_options(
                signal,
                true,
                true,
                private::Internal,
            );
        } else {
            self.pin.connect_input_to_peripheral_with_options(
                signal,
                invert,
                force_via_gpio_mux,
                private::Internal,
            );
        }
    }
}

/// Generic pin wrapper for pins which can only be Input.
pub struct AnyInputOnlyPin<'d> {
    pin: ErasedPin,
    inverted: Inverted,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyInputOnlyPin<'d> {
    /// Create wrapper for the given pin.
    #[inline]
    pub fn new<P: InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
    ) -> Self {
        crate::into_ref!(pin);
        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            inverted: Inverted::NonInverted,
            _phantom: PhantomData,
        }
    }
}

impl<'d> crate::peripheral::Peripheral for AnyInputOnlyPin<'d> {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            inverted: self.inverted,
            _phantom: PhantomData,
        }
    }
}

impl<'d> private::Sealed for AnyInputOnlyPin<'d> {}

impl<'d> Pin for AnyInputOnlyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn number(&self, _internal: private::Internal) -> u8;
            fn sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn set_alternate_function(&mut self, alternate: AlternateFunction, _internal: private::Internal);
            fn is_listening(&self, _internal: private::Internal) -> bool;
            fn listen_with_options(
                &mut self,
                event: Event,
                int_enable: bool,
                nmi_enable: bool,
                wake_up_from_light_sleep: bool,
                _internal: private::Internal,
            );
            fn unlisten(&mut self, _internal: private::Internal);
            fn is_interrupt_set(&self, _internal: private::Internal) -> bool;
            fn clear_interrupt(&mut self, _internal: private::Internal);
        }
    }
}

impl<'d> InputPin for AnyInputOnlyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn init_input(&self, pull_down: bool, pull_up: bool, _internal: private::Internal);
            fn set_to_input(&mut self, _internal: private::Internal);
            fn enable_input(&mut self, on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn connect_input_to_peripheral(&mut self, signal: InputSignal, _internal: private::Internal);
            fn connect_input_to_peripheral_with_options(
                &mut self,
                signal: InputSignal,
                invert: bool,
                force_via_gpio_mux: bool,
                _internal: private::Internal,
            );
            fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _internal: private::Internal);
        }
    }
}
