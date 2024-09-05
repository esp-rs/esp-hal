use super::*;

/// A type-erased GPIO pin, with additional configuration options.
///
/// Note that accessing unsupported pin functions (e.g. trying to use an
/// input-only pin as output) will panic.
pub struct AnyPin<'d> {
    pin: ErasedPin,
    is_inverted: bool,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyPin<'d> {
    /// Create wrapper for the given pin.
    #[inline]
    pub fn new<P>(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
    where
        P: Pin,
    {
        crate::into_ref!(pin);
        let pin = pin.degrade_internal(private::Internal);

        Self {
            pin,
            is_inverted: false,
            _phantom: PhantomData,
        }
    }

    /// Create wrapper for the given pin. The peripheral signal will be
    /// inverted.
    #[inline]
    pub fn new_inverted<P>(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
    where
        P: Pin,
    {
        crate::into_ref!(pin);
        let pin = pin.degrade_internal(private::Internal);

        Self {
            pin,
            is_inverted: true,
            _phantom: PhantomData,
        }
    }
}

impl<'d> crate::peripheral::Peripheral for AnyPin<'d> {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            is_inverted: self.is_inverted,
            _phantom: PhantomData,
        }
    }
}

impl<'d> private::Sealed for AnyPin<'d> {}

impl<'d> Pin for AnyPin<'d> {
    delegate::delegate! {
        to self.pin {
            fn number(&self, _internal: private::Internal) -> u8;
            fn degrade_internal(&self, _internal: private::Internal) -> ErasedPin;
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
            fn wakeup_enable(&mut self, enable: bool, event: WakeEvent, _internal: private::Internal);
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
            self.is_inverted,
            false,
            false,
            self.is_inverted,
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
        if self.is_inverted {
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
            fn enable_input(&mut self, on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _internal: private::Internal);
        }
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _internal: private::Internal) {
        self.pin.connect_input_to_peripheral_with_options(
            signal,
            self.is_inverted,
            self.is_inverted,
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
        if self.is_inverted {
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
