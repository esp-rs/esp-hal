//! Peripheral signal interconnect using IOMUX or GPIOMUX.

use crate::{
    gpio::{
        self,
        AlternateFunction,
        DummyPin,
        ErasedPin,
        GpioPin,
        InputPin,
        Level,
        OutputSignalType,
        PeripheralInputPin,
        PeripheralOutputPin,
        Pin,
        FUNC_IN_SEL_OFFSET,
        GPIO_FUNCTION,
        INPUT_SIGNAL_MAX,
        OUTPUT_SIGNAL_MAX,
    },
    peripheral::Peripheral,
    peripherals::GPIO,
    private::{self, Sealed},
};

/// Multiple input signal can be connected to one pin.
pub struct InputSignal {
    pin: ErasedPin,
    is_inverted: bool,
}

impl Clone for InputSignal {
    fn clone(&self) -> Self {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            is_inverted: self.is_inverted,
        }
    }
}

impl Peripheral for InputSignal {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        self.clone()
    }
}

impl Sealed for InputSignal {}

impl InputSignal {
    pub(crate) fn new(pin: ErasedPin) -> Self {
        Self {
            pin,
            is_inverted: false,
        }
    }

    /// Inverts the peripheral's input signal.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    fn connect(&self, signal: usize, invert: bool, input: u8) {
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(invert)
                    .in_sel()
                    .bits(input)
            });
    }
}

impl PeripheralInputPin for InputSignal {
    /// Connect the pin to a peripheral input signal.
    fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal, _: private::Internal) {
        let signal_nr = signal as usize;

        let af = if self.is_inverted {
            GPIO_FUNCTION
        } else if let Some(af) = self
            .pin
            .input_signals(private::Internal)
            .into_iter()
            .position(|s| s == Some(signal))
        {
            match af {
                0 => AlternateFunction::Function0,
                1 => AlternateFunction::Function1,
                2 => AlternateFunction::Function2,
                3 => AlternateFunction::Function3,
                4 => AlternateFunction::Function4,
                5 => AlternateFunction::Function5,
                _ => unreachable!(),
            }
        } else {
            GPIO_FUNCTION
        };

        if af == GPIO_FUNCTION && signal_nr > INPUT_SIGNAL_MAX as usize {
            panic!("Cannot connect GPIO to this peripheral");
        }

        self.pin.set_alternate_function(af, private::Internal);

        if signal_nr <= INPUT_SIGNAL_MAX as usize {
            self.connect(
                signal_nr,
                self.is_inverted,
                self.pin.number(private::Internal),
            );
        }
    }

    /// Remove this pin from a connected peripheral input.
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    fn disconnect_input_from_peripheral(
        &mut self,
        signal: gpio::InputSignal,
        _: private::Internal,
    ) {
        self.pin
            .set_alternate_function(GPIO_FUNCTION, private::Internal);

        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| w.sel().clear_bit());
    }

    delegate::delegate! {
        to self.pin {
            fn init_input(&self, pull_down: bool, pull_up: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
            fn enable_input(&mut self, on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
        }
    }
}

/// Multiple pins can be connected to one output signal.
pub struct OutputSignal {
    pin: ErasedPin,
    is_inverted: bool,
}

impl Peripheral for OutputSignal {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self {
            pin: self.pin.clone_unchecked(),
            is_inverted: self.is_inverted,
        }
    }
}

impl Sealed for OutputSignal {}

impl OutputSignal {
    pub(crate) fn new(pin: ErasedPin) -> Self {
        Self {
            is_inverted: false,
            pin,
        }
    }

    /// Inverts the peripheral's output signal.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    fn connect(
        &self,
        signal: OutputSignalType,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        output: u8,
    ) {
        unsafe { &*GPIO::PTR }
            .func_out_sel_cfg(output as usize)
            .modify(|_, w| unsafe {
                w.out_sel()
                    .bits(signal)
                    .inv_sel()
                    .bit(invert)
                    .oen_sel()
                    .bit(enable_from_gpio)
                    .oen_inv_sel()
                    .bit(invert_enable)
            });
    }
}

impl PeripheralOutputPin for OutputSignal {
    /// Connect the pin to a peripheral output signal.
    // TODO: the following options should be part of the struct:
    /// invert: Configures whether or not to invert the output value
    ///
    /// invert_enable: Configures whether or not to invert the output enable
    /// signal
    ///
    /// enable_from_gpio: Configures to select the source of output enable
    /// signal.
    /// - false = Use output enable signal from peripheral
    /// - true = Force the output enable signal to be sourced from bit n of
    ///   GPIO_ENABLE_REG
    ///
    /// force_via_gpio_mux: if true don't use the alternate function even if it
    /// matches
    fn connect_peripheral_to_output(&mut self, signal: gpio::OutputSignal, _: private::Internal) {
        let af = if self.is_inverted {
            GPIO_FUNCTION
        } else if let Some(af) = self
            .pin
            .output_signals(private::Internal)
            .into_iter()
            .position(|s| s == Some(signal))
        {
            match af {
                0 => AlternateFunction::Function0,
                1 => AlternateFunction::Function1,
                2 => AlternateFunction::Function2,
                3 => AlternateFunction::Function3,
                4 => AlternateFunction::Function4,
                5 => AlternateFunction::Function5,
                _ => unreachable!(),
            }
        } else {
            GPIO_FUNCTION
        };

        self.pin.set_alternate_function(af, private::Internal);

        let clipped_signal = if signal as usize <= OUTPUT_SIGNAL_MAX as usize {
            signal as OutputSignalType
        } else {
            OUTPUT_SIGNAL_MAX
        };

        self.connect(
            clipped_signal,
            self.is_inverted,
            false,
            false,
            self.pin.number(private::Internal),
        );
    }

    /// Remove this output pin from a connected [signal](`OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`OutputSignal`). Any other
    /// outputs connected to the signal remain intact.
    fn disconnect_from_peripheral_output(
        &mut self,
        signal: gpio::OutputSignal,
        _: private::Internal,
    ) {
        self.pin
            .set_alternate_function(GPIO_FUNCTION, private::Internal);

        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| w.sel().clear_bit());
    }

    delegate::delegate! {
        to self.pin {
            fn set_to_open_drain_output(&mut self, _internal: private::Internal);
            fn set_to_push_pull_output(&mut self, _internal: private::Internal);
            fn enable_output(&mut self, on: bool, _internal: private::Internal);
            fn set_output_high(&mut self, on: bool, _internal: private::Internal);
            fn set_drive_strength(&mut self, strength: gpio::DriveStrength, _internal: private::Internal);
            fn enable_open_drain(&mut self, on: bool, _internal: private::Internal);
            fn enable_output_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_up(&mut self, on: bool, _internal: private::Internal);
            fn internal_pull_down(&mut self, on: bool, _internal: private::Internal);
            fn is_set_high(&self, _internal: private::Internal) -> bool;
            fn output_signals(&self, _internal: private::Internal) -> [Option<gpio::OutputSignal>; 6];
        }
    }
}

#[derive(Clone)]
enum AnyInputSignalInner {
    Input(InputSignal),
    Constant(Level),
    Dummy(DummyPin),
}

/// A type-erased input signal.
#[derive(Clone)]
pub struct AnyInputSignal(AnyInputSignalInner);

impl Peripheral for AnyInputSignal {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        self.clone()
    }
}

impl From<InputSignal> for AnyInputSignal {
    fn from(input: InputSignal) -> Self {
        Self(AnyInputSignalInner::Input(input))
    }
}

impl From<Level> for AnyInputSignal {
    fn from(level: Level) -> Self {
        Self(AnyInputSignalInner::Constant(level))
    }
}

impl From<DummyPin> for AnyInputSignal {
    fn from(pin: DummyPin) -> Self {
        Self(AnyInputSignalInner::Dummy(pin))
    }
}

impl From<ErasedPin> for AnyInputSignal {
    fn from(input: ErasedPin) -> Self {
        Self(AnyInputSignalInner::Input(input.peripheral_input()))
    }
}

impl<const GPIONUM: u8> From<GpioPin<GPIONUM>> for AnyInputSignal
where
    GpioPin<GPIONUM>: InputPin,
{
    fn from(pin: GpioPin<GPIONUM>) -> Self {
        Self(AnyInputSignalInner::Input(pin.peripheral_input()))
    }
}

impl Sealed for AnyInputSignal {}
impl PeripheralInputPin for AnyInputSignal {
    delegate::delegate! {
        to match &self.0 {
            AnyInputSignalInner::Input(pin) => pin,
            AnyInputSignalInner::Constant(level) => level,
            AnyInputSignalInner::Dummy(pin) => pin,
        } {
            fn init_input(&self, pull_down: bool, pull_up: bool, _internal: private::Internal);
            fn is_input_high(&self, _internal: private::Internal) -> bool;
            fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
        }

        to match &mut self.0 {
            AnyInputSignalInner::Input(pin) => pin,
            AnyInputSignalInner::Constant(level) => level,
            AnyInputSignalInner::Dummy(pin) => pin,
        } {
            fn enable_input(&mut self, on: bool, _internal: private::Internal);
            fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            fn connect_input_to_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
            fn disconnect_input_from_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
        }
    }
}
