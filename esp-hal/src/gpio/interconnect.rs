//! Peripheral signal interconnect using IOMUX or GPIOMUX.

use crate::{
    gpio::{
        self,
        AlternateFunction,
        AnyPin,
        GpioPin,
        InputPin,
        Level,
        NoPin,
        OutputPin,
        OutputSignalType,
        Pin,
        Pull,
        FUNC_IN_SEL_OFFSET,
        GPIO_FUNCTION,
        INPUT_SIGNAL_MAX,
        OUTPUT_SIGNAL_MAX,
    },
    peripheral::Peripheral,
    peripherals::GPIO,
    private::{self, Sealed},
};

/// A configurable input signal between a peripheral and a GPIO pin.
///
/// Obtained by calling [`GpioPin::peripheral_input()`],
/// [`super::Flex::peripheral_input()`] or [`super::Input::peripheral_input()`].
///
/// Multiple input signals can be connected to one pin.
pub struct InputSignal {
    pin: AnyPin,
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

    unsafe fn clone_unchecked(&self) -> Self::P {
        self.clone()
    }
}

impl Sealed for InputSignal {}

impl InputSignal {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self {
            pin,
            is_inverted: false,
        }
    }

    /// Returns the current signal level.
    pub fn get_level(&self) -> Level {
        self.is_input_high(private::Internal).into()
    }

    /// Inverts the peripheral's input signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    /// Consumed the signal and returns a new one that inverts the peripheral's
    /// input signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn inverted(mut self) -> Self {
        self.invert();
        self
    }

    /// - signal: The input signal to connect to the pin
    /// - invert: Configures whether or not to invert the input value
    /// - input: The GPIO number to connect to the input signal
    fn connect(&self, signal: usize, invert: bool, input: u8) {
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| unsafe {
                w.sel().set_bit();
                w.in_inv_sel().bit(invert);
                w.in_sel().bits(input)
            });
    }

    #[doc(hidden)]
    /// Connect the pin to a peripheral input signal.
    ///
    /// Since there can only be one input signal connected to a peripheral at a
    /// time, this function will disconnect any previously connected input
    /// signals.
    pub fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal, _: private::Internal) {
        let signal_nr = signal as usize;

        let af = if self.is_inverted {
            GPIO_FUNCTION
        } else {
            self.input_signals(private::Internal)
                .into_iter()
                .position(|s| s == Some(signal))
                .ok_or(())
                .and_then(AlternateFunction::try_from)
                .unwrap_or(GPIO_FUNCTION)
        };

        if af == GPIO_FUNCTION && signal_nr > INPUT_SIGNAL_MAX as usize {
            panic!("Cannot connect GPIO to this peripheral");
        }

        self.pin.set_alternate_function(af, private::Internal);

        if signal_nr <= INPUT_SIGNAL_MAX as usize {
            self.connect(signal_nr, self.is_inverted, self.pin.number());
        }
    }

    #[doc(hidden)]
    /// Remove this pin from a connected peripheral input.
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    pub fn disconnect_input_from_peripheral(
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
        #[doc(hidden)]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull, _internal: private::Internal);
            pub fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
            pub fn init_input(&self, pull: Pull, _internal: private::Internal);
            pub fn is_input_high(&self, _internal: private::Internal) -> bool;
            pub fn enable_input(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
        }
    }
}

/// A configurable output signal between a peripheral and a GPIO pin.
///
/// Obtained by calling [`GpioPin::into_peripheral_output()`],
/// [`super::Flex::into_peripheral_output()`] or
/// [`super::Output::into_peripheral_output()`].
///
/// Multiple pins can be connected to one output signal.
pub struct OutputSignal {
    pin: AnyPin,
    is_inverted: bool,
}

impl Peripheral for OutputSignal {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self {
            pin: self.pin.clone_unchecked(),
            is_inverted: self.is_inverted,
        }
    }
}

impl Sealed for OutputSignal {}

impl OutputSignal {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self {
            pin,
            is_inverted: false,
        }
    }

    /// Inverts the peripheral's output signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    /// Consumed the signal and returns a new one that inverts the peripheral's
    /// output signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn inverted(mut self) -> Self {
        self.invert();
        self
    }

    /// - signal: The output signal to connect to the pin
    /// - invert: Configures whether or not to invert the output value
    /// - invert_enable: Configures whether or not to invert the output enable
    ///   signal
    /// - enable_from_gpio: Configures to select the source of output enable
    ///   signal.
    ///   - false: Use output enable signal from peripheral
    ///   - true: Force the output enable signal to be sourced from bit n of
    ///     GPIO_ENABLE_REG
    /// - output: The GPIO number to connect to the output signal
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
                w.out_sel().bits(signal);
                w.inv_sel().bit(invert);
                w.oen_sel().bit(enable_from_gpio);
                w.oen_inv_sel().bit(invert_enable)
            });
    }

    /// - signal: The input signal to connect to the pin
    /// - invert: Configures whether or not to invert the input value
    /// - input: The GPIO number to connect to the input signal
    fn connect_input(&self, signal: usize, invert: bool, input: u8) {
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| unsafe {
                w.sel().set_bit();
                w.in_inv_sel().bit(invert);
                w.in_sel().bits(input)
            });
    }

    #[doc(hidden)]
    /// Connect the pin to a peripheral input signal.
    ///
    /// Since there can only be one input signal connected to a peripheral at a
    /// time, this function will disconnect any previously connected input
    /// signals.
    pub fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal, _: private::Internal) {
        let signal_nr = signal as usize;

        let af = if self.is_inverted {
            GPIO_FUNCTION
        } else {
            self.input_signals(private::Internal)
                .into_iter()
                .position(|s| s == Some(signal))
                .ok_or(())
                .and_then(AlternateFunction::try_from)
                .unwrap_or(GPIO_FUNCTION)
        };

        if af == GPIO_FUNCTION && signal_nr > INPUT_SIGNAL_MAX as usize {
            panic!("Cannot connect GPIO to this peripheral");
        }

        self.pin.set_alternate_function(af, private::Internal);

        if signal_nr <= INPUT_SIGNAL_MAX as usize {
            self.connect_input(signal_nr, self.is_inverted, self.pin.number());
        }
    }

    #[doc(hidden)]
    /// Remove this pin from a connected peripheral input.
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    pub fn disconnect_input_from_peripheral(
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

    #[doc(hidden)]
    /// Connect the pin to a peripheral output signal.
    pub fn connect_peripheral_to_output(
        &mut self,
        signal: gpio::OutputSignal,
        _: private::Internal,
    ) {
        let af = if self.is_inverted {
            GPIO_FUNCTION
        } else {
            self.output_signals(private::Internal)
                .into_iter()
                .position(|s| s == Some(signal))
                .ok_or(())
                .and_then(AlternateFunction::try_from)
                .unwrap_or(GPIO_FUNCTION)
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
            self.pin.number(),
        );
    }

    #[doc(hidden)]
    /// Remove this output pin from a connected [signal](`OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`OutputSignal`). Any other
    /// outputs connected to the peripheral remain intact.
    pub fn disconnect_from_peripheral_output(
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
        #[doc(hidden)]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull, _internal: private::Internal);
            pub fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
            pub fn init_input(&self, pull: Pull, _internal: private::Internal);
            pub fn is_input_high(&self, _internal: private::Internal) -> bool;
            pub fn enable_input(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);

            pub fn output_signals(&self, _internal: private::Internal) -> [Option<gpio::OutputSignal>; 6];
            pub fn set_to_open_drain_output(&mut self, _internal: private::Internal);
            pub fn set_to_push_pull_output(&mut self, _internal: private::Internal);
            pub fn enable_output(&mut self, on: bool, _internal: private::Internal);
            pub fn set_output_high(&mut self, on: bool, _internal: private::Internal);
            pub fn set_drive_strength(&mut self, strength: gpio::DriveStrength, _internal: private::Internal);
            pub fn enable_open_drain(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_output_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn is_set_high(&self, _internal: private::Internal) -> bool;
        }
    }
}

enum OutputConnectionInner {
    Output(OutputSignal),
    Dummy(NoPin),
}

/// A type-erased (input and) output signal, intended for internal use.
///
/// Peripheral drivers are supposed to take this and [`InputConnection`] as
/// arguments instead of pin types.
pub struct OutputConnection(OutputConnectionInner);

impl Sealed for OutputConnection {}

impl Peripheral for OutputConnection {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        match self {
            Self(OutputConnectionInner::Output(signal)) => Self::from(signal.clone_unchecked()),
            Self(OutputConnectionInner::Dummy(pin)) => Self::from(*pin),
        }
    }
}

impl From<NoPin> for OutputConnection {
    fn from(pin: NoPin) -> Self {
        Self(OutputConnectionInner::Dummy(pin))
    }
}

impl From<AnyPin> for OutputConnection {
    fn from(input: AnyPin) -> Self {
        Self(OutputConnectionInner::Output(
            input.into_peripheral_output(),
        ))
    }
}

impl From<OutputSignal> for OutputConnection {
    fn from(signal: OutputSignal) -> Self {
        Self(OutputConnectionInner::Output(signal))
    }
}

impl<const GPIONUM: u8> From<GpioPin<GPIONUM>> for OutputConnection
where
    GpioPin<GPIONUM>: OutputPin,
{
    fn from(pin: GpioPin<GPIONUM>) -> Self {
        Self(OutputConnectionInner::Output(pin.into_peripheral_output()))
    }
}

impl OutputConnection {
    delegate::delegate! {
        #[doc(hidden)]
        to match &self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::Dummy(_) => Level::Low,
        } {
            pub fn pull_direction(&self, pull: Pull, _internal: private::Internal);
            pub fn init_input(&self, pull: Pull, _internal: private::Internal);
            pub fn is_input_high(&self, _internal: private::Internal) -> bool;
            pub fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
        }
        #[doc(hidden)]
        to match &mut self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::Dummy(_) => Level::Low,
        } {
            pub fn enable_input(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn connect_input_to_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
            pub fn disconnect_input_from_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
        }

        #[doc(hidden)]
        to match &self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::Dummy(pin) => pin,
        } {
            pub fn is_set_high(&self, _internal: private::Internal) -> bool;
            pub fn output_signals(&self, _internal: private::Internal) -> [Option<gpio::OutputSignal>; 6];
        }

        #[doc(hidden)]
        to match &mut self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::Dummy(pin) => pin,
        } {
            pub fn set_to_open_drain_output(&mut self, _internal: private::Internal);
            pub fn set_to_push_pull_output(&mut self, _internal: private::Internal);
            pub fn enable_output(&mut self, on: bool, _internal: private::Internal);
            pub fn set_output_high(&mut self, on: bool, _internal: private::Internal);
            pub fn set_drive_strength(&mut self, strength: gpio::DriveStrength, _internal: private::Internal);
            pub fn enable_open_drain(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_output_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn connect_peripheral_to_output(&mut self, signal: gpio::OutputSignal, _internal: private::Internal);
            pub fn disconnect_from_peripheral_output(&mut self, signal: gpio::OutputSignal, _internal: private::Internal);
        }
    }
}

#[derive(Clone)]
enum InputConnectionInner {
    Input(InputSignal),
    Constant(Level),
}

/// A type-erased input signal, intended for internal use.
///
/// Peripheral drivers are supposed to take this and [`OutputConnection`] as
/// arguments instead of pin types.
#[derive(Clone)]
pub struct InputConnection(InputConnectionInner);

impl Sealed for InputConnection {}

impl Peripheral for InputConnection {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        self.clone()
    }
}

impl From<InputSignal> for InputConnection {
    fn from(input: InputSignal) -> Self {
        Self(InputConnectionInner::Input(input))
    }
}

impl From<Level> for InputConnection {
    fn from(level: Level) -> Self {
        Self(InputConnectionInner::Constant(level))
    }
}

impl From<NoPin> for InputConnection {
    fn from(_pin: NoPin) -> Self {
        Self(InputConnectionInner::Constant(Level::Low))
    }
}

impl From<AnyPin> for InputConnection {
    fn from(input: AnyPin) -> Self {
        Self(InputConnectionInner::Input(input.peripheral_input()))
    }
}

impl<const GPIONUM: u8> From<GpioPin<GPIONUM>> for InputConnection
where
    GpioPin<GPIONUM>: InputPin,
{
    fn from(pin: GpioPin<GPIONUM>) -> Self {
        Self(InputConnectionInner::Input(pin.peripheral_input()))
    }
}

impl InputConnection {
    delegate::delegate! {
        #[doc(hidden)]
        to match &self.0 {
            InputConnectionInner::Input(pin) => pin,
            InputConnectionInner::Constant(level) => level,
        } {
            pub fn pull_direction(&self, pull: Pull, _internal: private::Internal);
            pub fn init_input(&self, pull: Pull, _internal: private::Internal);
            pub fn is_input_high(&self, _internal: private::Internal) -> bool;
            pub fn input_signals(&self, _internal: private::Internal) -> [Option<gpio::InputSignal>; 6];
        }

        #[doc(hidden)]
        to match &mut self.0 {
            InputConnectionInner::Input(pin) => pin,
            InputConnectionInner::Constant(level) => level,
        } {
            pub fn enable_input(&mut self, on: bool, _internal: private::Internal);
            pub fn enable_input_in_sleep_mode(&mut self, on: bool, _internal: private::Internal);
            pub fn connect_input_to_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
            pub fn disconnect_input_from_peripheral(&mut self, signal: crate::gpio::InputSignal, _internal: private::Internal);
        }
    }
}
