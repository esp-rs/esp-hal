//! Peripheral signal interconnect using IOMUX or GPIOMUX.

// FIXME: https://github.com/esp-rs/esp-hal/issues/2954 The GPIO implementation does not contain any
// locking. This is okay there, because the implementation uses either W1TS/W1TC
// registers to set certain bits, or separate registers for each pin - and there
// can only be one instance of every GPIO struct in safe code. However, with the
// interconnect module we allow multiple handles, which means possible RMW
// operations on the pin registers cause data races.

use crate::{
    gpio::{
        self,
        AlternateFunction,
        AnyPin,
        Flex,
        InputPin,
        InputSignalType,
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

/// A signal that can be connected to a peripheral input.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralOutput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "InputConnection is unstable, but the trait needs to be public"
)]
pub trait PeripheralInput: Into<InputConnection> + 'static + crate::private::Sealed {}

/// A signal that can be connected to a peripheral input and/or output.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralInput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "OutputConnection is unstable, but the trait needs to be public"
)]
pub trait PeripheralOutput: Into<OutputConnection> + 'static + crate::private::Sealed {}

// Pins
impl<P: InputPin> PeripheralInput for P {}
impl<P: OutputPin> PeripheralOutput for P {}

// Pin drivers
impl PeripheralInput for Flex<'static> {}
impl PeripheralOutput for Flex<'static> {}

// Placeholders
impl PeripheralInput for NoPin {}
impl PeripheralOutput for NoPin {}

impl PeripheralInput for Level {}
impl PeripheralOutput for Level {}

// Split signals
impl PeripheralInput for InputSignal {}
impl PeripheralInput for OutputSignal {}
impl PeripheralOutput for OutputSignal {}

// Type-erased signals
impl PeripheralInput for InputConnection {}
impl PeripheralInput for OutputConnection {}
impl PeripheralOutput for OutputConnection {}

impl gpio::InputSignal {
    fn can_use_gpio_matrix(self) -> bool {
        self as InputSignalType <= INPUT_SIGNAL_MAX
    }

    /// Connects a peripheral input signal to a GPIO or a constant level.
    ///
    /// Note that connecting multiple GPIOs to a single peripheral input is not
    /// possible and the previous connection will be replaced.
    ///
    /// Also note that a peripheral input must always be connected to something,
    /// so if you want to disconnect it from GPIOs, you should connect it to a
    /// constant level.
    #[inline]
    pub fn connect_to(self, pin: impl Peripheral<P = impl PeripheralInput>) {
        crate::into_mapped_ref!(pin);

        pin.connect_input_to_peripheral(self);
    }
}

impl gpio::OutputSignal {
    fn can_use_gpio_matrix(self) -> bool {
        self as OutputSignalType <= OUTPUT_SIGNAL_MAX
    }

    /// Connects a peripheral output signal to a GPIO.
    ///
    /// Note that connecting multiple output signals to a single GPIO is not
    /// possible and the previous connection will be replaced.
    ///
    /// Also note that it is possible to connect a peripheral output signal to
    /// multiple GPIOs, and old connections will not be cleared automatically.
    #[inline]
    pub fn connect_to(self, pin: impl Peripheral<P = impl PeripheralOutput>) {
        crate::into_mapped_ref!(pin);

        // FIXME: disconnect previous connection(s)
        pin.connect_peripheral_to_output(self);
    }

    /// Disconnects a peripheral output signal from a GPIO.
    #[inline]
    pub fn disconnect_from(self, pin: impl Peripheral<P = impl PeripheralOutput>) {
        crate::into_mapped_ref!(pin);

        pin.disconnect_from_peripheral_output(self);
    }
}

/// Connects a peripheral input (`signal`, e.g. SPI MISO) to a GPIO or a
/// constant level (`input`).
///
/// - `signal`: The input signal to connect to the pin
/// - `input`: The GPIO (or constant level) number to connect to the input
///   signal.
/// - `invert`: Configures whether or not to invert the input value
/// - `use_gpio_matrix`: true to route through the GPIO matrix
pub(crate) fn connect_input_signal(
    signal: gpio::InputSignal,
    input: u8,
    invert: bool,
    use_gpio_matrix: bool,
) {
    assert!(
        signal.can_use_gpio_matrix() || !use_gpio_matrix,
        "{:?} cannot be routed through the GPIO matrix",
        signal
    );
    GPIO::regs()
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .write(|w| unsafe {
            w.sel().bit(use_gpio_matrix);
            w.in_inv_sel().bit(invert);
            w.in_sel().bits(input) // Connect to GPIO or constant level
        });
}

fn connect_pin_to_input_signal(
    pin: &mut AnyPin,
    signal: gpio::InputSignal,
    is_inverted: bool,
    force_gpio: bool,
) {
    let af = if is_inverted || force_gpio {
        GPIO_FUNCTION
    } else {
        pin.input_signals(private::Internal)
            .iter()
            .find(|(_af, s)| *s == signal)
            .map(|(af, _)| *af)
            .unwrap_or(GPIO_FUNCTION)
    };

    pin.set_alternate_function(af);

    connect_input_signal(signal, pin.number(), is_inverted, af == GPIO_FUNCTION);
}

fn connect_peripheral_to_output(
    pin: &mut AnyPin,
    signal: gpio::OutputSignal,
    is_inverted: bool,
    force_gpio: bool,
    peripheral_control_output_enable: bool,
    invert_output_enable: bool,
) {
    let af = if is_inverted || force_gpio {
        GPIO_FUNCTION
    } else {
        pin.output_signals(private::Internal)
            .iter()
            .find(|(_af, s)| *s == signal)
            .map(|(af, _)| *af)
            .unwrap_or(GPIO_FUNCTION)
    };

    assert!(
        signal.can_use_gpio_matrix() || af != GPIO_FUNCTION,
        "{:?} cannot be routed through the GPIO matrix",
        signal
    );

    pin.set_alternate_function(af);

    // Inlined because output signals can only be connected to pins or nothing, so
    // there is no other user.
    GPIO::regs()
        .func_out_sel_cfg(pin.number() as usize)
        .write(|w| unsafe {
            if af == GPIO_FUNCTION {
                // Ignored if the signal is not routed through the GPIO matrix - alternate
                // function selects peripheral signal directly.
                w.out_sel().bits(signal as _);
                w.inv_sel().bit(is_inverted);
            }
            w.oen_sel().bit(!peripheral_control_output_enable);
            w.oen_inv_sel().bit(invert_output_enable)
        });
}

fn disconnect_peripheral_output_from_pin(pin: &mut AnyPin, signal: gpio::OutputSignal) {
    pin.set_alternate_function(GPIO_FUNCTION);

    GPIO::regs()
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .write(|w| w.sel().clear_bit());
}

/// A configurable input signal between a peripheral and a GPIO pin.
///
/// Multiple input signals can be connected to one pin.
#[instability::unstable]
pub struct InputSignal {
    pin: AnyPin,
    is_inverted: bool,
}

impl<P> From<P> for InputSignal
where
    P: InputPin,
{
    fn from(input: P) -> Self {
        Self::new(input.degrade())
    }
}

impl From<Flex<'static>> for InputSignal {
    fn from(input: Flex<'static>) -> Self {
        Self::new(unsafe { AnyPin::steal(input.pin.number()) })
    }
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

    /// Returns the GPIO number of the underlying pin.
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Returns the current signal level.
    pub fn level(&self) -> Level {
        self.is_input_high().into()
    }

    /// Inverts the peripheral's input signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// input signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn inverted(mut self) -> Self {
        self.invert();
        self
    }

    /// Connect the pin to a peripheral input signal.
    ///
    /// Since there can only be one input signal connected to a peripheral at a
    /// time, this function will disconnect any previously connected input
    /// signals.
    fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&mut self.pin, signal, self.is_inverted, true);
    }

    delegate::delegate! {
        #[doc(hidden)]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull);
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn init_input(&self, pull: Pull);
            pub fn is_input_high(&self) -> bool;
            pub fn enable_input(&mut self, on: bool);
        }
    }
}

/// A limited, private version of [InputSignal] that allows bypassing the GPIO
/// matrix. This is only usable when the GPIO pin connected to the signal is not
/// split.
struct DirectInputSignal {
    pin: AnyPin,
}

impl Clone for DirectInputSignal {
    fn clone(&self) -> Self {
        Self::new(unsafe { self.pin.clone_unchecked() })
    }
}

impl DirectInputSignal {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self { pin }
    }

    /// Connect the pin to a peripheral input signal.
    ///
    /// Since there can only be one input signal connected to a peripheral at a
    /// time, this function will disconnect any previously connected input
    /// signals.
    fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&mut self.pin, signal, false, false);
    }

    delegate::delegate! {
        to self.pin {
            fn pull_direction(&self, pull: Pull);
            fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            fn init_input(&self, pull: Pull);
            fn is_input_high(&self) -> bool;
            fn enable_input(&mut self, on: bool);
        }
    }
}

/// A configurable output signal between a peripheral and a GPIO pin.
///
/// Multiple pins can be connected to one output signal.
#[instability::unstable]
pub struct OutputSignal {
    pin: AnyPin,
    is_inverted: bool,
}

impl<P> From<P> for OutputSignal
where
    P: OutputPin,
{
    fn from(output: P) -> Self {
        Self::new(output.degrade())
    }
}

impl From<Flex<'static>> for OutputSignal {
    fn from(output: Flex<'static>) -> Self {
        Self::new(unsafe { AnyPin::steal(output.pin.number()) })
    }
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

    /// Returns the GPIO number of the underlying pin.
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Inverts the peripheral's output signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn invert(&mut self) {
        self.is_inverted = !self.is_inverted;
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// output signal.
    ///
    /// Calling this function multiple times toggles the setting.
    pub fn inverted(mut self) -> Self {
        self.invert();
        self
    }

    /// Connect the pin to a peripheral output signal.
    fn connect_peripheral_to_output(&mut self, signal: gpio::OutputSignal) {
        connect_peripheral_to_output(&mut self.pin, signal, self.is_inverted, true, true, false);
    }

    /// Remove this output pin from a connected [signal](`gpio::OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`gpio::OutputSignal`). Any
    /// other outputs connected to the peripheral remain intact.
    fn disconnect_from_peripheral_output(&mut self, signal: gpio::OutputSignal) {
        disconnect_peripheral_output_from_pin(&mut self.pin, signal);
    }

    delegate::delegate! {
        #[instability::unstable]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull);
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn init_input(&self, pull: Pull);
            pub fn is_input_high(&self) -> bool;
            pub fn enable_input(&mut self, on: bool);

            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            pub fn set_to_open_drain_output(&mut self);
            pub fn set_to_push_pull_output(&mut self);
            pub fn enable_output(&mut self, on: bool);
            pub fn set_output_high(&mut self, on: bool);
            pub fn set_drive_strength(&mut self, strength: gpio::DriveStrength);
            pub fn enable_open_drain(&mut self, on: bool);
            pub fn is_set_high(&self) -> bool;
        }
    }
}

/// A limited, private version of [OutputSignal] that allows bypassing the GPIO
/// matrix. This is only usable when the GPIO pin connected to the signal is not
/// split.
struct DirectOutputSignal {
    pin: AnyPin,
}

impl DirectOutputSignal {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self { pin }
    }

    /// Connect the pin to a peripheral output signal.
    fn connect_peripheral_to_output(&mut self, signal: gpio::OutputSignal) {
        connect_peripheral_to_output(&mut self.pin, signal, false, false, true, false);
    }

    /// Remove this output pin from a connected [signal](`gpio::OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`gpio::OutputSignal`). Any
    /// other outputs connected to the peripheral remain intact.
    fn disconnect_from_peripheral_output(&mut self, signal: gpio::OutputSignal) {
        disconnect_peripheral_output_from_pin(&mut self.pin, signal);
    }

    delegate::delegate! {
        to self.pin {
            fn pull_direction(&self, pull: Pull);
            fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            fn init_input(&self, pull: Pull);
            fn is_input_high(&self) -> bool;
            fn enable_input(&mut self, on: bool);

            fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            fn set_to_open_drain_output(&mut self);
            fn set_to_push_pull_output(&mut self);
            fn enable_output(&mut self, on: bool);
            fn set_output_high(&mut self, on: bool);
            fn set_drive_strength(&mut self, strength: gpio::DriveStrength);
            fn enable_open_drain(&mut self, on: bool);
            fn is_set_high(&self) -> bool;
        }
    }
}

#[derive(Clone)]
enum InputConnectionInner {
    Input(InputSignal),
    DirectInput(DirectInputSignal),
    Constant(Level),
}

/// A peripheral input signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
#[derive(Clone)]
#[instability::unstable]
pub struct InputConnection(InputConnectionInner);

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

impl<P> From<P> for InputConnection
where
    P: InputPin,
{
    fn from(input: P) -> Self {
        Self(InputConnectionInner::Input(InputSignal::from(input)))
    }
}

impl From<OutputSignal> for InputConnection {
    fn from(output_signal: OutputSignal) -> Self {
        Self(InputConnectionInner::Input(InputSignal {
            pin: output_signal.pin,
            is_inverted: output_signal.is_inverted,
        }))
    }
}

impl From<DirectOutputSignal> for InputConnection {
    fn from(output_signal: DirectOutputSignal) -> Self {
        Self(InputConnectionInner::DirectInput(DirectInputSignal::new(
            output_signal.pin,
        )))
    }
}

impl From<OutputConnection> for InputConnection {
    fn from(conn: OutputConnection) -> Self {
        match conn.0 {
            OutputConnectionInner::Output(inner) => inner.into(),
            OutputConnectionInner::DirectOutput(inner) => inner.into(),
            OutputConnectionInner::Constant(inner) => inner.into(),
        }
    }
}

impl From<Flex<'static>> for InputConnection {
    fn from(pin: Flex<'static>) -> Self {
        pin.peripheral_input().into()
    }
}

impl Sealed for InputConnection {}

impl InputConnection {
    delegate::delegate! {
        #[instability::unstable]
        to match &self.0 {
            InputConnectionInner::Input(pin) => pin,
            InputConnectionInner::DirectInput(pin) => pin,
            InputConnectionInner::Constant(level) => level,
        } {
            pub fn pull_direction(&self, pull: Pull);
            pub fn init_input(&self, pull: Pull);
            pub fn is_input_high(&self) -> bool;
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
        }

        #[instability::unstable]
        to match &mut self.0 {
            InputConnectionInner::Input(pin) => pin,
            InputConnectionInner::DirectInput(pin) => pin,
            InputConnectionInner::Constant(level) => level,
        } {
            pub fn enable_input(&mut self, on: bool);

            // This doesn't need to be public, the intended way is `connect_to` and `disconnect_from`
            fn connect_input_to_peripheral(&mut self, signal: gpio::InputSignal);
        }
    }
}

enum OutputConnectionInner {
    Output(OutputSignal),
    DirectOutput(DirectOutputSignal),
    Constant(Level),
}

/// A peripheral (input and) output signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
#[instability::unstable]
pub struct OutputConnection(OutputConnectionInner);

impl Sealed for OutputConnection {}

impl Peripheral for OutputConnection {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        match self {
            Self(OutputConnectionInner::Output(signal)) => Self::from(signal.clone_unchecked()),
            Self(OutputConnectionInner::DirectOutput(signal)) => {
                Self::from(DirectOutputSignal::new(signal.pin.clone_unchecked()))
            }
            Self(OutputConnectionInner::Constant(level)) => Self::from(*level),
        }
    }
}

impl From<NoPin> for OutputConnection {
    fn from(_pin: NoPin) -> Self {
        Self(OutputConnectionInner::Constant(Level::Low))
    }
}

impl From<Level> for OutputConnection {
    fn from(level: Level) -> Self {
        Self(OutputConnectionInner::Constant(level))
    }
}

impl<P> From<P> for OutputConnection
where
    P: OutputPin,
{
    fn from(input: P) -> Self {
        Self(OutputConnectionInner::Output(OutputSignal::from(input)))
    }
}

impl From<OutputSignal> for OutputConnection {
    fn from(signal: OutputSignal) -> Self {
        Self(OutputConnectionInner::Output(signal))
    }
}

impl From<Flex<'static>> for OutputConnection {
    fn from(pin: Flex<'static>) -> Self {
        pin.into_peripheral_output().into()
    }
}

impl From<DirectOutputSignal> for OutputConnection {
    fn from(signal: DirectOutputSignal) -> Self {
        Self(OutputConnectionInner::DirectOutput(signal))
    }
}

impl OutputConnection {
    delegate::delegate! {
        #[instability::unstable]
        to match &self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::DirectOutput(pin) => pin,
            OutputConnectionInner::Constant(level) => level,
        } {
            pub fn is_input_high(&self) -> bool;
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];

            pub fn is_set_high(&self) -> bool;
            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
        }
        #[instability::unstable]
        to match &mut self.0 {
            OutputConnectionInner::Output(pin) => pin,
            OutputConnectionInner::DirectOutput(pin) => pin,
            OutputConnectionInner::Constant(level) => level,
        } {
            pub fn pull_direction(&mut self, pull: Pull);
            pub fn init_input(&mut self, pull: Pull);
            pub fn enable_input(&mut self, on: bool);

            pub fn set_to_open_drain_output(&mut self);
            pub fn set_to_push_pull_output(&mut self);
            pub fn enable_output(&mut self, on: bool);
            pub fn set_output_high(&mut self, on: bool);
            pub fn set_drive_strength(&mut self, strength: gpio::DriveStrength);
            pub fn enable_open_drain(&mut self, on: bool);

            // These don't need to be public, the intended way is `connect_to` and `disconnect_from`
            fn connect_peripheral_to_output(&mut self, signal: gpio::OutputSignal);
            fn disconnect_from_peripheral_output(&mut self, signal: gpio::OutputSignal);
        }
    }
}
