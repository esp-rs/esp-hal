//! Peripheral signal interconnect using IOMUX or GPIOMUX.

// FIXME: https://github.com/esp-rs/esp-hal/issues/2954 The GPIO implementation does not contain any
// locking. This is okay there, because the implementation uses either W1TS/W1TC
// registers to set certain bits, or separate registers for each pin - and there
// can only be one instance of every GPIO struct in safe code. However, with the
// interconnect module we allow multiple handles, which means possible RMW
// operations on the pin registers cause data races.

use core::marker::PhantomData;

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
        PinGuard,
        Pull,
        FUNC_IN_SEL_OFFSET,
        GPIO_FUNCTION,
        INPUT_SIGNAL_MAX,
        OUTPUT_SIGNAL_MAX,
    },
    peripheral::{Peripheral, PeripheralRef},
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
pub trait PeripheralInput<'d>: Into<InputConnection<'d>> + crate::private::Sealed {
    /// Connects the peripheral input to an input signal source.
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal);
}

/// A signal that can be connected to a peripheral input and/or output.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralInput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "OutputConnection is unstable, but the trait needs to be public"
)]
pub trait PeripheralOutput<'d>: Into<OutputConnection<'d>> + crate::private::Sealed {
    /// Connects the peripheral output to an output signal target.
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal);

    /// Disconnects the peripheral output from an output signal target.
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal);
}

// Pins
impl<'d, P, IP> PeripheralInput<'d> for P
where
    P: Peripheral<P = IP> + Into<InputConnection<'d>> + crate::private::Sealed,
    IP: InputPin,
{
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        let this = PeripheralRef::new(unsafe { self.clone_unchecked() });
        let pin = unsafe { AnyPin::steal(this.number()) };
        connect_pin_to_input_signal(&pin, signal, false, false);
    }
}

impl<'d, P, OP> PeripheralOutput<'d> for P
where
    P: Peripheral<P = OP> + Into<OutputConnection<'d>> + crate::private::Sealed,
    OP: OutputPin,
{
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        let this = PeripheralRef::new(unsafe { self.clone_unchecked() });
        let pin = unsafe { AnyPin::steal(this.number()) };
        connect_peripheral_to_output(&pin, signal, false, false, true, false);
    }
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        let this = PeripheralRef::new(unsafe { self.clone_unchecked() });
        let pin = unsafe { AnyPin::steal(this.number()) };
        disconnect_peripheral_output_from_pin(&pin, signal);
    }
}

// Pin drivers
impl<'d> PeripheralInput<'d> for Flex<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.pin.connect_input_to_peripheral(signal);
    }
}
impl<'d> PeripheralOutput<'d> for Flex<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.pin.connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        self.pin.disconnect_from_peripheral_output(signal);
    }
}

// Placeholders
impl PeripheralInput<'static> for NoPin {
    fn connect_input_to_peripheral(&self, _: gpio::InputSignal) {
        // One might argue we should be removing the previous mapping
        // here. However, looking that up needs looping through all GPIOs and
        // overwriting connections made outside of a peripheral driver
        // may be unwanted.
    }
}
impl PeripheralOutput<'static> for NoPin {
    fn connect_peripheral_to_output(&self, _: gpio::OutputSignal) {
        // A peripheral's outputs may be connected to any number of GPIOs.
        // Connecting to, and disconnecting from a NoPin is therefore a
        // no-op, as we are adding and removing nothing from that list of
        // connections.
    }
    fn disconnect_from_peripheral_output(&self, _: gpio::OutputSignal) {
        // A peripheral's outputs may be connected to any number of GPIOs.
        // Connecting to, and disconnecting from a NoPin is therefore a
        // no-op, as we are adding and removing nothing from that list of
        // connections.
    }
}

impl PeripheralInput<'static> for Level {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        let value = match self {
            Level::High => gpio::ONE_INPUT,
            Level::Low => gpio::ZERO_INPUT,
        };

        connect_input_signal(signal, value, false, true);
    }
}
impl PeripheralOutput<'static> for Level {
    fn connect_peripheral_to_output(&self, _: gpio::OutputSignal) {
        // There is no such thing as a constant-high level peripheral output,
        // the implementation just exists for convenience.
    }
    fn disconnect_from_peripheral_output(&self, _: gpio::OutputSignal) {
        // There is no such thing as a constant-high level peripheral output,
        // the implementation just exists for convenience.
    }
}

// Split signals
impl<'d> PeripheralInput<'d> for InputSignal<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}
impl<'d> PeripheralInput<'d> for OutputSignal<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}
impl<'d> PeripheralOutput<'d> for OutputSignal<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        self.connect_peripheral_to_output(signal);
    }
}

// Type-erased signals
impl<'d> PeripheralInput<'d> for InputConnection<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}
impl<'d> PeripheralInput<'d> for OutputConnection<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}
impl<'d> PeripheralOutput<'d> for OutputConnection<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        self.disconnect_from_peripheral_output(signal);
    }
}

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
    pub fn connect_to<'d>(self, pin: &impl PeripheralInput<'d>) {
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
    pub fn connect_to<'d>(self, pin: &impl PeripheralOutput<'d>) {
        pin.connect_peripheral_to_output(self);
    }

    /// Disconnects a peripheral output signal from a GPIO.
    #[inline]
    pub fn disconnect_from<'d>(self, pin: &impl PeripheralOutput<'d>) {
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
    pin: &AnyPin,
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
    pin: &AnyPin,
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

fn disconnect_peripheral_output_from_pin(pin: &AnyPin, signal: gpio::OutputSignal) {
    pin.set_alternate_function(GPIO_FUNCTION);

    GPIO::regs()
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .write(|w| w.sel().clear_bit());
}

/// A configurable input signal between a peripheral and a GPIO pin.
///
/// Multiple input signals can be connected to one pin.
#[instability::unstable]
pub struct InputSignal<'d> {
    pin: AnyPin,
    is_inverted: bool,
    _lifetime: PhantomData<&'d mut ()>,
}

impl Sealed for InputSignal<'_> {}

impl<P> From<P> for InputSignal<'static>
where
    P: InputPin,
{
    fn from(input: P) -> Self {
        Self::new(input.degrade())
    }
}

impl<'d> From<Flex<'d>> for InputSignal<'d> {
    fn from(input: Flex<'d>) -> Self {
        InputSignal::new(unsafe { AnyPin::steal(input.pin.number()) })
    }
}

impl Clone for InputSignal<'_> {
    fn clone(&self) -> Self {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            is_inverted: self.is_inverted,
            _lifetime: PhantomData,
        }
    }
}

impl InputSignal<'static> {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self {
            pin,
            is_inverted: false,
            _lifetime: PhantomData,
        }
    }
}

impl InputSignal<'_> {
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
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&self.pin, signal, self.is_inverted, true);
    }

    delegate::delegate! {
        #[doc(hidden)]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull);
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn init_input(&self, pull: Pull);
            pub fn is_input_high(&self) -> bool;
            pub fn enable_input(&self, on: bool);
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
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&self.pin, signal, false, false);
    }

    delegate::delegate! {
        to self.pin {
            fn pull_direction(&self, pull: Pull);
            fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            fn init_input(&self, pull: Pull);
            fn is_input_high(&self) -> bool;
            fn enable_input(&self, on: bool);
        }
    }
}

/// A configurable output signal between a peripheral and a GPIO pin.
///
/// Multiple pins can be connected to one output signal.
#[instability::unstable]
pub struct OutputSignal<'d> {
    pin: AnyPin,
    is_inverted: bool,
    _lifetime: PhantomData<&'d mut ()>,
}

impl Sealed for OutputSignal<'_> {}

impl<P> From<P> for OutputSignal<'static>
where
    P: OutputPin,
{
    fn from(output: P) -> Self {
        Self::new(output.degrade())
    }
}

impl<'d> From<Flex<'d>> for OutputSignal<'d> {
    fn from(output: Flex<'d>) -> Self {
        OutputSignal::new(unsafe { AnyPin::steal(output.pin.number()) })
    }
}

impl OutputSignal<'static> {
    pub(crate) fn new(pin: AnyPin) -> Self {
        Self {
            pin,
            is_inverted: false,
            _lifetime: PhantomData,
        }
    }
}

impl OutputSignal<'_> {
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

    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&self.pin, signal, self.is_inverted, true);
    }

    /// Connect the pin to a peripheral output signal.
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        connect_peripheral_to_output(&self.pin, signal, self.is_inverted, true, true, false);
    }

    /// Remove this output pin from a connected [signal](`gpio::OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`gpio::OutputSignal`). Any
    /// other outputs connected to the peripheral remain intact.
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        disconnect_peripheral_output_from_pin(&self.pin, signal);
    }

    delegate::delegate! {
        #[instability::unstable]
        to self.pin {
            pub fn pull_direction(&self, pull: Pull);
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn init_input(&self, pull: Pull);
            pub fn is_input_high(&self) -> bool;
            pub fn enable_input(&self, on: bool);

            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            pub fn set_to_open_drain_output(&self);
            pub fn set_to_push_pull_output(&self);
            pub fn enable_output(&self, on: bool);
            pub fn set_output_high(&self, on: bool);
            pub fn set_drive_strength(&self, strength: gpio::DriveStrength);
            pub fn enable_open_drain(&self, on: bool);
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

    /// Connect the pin to a peripheral input signal.
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&self.pin, signal, false, false);
    }

    /// Connect the pin to a peripheral output signal.
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        connect_peripheral_to_output(&self.pin, signal, false, false, true, false);
    }

    /// Remove this output pin from a connected [signal](`gpio::OutputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`gpio::OutputSignal`). Any
    /// other outputs connected to the peripheral remain intact.
    fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal) {
        disconnect_peripheral_output_from_pin(&self.pin, signal);
    }

    delegate::delegate! {
        to self.pin {
            fn pull_direction(&self, pull: Pull);
            fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            fn init_input(&self, pull: Pull);
            fn is_input_high(&self) -> bool;
            fn enable_input(&self, on: bool);

            fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            fn set_to_open_drain_output(&self);
            fn set_to_push_pull_output(&self);
            fn enable_output(&self, on: bool);
            fn set_output_high(&self, on: bool);
            fn set_drive_strength(&self, strength: gpio::DriveStrength);
            fn enable_open_drain(&self, on: bool);
            fn is_set_high(&self) -> bool;
        }
    }
}

#[derive(Clone)]
enum InputConnectionInner<'d> {
    Input(InputSignal<'d>),
    DirectInput(DirectInputSignal),
    Constant(Level),
}

/// A peripheral input signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
#[derive(Clone)]
#[instability::unstable]
pub struct InputConnection<'d>(InputConnectionInner<'d>);
impl Sealed for InputConnection<'_> {}

impl<'d> From<InputSignal<'d>> for InputConnection<'d> {
    fn from(input: InputSignal<'d>) -> Self {
        Self(InputConnectionInner::Input(input))
    }
}

impl From<Level> for InputConnection<'static> {
    fn from(level: Level) -> Self {
        Self(InputConnectionInner::Constant(level))
    }
}

impl From<NoPin> for InputConnection<'static> {
    fn from(_pin: NoPin) -> Self {
        Self(InputConnectionInner::Constant(Level::Low))
    }
}

impl<P> From<P> for InputConnection<'static>
where
    P: InputPin,
{
    fn from(input: P) -> Self {
        Self::from(input.degrade().into_ref())
    }
}

impl<'d, P> From<PeripheralRef<'d, P>> for InputConnection<'d>
where
    P: InputPin,
{
    fn from(input: PeripheralRef<'d, P>) -> Self {
        Self(InputConnectionInner::DirectInput(DirectInputSignal::new(
            unsafe { AnyPin::steal(input.number()) },
        )))
    }
}

impl<'d> From<OutputSignal<'d>> for InputConnection<'d> {
    fn from(output_signal: OutputSignal<'d>) -> Self {
        Self(InputConnectionInner::Input(InputSignal {
            pin: output_signal.pin,
            is_inverted: output_signal.is_inverted,
            _lifetime: PhantomData,
        }))
    }
}

impl From<DirectOutputSignal> for InputConnection<'static> {
    fn from(output_signal: DirectOutputSignal) -> Self {
        Self(InputConnectionInner::DirectInput(DirectInputSignal::new(
            output_signal.pin,
        )))
    }
}

impl<'d> From<OutputConnection<'d>> for InputConnection<'d> {
    fn from(conn: OutputConnection<'d>) -> Self {
        match conn.0 {
            OutputConnectionInner::Output(inner) => inner.into(),
            OutputConnectionInner::DirectOutput(inner) => inner.into(),
            OutputConnectionInner::Constant(inner) => inner.into(),
        }
    }
}

impl<'d> From<Flex<'d>> for InputConnection<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.peripheral_input().into()
    }
}

impl InputConnection<'_> {
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
            pub fn enable_input(&self, on: bool);

            // This doesn't need to be public, the intended way is `connect_to` and `disconnect_from`
            fn connect_input_to_peripheral(&self, signal: gpio::InputSignal);
        }
    }
}

enum OutputConnectionInner<'d> {
    Output(OutputSignal<'d>),
    DirectOutput(DirectOutputSignal),
    Constant(Level),
}

/// A peripheral (input and) output signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
#[instability::unstable]
pub struct OutputConnection<'d>(OutputConnectionInner<'d>);
impl Sealed for OutputConnection<'_> {}

impl From<NoPin> for OutputConnection<'static> {
    fn from(_pin: NoPin) -> Self {
        Self(OutputConnectionInner::Constant(Level::Low))
    }
}

impl From<Level> for OutputConnection<'static> {
    fn from(level: Level) -> Self {
        Self(OutputConnectionInner::Constant(level))
    }
}

impl<P> From<P> for OutputConnection<'static>
where
    P: OutputPin,
{
    fn from(output: P) -> Self {
        Self::from(output.degrade().into_ref())
    }
}

impl<'d, P> From<PeripheralRef<'d, P>> for OutputConnection<'d>
where
    P: OutputPin,
{
    fn from(output: PeripheralRef<'d, P>) -> Self {
        Self(OutputConnectionInner::DirectOutput(
            DirectOutputSignal::new(unsafe { AnyPin::steal(output.number()) }),
        ))
    }
}

impl<'d> From<OutputSignal<'d>> for OutputConnection<'d> {
    fn from(signal: OutputSignal<'d>) -> Self {
        Self(OutputConnectionInner::Output(signal))
    }
}

impl<'d> From<Flex<'d>> for OutputConnection<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.into_peripheral_output().into()
    }
}

impl From<DirectOutputSignal> for OutputConnection<'static> {
    fn from(signal: DirectOutputSignal) -> Self {
        Self(OutputConnectionInner::DirectOutput(signal))
    }
}

impl OutputConnection<'_> {
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
            pub fn pull_direction(&self, pull: Pull);
            pub fn init_input(&self, pull: Pull);
            pub fn enable_input(&self, on: bool);

            pub fn set_to_open_drain_output(&self);
            pub fn set_to_push_pull_output(&self);
            pub fn enable_output(&self, on: bool);
            pub fn set_output_high(&self, on: bool);
            pub fn set_drive_strength(&self, strength: gpio::DriveStrength);
            pub fn enable_open_drain(&self, on: bool);

            // These don't need to be public, the intended way is `connect_to` and `disconnect_from`
            fn connect_input_to_peripheral(&self, signal: gpio::InputSignal);
            fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal);
            fn disconnect_from_peripheral_output(&self, signal: gpio::OutputSignal);
        }
    }

    pub(crate) fn connect_with_guard<'d>(self, signal: crate::gpio::OutputSignal) -> PinGuard {
        match self.0 {
            OutputConnectionInner::Output(pin) => PinGuard::new(pin.pin, signal),
            OutputConnectionInner::DirectOutput(pin) => PinGuard::new(pin.pin, signal),
            OutputConnectionInner::Constant(_) => PinGuard::new_unconnected(signal),
        }
    }
}
