//! # Peripheral signal interconnect using the GPIO matrix.
//!
//! The GPIO matrix offers flexible connection options between GPIO pins and
//! peripherals. This module offers capabilities not covered by GPIO pin types
//! and drivers.
#![doc = concat!("## Relation to the", crate::trm_markdown_link!())]
//! The GPIO drivers implement IO MUX and pin functionality (input/output
//! buffers, pull resistors, etc.). The GPIO matrix is represented by signals
//! and the [`PeripheralInput`] and [`PeripheralOutput`] traits. There is some
//! overlap between them: signal routing depends on what type is passed to a
//! peripheral driver's pin setter functions.
//!
//! ## Signals
//!
//! Each GPIO pin driver such as [`Input`], can be converted
//! into input or output signals. [`Flex`], which can be either input or output,
//! can be [`split`][Flex::split] into both signals at once. These signals can
//! then be individually connected to a peripheral input or output signal. This
//! allows for flexible routing of signals between peripherals and GPIO pins.
//!
//! Note that only configured GPIO drivers can be safely turned into signals,
//! and this conversion freezes the pin configuration, otherwise it would be
//! possible for multiple peripheral drivers to configure the same GPIO pin at
//! the same time, which is undefined behavior.
//!
//! GPIO signals are represented by the [`InputSignal`] and [`OutputSignal`]
//! structs. Sometimes, however, you want to connect a fixed signal level
//! to your peripheral's input, or don't want to route the output to any pin. To
//! support these use cases, peripheral drivers accept [`PeripheralInput`] and
//! [`PeripheralOutput`] implementations instead of pin types. [`Level`] and
//! [`NoPin`] also implement these traits and so they can be passed to
//! peripherals, and you can also use the [`InputConnection`] and
//! [`OutputConnection`] types to represent a signal that can either
//! be a GPIO pin or a fixed signal level.
//!
//! A GPIO pin can be configured either with a GPIO driver such as
//! [`Input`][crate::gpio::Input], or by a peripheral driver using a pin
//! assignment method such as
//! [`Spi::with_mosi`](crate::spi::master::Spi::with_mosi). The peripheral
//! drivers' preferences can be overridden by passing a pin driver to the
//! peripheral driver. When converting a driver to signals, the underlying
//! signals will be initially [frozen](InputSignal::freeze) to support this
//! use case.
//!
//! ## Inverting inputs and outputs
//!
//! The GPIO matrix allows for inverting the input and output signals. This can
//! be configured by setting the [`InputSignal::invert_input`] and
//! [`OutputSignal::invert_output`]. The hardware is configured accordingly when
//! the signal is connected to a peripheral input or output.
//!
//! ## Connection rules
//!
//! Peripheral signals and GPIOs can be connected with the following
//! constraints:
//!
//! - A peripheral input signal must be driven by exactly one signal, which can
//!   be a GPIO input or a constant level.
//! - A peripheral output signal can be connected to any number of GPIOs. These
//!   GPIOs can be configured differently. The peripheral drivers will only
//!   support a single connection (that is, they disconnect previously
//!   configured signals on repeat calls to the same function), but you can use
//!   [`gpio::OutputSignal::connect_to`] to connect multiple GPIOs to the same
//!   output signal.
//! - A GPIO input signal can be connected to any number of peripheral inputs.
//! - A GPIO output can be driven by only one peripheral output.

#[cfg(feature = "unstable")]
use crate::gpio::{Input, Output};
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
        FUNC_IN_SEL_OFFSET,
        GPIO_FUNCTION,
        INPUT_SIGNAL_MAX,
        OUTPUT_SIGNAL_MAX,
    },
    peripherals::GPIO,
    private::{self, Sealed},
};

/// The base of all peripheral signal.
///
/// All signals can be peripheral inputs, but not all output-like types should
/// be allowed to be passed as inputs. This trait bridges this gap by defining
/// the logic, but not declaring the signal to be an actual Input signal.
pub trait PeripheralSignal<'d>: Sealed {
    /// Connects the peripheral input to an input signal source.
    #[doc(hidden)] // Considered unstable
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal);
}

/// A signal that can be connected to a peripheral input.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralOutput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "InputConnection is unstable, but the trait needs to be public"
)]
pub trait PeripheralInput<'d>: Into<InputConnection<'d>> + PeripheralSignal<'d> {}

/// A signal that can be connected to a peripheral input and/or output.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralInput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "OutputConnection is unstable, but the trait needs to be public"
)]
pub trait PeripheralOutput<'d>: Into<OutputConnection<'d>> + PeripheralSignal<'d> {
    /// Connects the peripheral output to an output signal target.
    #[doc(hidden)] // Considered unstable
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal);

    /// Disconnects the peripheral output from an output signal target.
    ///
    /// This function clears the entry in the IO MUX that
    /// associates this output pin with a previously connected
    /// [signal](`gpio::OutputSignal`). Any other outputs connected to the
    /// peripheral remain intact.
    #[doc(hidden)] // Considered unstable
    fn disconnect_from_peripheral_output(&self);
}

// Pins
impl<'d, P> PeripheralSignal<'d> for P
where
    P: Pin + 'd,
{
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        let pin = unsafe { AnyPin::steal(self.number()) };
        InputSignal::new(pin).connect_input_to_peripheral(signal);
    }
}
impl<'d, P> PeripheralInput<'d> for P where P: InputPin + 'd {}

impl<'d, P> PeripheralOutput<'d> for P
where
    P: OutputPin + 'd,
{
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        let pin = unsafe { AnyPin::steal(self.number()) };
        OutputSignal::new(pin).connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self) {
        let pin = unsafe { AnyPin::steal(self.number()) };
        OutputSignal::new(pin).disconnect_from_peripheral_output();
    }
}

// Pin drivers
#[instability::unstable]
impl<'d> PeripheralSignal<'d> for Flex<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.pin.connect_input_to_peripheral(signal);
    }
}
#[instability::unstable]
impl<'d> PeripheralInput<'d> for Flex<'d> {}
#[instability::unstable]
impl<'d> PeripheralOutput<'d> for Flex<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.pin.connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self) {
        self.pin.disconnect_from_peripheral_output();
    }
}

#[instability::unstable]
impl<'d> PeripheralSignal<'d> for Input<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.pin.connect_input_to_peripheral(signal);
    }
}
#[instability::unstable]
impl<'d> PeripheralInput<'d> for Input<'d> {}

#[instability::unstable]
impl<'d> PeripheralSignal<'d> for Output<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.pin.connect_input_to_peripheral(signal);
    }
}
#[instability::unstable]
impl<'d> PeripheralOutput<'d> for Output<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.pin.connect_peripheral_to_output(signal);
    }
    fn disconnect_from_peripheral_output(&self) {
        self.pin.disconnect_from_peripheral_output();
    }
}

// Placeholders
impl PeripheralSignal<'_> for NoPin {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        // Arbitrary choice but we need to overwrite a previous signal input
        // association.
        Level::Low.connect_input_to_peripheral(signal);
    }
}
impl PeripheralInput<'_> for NoPin {}
impl PeripheralOutput<'_> for NoPin {
    fn connect_peripheral_to_output(&self, _: gpio::OutputSignal) {
        // A peripheral's outputs may be connected to any number of GPIOs.
        // Connecting to, and disconnecting from a NoPin is therefore a
        // no-op, as we are adding and removing nothing from that list of
        // connections.
    }
    fn disconnect_from_peripheral_output(&self) {
        // A peripheral's outputs may be connected to any number of GPIOs.
        // Connecting to, and disconnecting from a NoPin is therefore a
        // no-op, as we are adding and removing nothing from that list of
        // connections.
    }
}

impl PeripheralSignal<'_> for Level {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        let value = match self {
            Level::High => gpio::ONE_INPUT,
            Level::Low => gpio::ZERO_INPUT,
        };

        connect_input_signal(signal, value, false, true);
    }
}
impl PeripheralInput<'_> for Level {}
impl PeripheralOutput<'_> for Level {
    fn connect_peripheral_to_output(&self, _: gpio::OutputSignal) {
        // There is no such thing as a constant-high level peripheral output,
        // the implementation just exists for convenience.
    }
    fn disconnect_from_peripheral_output(&self) {
        // There is no such thing as a constant-high level peripheral output,
        // the implementation just exists for convenience.
    }
}

// Split signals
impl<'d> PeripheralSignal<'d> for InputSignal<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        // Since there can only be one input signal connected to a peripheral
        // at a time, this function will disconnect any previously
        // connected input signals.
        connect_pin_to_input_signal(&self.pin, signal, self.invert_input, self.force_gpio_matrix);
    }
}
impl<'d> PeripheralInput<'d> for InputSignal<'d> {}

impl<'d> PeripheralSignal<'d> for OutputSignal<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        connect_pin_to_input_signal(&self.pin, signal, self.invert_input, self.force_gpio_matrix);
    }
}
impl<'d> PeripheralOutput<'d> for OutputSignal<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        connect_peripheral_to_output(
            &self.pin,
            signal,
            self.invert_output,
            self.force_gpio_matrix,
            true,
            false,
        );
    }
    fn disconnect_from_peripheral_output(&self) {
        GPIO::regs()
            .func_out_sel_cfg(self.pin.number() as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(gpio::OutputSignal::GPIO as _) });
    }
}

// Type-erased signals
impl<'d> PeripheralSignal<'d> for InputConnection<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        match &self.0 {
            InputConnectionInner::Signal(pin) => pin.connect_input_to_peripheral(signal),
            InputConnectionInner::Constant(level) => level.connect_input_to_peripheral(signal),
        }
    }
}
impl<'d> PeripheralInput<'d> for InputConnection<'d> {}

impl<'d> PeripheralSignal<'d> for OutputConnection<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        match &self.0 {
            OutputConnectionInner::Signal(pin) => pin.connect_input_to_peripheral(signal),
            OutputConnectionInner::Constant(level) => level.connect_input_to_peripheral(signal),
        }
    }
}
impl<'d> PeripheralOutput<'d> for OutputConnection<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        match &self.0 {
            OutputConnectionInner::Signal(pin) => pin.connect_peripheral_to_output(signal),
            OutputConnectionInner::Constant(level) => level.connect_peripheral_to_output(signal),
        }
    }
    fn disconnect_from_peripheral_output(&self) {
        match &self.0 {
            OutputConnectionInner::Signal(pin) => pin.disconnect_from_peripheral_output(),
            OutputConnectionInner::Constant(level) => level.disconnect_from_peripheral_output(),
        }
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
    ///
    /// This function allows connecting a peripheral input to either a
    /// [`PeripheralInput`] or [`PeripheralOutput`] implementation.
    #[inline]
    #[instability::unstable]
    pub fn connect_to<'a>(self, pin: &impl PeripheralSignal<'a>) {
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
    #[instability::unstable]
    pub fn connect_to<'d>(self, pin: &impl PeripheralOutput<'d>) {
        pin.connect_peripheral_to_output(self);
    }

    /// Disconnects a peripheral output signal from a GPIO.
    #[inline]
    #[instability::unstable]
    pub fn disconnect_from<'d>(self, pin: &impl PeripheralOutput<'d>) {
        pin.disconnect_from_peripheral_output();
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
fn connect_input_signal(signal: gpio::InputSignal, input: u8, invert: bool, use_gpio_matrix: bool) {
    assert!(
        signal.can_use_gpio_matrix() || !use_gpio_matrix,
        "{:?} cannot be routed through the GPIO matrix",
        signal
    );
    // No need for a critical section, this is a write and not a modify operation.
    GPIO::regs()
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .write(|w| unsafe {
            w.sel().bit(use_gpio_matrix);
            w.in_inv_sel().bit(invert);
            w.in_sel().bits(input) // Connect to GPIO or constant level
        });
}

fn connect_pin_to_input_signal(
    pin: &AnyPin<'_>,
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
    pin: &AnyPin<'_>,
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

/// An input signal between a peripheral and a GPIO pin.
///
/// If the `InputSignal` was obtained from a pin driver such as
/// [`Input`][crate::gpio::Input::split], the GPIO driver will be responsible
/// for configuring the pin with the correct settings, peripheral drivers will
/// not be able to modify the pin settings.
///
/// Multiple input signals can be connected to one pin.
#[instability::unstable]
pub struct InputSignal<'d> {
    pin: AnyPin<'d>,
    /// If `true`, forces the GPIO matrix to be used for this signal.
    pub(crate) force_gpio_matrix: bool,
    /// Inverts the peripheral's input signal.
    pub invert_input: bool,
    /// If `true`, prevents peripheral drivers from modifying the pin settings.
    pub(crate) frozen: bool,
}

impl<'d, P> From<P> for InputSignal<'d>
where
    P: Pin + 'd,
{
    fn from(input: P) -> Self {
        InputSignal::new(input.degrade())
    }
}

impl<'d> From<Flex<'d>> for InputSignal<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.peripheral_input()
    }
}

#[instability::unstable]
impl<'d> From<Input<'d>> for InputSignal<'d> {
    fn from(pin: Input<'d>) -> Self {
        pin.pin.into()
    }
}

impl Sealed for InputSignal<'_> {}

impl Clone for InputSignal<'_> {
    fn clone(&self) -> Self {
        Self {
            pin: unsafe { self.pin.clone_unchecked() },
            force_gpio_matrix: self.force_gpio_matrix,
            invert_input: self.invert_input,
            frozen: self.frozen,
        }
    }
}

impl<'d> InputSignal<'d> {
    pub(crate) fn new(pin: AnyPin<'d>) -> Self {
        Self {
            pin,
            force_gpio_matrix: false,
            invert_input: false,
            frozen: false,
        }
    }

    /// Freezes the pin configuration.
    ///
    /// This will prevent the associated peripheral from modifying the pin
    /// settings.
    pub fn freeze(mut self) -> Self {
        self.frozen = true;
        self
    }

    /// Unfreezes the pin configuration.
    ///
    /// This will enable the associated peripheral to modify the pin settings
    /// again.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it allows peripherals to modify the pin
    /// configuration again. This can lead to undefined behavior if the pin
    /// is being configured by multiple peripherals at the same time. It can
    /// also lead to surprising behavior if the pin is passed to multiple
    /// peripherals that expect conflicting settings.
    pub unsafe fn unfreeze(&mut self) {
        self.frozen = false;
    }

    /// Returns the GPIO number of the underlying pin.
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Returns the current signal level.
    ///
    /// Note that this does not take [`Self::invert_input`] into account.
    pub fn level(&self) -> Level {
        self.is_input_high().into()
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// input signal.
    pub fn with_inverted_input(mut self, invert: bool) -> Self {
        self.invert_input = invert;
        self
    }

    delegate::delegate! {
        #[doc(hidden)]
        to self.pin {
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn is_input_high(&self) -> bool;
        }

        #[doc(hidden)]
        to match &self.pin {
            p if self.frozen => Frozen(p),
            p => p
        } {
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn set_input_enable(&self, _on: bool);
        }
    }
}

/// An (input and) output signal between a peripheral and a GPIO pin.
///
/// If the `OutputSignal` was obtained from a pin driver such as
/// [`Output`][crate::gpio::Output::split], the GPIO driver will be responsible
/// for configuring the pin with the correct settings, peripheral drivers will
/// not be able to modify the pin settings.
///
/// Note that connecting this to a peripheral input will enable the input stage
/// of the GPIO pin.
///
/// Multiple pins can be connected to one output signal.
#[instability::unstable]
pub struct OutputSignal<'d> {
    pin: AnyPin<'d>,
    /// If `true`, forces the GPIO matrix to be used for this signal.
    pub(crate) force_gpio_matrix: bool,

    /// Inverts the peripheral's output signal.
    pub invert_output: bool,

    /// Inverts the peripheral's input signal.
    pub invert_input: bool,

    /// If `true`, prevents peripheral drivers from modifying the pin settings.
    pub(crate) frozen: bool,
}

impl Sealed for OutputSignal<'_> {}

impl<'d, P> From<P> for OutputSignal<'d>
where
    P: OutputPin + 'd,
{
    fn from(output: P) -> Self {
        OutputSignal::new(output.degrade())
    }
}

impl<'d> From<Flex<'d>> for OutputSignal<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.into_peripheral_output()
    }
}

#[instability::unstable]
impl<'d> From<Output<'d>> for OutputSignal<'d> {
    fn from(pin: Output<'d>) -> Self {
        pin.pin.into()
    }
}

impl<'d> OutputSignal<'d> {
    pub(crate) fn new(pin: AnyPin<'d>) -> Self {
        Self {
            pin,
            force_gpio_matrix: false,
            invert_output: false,
            invert_input: false,
            frozen: false,
        }
    }

    /// Unfreezes the pin configuration.
    ///
    /// This will enable the associated peripheral to modify the pin settings
    /// again.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it allows peripherals to modify the pin
    /// configuration again. This can lead to undefined behavior if the pin
    /// is being configured by multiple peripherals at the same time.
    /// It can also lead to surprising behavior if the pin is passed to multiple
    /// peripherals that expect conflicting settings.
    pub unsafe fn unfreeze(&mut self) {
        self.frozen = false;
    }

    /// Returns the GPIO number of the underlying pin.
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// output signal.
    pub fn with_inverted_output(mut self, invert: bool) -> Self {
        self.invert_output = invert;
        self
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// input signal.
    pub fn with_inverted_input(mut self, invert: bool) -> Self {
        self.invert_input = invert;
        self
    }

    delegate::delegate! {
        #[instability::unstable]
        to self.pin {
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn is_input_high(&self) -> bool;

            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            pub fn is_set_high(&self) -> bool;
        }

        #[doc(hidden)]
        to match &self.pin {
            p if self.frozen => Frozen(p),
            p => p
        } {
            pub fn apply_output_config(&self, _config: &gpio::OutputConfig);
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn set_input_enable(&self, _on: bool);
            pub fn set_output_enable(&self, _on: bool);
            pub fn set_output_high(&self, _on: bool);
        }
    }
}

enum InputConnectionInner<'d> {
    /// InputSignal derived from a peripheral driver
    Signal(InputSignal<'d>),
    Constant(Level),
}

impl Clone for InputConnectionInner<'_> {
    fn clone(&self) -> Self {
        match self {
            Self::Signal(signal) => Self::Signal(signal.clone()),
            Self::Constant(level) => Self::Constant(*level),
        }
    }
}

/// A peripheral input signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
#[instability::unstable]
pub struct InputConnection<'d>(InputConnectionInner<'d>);
impl Sealed for InputConnection<'_> {}

impl<'d> From<InputSignal<'d>> for InputConnection<'d> {
    fn from(input: InputSignal<'d>) -> Self {
        Self(InputConnectionInner::Signal(input))
    }
}

impl From<Level> for InputConnection<'_> {
    fn from(level: Level) -> Self {
        Self(InputConnectionInner::Constant(level))
    }
}

impl From<NoPin> for InputConnection<'_> {
    fn from(_pin: NoPin) -> Self {
        Self::from(Level::Low)
    }
}

impl<'d, P> From<P> for InputConnection<'d>
where
    P: Pin + 'd,
{
    fn from(input: P) -> Self {
        Self::from(InputSignal::new(input.degrade()))
    }
}

impl<'d> From<Flex<'d>> for InputConnection<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.peripheral_input().into()
    }
}

#[instability::unstable]
impl<'d> From<Input<'d>> for InputConnection<'d> {
    fn from(pin: Input<'d>) -> Self {
        pin.pin.into()
    }
}

impl InputConnection<'_> {
    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.0 {
            InputConnectionInner::Signal(signal) => signal,
            InputConnectionInner::Constant(_) => NoOp,
        } {
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn set_input_enable(&self, on: bool);
        }
    }

    #[instability::unstable]
    #[doc(hidden)]
    pub fn is_input_high(&self) -> bool {
        match &self.0 {
            InputConnectionInner::Signal(signal) => signal.is_input_high(),
            InputConnectionInner::Constant(level) => *level == Level::High,
        }
    }
}

enum OutputConnectionInner<'d> {
    Signal(OutputSignal<'d>),
    Constant(Level),
}

/// A peripheral (input and) output signal connection.
///
/// This is mainly intended for internal use, but it can be used to connect
/// peripherals within the MCU without external hardware.
///
/// Note that connecting this to a peripheral input will enable the input stage
/// of the GPIO pin.
#[instability::unstable]
pub struct OutputConnection<'d>(OutputConnectionInner<'d>);
impl Sealed for OutputConnection<'_> {}

impl<'d> From<OutputSignal<'d>> for OutputConnection<'d> {
    fn from(signal: OutputSignal<'d>) -> Self {
        Self(OutputConnectionInner::Signal(signal))
    }
}

impl From<NoPin> for OutputConnection<'_> {
    fn from(_pin: NoPin) -> Self {
        Self::from(Level::Low)
    }
}

impl From<Level> for OutputConnection<'_> {
    fn from(level: Level) -> Self {
        Self(OutputConnectionInner::Constant(level))
    }
}

impl<'d, P> From<P> for OutputConnection<'d>
where
    P: OutputPin + 'd,
{
    fn from(output: P) -> Self {
        OutputConnection::from(OutputSignal::new(output.degrade()))
    }
}

impl<'d> From<Flex<'d>> for OutputConnection<'d> {
    fn from(pin: Flex<'d>) -> Self {
        pin.into_peripheral_output().into()
    }
}

#[instability::unstable]
impl<'d> From<Output<'d>> for OutputConnection<'d> {
    fn from(pin: Output<'d>) -> Self {
        pin.pin.into()
    }
}

impl OutputConnection<'_> {
    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.0 {
            OutputConnectionInner::Signal(signal) => signal,
            OutputConnectionInner::Constant(_) => NoOp,
        } {
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn apply_output_config(&self, _config: &gpio::OutputConfig);
            pub fn set_input_enable(&self, on: bool);
            pub fn set_output_enable(&self, on: bool);
            pub fn set_output_high(&self, on: bool);
        }
    }

    #[doc(hidden)]
    #[instability::unstable]
    pub fn is_set_high(&self) -> bool {
        match &self.0 {
            OutputConnectionInner::Signal(signal) => signal.is_set_high(),
            OutputConnectionInner::Constant(level) => *level == Level::High,
        }
    }

    #[doc(hidden)]
    #[instability::unstable]
    pub fn is_input_high(&self) -> bool {
        match &self.0 {
            OutputConnectionInner::Signal(signal) => signal.is_input_high(),
            OutputConnectionInner::Constant(level) => *level == Level::High,
        }
    }

    #[doc(hidden)]
    #[instability::unstable]
    pub(crate) fn connect_with_guard(self, signal: crate::gpio::OutputSignal) -> PinGuard {
        signal.connect_to(&self);
        match self.0 {
            OutputConnectionInner::Signal(pin) => PinGuard::new(pin.pin, signal),
            OutputConnectionInner::Constant(_) => PinGuard::new_unconnected(signal),
        }
    }
}

struct Frozen<'a>(&'a AnyPin<'a>);

impl Frozen<'_> {
    // Even though settings are frozen, the user probably wants the input stage to
    // be enabled. It's cheap and safe to do. The user can still disconnect the
    // peripheral from the outside world if they want with some creative
    // applicaiton of `Flex`.
    fn set_input_enable(&self, _on: bool) {
        self.0.set_input_enable(true);
    }
    fn set_output_enable(&self, _on: bool) {}
    fn set_output_high(&self, _on: bool) {}
    fn apply_input_config(&self, _config: &gpio::InputConfig) {}
    fn apply_output_config(&self, _config: &gpio::OutputConfig) {}
}

struct NoOp;

impl NoOp {
    fn set_input_enable(&self, _on: bool) {}
    fn set_output_enable(&self, _on: bool) {}
    fn set_output_high(&self, _on: bool) {}
    fn apply_input_config(&self, _config: &gpio::InputConfig) {}
    fn apply_output_config(&self, _config: &gpio::OutputConfig) {}

    fn input_signals(
        &self,
        _: private::Internal,
    ) -> &'static [(AlternateFunction, gpio::InputSignal)] {
        &[]
    }

    fn output_signals(
        &self,
        _: private::Internal,
    ) -> &'static [(AlternateFunction, gpio::OutputSignal)] {
        &[]
    }
}

/// ```rust,compile_fail
/// // Regression test for <https://github.com/esp-rs/esp-hal/issues/3313>
/// // This test case is expected to generate the following error:
/// // error[E0277]: the trait bound `Output<'_>: PeripheralInput<'_>` is not satisfied
/// //   --> src\gpio\interconnect.rs:977:5
/// //    |
/// // 31 |   function_expects_input(
/// //    |   ---------------------- required by a bound introduced by this call
/// // 32 | /     Output::new(peripherals.GPIO0,
/// // 33 | |     Level::Low,
/// // 34 | |     Default::default()),
/// //    | |_______________________^ the trait `InputPin` is not implemented for `Output<'_>`
/// // FIXME: due to <https://github.com/rust-lang/rust/issues/139924> this test may be ineffective.
/// //        It can be manually verified by changing it to `no_run` for a `run-doc-tests` run.
#[doc = crate::before_snippet!()]
/// use esp_hal::gpio::{Output, Level, interconnect::PeripheralInput};
///
/// fn function_expects_input<'d>(_: impl PeripheralInput<'d>) {}
///
/// function_expects_input(
///     Output::new(peripherals.GPIO0,
///     Level::Low,
///     Default::default()),
/// );
///
/// # Ok(())
/// # }
/// ```
fn _compile_tests() {}
