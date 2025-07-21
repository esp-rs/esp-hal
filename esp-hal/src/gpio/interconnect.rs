//! # Peripheral signal interconnect using the GPIO matrix.
//!
//! The GPIO matrix offers flexible connection options between GPIO pins and
//! peripherals. This module offers capabilities not covered by GPIO pin types
//! and drivers, like routing fixed logic levels to peripheral inputs, or
//! inverting input and output signals.
//!
//! > Note that routing a signal through the GPIO matrix adds some latency to
//! > the signal. This is not a problem for most peripherals, but it can be an
//! > issue for high-speed peripherals like SPI or I2S. `esp-hal` tries to
//! > bypass the GPIO matrix when possible (e.g. when the pin can be configured
//! > as a suitable Alternate Function for the peripheral signal, and other
//! > settings are compatible), but silently falls back to the GPIO matrix for
//! > flexibility.
#![doc = concat!("## Relation to the ", crate::trm_markdown_link!("iomuxgpio"))]
//! The GPIO drivers implement IO MUX and pin functionality (input/output
//! buffers, pull resistors, etc.). The GPIO matrix is represented by signals
//! and the [`PeripheralInput`] and [`PeripheralOutput`] traits. There is some
//! overlap between them: signal routing depends on what type is passed to a
//! peripheral driver's pin setter functions.
//!
//! ## Signals
//!
//! GPIO signals are represented by the [`InputSignal`] and [`OutputSignal`]
//! structs. Peripheral drivers accept [`PeripheralInput`] and
//! [`PeripheralOutput`] implementations which are implemented for anything that
//! can be converted into the signal types:
//! - GPIO pins and drivers
//! - A fixed logic [`Level`]
//! - [`NoPin`]
//!
//! Note that some of these exist for convenience only. `Level` is meaningful as
//! a peripheral input, but not as a peripheral output. `NoPin` is a placeholder
//! for when a peripheral driver does not require a pin, but the API requires
//! one. It is equivalent to [`Level::Low`].
//!
//! ### Splitting drivers into signals
//!
//! Each GPIO pin driver such as [`Input`], can be converted
//! into input or output signals. [`Flex`], which can be either input or output,
//! can be [`split`](Flex::split) into both signals at once. These signals can
//! then be individually connected to a peripheral input or output signal. This
//! allows for flexible routing of signals between peripherals and GPIO pins.
//!
//! Note that only configured GPIO drivers can be safely turned into signals.
//! This conversion freezes the pin configuration, otherwise it would be
//! possible for multiple peripheral drivers to configure the same GPIO pin at
//! the same time, which is undefined behavior.
//!
//! ### Splitting pins into signals
//!
//! GPIO pin types such as [`GPIO0`] or [`AnyPin`] can be **unsafely**
//! [split](AnyPin::split) into signals. In this case you need to carefully
//! ensure that only a single driver configures the split pin, by selectively
//! [freezing](`InputSignal::freeze`) the signals.
//!
//! For example, if you want to route GPIO3 to both a Pulse Counter
//! input and a [UART](crate::uart::Uart) RX line, you will need to make sure
//! one of the signals is frozen, otherwise the driver that is configured later
//! will overwrite the other driver's configuration. Configuring the signals on
//! multiple cores is undefined behaviour unless you ensure the configuration
//! does not happen at the same time.
//!
//! ### Using pins and signals
//!
//! A GPIO pin can be configured either with a GPIO driver such as [`Input`], or
//! by a peripheral driver using a pin assignment method such as
//! [`Spi::with_mosi`]. The peripheral drivers' preferences can be overridden by
//! passing a pin driver to the peripheral driver. When converting a driver to
//! signals, the underlying signals will be initially
//! [frozen](InputSignal::freeze) to support this use case.
//!
//! ## Inverting inputs and outputs
//!
//! The GPIO matrix allows for inverting the input and output signals. This can
//! be configured via [`InputSignal::with_input_inverter`] and
//! [`OutputSignal::with_input_inverter`]. The hardware is configured
//! accordingly when the signal is connected to a peripheral input or output.
//!
//! ## Connection rules
//!
//! Peripheral signals and GPIOs can be connected with the following
//! constraints:
//!
//! - A peripheral input signal must be driven by exactly one signal, which can be a GPIO input or a
//!   constant level.
//! - A peripheral output signal can be connected to any number of GPIOs. These GPIOs can be
//!   configured differently. The peripheral drivers will only support a single connection (that is,
//!   they disconnect previously configured signals on repeat calls to the same function), but you
//!   can use `esp_hal::gpio::OutputSignal::connect_to` (note that the type is currently hidden from
//!   the documentation) to connect multiple GPIOs to the same output signal.
//! - A GPIO input signal can be connected to any number of peripheral inputs.
//! - A GPIO output can be driven by only one peripheral output.
//!
//! [`GPIO0`]: crate::peripherals::GPIO0
//! [`Spi::with_mosi`]: crate::spi::master::Spi::with_mosi

#[cfg(feature = "unstable")]
use crate::gpio::{Input, Output};
use crate::{
    gpio::{
        self,
        AlternateFunction,
        AnyPin,
        Flex,
        InputPin,
        Level,
        NoPin,
        OutputPin,
        Pin,
        PinGuard,
    },
    peripherals::GPIO,
    private::{self, Sealed},
};

/// The base of all peripheral signals.
///
/// This trait represents a signal in the GPIO matrix. Signals are converted or
/// split from GPIO pins and can be connected to peripheral inputs and outputs.
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
    reason = "InputSignal is unstable, but the trait needs to be public"
)]
pub trait PeripheralInput<'d>: Into<InputSignal<'d>> + PeripheralSignal<'d> {}

/// A signal that can be connected to a peripheral input and/or output.
///
/// Peripheral drivers are encouraged to accept types that implement this and
/// [`PeripheralInput`] as arguments instead of pin types.
#[allow(
    private_bounds,
    reason = "OutputSignal is unstable, but the trait needs to be public"
)]
pub trait PeripheralOutput<'d>: Into<OutputSignal<'d>> + PeripheralSignal<'d> {
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
        Signal::Level(*self).connect_to_peripheral_input(signal, false, true);
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
        self.pin.connect_to_peripheral_input(
            signal,
            self.is_input_inverted(),
            self.is_gpio_matrix_forced(),
        );
    }
}
impl<'d> PeripheralInput<'d> for InputSignal<'d> {}

impl<'d> PeripheralSignal<'d> for OutputSignal<'d> {
    fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
        self.pin.connect_to_peripheral_input(
            signal,
            self.is_input_inverted(),
            self.is_gpio_matrix_forced(),
        );
    }
}
impl<'d> PeripheralOutput<'d> for OutputSignal<'d> {
    fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
        self.pin.connect_peripheral_to_output(
            signal,
            self.is_output_inverted(),
            self.is_gpio_matrix_forced(),
            true,
            false,
        );
    }
    fn disconnect_from_peripheral_output(&self) {
        self.pin.disconnect_from_peripheral_output();
    }
}

impl gpio::InputSignal {
    fn can_use_gpio_matrix(self) -> bool {
        self as usize <= property!("gpio.input_signal_max")
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
        self as usize <= property!("gpio.output_signal_max")
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

enum Signal<'d> {
    Pin(AnyPin<'d>),
    Level(Level),
}
impl Signal<'_> {
    fn gpio_number(&self) -> Option<u8> {
        match &self {
            Signal::Pin(pin) => Some(pin.number()),
            Signal::Level(_) => None,
        }
    }

    unsafe fn clone_unchecked(&self) -> Self {
        match self {
            Signal::Pin(pin) => Signal::Pin(unsafe { pin.clone_unchecked() }),
            Signal::Level(level) => Signal::Level(*level),
        }
    }

    fn is_set_high(&self) -> bool {
        match &self {
            Signal::Pin(signal) => signal.is_set_high(),
            Signal::Level(level) => *level == Level::High,
        }
    }

    fn is_input_high(&self) -> bool {
        match &self {
            Signal::Pin(signal) => signal.is_input_high(),
            Signal::Level(level) => *level == Level::High,
        }
    }

    fn connect_with_guard(self, signal: crate::gpio::OutputSignal) -> PinGuard {
        match self {
            Signal::Pin(pin) => PinGuard::new(pin, signal),
            Signal::Level(_) => PinGuard::new_unconnected(signal),
        }
    }

    fn connect_to_peripheral_input(
        &self,
        signal: gpio::InputSignal,
        is_inverted: bool,
        force_gpio: bool,
    ) {
        let use_gpio_matrix = match self {
            Signal::Pin(pin) => {
                let af = if is_inverted || force_gpio {
                    AlternateFunction::GPIO
                } else {
                    pin.input_signals(private::Internal)
                        .iter()
                        .find(|(_af, s)| *s == signal)
                        .map(|(af, _)| *af)
                        .unwrap_or(AlternateFunction::GPIO)
                };
                pin.disable_usb_pads();
                pin.set_alternate_function(af);
                af == AlternateFunction::GPIO
            }
            Signal::Level(_) => true,
        };

        let input = match self {
            Signal::Pin(pin) => pin.number(),
            Signal::Level(Level::Low) => property!("gpio.constant_0_input"),
            Signal::Level(Level::High) => property!("gpio.constant_1_input"),
        };

        assert!(
            signal.can_use_gpio_matrix() || !use_gpio_matrix,
            "{:?} cannot be routed through the GPIO matrix",
            signal
        );
        // No need for a critical section, this is a write and not a modify operation.
        let offset = property!("gpio.func_in_sel_offset");
        GPIO::regs()
            .func_in_sel_cfg(signal as usize - offset)
            .write(|w| unsafe {
                w.sel().bit(use_gpio_matrix);
                w.in_inv_sel().bit(is_inverted);
                // Connect to GPIO or constant level
                w.in_sel().bits(input)
            });
    }

    fn connect_peripheral_to_output(
        &self,
        signal: gpio::OutputSignal,
        is_inverted: bool,
        force_gpio: bool,
        peripheral_control_output_enable: bool,
        invert_output_enable: bool,
    ) {
        let Signal::Pin(pin) = self else {
            return;
        };
        let af = if is_inverted || force_gpio {
            AlternateFunction::GPIO
        } else {
            pin.output_signals(private::Internal)
                .iter()
                .find(|(_af, s)| *s == signal)
                .map(|(af, _)| *af)
                .unwrap_or(AlternateFunction::GPIO)
        };
        pin.disable_usb_pads();
        pin.set_alternate_function(af);

        let use_gpio_matrix = af == AlternateFunction::GPIO;

        assert!(
            signal.can_use_gpio_matrix() || !use_gpio_matrix,
            "{:?} cannot be routed through the GPIO matrix",
            signal
        );

        GPIO::regs()
            .func_out_sel_cfg(pin.number() as usize)
            .write(|w| unsafe {
                if use_gpio_matrix {
                    // Ignored if the signal is not routed through the GPIO matrix - alternate
                    // function selects peripheral signal directly.
                    w.out_sel().bits(signal as _);
                    w.inv_sel().bit(is_inverted);
                }
                w.oen_sel().bit(!peripheral_control_output_enable);
                w.oen_inv_sel().bit(invert_output_enable)
            });
    }

    fn disconnect_from_peripheral_output(&self) {
        let Some(number) = self.gpio_number() else {
            return;
        };
        GPIO::regs()
            .func_out_sel_cfg(number as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(gpio::OutputSignal::GPIO as _) });
    }
}

bitflags::bitflags! {
    #[derive(Clone, Copy)]
    struct InputFlags: u8 {
        const ForceGpioMatrix = 1 << 0;
        const Frozen          = 1 << 1;
        const InvertInput     = 1 << 2;
    }
}

/// An input signal between a peripheral and a GPIO pin.
///
/// If the `InputSignal` was obtained from a pin driver such as
/// [`Input`](crate::gpio::Input::split), the GPIO driver will be responsible
/// for configuring the pin with the correct settings, peripheral drivers will
/// not be able to modify the pin settings.
///
/// Multiple input signals can be connected to one pin.
#[instability::unstable]
pub struct InputSignal<'d> {
    pin: Signal<'d>,
    flags: InputFlags,
}

impl From<Level> for InputSignal<'_> {
    fn from(level: Level) -> Self {
        InputSignal::new_level(level)
    }
}

impl From<NoPin> for InputSignal<'_> {
    fn from(_pin: NoPin) -> Self {
        InputSignal::new_level(Level::Low)
    }
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
            flags: self.flags,
        }
    }
}

impl<'d> InputSignal<'d> {
    fn new_inner(inner: Signal<'d>) -> Self {
        Self {
            pin: inner,
            flags: InputFlags::empty(),
        }
    }

    pub(crate) fn new(pin: AnyPin<'d>) -> Self {
        Self::new_inner(Signal::Pin(pin))
    }

    pub(crate) fn new_level(level: Level) -> Self {
        Self::new_inner(Signal::Level(level))
    }

    /// Freezes the pin configuration.
    ///
    /// This will prevent peripheral drivers using this signal from modifying
    /// the pin settings.
    pub fn freeze(mut self) -> Self {
        self.flags.insert(InputFlags::Frozen);
        self
    }

    /// Unfreezes the pin configuration.
    ///
    /// This will enable peripheral drivers to modify the pin settings
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
        self.flags.remove(InputFlags::Frozen);
    }

    /// Returns the GPIO number of the underlying pin.
    ///
    /// Returns `None` if the signal is a constant level.
    pub fn gpio_number(&self) -> Option<u8> {
        self.pin.gpio_number()
    }

    /// Returns `true` if the input signal is high.
    ///
    /// Note that this does not take [`Self::with_input_inverter`] into account.
    pub fn is_input_high(&self) -> bool {
        self.pin.is_input_high()
    }

    /// Returns the current signal level.
    ///
    /// Note that this does not take [`Self::with_input_inverter`] into account.
    pub fn level(&self) -> Level {
        self.is_input_high().into()
    }

    /// Returns `true` if the input signal is configured to be inverted.
    ///
    /// Note that the hardware is not configured until the signal is actually
    /// connected to a peripheral.
    pub fn is_input_inverted(&self) -> bool {
        self.flags.contains(InputFlags::InvertInput)
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// input signal.
    pub fn with_input_inverter(mut self, invert: bool) -> Self {
        self.flags.set(InputFlags::InvertInput, invert);
        self
    }

    /// Consumes the signal and returns a new one that forces the GPIO matrix
    /// to be used.
    pub fn with_gpio_matrix_forced(mut self, force: bool) -> Self {
        self.flags.set(InputFlags::ForceGpioMatrix, force);
        self
    }

    /// Returns `true` if the input signal must be routed through the GPIO
    /// matrix.
    pub fn is_gpio_matrix_forced(&self) -> bool {
        self.flags.contains(InputFlags::ForceGpioMatrix)
    }

    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.pin {
            Signal::Pin(signal) => signal,
            Signal::Level(_) => NoOp,
        } {
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
        }
    }

    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.pin {
            Signal::Pin(_) if self.flags.contains(InputFlags::Frozen) => NoOp,
            Signal::Pin(signal) => signal,
            Signal::Level(_) => NoOp,
        } {
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn set_input_enable(&self, on: bool);
        }
    }
}

bitflags::bitflags! {
    #[derive(Clone, Copy)]
    struct OutputFlags: u8 {
        const ForceGpioMatrix = 1 << 0;
        const Frozen          = 1 << 1;
        const InvertInput     = 1 << 2;
        const InvertOutput    = 1 << 3;
    }
}

/// An (input and) output signal between a peripheral and a GPIO pin.
///
/// If the `OutputSignal` was obtained from a pin driver such as
/// [`Output`](crate::gpio::Output::split), the GPIO driver will be responsible
/// for configuring the pin with the correct settings, peripheral drivers will
/// not be able to modify the pin settings.
///
/// Note that connecting this to a peripheral input will enable the input stage
/// of the GPIO pin.
///
/// Multiple pins can be connected to one output signal.
#[instability::unstable]
pub struct OutputSignal<'d> {
    pin: Signal<'d>,
    flags: OutputFlags,
}

impl Sealed for OutputSignal<'_> {}

impl From<Level> for OutputSignal<'_> {
    fn from(level: Level) -> Self {
        OutputSignal::new_level(level)
    }
}

impl From<NoPin> for OutputSignal<'_> {
    fn from(_pin: NoPin) -> Self {
        OutputSignal::new_level(Level::Low)
    }
}

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
    fn new_inner(inner: Signal<'d>) -> Self {
        Self {
            pin: inner,
            flags: OutputFlags::empty(),
        }
    }

    pub(crate) fn new(pin: AnyPin<'d>) -> Self {
        Self::new_inner(Signal::Pin(pin))
    }

    pub(crate) fn new_level(level: Level) -> Self {
        Self::new_inner(Signal::Level(level))
    }

    /// Freezes the pin configuration.
    ///
    /// This will prevent peripheral drivers using this signal from
    /// modifying the pin settings.
    pub fn freeze(mut self) -> Self {
        self.flags.insert(OutputFlags::Frozen);
        self
    }

    /// Unfreezes the pin configuration.
    ///
    /// This will enable peripheral drivers to modify the pin settings
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
        self.flags.remove(OutputFlags::Frozen);
    }

    /// Returns the GPIO number of the underlying pin.
    ///
    /// Returns `None` if the signal is a constant level.
    pub fn gpio_number(&self) -> Option<u8> {
        self.pin.gpio_number()
    }

    /// Returns `true` if the input signal is configured to be inverted.
    ///
    /// Note that the hardware is not configured until the signal is actually
    /// connected to a peripheral.
    pub fn is_input_inverted(&self) -> bool {
        self.flags.contains(OutputFlags::InvertInput)
    }

    /// Returns `true` if the output signal is configured to be inverted.
    ///
    /// Note that the hardware is not configured until the signal is actually
    /// connected to a peripheral.
    pub fn is_output_inverted(&self) -> bool {
        self.flags.contains(OutputFlags::InvertOutput)
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// output signal.
    pub fn with_output_inverter(mut self, invert: bool) -> Self {
        self.flags.set(OutputFlags::InvertOutput, invert);
        self
    }

    /// Consumes the signal and returns a new one that inverts the peripheral's
    /// input signal.
    pub fn with_input_inverter(mut self, invert: bool) -> Self {
        self.flags.set(OutputFlags::InvertInput, invert);
        self
    }

    /// Consumes the signal and returns a new one that forces the GPIO matrix
    /// to be used.
    pub fn with_gpio_matrix_forced(mut self, force: bool) -> Self {
        self.flags.set(OutputFlags::ForceGpioMatrix, force);
        self
    }

    /// Returns `true` if the input signal must be routed through the GPIO
    /// matrix.
    pub fn is_gpio_matrix_forced(&self) -> bool {
        self.flags.contains(OutputFlags::ForceGpioMatrix)
    }

    /// Returns `true` if the input signal is high.
    ///
    /// Note that this does not take [`Self::with_input_inverter`] into account.
    pub fn is_input_high(&self) -> bool {
        self.pin.is_input_high()
    }

    /// Returns `true` if the output signal is set high.
    ///
    /// Note that this does not take [`Self::with_output_inverter`] into
    /// account.
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    #[doc(hidden)]
    #[instability::unstable]
    pub(crate) fn connect_with_guard(self, signal: crate::gpio::OutputSignal) -> PinGuard {
        signal.connect_to(&self);
        self.pin.connect_with_guard(signal)
    }

    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.pin {
            Signal::Pin(signal) => signal,
            Signal::Level(_) => NoOp,
        } {
            pub fn input_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::InputSignal)];
            pub fn output_signals(&self, _internal: private::Internal) -> &'static [(AlternateFunction, gpio::OutputSignal)];
        }
    }

    delegate::delegate! {
        #[instability::unstable]
        #[doc(hidden)]
        to match &self.pin {
            Signal::Pin(_) if self.flags.contains(OutputFlags::Frozen) => NoOp,
            Signal::Pin(pin) => pin,
            Signal::Level(_) => NoOp,
        } {
            pub fn apply_input_config(&self, _config: &gpio::InputConfig);
            pub fn apply_output_config(&self, _config: &gpio::OutputConfig);
            pub fn set_input_enable(&self, on: bool);
            pub fn set_output_enable(&self, on: bool);
            pub fn set_output_high(&self, on: bool);
        }
    }
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

#[procmacros::doc_replace]
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
/// # {before_snippet}
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
/// # {after_snippet}
/// ```
fn _compile_tests() {}
