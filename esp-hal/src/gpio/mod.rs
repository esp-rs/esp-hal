#![cfg_attr(docsrs, procmacros::doc_replace(
    "etm_availability" => {
        cfg(etm_driver_supported) => "The GPIO pins also provide tasks and events via the ETM interconnect system. For more information, see the [etm] module."
    }
))]
//! # General Purpose Input/Output (GPIO)
//!
//! ## Overview
//!
//! Each pin can be used as a general-purpose I/O, or be connected to one or
//! more internal peripheral signals.
//! # {etm_availability}
//! ## Working with pins
//!
//! After initializing the HAL, you can access the individual pins using the
//! [`crate::Peripherals`] struct. These pins can then be used as general
//! purpose digital IO using pin drivers, or they can be passed to peripherals
//! (such as SPI, UART, I2C, etc.), or can be [`split`]
//! into peripheral signals for advanced use.
//!
//! Pin drivers can be created using [`Flex::new`], [`Input::new`] and
//! [`Output::new`]
//!
//! Output pins can be configured to either push-pull or open-drain (active low)
//! mode, with configurable drive strength and pull-up/pull-down resistors.
//!
//! Each pin is a different type initially. Internally, `esp-hal` will erase
//! their types automatically, but they can also be converted into [`AnyPin`]
//! manually by calling [`Pin::degrade`]
//!
//! The [`Io`] struct can also be used to configure the interrupt handler for
//! GPIO interrupts. For more information, see the
//! [`InterruptConfigurable::set_interrupt_handler`](crate::interrupt::InterruptConfigurable::set_interrupt_handler)
//!
//! This driver also implements pin-related traits from [embedded-hal] and
//! [Wait](embedded_hal_async::digital::Wait) trait from [embedded-hal-async].
//!
//! ## GPIO interconnect
//!
//! Sometimes you may want to connect peripherals together without using
//! external hardware. The [`interconnect`] module provides tools to achieve
//! this using GPIO pins.
//!
//! To obtain peripheral signals, use the [`split`] method to split a
//! pin into an input and output signal. Alternatively, you may use
//! [`Flex::split`], [`Flex::into_peripheral_output`],
//! [`Flex::peripheral_input`], and similar methods to split a pin driver into
//! an input and output signal. You can then pass these signals to the
//! peripheral drivers similar to how you would pass a pin.
//!
//! [embedded-hal]: embedded_hal
//! [embedded-hal-async]: embedded_hal_async
//! [`split`]: crate::peripherals::GPIO0::split

crate::unstable_module! {
    pub mod interconnect;

    #[cfg(etm_driver_supported)]
    pub mod etm;

    #[cfg(lp_io_driver_supported)]
    pub mod lp_io;

    #[cfg(dedicated_gpio_driver_supported)]
    pub mod dedicated;
}
use interconnect::PeripheralOutput;

mod asynch;
mod embedded_hal_impls;
pub(crate) mod interrupt;
mod low_level;
use interrupt::*;
pub use low_level::GpioBank;
use low_level::{gpio_intr_enable, is_int_enabled, set_int_enable};

mod placeholder;

#[cfg(sleep_driver_supported)]
pub(crate) mod wakeup;
use core::fmt::Display;

use esp_sync::RawMutex;
pub use placeholder::NoPin;
#[cfg(sleep_driver_supported)]
#[instability::unstable]
pub use wakeup::WakeupConfig;

use crate::{
    asynch::AtomicWaker,
    interrupt::{InterruptHandler, Priority},
    peripherals::{GPIO, IO_MUX},
    private::{self, Sealed},
};

define_io_mux_signals!();

pub(crate) static GPIO_LOCK: RawMutex = RawMutex::new();

/// Represents a pin-peripheral connection that, when dropped, disconnects the
/// peripheral from the pin.
///
/// This only needs to be applied to output signals, as it is not possible to connect multiple
/// inputs to the same peripheral signal.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PinGuard {
    pin: u8,
}

impl crate::private::Sealed for PinGuard {}

impl PinGuard {
    // This must only be used with a pin currently configured for output, and the PinGuard must be
    // dropped before the pin can be reconfigured (e.g. for input).
    fn new(pin: AnyPin<'_>) -> Self {
        Self { pin: pin.number() }
    }

    pub(crate) const fn new_unconnected() -> Self {
        Self { pin: u8::MAX }
    }

    #[allow(unused)]
    pub(crate) fn pin_number(&self) -> Option<u8> {
        if self.pin == u8::MAX {
            None
        } else {
            Some(self.pin)
        }
    }
}

impl Drop for PinGuard {
    fn drop(&mut self) {
        if self.pin != u8::MAX {
            let pin = unsafe { AnyPin::steal(self.pin) };
            pin.disconnect_from_peripheral_output();
        }
    }
}

/// Event used to trigger interrupts.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum Event {
    /// Interrupts trigger on rising pin edge.
    RisingEdge  = 1,
    /// Interrupts trigger on falling pin edge.
    FallingEdge = 2,
    /// Interrupts trigger on either rising or falling pin edges.
    AnyEdge     = 3,
    /// Interrupts trigger on low level
    LowLevel    = 4,
    /// Interrupts trigger on high level
    HighLevel   = 5,
}

/// Digital input or output level.
///
/// `Level` can be used to control a GPIO output, and it can act as a peripheral
/// signal and be connected to peripheral inputs and outputs.
///
/// When connected to a peripheral
/// input, the peripheral will read the corresponding level from that signal.
///
/// When connected to a peripheral output, the level will be ignored.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// Low
    Low,
    /// High
    High,
}

impl Sealed for Level {}

impl core::ops::Not for Level {
    type Output = Self;

    fn not(self) -> Self {
        match self {
            Self::Low => Self::High,
            Self::High => Self::Low,
        }
    }
}

impl Level {
    /// Creates a new [`Level`] from [`bool`].
    ///
    /// Like `<Level as From<bool>>::from(val)`, but `const`.
    pub(crate) const fn const_from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }

    /// Converts a [`Level`] to [`bool`].
    ///
    /// Like `<bool as From<Level>>::from(self)`, but `const`.
    pub(crate) const fn const_into(self) -> bool {
        match self {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        Self::const_from(val)
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        level.const_into()
    }
}

/// Errors that can occur when configuring a pin to be a wakeup source.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum WakeConfigError {
    /// The pad has no low-power path. It cannot wake the chip while the high-performance GPIO
    /// peripheral is powered down.
    NoLowPowerPath,
}

impl Display for WakeConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WakeConfigError::NoLowPowerPath => {
                write!(f, "The pad is not a low-power pad")
            }
        }
    }
}

impl core::error::Error for WakeConfigError {}

/// Pull setting for a GPIO.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    /// No pull
    None,
    /// Pull up
    Up,
    /// Pull down
    Down,
}

/// Drive strength (values are approximates)
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    /// Drive strength of approximately 5mA.
    _5mA  = 0,
    /// Drive strength of approximately 10mA.
    _10mA = 1,
    /// Drive strength of approximately 20mA.
    _20mA = 2,
    /// Drive strength of approximately 40mA.
    _40mA = 3,
}

/// Alternate functions
///
/// GPIO pins can be configured for various functions, such as GPIO
/// or being directly connected to a peripheral's signal like UART, SPI, etc.
/// The `AlternateFunction` enum allows selecting one of several functions that
/// a pin can perform, rather than using it as a general-purpose input or
/// output.
///
/// The different variants correspond to different functionality depending on
/// the chip and the specific pin. For more information, refer to the chip's
#[doc(hidden)]
#[doc = crate::trm_markdown_link!("iomuxgpio")]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlternateFunction {
    /// Alternate function 0.
    _0 = 0,
    /// Alternate function 1.
    _1 = 1,
    /// Alternate function 2.
    _2 = 2,
    /// Alternate function 3.
    _3 = 3,
    /// Alternate function 4.
    _4 = 4,
    /// Alternate function 5.
    _5 = 5,
}

impl AlternateFunction {
    pub(crate) const GPIO: Self = match Self::const_try_from(property!("gpio.gpio_function")) {
        Ok(func) => func,
        Err(_) => ::core::panic!("Invalid GPIO function"),
    };

    const fn const_try_from(value: usize) -> Result<Self, ()> {
        match value {
            0 => Ok(Self::_0),
            1 => Ok(Self::_1),
            2 => Ok(Self::_2),
            3 => Ok(Self::_3),
            4 => Ok(Self::_4),
            5 => Ok(Self::_5),
            _ => Err(()),
        }
    }
}

impl TryFrom<usize> for AlternateFunction {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Self::const_try_from(value)
    }
}

/// Trait implemented by the pins that the low-power domain can reach.
///
/// The low-power domain has its own numbers for these pads, and the low-power registers take such a
/// number. Only some chips give a pad the same number in both domains, so do not mix the two
/// numbers. Give [`Self::lp_number`] to the low-power registers, and [`Pin::number`] to the digital
/// registers.
#[instability::unstable]
#[cfg(lp_io_driver_supported)]
pub trait LpPin: Pin {
    /// LP number of the pin
    fn lp_number(&self) -> u8;
}

/// Common trait implemented by pins
pub trait Pin: Sealed {
    /// GPIO number
    fn number(&self) -> u8;

    #[procmacros::doc_replace]
    /// Type-erases this pin into an [`AnyPin`].
    ///
    /// Converts pin singletons (`GPIO0<'_>`, …), which are all different types,
    /// into the same type. Useful for creating arrays of pins, or avoiding
    /// generics.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     delay::Delay,
    ///     gpio::{AnyPin, Level, Output, OutputConfig, Pin},
    /// };
    ///
    /// fn toggle_pins(pins: [AnyPin; 2], delay: &mut Delay) {
    ///     let [red, blue] = pins;
    ///     let mut red = Output::new(red, Level::High, OutputConfig::default());
    ///     let mut blue = Output::new(blue, Level::Low, OutputConfig::default());
    ///
    ///     loop {
    ///         red.toggle();
    ///         blue.toggle();
    ///         delay.delay_millis(500);
    ///     }
    /// }
    ///
    /// let pins: [AnyPin; 2] = [peripherals.GPIO5.degrade(), peripherals.GPIO4.degrade()];
    ///
    /// let mut delay = Delay::new();
    /// toggle_pins(pins, &mut delay);
    /// # {after_snippet}
    /// ```
    fn degrade<'d>(self) -> AnyPin<'d>
    where
        Self: Sized + 'd,
    {
        unsafe { AnyPin::steal(self.number()) }
    }

    #[doc(hidden)]
    fn output_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, OutputSignal)];

    #[doc(hidden)]
    fn input_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, InputSignal)];
}

/// Trait implemented by pins which can be used as inputs.
pub trait InputPin: Pin {
    #[doc(hidden)]
    fn waker(&self) -> &'static AtomicWaker;
}

/// Trait implemented by pins which can be used as outputs.
pub trait OutputPin: Pin {}

/// Trait implemented by pins which can be used as analog pins.
#[instability::unstable]
pub trait AnalogPin: Pin {
    /// Configures the pin for analog operation.
    #[doc(hidden)]
    fn set_analog(&self, _: private::Internal);
}

/// Trait implemented by pins which can be used as Touchpad pins.
#[cfg(touch_driver_supported)]
#[instability::unstable]
pub trait TouchPin: Pin {
    /// Configures the pin for analog operation.
    #[doc(hidden)]
    fn set_touch(&self, _: private::Internal);

    /// Reads the pin's touch measurement register.
    #[doc(hidden)]
    fn touch_measurement(&self, _: private::Internal) -> u16;

    /// Maps the pin nr to the touch pad nr.
    #[doc(hidden)]
    fn touch_nr(&self, _: private::Internal) -> u8;

    /// Sets a pins touch threshold for interrupts.
    #[doc(hidden)]
    fn set_threshold(&self, threshold: u16, _: private::Internal);
}

/// Any GPIO pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyPin<'lt> {
    pub(crate) pin: u8,
    pub(crate) _lifetime: core::marker::PhantomData<&'lt mut ()>,
}

/// General Purpose Input/Output driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Io<'d> {
    _io_mux: IO_MUX<'d>,
}

impl<'d> Io<'d> {
    /// Initializes the I/O driver.
    #[instability::unstable]
    pub fn new(_io_mux: IO_MUX<'d>) -> Self {
        Io { _io_mux }
    }

    #[doc = cfg_select!{
        any(single_core, esp32s3) => "Sets the interrupt priority and enables GPIO interrupts.",
        _ => "Sets the interrupt priority and enables GPIO interrupts on all cores.",
    }]
    #[instability::unstable]
    pub fn set_interrupt_priority(&self, prio: Priority) {
        low_level::set_interrupt_priority(prio);
    }

    #[doc = cfg_select!{
        any(single_core, esp32s3) => "Registers an interrupt handler for all GPIO pins.",
        _ => "Registers an interrupt handler for all GPIO pins. Enables interrupts on all cores.",
    }]
    #[doc = ""]
    /// When using interrupt handlers registered by this method, or by defining a
    /// `#[no_mangle] unsafe extern "C" fn GPIO()` function, the interrupt status
    /// register and the interrupt enable setting for the GPIO pin are **not**
    /// cleared automatically. Based on the use case, do one of the following:
    ///
    /// - Disabling the interrupt enable setting for the GPIO pin lets an event be handled once per
    ///   call to [`listen()`]. Using this method, the [`is_interrupt_set()`] method returns `true`
    ///   if the interrupt is set even after the handler has finished running.
    /// - Clearing the interrupt status register lets an event be handled repeatedly after
    ///   [`listen()`] is called. Using this method, [`is_interrupt_set()`] returns `false` after
    ///   the handler has finished running.
    ///
    /// [`listen()`]: Input::listen
    /// [`is_interrupt_set()`]: Input::is_interrupt_set
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        USER_INTERRUPT_HANDLER.store(handler.handler().callback());
        low_level::enable_interrupt(InterruptHandler::new(
            user_gpio_interrupt_handler,
            handler.priority(),
        ));
    }
}

impl crate::private::Sealed for Io<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Io<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

/// Drives the GPIO async API from a user-installed raw GPIO interrupt handler.
///
/// Entry point for code that bypasses esp-hal's GPIO ISR dispatch (typically
/// by defining a custom `#[unsafe(no_mangle)] unsafe extern "C" fn GPIO()`, or
/// by registering a handler via [`crate::interrupt::bind_handler`]) and needs
/// the GPIO async API to keep working.
///
/// # Safety
///
/// Must be called from a GPIO interrupt context.
#[instability::unstable]
pub unsafe fn handle_gpio_interrupt() {
    unsafe { interrupt::handle_gpio_interrupt_impl() }
}

/// Completes any in-flight async wait on a single GPIO pin.
///
/// Per-pin counterpart of [`handle_gpio_interrupt`]
///
/// # Safety
///
/// Must be called from a GPIO interrupt context.
#[instability::unstable]
pub unsafe fn wake_pin(pin: u8) {
    unsafe { interrupt::wake_pin_impl(pin) }
}

for_each_analog_function! {
    (($_ch:ident, ADCn_CHm, $_n:literal, $_m:literal), $gpio:ident) => {
        #[instability::unstable]
        impl $crate::gpio::AnalogPin for crate::peripherals::$gpio<'_> {
            fn set_analog(&self, _: private::Internal) {
                cfg_select! {
                    xtensa => { self.set_analog_impl(); }
                    riscv => {
                        // Just configure as a floating GPIO.
                        io_mux_reg(self.number()).modify(|_, w| unsafe {
                            w.mcu_sel().bits(AlternateFunction::GPIO as u8);
                            w.fun_ie().clear_bit();
                            w.fun_wpu().clear_bit();
                            w.fun_wpd().clear_bit()
                        });

                        GPIO::regs()
                            .enable_w1tc()
                            .write(|w| unsafe { w.bits(1 << self.number()) });
                    }
                }
            }
        }
    };
}

/// The drive mode of the output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveMode {
    /// Push-pull output.
    ///
    /// The driver actively sets the output voltage level for both high and low
    /// logical [`Level`]s
    PushPull,

    /// Open drain output.
    ///
    /// The driver actively pulls the output voltage level low for the low
    /// logical [`Level`], but leaves the high level floating, which is then
    /// determined by external hardware, or internal pull-up/pull-down
    /// resistors.
    #[cfg_attr(
        feature = "unstable",
        doc = "\n\nEnable the input related functionality by using [Output::into_flex] and enabling input via [Flex::set_input_enable]"
    )]
    OpenDrain,
}

/// Output pin configuration.
///
/// Configures the drive mode, drive strength, and pull direction of an output
/// pin. By default, the configuration is set to:
/// - Drive mode: [`DriveMode::PushPull`]
/// - Drive strength: [`DriveStrength::_20mA`]
/// - Pull direction: [`Pull::None`] (no pull resistors connected)
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct OutputConfig {
    /// Output drive mode.
    drive_mode: DriveMode,

    /// Pin drive strength.
    drive_strength: DriveStrength,

    /// Pin pull direction.
    pull: Pull,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            drive_mode: DriveMode::PushPull,
            drive_strength: DriveStrength::_20mA,
            pull: Pull::None,
        }
    }
}

/// Push-pull digital output.
///
/// This driver configures the GPIO pin to be an output driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl private::Sealed for Output<'_> {}
impl private::Sealed for &mut Output<'_> {}

impl<'d> Output<'d> {
    #[procmacros::doc_replace]
    /// Creates a new GPIO output driver.
    ///
    /// The `initial_level` parameter sets the initial output level of the pin.
    /// The `config` parameter sets the drive mode, drive strength, and pull
    /// direction of the pin.
    ///
    /// # Examples
    ///
    /// The following example configures `GPIO5` to pulse a LED once. The
    /// example assumes that the LED is connected such that it is on when
    /// the pin is low.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     delay::Delay,
    ///     gpio::{Level, Output, OutputConfig},
    /// };
    ///
    /// fn blink_once(led: &mut Output<'_>, delay: &mut Delay) {
    ///     led.set_low();
    ///     delay.delay_millis(500);
    ///     led.set_high();
    /// }
    ///
    /// let config = OutputConfig::default();
    /// let mut led = Output::new(peripherals.GPIO5, Level::High, config);
    /// let mut delay = Delay::new();
    ///
    /// blink_once(&mut led, &mut delay);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn new(pin: impl OutputPin + 'd, initial_level: Level, config: OutputConfig) -> Self {
        // Set up the pin
        let mut this = Self {
            pin: Flex::new(pin),
        };
        this.set_level(initial_level);
        this.apply_config(&config);
        this.pin.pin.set_output_enable(true);

        this
    }

    #[procmacros::doc_replace]
    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    ///
    /// The returned signal is [frozen](interconnect::OutputSignal::freeze).
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// # let config = OutputConfig::default();
    /// let pin1_gpio = Output::new(peripherals.GPIO1, Level::High, config);
    /// let output = pin1_gpio.into_peripheral_output();
    /// # {after_snippet}
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal<'d> {
        self.pin.into_peripheral_output()
    }

    #[procmacros::doc_replace]
    /// Changes the configuration.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
    /// let mut pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    ///
    /// pin.apply_config(&OutputConfig::default().with_drive_mode(DriveMode::OpenDrain));
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn apply_config(&mut self, config: &OutputConfig) {
        self.pin.apply_output_config(config)
    }
    #[procmacros::doc_replace]
    /// Sets the output as high.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let mut pin = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    /// pin.set_high();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn set_high(&mut self) {
        self.set_level(Level::High)
    }

    #[procmacros::doc_replace]
    /// Sets the output as low.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let mut pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// pin.set_low();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn set_low(&mut self) {
        self.set_level(Level::Low)
    }

    #[procmacros::doc_replace]
    /// Sets the output level.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let mut pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// pin.set_level(Level::Low);
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    #[procmacros::doc_replace]
    /// Returns whether the pin is set to high level.
    ///
    /// Reads back the value set using `set_level`, `set_high` or `set_low`. Does
    /// not need the input stage to be enabled.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// let is_high = pin.is_set_high();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.output_level() == Level::High
    }

    #[procmacros::doc_replace]
    /// Returns whether the pin is set to low level.
    ///
    /// Reads back the value set using `set_level`, `set_high` or `set_low`. Does
    /// not need the input stage to be enabled.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// let is_low = pin.is_set_low();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.output_level() == Level::Low
    }

    #[procmacros::doc_replace]
    /// Returns which level the pin is set to.
    ///
    /// Reads back the value set using `set_level`, `set_high` or `set_low`. Does
    /// not need the input stage to be enabled.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// let level = pin.output_level();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }

    #[procmacros::doc_replace]
    /// Toggles the pin output.
    ///
    /// If the pin was previously set to high, it will be set to low, and vice
    /// versa.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// let mut pin = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    /// pin.toggle();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// Takes or releases the hold of the pad.
    ///
    /// A held pad keeps its level, its function and its resistors, and it ignores this driver.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn set_pad_hold(&mut self, enable: bool) {
        self.pin.set_pad_hold(enable);
    }

    /// Returns whether something holds the pad.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn is_pad_held(&self) -> bool {
        self.pin.is_pad_held()
    }

    /// Converts the pin driver into a [`Flex`] driver.
    #[inline]
    #[instability::unstable]
    pub fn into_flex(self) -> Flex<'d> {
        self.pin
    }
}

/// Configures the clock edge used in one of the two input synchronization stages to synchronize the
/// GPIO input signal to the system clock, reducing metastability.
///
/// See [`InputConfig::with_input_sync_stage1`] and
/// [`InputConfig::with_input_sync_stage2`]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[cfg(gpio_has_input_sync)]
pub enum InputSync {
    /// Stage is disabled; the signal passes through without latching (hardware
    /// default).
    #[default]
    Disabled     = 0,
    /// Stage latches on the falling edge of the GPIO clock.
    NegativeEdge = 1,
    /// Stage latches on the rising edge of the GPIO clock.
    PositiveEdge = 2,
}

/// Input pin configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct InputConfig {
    /// Initial pull of the pin.
    pull: Pull,

    /// First input synchronization stage.
    ///
    /// Default value: [`InputSync::Disabled`]
    #[cfg(gpio_has_input_sync)]
    #[builder_lite(unstable)]
    input_sync_stage1: InputSync,

    /// Second input synchronization stage.
    ///
    /// Default value: [`InputSync::Disabled`]
    #[cfg(gpio_has_input_sync)]
    #[builder_lite(unstable)]
    input_sync_stage2: InputSync,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self {
            pull: Pull::None,
            #[cfg(gpio_has_input_sync)]
            input_sync_stage1: Default::default(),
            #[cfg(gpio_has_input_sync)]
            input_sync_stage2: Default::default(),
        }
    }
}

/// Digital input.
///
/// This driver configures the GPIO pin to be an input. Input drivers read the
/// voltage of their pins and convert it to a logical [`Level`]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl private::Sealed for Input<'_> {}
impl private::Sealed for &mut Input<'_> {}

impl<'d> Input<'d> {
    #[procmacros::doc_replace]
    /// Creates a new GPIO input.
    ///
    /// The `pull` parameter configures internal pull-up or pull-down
    /// resistors.
    ///
    /// # Examples
    ///
    /// The following example configures `GPIO5` to read a button press. The
    /// example assumes that the button is connected such that the pin is low
    /// when the button is pressed.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     delay::Delay,
    ///     gpio::{Input, InputConfig, Level, Pull},
    /// };
    ///
    /// fn print_when_pressed(button: &mut Input<'_>, delay: &mut Delay) {
    ///     let mut was_pressed = false;
    ///     loop {
    ///         let is_pressed = button.is_low();
    ///         if is_pressed && !was_pressed {
    ///             println!("Button pressed!");
    ///         }
    ///         was_pressed = is_pressed;
    ///         delay.delay_millis(100);
    ///     }
    /// }
    ///
    /// let config = InputConfig::default().with_pull(Pull::Up);
    /// let mut button = Input::new(peripherals.GPIO5, config);
    /// let mut delay = Delay::new();
    ///
    /// print_when_pressed(&mut button, &mut delay);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn new(pin: impl InputPin + 'd, config: InputConfig) -> Self {
        let mut pin = Flex::new(pin);

        pin.set_output_enable(false);
        pin.set_input_enable(true);
        pin.apply_input_config(&config);

        Self { pin }
    }

    #[procmacros::doc_replace]
    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    ///
    /// The returned signal is [frozen](interconnect::InputSignal::freeze).
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::gpio::{Input, InputConfig, Pull};
    /// let config = InputConfig::default().with_pull(Pull::Up);
    /// let pin1_gpio = Input::new(peripherals.GPIO1, config);
    /// let pin1 = pin1_gpio.peripheral_input();
    /// #
    /// # {after_snippet}
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal<'d> {
        self.pin.peripheral_input()
    }

    #[procmacros::doc_replace]
    /// Returns whether the pin input level is high.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Input, InputConfig};
    /// let pin = Input::new(peripherals.GPIO5, InputConfig::default());
    /// let is_high = pin.is_high();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    #[procmacros::doc_replace]
    /// Returns whether the pin input level is low.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Input, InputConfig};
    /// let pin = Input::new(peripherals.GPIO5, InputConfig::default());
    /// let is_low = pin.is_low();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    #[procmacros::doc_replace]
    /// Returns the current pin input level.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Input, InputConfig, Level};
    /// let pin = Input::new(peripherals.GPIO5, InputConfig::default());
    /// let level = pin.level();
    ///
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn level(&self) -> Level {
        self.pin.level()
    }

    #[procmacros::doc_replace]
    /// Changes the configuration.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Input, InputConfig, Level, Pull};
    /// let mut pin = Input::new(peripherals.GPIO5, InputConfig::default());
    /// pin.apply_config(&InputConfig::default().with_pull(Pull::Up));
    ///
    /// # {after_snippet}
    /// ```
    pub fn apply_config(&mut self, config: &InputConfig) {
        self.pin.apply_input_config(config)
    }

    #[procmacros::doc_replace]
    /// Listens for interrupts.
    ///
    /// The interrupts will be handled by the handler set using
    /// [`Io::set_interrupt_handler`]. All GPIO pins share the same
    /// interrupt handler.
    ///
    /// [`Event::LowLevel`] and [`Event::HighLevel`] are fired continuously when
    /// the pin is low or high, respectively. A custom interrupt handler is
    /// required to stop listening for these events; otherwise the program can
    /// remain in a loop as long as the pin reads the corresponding level.
    ///
    /// A listening pin also ends a light sleep. On most chips, sleep entry must give the trigger as
    /// a level, so an edge trigger becomes the level at the end of the edge. A rising edge becomes
    /// a high level, a falling edge becomes a low level, and any edge becomes the level that
    /// the pin is not at when the sleep starts. A pin that listens for a rising edge on a line
    /// that is already high therefore ends each light sleep immediately, and without an
    /// interrupt, because no edge occurred. Automatic light sleep then makes no sleep at all.
    ///
    /// # Examples
    ///
    /// ### Print something when a button is pressed.
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{Event, Input, InputConfig, Io, Pull};
    ///
    /// let mut io = Io::new(peripherals.IO_MUX);
    /// io.set_interrupt_handler(handler);
    ///
    /// // Set up the input and store it in the static variable.
    /// // This example uses a push button that is high when not
    /// // pressed and low when pressed.
    /// let config = InputConfig::default().with_pull(Pull::Up);
    /// let mut button = Input::new(peripherals.GPIO5, config);
    ///
    /// critical_section::with(|cs| {
    ///     // Here we are listening for a low level to demonstrate
    ///     // that you need to stop listening for level interrupts,
    ///     // but usually you'd probably use `FallingEdge`
    ///     button.listen(Event::LowLevel);
    ///     BUTTON.borrow_ref_mut(cs).replace(button);
    /// });
    /// # {after_snippet}
    ///
    /// // Outside of your `main` function:
    ///
    /// use core::cell::RefCell;
    ///
    /// use critical_section::Mutex;
    /// use esp_hal::gpio::Input;
    ///
    /// // You will need to store the `Input` object in a static variable so
    /// // that the interrupt handler can access it.
    /// static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
    ///
    /// #[esp_hal::handler]
    /// fn handler() {
    ///     critical_section::with(|cs| {
    ///         let mut button = BUTTON.borrow_ref_mut(cs);
    ///         let Some(button) = button.as_mut() else {
    ///             // Some other interrupt has occurred
    ///             // before the button was set up.
    ///             return;
    ///         };
    ///
    ///         if button.is_interrupt_set() {
    ///             print!("Button pressed");
    ///
    ///             // If you want to stop listening for interrupts, you need to
    ///             // call `unlisten` here. If you comment this line, the
    ///             // interrupt will fire continuously while the button
    ///             // is pressed.
    ///             button.unlisten();
    ///         }
    ///     });
    /// }
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }

    /// Stops listening for interrupts.
    #[inline]
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        self.pin.unlisten();
    }

    /// Clears the interrupt status bit for this Pin.
    #[inline]
    #[instability::unstable]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt();
    }

    /// Returns whether the interrupt status bit for this Pin is set.
    #[inline]
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.is_interrupt_set()
    }

    /// Configures whether the pin can wake the chip from sleep.
    ///
    /// [`Flex::apply_wakeup_config`] describes the configuration, and the conditions that wake the
    /// chip.
    ///
    /// # Errors
    ///
    /// [`WakeConfigError::NoLowPowerPath`] when the configuration requests the low-power path
    /// for a pad that has no such path.
    #[cfg(sleep_driver_supported)]
    #[instability::unstable]
    #[inline]
    pub fn apply_wakeup_config(&mut self, config: &WakeupConfig) -> Result<(), WakeConfigError> {
        self.pin.apply_wakeup_config(config)
    }

    /// Returns whether this pin ended the most recent sleep.
    ///
    /// See [`Flex::caused_wakeup`]
    #[cfg(sleep_driver_supported)]
    #[instability::unstable]
    #[inline]
    pub fn caused_wakeup(&self) -> bool {
        self.pin.caused_wakeup()
    }

    /// Takes or releases the hold of the pad.
    ///
    /// A held pad keeps its level, its function and its resistors, and it ignores this driver.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn set_pad_hold(&mut self, enable: bool) {
        self.pin.set_pad_hold(enable);
    }

    /// Returns whether something holds the pad.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn is_pad_held(&self) -> bool {
        self.pin.is_pad_held()
    }

    /// Converts the pin driver into a [`Flex`] driver.
    #[inline]
    #[instability::unstable]
    pub fn into_flex(self) -> Flex<'d> {
        self.pin
    }
}

/// Flexible pin driver.
///
/// This pin driver can act as either input, or output, or both at the same
/// time. The input and output are (not counting the shared pull direction)
/// separately configurable, and they have independent enable states.
///
/// Enabling the input stage does not change the output stage, and vice versa.
/// Disabling the input or output stages do not forget their configuration.
/// Disabling the output stage will not change the output level, but it will
/// disable the driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Flex<'d> {
    pin: AnyPin<'d>,
}

impl private::Sealed for Flex<'_> {}
impl private::Sealed for &mut Flex<'_> {}

impl<'d> Flex<'d> {
    /// Creates flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    #[instability::unstable]
    pub fn new(pin: impl Pin + 'd) -> Self {
        let pin = pin.degrade();

        // Before each use, reset the GPIO to a known state.
        pin.init_gpio();

        Self { pin }
    }

    // Input functions

    /// Applies the given input configuration to the pin.
    ///
    /// Does not set the pin as input (does not enable the input buffer). The pull
    /// direction is common between the input and output configuration.
    #[inline]
    #[instability::unstable]
    pub fn apply_input_config(&mut self, config: &InputConfig) {
        self.pin.apply_input_config(config);
    }

    /// Enables or disables the GPIO pin input buffer.
    #[inline]
    #[instability::unstable]
    pub fn set_input_enable(&mut self, enable_input: bool) {
        self.pin.set_input_enable(enable_input);
    }

    /// Takes or releases the hold of the pad.
    ///
    /// A held pad keeps its level, its function and its resistors, and it ignores this driver.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn set_pad_hold(&mut self, enable: bool) {
        self.pin.set_pad_hold(enable);
    }

    /// Returns whether something holds the pad.
    #[inline]
    #[instability::unstable]
    #[cfg(lp_io_driver_supported)]
    pub fn is_pad_held(&self) -> bool {
        self.pin.is_pad_held()
    }

    /// Returns whether the pin input level is high.
    #[inline]
    #[instability::unstable]
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    /// Returns whether the pin input level is low.
    #[inline]
    #[instability::unstable]
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    /// Returns the current pin input level.
    #[inline]
    #[instability::unstable]
    pub fn level(&self) -> Level {
        self.pin.is_input_high().into()
    }

    /// Listens for interrupts.
    ///
    /// See [`Input::listen`] for more information and an example
    #[inline]
    #[instability::unstable]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }

    /// Stops listening for interrupts.
    #[inline]
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        GPIO_LOCK.lock(|| {
            set_int_enable(self.pin.number(), Some(0), 0, false);
        });
    }

    fn unlisten_and_clear(&mut self) {
        GPIO_LOCK.lock(|| {
            set_int_enable(self.pin.number(), Some(0), 0, false);
            self.clear_interrupt();
        });
    }

    /// Returns whether the pin is listening for interrupts.
    #[inline]
    #[instability::unstable]
    pub fn is_listening(&self) -> bool {
        is_int_enabled(self.pin.number())
    }

    /// Clears the interrupt status bit for this Pin.
    #[inline]
    #[instability::unstable]
    pub fn clear_interrupt(&mut self) {
        self.pin
            .bank()
            .write_interrupt_status_clear(self.pin.mask());
    }

    /// Returns whether the interrupt status bit for this Pin is set.
    #[inline]
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.bank().read_interrupt_status() & self.pin.mask() != 0
    }

    /// Configures whether the pin can wake the chip from sleep.
    ///
    /// The configuration selects the hardware paths that the pin can use. The wake condition is the
    /// interrupt trigger, so **a pin that does not listen is not a wakeup source**. A pin that
    /// listens already wakes the chip from light sleep through the digital path. See
    /// [`WakeupConfig`]
    ///
    /// The configuration stays after the driver is dropped, because a pad that wakes the chip from
    /// deep sleep must continue to do so while no driver owns it. To remove the configuration, call
    /// this function again with [`WakeupConfig::default()`]
    ///
    /// # Errors
    ///
    /// [`WakeConfigError::NoLowPowerPath`] when the configuration requests the low-power path
    /// for a pad that has no such path.
    #[cfg(sleep_driver_supported)]
    #[inline]
    #[instability::unstable]
    pub fn apply_wakeup_config(&mut self, config: &WakeupConfig) -> Result<(), WakeConfigError> {
        wakeup::apply_config(&self.pin, config)
    }

    /// Returns whether this pin ended the most recent sleep.
    ///
    /// More than one pin can end a sleep, so this function can return `true` for several pins. It
    /// returns `false` if the chip did not wake from a sleep, and `false` for a pin that ended an
    /// earlier sleep only.
    ///
    /// esp-hal reads the result while the sleep ends, so a later clear of the interrupt of the pin
    /// does not change it. One case depends on the interrupt status: a pin that ends a light sleep
    /// without [`WakeupConfig::low_power_path`]. The interrupt handler of that pin clears the
    /// status, so the pin reports `false` if the handler runs before the sleep call returns
    /// This can only occur if interrupts were enabled during the sleep.
    #[cfg(sleep_driver_supported)]
    #[inline]
    #[instability::unstable]
    pub fn caused_wakeup(&self) -> bool {
        wakeup::caused_wakeup(&self.pin)
    }

    // Output functions

    /// Applies the given output configuration to the pin.
    ///
    /// Does not set the pin to output (does not enable the output driver). The
    /// pull direction is common between the input and output configuration.
    #[inline]
    #[instability::unstable]
    pub fn apply_output_config(&mut self, config: &OutputConfig) {
        self.pin.apply_output_config(config);
    }

    /// Enables or disables the GPIO pin output driver.
    ///
    /// The output level will be set to the last value. Use [`Self::set_high`],
    /// [`Self::set_low`] or [`Self::set_level`] to set the output level before
    /// enabling the output.
    ///
    /// Does not disable the input buffer.
    #[inline]
    #[instability::unstable]
    pub fn set_output_enable(&mut self, enable_output: bool) {
        self.pin.set_output_enable(enable_output);
    }

    /// Sets the output as high.
    #[inline]
    #[instability::unstable]
    pub fn set_high(&mut self) {
        self.set_level(Level::High)
    }

    /// Sets the output as low.
    #[inline]
    #[instability::unstable]
    pub fn set_low(&mut self) {
        self.set_level(Level::Low)
    }

    /// Sets the output level.
    #[inline]
    #[instability::unstable]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into());
    }

    /// Is the output pin set as high?
    #[inline]
    #[instability::unstable]
    pub fn is_set_high(&self) -> bool {
        self.output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    #[instability::unstable]
    pub fn is_set_low(&self) -> bool {
        self.output_level() == Level::Low
    }

    /// What level output is set to.
    #[inline]
    #[instability::unstable]
    pub fn output_level(&self) -> Level {
        self.pin.is_set_high().into()
    }

    /// Toggles pin output.
    #[inline]
    #[instability::unstable]
    pub fn toggle(&mut self) {
        let level = self.output_level();
        self.set_level(!level);
    }

    // Other/common functions

    #[procmacros::doc_replace]
    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    ///
    /// The returned signal is [frozen](interconnect::InputSignal::freeze).
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::Flex;
    /// let pin1_gpio = Flex::new(peripherals.GPIO1);
    /// // Can be passed as an input.
    /// let pin1 = pin1_gpio.peripheral_input();
    /// // You can keep using the Flex, as well as connect the pin to a
    /// // peripheral input.
    /// # {after_snippet}
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal<'d> {
        self.pin.set_input_enable(true);
        unsafe {
            // Safety: the signal is frozen by this function.
            self.pin.clone_unchecked().split_no_init().0.freeze()
        }
    }

    #[procmacros::doc_replace]
    /// Splits the pin into an input and output signal pair.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    ///
    /// The returned signals are [frozen](interconnect::InputSignal::freeze).
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::Flex;
    /// let pin1 = Flex::new(peripherals.GPIO1);
    /// let (input, output) = pin1.split();
    /// # {after_snippet}
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn split(
        self,
    ) -> (
        interconnect::InputSignal<'d>,
        interconnect::OutputSignal<'d>,
    ) {
        let input = self.peripheral_input();
        let output = self.into_peripheral_output();

        (input, output)
    }

    /// Splits the pin into an [Input] and an [Output] driver pair.
    ///
    /// The returned input signal is [frozen](interconnect::InputSignal::freeze). The
    /// pin driver is free to change settings.
    ///
    /// Lets an input-output pin be configured, then keeps working with the output
    /// half. Mainly intended for testing, to drive a peripheral from a signal
    /// generated by software.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the pins are not being configured via their
    /// `apply_config` functions in the same time in multiple places. The pin
    /// drivers must not be turned back into `Flex`, unless one of the
    /// drivers is dropped first.
    // TODO is this enough? Register-wise config is the only non-atomic operation, but is it
    // actually safe to have two drivers on the same pin, otherwise? Perhaps it would be better
    // to implement ehal traits for signals?
    #[inline]
    #[instability::unstable]
    pub unsafe fn split_into_drivers(self) -> (Input<'d>, Output<'d>) {
        self.pin.set_input_enable(true);
        let input = Input {
            pin: Flex {
                pin: unsafe { self.pin.clone_unchecked() },
            },
        };
        let output = Output { pin: self };

        (input, output)
    }

    #[procmacros::doc_replace]
    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    ///
    /// The returned signal is [frozen](interconnect::OutputSignal::freeze).
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::Flex;
    /// let pin1_gpio = Flex::new(peripherals.GPIO1);
    /// // Can be passed as an output.
    /// let pin1 = pin1_gpio.into_peripheral_output();
    /// # {after_snippet}
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal<'d> {
        unsafe {
            // Safety: the signals are frozen by this function.
            self.pin.split_no_init().1.freeze()
        }
    }
}

impl private::Sealed for AnyPin<'_> {}

impl<'lt> AnyPin<'lt> {
    fn bank(&self) -> GpioBank {
        low_level::bank(self.number())
    }

    pub(crate) fn disable_usb_pads(&self) {
        #[cfg(soc_has_usb_device)]
        {
            /// Workaround to make D+ and D- work when the pin is assigned to
            /// the `USB_SERIAL_JTAG` peripheral by default
            fn disable_usb_pads(_gpionum: u8) {
                crate::peripherals::USB_DEVICE::regs()
                    .conf0()
                    .modify(|_, w| {
                        w.usb_pad_enable().clear_bit();
                        w.dm_pullup().clear_bit();
                        w.dm_pulldown().clear_bit();
                        w.dp_pullup().clear_bit();
                        w.dp_pulldown().clear_bit()
                    });
            }

            #[cfg(esp32p4)]
            fn disable_usb_fs_pads(_gpionum: u8) {
                crate::peripherals::USB_WRAP::regs()
                    .otg_conf()
                    .modify(|_, w| {
                        w.pad_pull_override().set_bit();
                        w.dm_pullup().clear_bit();
                        w.dm_pulldown().clear_bit();
                        w.dp_pullup().clear_bit();
                        w.dp_pulldown().clear_bit()
                    });
            }

            #[cfg(soc_has_usb_fs)]
            macro_rules! disable_usb_fs_pads {
                ($gpio:ident) => {
                    if self.number() == crate::peripherals::$gpio::NUMBER {
                        cfg_select! {
                            esp32p4 => {
                                disable_usb_fs_pads(crate::peripherals::$gpio::NUMBER);
                            }
                            _ => {
                                disable_usb_pads(crate::peripherals::$gpio::NUMBER);
                            }
                        }
                    }
                };
            }

            for_each_analog_function! {
                (USJ_DM, $gpio:ident) => {
                    if self.number() == crate::peripherals::$gpio::NUMBER {
                        disable_usb_pads(crate::peripherals::$gpio::NUMBER);
                    }
                };
                (USJ_DP, $gpio:ident) => {
                    if self.number() == crate::peripherals::$gpio::NUMBER {
                        disable_usb_pads(crate::peripherals::$gpio::NUMBER);
                    }
                };
                (USB_FS_DM, $gpio:ident) => { disable_usb_fs_pads!($gpio) };
                (USB_FS_DP, $gpio:ident) => { disable_usb_fs_pads!($gpio) };
            }
        }
    }

    /// Takes or releases the hold of the pad.
    ///
    /// A held pad keeps its level, its function and its resistors, and it ignores its driver. Sleep
    /// takes the hold of a wakeup pad, because a deep sleep powers the circuit that drives the pad
    /// down.
    ///
    /// A pad that the low-power registers reach keeps its hold in the low-power domain, and every
    /// other pad has a bit in a register of the digital pads.
    #[cfg(lp_io_driver_supported)]
    pub(crate) fn set_pad_hold(&self, enable: bool) {
        GPIO_LOCK.lock(|| {
            if let Some(lp) = lp_io::lp_number(self.number()) {
                lp_io::low_level::pad_hold(lp, enable);
            } else {
                lp_io::low_level::digital_pad_hold(self.number(), enable);
            }
        })
    }

    /// Returns whether something holds the pad.
    #[cfg(lp_io_driver_supported)]
    pub(crate) fn is_pad_held(&self) -> bool {
        // No lock necessary, as this pin cannot be written
        // while this function is being called.
        if let Some(lp) = lp_io::lp_number(self.number()) {
            lp_io::low_level::is_pad_held(lp)
        } else {
            lp_io::low_level::is_digital_pad_held(self.number())
        }
    }

    #[inline]
    /// Resets the GPIO to a known state.
    ///
    /// Must be called before using the GPIO pin:
    /// - Before converting it into signals
    /// - Before using it as an input or output
    pub(crate) fn init_gpio(&self) {
        self.set_output_enable(false);
        self.disable_usb_pads();

        // A held pad ignores every configuration that follows the hold, and only a power-on reset
        // releases the hold by itself. Without this, a pad that an earlier program held keeps its
        // level for the rest of the life of the chip.
        #[cfg(lp_io_driver_supported)]
        self.set_pad_hold(false);

        #[cfg(lp_io_driver_supported)]
        for_each_lp_function! {
            (($_signal:ident, LP_GPIOn, $lp_pin:literal), $gpio:ident, $af:ident, $_lp_in:tt $_lp_out:tt) => {
                if self.number() == crate::peripherals::$gpio::NUMBER {
                    lp_io::low_level::set_config($lp_pin, false, false, lp_io::LpFunction::$af);
                }
            };
        }

        GPIO::regs()
            .func_out_sel_cfg(self.number() as usize)
            .write(|w| unsafe { w.out_sel().bits(OutputSignal::GPIO as _) });

        // Use RMW to not overwrite sleep configuration
        io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(AlternateFunction::GPIO as u8);
            w.fun_ie().clear_bit();
            w.slp_sel().clear_bit()
        });
    }

    #[procmacros::doc_replace]
    /// Splits the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// Creating an input signal enables the pin's input buffer.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{AnyPin, Pin};
    /// let pin1 = peripherals.GPIO1.degrade();
    /// let (input, output) = unsafe { pin1.split() };
    /// # {after_snippet}
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if the pin is not an output pin.
    ///
    /// # Safety
    ///
    /// The caller must ensure that peripheral drivers do not configure the same
    /// GPIO at the same time in multiple places. This includes clones of the
    /// `InputSignal` struct, as well as the `OutputSignal` struct
    #[inline]
    #[instability::unstable]
    pub unsafe fn split(
        self,
    ) -> (
        interconnect::InputSignal<'lt>,
        interconnect::OutputSignal<'lt>,
    ) {
        assert!(self.is_output());

        // Before each use, reset the GPIO to a known state.
        self.init_gpio();
        self.set_input_enable(true);

        let (input, output) = unsafe { self.split_no_init() };

        // We don't know if the input signal(s) will support bypassing the GPIO matrix.
        // Since the bypass option is common between input and output halves of
        // a single GPIO, we can't assume anything about the output, either.
        let output = output.with_gpio_matrix_forced(true);

        (input, output)
    }

    /// Converts the pin into an input signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// Creating an input signal enables the pin's input buffer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that peripheral drivers do not configure the same
    /// GPIO at the same time in multiple places. This includes clones of the
    /// `InputSignal` struct
    #[inline]
    #[instability::unstable]
    pub unsafe fn into_input_signal(self) -> interconnect::InputSignal<'lt> {
        // Before each use, reset the GPIO to a known state.
        self.init_gpio();
        self.set_input_enable(true);

        let (input, _) = unsafe { self.split_no_init() };

        input
    }

    /// Converts the pin into an output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// # Panics
    ///
    /// Panics if the pin is not an output pin.
    #[inline]
    #[instability::unstable]
    pub fn into_output_signal(self) -> interconnect::OutputSignal<'lt> {
        assert!(self.is_output());

        // Before each use, reset the GPIO to a known state.
        self.init_gpio();

        // AnyPin is used as output only, we can allow bypassing the GPIO matrix.
        let (_, output) = unsafe { self.split_no_init() };

        output
    }

    unsafe fn split_no_init(
        self,
    ) -> (
        interconnect::InputSignal<'lt>,
        interconnect::OutputSignal<'lt>,
    ) {
        let input = interconnect::InputSignal::new(unsafe { self.clone_unchecked() });
        let output = interconnect::OutputSignal::new(self);

        // Since InputSignal can be cloned, we have no way of knowing how many signals
        // end up being configured, and in what order. If multiple signals are
        // passed to peripherals, and one of them would allow GPIO alternate
        // function configurations, it would mean that the GPIO MCU_SEL bit's
        // final value would depend on the order of operations.
        let input = input.with_gpio_matrix_forced(true);

        (input, output)
    }

    #[inline]
    pub(crate) fn set_alternate_function(&self, alternate: AlternateFunction) {
        io_mux_reg(self.number()).modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
    }

    // /// Enable/disable sleep-mode
    // #[inline]
    // fn sleep_mode(&mut self, on: bool, _: private::Internal) {
    //     io_mux_reg(self.number()).modify(|_, w| w.slp_sel().bit(on));
    // }

    /// Enables or disables the GPIO pin output buffer.
    #[inline]
    pub(crate) fn set_output_enable(&self, enable: bool) {
        assert!(self.is_output() || !enable);
        self.bank().write_out_en(self.mask(), enable);
    }

    /// Enables input for the pin.
    #[inline]
    pub(crate) fn set_input_enable(&self, on: bool) {
        io_mux_reg(self.number()).modify(|_, w| w.fun_ie().bit(on));
    }

    #[inline]
    pub(crate) fn apply_input_config(&self, config: &InputConfig) {
        let pull_up = config.pull == Pull::Up;
        let pull_down = config.pull == Pull::Down;

        low_level::prepare_pin_pull(self, pull_up, pull_down);

        io_mux_reg(self.number()).modify(|_, w| {
            w.fun_wpd().bit(pull_down);
            w.fun_wpu().bit(pull_up)
        });

        #[cfg(gpio_has_input_sync)]
        self.with_gpio_lock(|| {
            GPIO::regs()
                .pin(self.number() as usize)
                .modify(|_, w| unsafe {
                    w.sync1_bypass().bits(config.input_sync_stage1 as u8);
                    w.sync2_bypass().bits(config.input_sync_stage2 as u8)
                });
        });
    }

    fn clear_interrupt(&self) {
        self.bank().write_interrupt_status_clear(self.mask());
    }

    fn with_gpio_lock<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        // If the pin is listening, we need to take a critical section to prevent racing
        // with the interrupt handler.
        if is_int_enabled(self.number()) {
            GPIO_LOCK.lock(f)
        } else {
            f()
        }
    }

    /// Starts to listen for `event`, which also makes the pin a light-sleep wakeup source.
    ///
    /// One register write sets the interrupt enable and the wakeup enable of the pad. A pin that
    /// cannot end a light sleep also cannot deliver its interrupt. A pin that listens therefore
    /// always wakes the chip through the digital path, and needs no configuration for that.
    fn listen(&self, event: Event) {
        self.with_gpio_lock(|| {
            // Clear the interrupt status bit for this Pin, just in case the user forgot.
            // Since we disabled the interrupt in the handler, it's not possible to
            // trigger a new interrupt before we re-enable it here.
            self.clear_interrupt();

            set_int_enable(
                self.number(),
                Some(gpio_intr_enable(true)),
                event as u8,
                true,
            );
        });

        // The mask bit tells sleep entry to look at the pads.
        #[cfg(sleep_driver_supported)]
        wakeup::enable();
    }

    #[inline]
    fn apply_output_config(&self, config: &OutputConfig) {
        let pull_up = config.pull == Pull::Up;
        let pull_down = config.pull == Pull::Down;

        low_level::prepare_pin_pull(self, pull_up, pull_down);

        io_mux_reg(self.number()).modify(|_, w| {
            unsafe { w.fun_drv().bits(config.drive_strength as u8) };
            w.fun_wpu().bit(pull_up);
            w.fun_wpd().bit(pull_down);
            w
        });

        self.with_gpio_lock(|| {
            GPIO::regs().pin(self.number() as usize).modify(|_, w| {
                w.pad_driver()
                    .bit(config.drive_mode == DriveMode::OpenDrain)
            });
        });
    }

    #[inline]
    fn mask(&self) -> u32 {
        1 << (self.number() % 32)
    }

    /// The current state of the input.
    #[inline]
    pub(crate) fn is_input_high(&self) -> bool {
        self.bank().read_input() & self.mask() != 0
    }

    /// Sets the pin's level to high or low.
    #[inline]
    pub(crate) fn set_output_high(&self, high: bool) {
        self.bank().write_output(self.mask(), high);
    }

    /// Is the output set to high.
    #[inline]
    pub(crate) fn is_set_high(&self) -> bool {
        self.bank().read_output() & self.mask() != 0
    }
}

impl Pin for AnyPin<'_> {
    #[inline(always)]
    fn number(&self) -> u8 {
        self.pin
    }

    fn output_signals(
        &self,
        private: private::Internal,
    ) -> &'static [(AlternateFunction, OutputSignal)] {
        for_each_gpio! {
            (all $( ($n:literal, $gpio:ident $in_afs:tt $out_afs:tt ($input:tt [$($is_output:ident)?]) ) ),* ) => {
                match self.number() {
                    $($(
                        $n => {
                            crate::ignore!($is_output);
                            let inner = unsafe { crate::peripherals::$gpio::steal() };
                            return Pin::output_signals(&inner, private);
                        }
                    )?)*
                    other => panic!("Pin {} is not an OutputPin", other)
                }
            };
        }
    }

    fn input_signals(
        &self,
        private: private::Internal,
    ) -> &'static [(AlternateFunction, InputSignal)] {
        for_each_gpio! {
            (all $( ($n:literal, $gpio:ident $in_afs:tt $out_afs:tt ([$($is_input:ident)?] $output:tt) ) ),* ) => {
                match self.number() {
                    $($(
                        $n => {
                            crate::ignore!($is_input);
                            let inner = unsafe { crate::peripherals::$gpio::steal() };
                            return Pin::input_signals(&inner, private);
                        }
                    )?)*
                    other => panic!("Pin {} is not an InputPin", other)
                }
            };
        }
    }
}

impl InputPin for AnyPin<'_> {
    fn waker(&self) -> &'static AtomicWaker {
        for_each_gpio! {
            (all $( ($n:literal, $gpio:ident $in_afs:tt $out_afs:tt ([$($is_input:ident)?] $output:tt) ) ),* ) => {
                match self.number() {
                    $($(
                        $n => {
                            crate::ignore!($is_input);
                            let inner = unsafe { crate::peripherals::$gpio::steal() };
                            return InputPin::waker(&inner);
                        }
                    )?)*
                    other => panic!("Pin {} is not an InputPin", other)
                }
            };
        }
    }
}
impl OutputPin for AnyPin<'_> {}

for_each_gpio! {
    ($n:literal, $gpio:ident $($_rest:tt)*) => {
        impl<'lt> TryFrom<AnyPin<'lt>> for crate::peripherals::$gpio<'lt> {
            type Error = AnyPin<'lt>;

            fn try_from(any_pin: AnyPin<'lt>) -> Result<Self, Self::Error> {
                if any_pin.number() == $n {
                    Ok(unsafe { Self::steal() })
                } else {
                    Err(any_pin)
                }
            }
        }
    };
}

impl AnyPin<'_> {
    #[procmacros::doc_replace]
    /// Attempts to downcast the pin into the underlying GPIO instance.
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::{
    ///     gpio::AnyPin,
    ///     peripherals::{GPIO2, GPIO4},
    /// };
    ///
    /// let any_pin2 = AnyPin::from(peripherals.GPIO2);
    /// let any_pin3 = AnyPin::from(peripherals.GPIO3);
    ///
    /// let gpio2 = any_pin2
    ///     .downcast::<GPIO2>()
    ///     .expect("This downcast succeeds because AnyPin was created from GPIO2");
    /// let gpio4 = any_pin3
    ///     .downcast::<GPIO4>()
    ///     .expect_err("This AnyPin was created from GPIO3, it cannot be downcast to GPIO4");
    /// #
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn downcast<P: Pin>(self) -> Result<P, Self>
    where
        Self: TryInto<P, Error = Self>,
    {
        self.try_into()
    }

    #[procmacros::doc_replace]
    /// Conjure a new GPIO pin out of thin air.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::gpio::AnyPin;
    /// let pin = unsafe { AnyPin::steal(1) };
    /// #
    /// # {after_snippet}
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if the pin with the given number does not exist.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a pin is in use at one time.
    pub unsafe fn steal(pin: u8) -> Self {
        for_each_gpio! {
            (all $( ($n:literal $($any:tt)*) ),*) => { const PINS: &[u8] = &[ $($n),* ]; };
        };
        assert!(PINS.contains(&pin), "Pin {} does not exist", pin);
        Self {
            pin,
            _lifetime: core::marker::PhantomData,
        }
    }

    #[procmacros::doc_replace]
    /// Unsafely clone the pin.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::gpio::{AnyPin, Pin};
    /// let pin = peripherals.GPIO1.degrade();
    /// let pin_cloned = unsafe { pin.clone_unchecked() };
    /// #
    /// # {after_snippet}
    /// ```
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a pin is in use at one time.
    pub unsafe fn clone_unchecked(&self) -> Self {
        Self {
            pin: self.pin,
            _lifetime: core::marker::PhantomData,
        }
    }

    #[procmacros::doc_replace]
    /// Creates a new AnyPin object that is limited to the lifetime of the
    /// passed reference.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::gpio::{AnyPin, Pin};
    /// let mut pin = peripherals.GPIO1.degrade();
    /// let pin_reborrowed = pin.reborrow();
    /// #
    /// # {after_snippet}
    /// ```
    pub fn reborrow(&mut self) -> AnyPin<'_> {
        unsafe { self.clone_unchecked() }
    }

    pub(crate) fn is_output(&self) -> bool {
        for_each_gpio! {
            (all $( ($n:literal, $gpio:ident $in_afs:tt $out_afs:tt ($input:tt [$($is_output:ident)?]) ) ),* ) => {
                return match self.number() {
                    $($(
                        // This code is generated if the Output attribute is present
                        $n => {
                            crate::ignore!($is_output);
                            true
                        }
                    )?)*
                    other => false,
                };
            };
        }
    }
}

for_each_gpio! {
    ($n:literal, $gpio:ident $af_ins:tt $af_outs:tt ([Input] $output:tt)) => {
        impl InputPin for crate::peripherals::$gpio<'_> {
            #[doc(hidden)]
            #[inline]
            fn waker(&self) -> &'static $crate::asynch::AtomicWaker {
                static WAKER: $crate::asynch::AtomicWaker = $crate::asynch::AtomicWaker::new();
                &WAKER
            }
        }
    };
}
for_each_gpio! {
    ($n:literal, $gpio:ident $af_ins:tt $af_outs:tt ($input:tt [Output])) => {
        impl OutputPin for crate::peripherals::$gpio<'_> {}
    };
}
for_each_gpio! {
    ($n:literal, $gpio:ident ($( $af_input_num:ident => $af_input_signal:ident )*) ($( $af_output_num:ident => $af_output_signal:ident )*) $attrs:tt) => {
        impl<'d> crate::peripherals::$gpio<'d> {
            #[allow(unused)]
            pub(crate) const NUMBER: u8 = $n;

            #[procmacros::doc_replace]
            /// Splits the pin into an input and output signal.
            ///
            /// Peripheral signals allow connecting peripherals together without using
            /// external hardware.
            ///
            /// # Safety
            ///
            /// The caller must ensure that peripheral drivers do not configure the same
            /// GPIO at the same time in multiple places. This includes clones of the
            /// `InputSignal` struct, as well as the `OutputSignal` struct
            ///
            /// ```rust, no_run
            /// # {before_snippet}
            /// #
            /// let (rx, tx) = unsafe { peripherals.GPIO2.split() };
            /// // rx and tx can then be passed to different peripherals to connect them.
            /// #
            /// # {after_snippet}
            /// ```
            #[instability::unstable]
            pub unsafe fn split(self) -> (interconnect::InputSignal<'d>, interconnect::OutputSignal<'d>) {
                // FIXME: we should implement this in the gpio macro for output pins, but we
                // should also have an input-only alternative for pins that can't be used as
                // outputs.

                // This goes through AnyPin which calls `init_gpio` as needed.
                unsafe { self.degrade().split() }
            }
        }

        impl Pin for crate::peripherals::$gpio<'_> {
            #[inline(always)]
            fn number(&self) -> u8 {
                $n
            }

            fn output_signals(&self, _: crate::private::Internal) -> &'static [(AlternateFunction, OutputSignal)] {
                &[$(
                    (AlternateFunction::$af_output_num, OutputSignal::$af_output_signal),
                )*]
            }

            fn input_signals(&self, _: crate::private::Internal) -> &'static [(AlternateFunction, InputSignal)] {
                &[$(
                    (AlternateFunction::$af_input_num, InputSignal::$af_input_signal),
                )*]
            }
        }

        impl<'lt> From<crate::peripherals::$gpio<'lt>> for AnyPin<'lt> {
            fn from(pin: crate::peripherals::$gpio<'lt>) -> Self {
                Pin::degrade(pin)
            }
        }
    };
}

define_io_mux_reg!();

// Implement the signal traits outside of interconnect, so that the impls
// don't inherit the module-level instability.
mod io_matrix_impls {
    use crate::gpio::{
        self,
        AnyPin,
        interconnect::{
            InputSignal,
            OutputSignal,
            PeripheralInput,
            PeripheralOutput,
            PeripheralSignal,
        },
    };

    impl<'d> PeripheralSignal<'d> for AnyPin<'d> {
        fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
            let pin = unsafe { self.clone_unchecked() };
            InputSignal::new(pin).connect_input_to_peripheral(signal);
        }
    }
    impl<'d> PeripheralInput<'d> for AnyPin<'d> {}

    impl<'d> PeripheralOutput<'d> for AnyPin<'d> {
        fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
            let pin = unsafe { self.clone_unchecked() };
            OutputSignal::new(pin).connect_peripheral_to_output(signal);
        }
        fn disconnect_from_peripheral_output(&self) {
            let pin = unsafe { self.clone_unchecked() };
            OutputSignal::new(pin).disconnect_from_peripheral_output();
        }
    }

    for_each_gpio! {
        ($n:literal, $gpio:ident $($_rest:tt)*) => {
            impl<'d> PeripheralSignal<'d> for crate::peripherals::$gpio<'d> {
                fn connect_input_to_peripheral(&self, signal: gpio::InputSignal) {
                    let pin = unsafe { AnyPin::steal($n) };
                    pin.connect_input_to_peripheral(signal);
                }
            }
            impl<'d> PeripheralInput<'d> for crate::peripherals::$gpio<'d> {}
        };
    }

    for_each_gpio! {
        ($n:literal, $gpio:ident $_af_ins:tt $_af_outs:tt ($input:tt [Output])) => {
            impl<'d> PeripheralOutput<'d> for crate::peripherals::$gpio<'d>
            {
                fn connect_peripheral_to_output(&self, signal: gpio::OutputSignal) {
                    let pin = unsafe { AnyPin::steal($n) };
                    pin.connect_peripheral_to_output(signal);
                }
                fn disconnect_from_peripheral_output(&self) {
                    let pin = unsafe { AnyPin::steal($n) };
                    pin.disconnect_from_peripheral_output();
                }
            }
        };
    }
}
