#![cfg_attr(docsrs, procmacros::doc_replace(
    "etm_availability" => {
        cfg(soc_has_etm) => "The GPIO pins also provide tasks and events via the ETM interconnect system. For more information, see the [etm] module."
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
//! [`Output::new`].
//!
//! Output pins can be configured to either push-pull or open-drain (active low)
//! mode, with configurable drive strength and pull-up/pull-down resistors.
//!
//! Each pin is a different type initially. Internally, `esp-hal` will erase
//! their types automatically, but they can also be converted into [`AnyPin`]
//! manually by calling [`Pin::degrade`].
//!
//! The [`Io`] struct can also be used to configure the interrupt handler for
//! GPIO interrupts. For more information, see the
//! [`InterruptConfigurable::set_interrupt_handler`](crate::interrupt::InterruptConfigurable::set_interrupt_handler).
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

    #[cfg(soc_has_etm)]
    pub mod etm;

    #[cfg(soc_has_lp_io)]
    pub mod lp_io;

    #[cfg(all(soc_has_rtc_io, not(esp32)))]
    pub mod rtc_io;
}

mod asynch;
mod embedded_hal_impls;
pub(crate) mod interrupt;
use interrupt::*;

mod placeholder;

use core::fmt::Display;

use esp_sync::RawMutex;
pub use placeholder::NoPin;
use portable_atomic::AtomicU32;
use strum::EnumCount;

use crate::{
    asynch::AtomicWaker,
    interrupt::{InterruptHandler, Priority},
    peripherals::{GPIO, IO_MUX, Interrupt},
    private::{self, Sealed},
};

define_io_mux_signals!();

pub(crate) static GPIO_LOCK: RawMutex = RawMutex::new();

/// Represents a pin-peripheral connection that, when dropped, disconnects the
/// peripheral from the pin.
///
/// This only needs to be applied to output signals, as it's not possible to
/// connect multiple inputs to the same peripheral signal.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PinGuard {
    pin: u8,
    signal: OutputSignal,
}

impl crate::private::Sealed for PinGuard {}

impl PinGuard {
    pub(crate) fn new(pin: AnyPin<'_>, signal: OutputSignal) -> Self {
        Self {
            pin: pin.number(),
            signal,
        }
    }

    pub(crate) fn new_unconnected(signal: OutputSignal) -> Self {
        Self {
            pin: u8::MAX,
            signal,
        }
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
            self.signal.disconnect_from(&pin);
        }
    }
}

/// Event used to trigger interrupts.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

impl From<WakeEvent> for Event {
    fn from(value: WakeEvent) -> Self {
        match value {
            WakeEvent::LowLevel => Event::LowLevel,
            WakeEvent::HighLevel => Event::HighLevel,
        }
    }
}

/// Event used to wake up from light sleep.
#[instability::unstable]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeEvent {
    /// Wake on low level
    LowLevel  = 4,
    /// Wake on high level
    HighLevel = 5,
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
    /// Create a [`Level`] from [`bool`].
    ///
    /// Like `<Level as From<bool>>::from(val)`, but `const`.
    pub(crate) const fn const_from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }

    /// Convert a [`Level`] to [`bool`].
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
    /// Returned when trying to configure a pin to wake up from light sleep on
    /// an edge trigger, which is not supported.
    EdgeTriggeringNotSupported,
}

impl Display for WakeConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WakeConfigError::EdgeTriggeringNotSupported => {
                write!(
                    f,
                    "Edge triggering is not supported for wake-up from light sleep"
                )
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
/// the chip and the specific pin. For more information, refer to your chip's
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
    const GPIO: Self = match Self::const_try_from(property!("gpio.gpio_function")) {
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

/// RTC function
#[instability::unstable]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(not(esp32h2))]
pub enum RtcFunction {
    /// RTC mode.
    Rtc     = 0,
    /// Digital mode.
    Digital = 1,
    /// RTC_I2C mode.
    #[cfg(soc_has_rtc_i2c)]
    I2c     = 3,
}

/// Trait implemented by RTC pins
#[instability::unstable]
#[cfg(not(esp32h2))] // H2 has no low-power mux, but it's not currently encoded in metadata.
pub trait RtcPin: Pin {
    /// RTC number of the pin
    #[cfg(xtensa)]
    fn rtc_number(&self) -> u8;

    /// Configure the pin
    #[cfg(any(xtensa, esp32c6))]
    #[doc(hidden)]
    fn rtc_set_config(&self, input_enable: bool, mux: bool, func: RtcFunction);

    /// Enable or disable PAD_HOLD
    #[doc(hidden)]
    fn rtcio_pad_hold(&self, enable: bool);

    /// # Safety
    ///
    /// The `level` argument needs to be a valid setting for the
    /// `rtc_cntl.gpio_wakeup.gpio_pinX_int_type`.
    #[cfg(any(esp32c3, esp32c2, esp32c6))]
    #[doc(hidden)]
    unsafe fn apply_wakeup(&self, wakeup: bool, level: u8);
}

/// Trait implemented by RTC pins which supporting internal pull-up / pull-down
/// resistors.
#[instability::unstable]
#[cfg(not(esp32h2))]
pub trait RtcPinWithResistors: RtcPin {
    /// Enable/disable the internal pull-up resistor
    #[doc(hidden)]
    fn rtcio_pullup(&self, enable: bool);
    /// Enable/disable the internal pull-down resistor
    #[doc(hidden)]
    fn rtcio_pulldown(&self, enable: bool);
}

/// Common trait implemented by pins
pub trait Pin: Sealed {
    /// GPIO number
    fn number(&self) -> u8;

    #[procmacros::doc_replace]
    /// Type-erase this pin into an [`AnyPin`].
    ///
    /// This function converts pin singletons (`GPIO0<'_>`, …), which are all
    /// different types, into the same type. It is useful for creating
    /// arrays of pins, or avoiding generics.
    ///
    /// ## Example
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

/// Trait implemented by pins which can be used as analog pins
#[instability::unstable]
pub trait AnalogPin: Pin {
    /// Configure the pin for analog operation
    #[doc(hidden)]
    fn set_analog(&self, _: private::Internal);
}

/// Trait implemented by pins which can be used as Touchpad pins
#[cfg(touch)]
#[instability::unstable]
pub trait TouchPin: Pin {
    /// Configure the pin for analog operation
    #[doc(hidden)]
    fn set_touch(&self, _: private::Internal);

    /// Reads the pin's touch measurement register
    #[doc(hidden)]
    fn touch_measurement(&self, _: private::Internal) -> u16;

    /// Maps the pin nr to the touch pad nr
    #[doc(hidden)]
    fn touch_nr(&self, _: private::Internal) -> u8;

    /// Set a pins touch threshold for interrupts.
    #[doc(hidden)]
    fn set_threshold(&self, threshold: u16, _: private::Internal);
}

#[doc(hidden)]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash, EnumCount)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GpioBank {
    _0,
    #[cfg(gpio_has_bank_1)]
    _1,
}

impl GpioBank {
    fn async_operations(self) -> &'static AtomicU32 {
        static FLAGS: [AtomicU32; GpioBank::COUNT] = [const { AtomicU32::new(0) }; GpioBank::COUNT];

        &FLAGS[self as usize]
    }

    fn offset(self) -> u8 {
        match self {
            Self::_0 => 0,
            #[cfg(gpio_has_bank_1)]
            Self::_1 => 32,
        }
    }

    fn write_out_en(self, word: u32, enable: bool) {
        if enable {
            self.write_out_en_set(word);
        } else {
            self.write_out_en_clear(word);
        }
    }

    fn write_out_en_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .enable_w1tc()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .enable1_w1tc()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    fn write_out_en_set(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .enable_w1ts()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .enable1_w1ts()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    fn read_input(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().in_().read().bits(),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().in1().read().bits(),
        }
    }

    fn read_output(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().out().read().bits(),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1().read().bits(),
        }
    }

    fn read_interrupt_status(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().status().read().bits(),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().status1().read().bits(),
        }
    }

    fn write_interrupt_status_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .status_w1tc()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .status1_w1tc()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    fn write_output(self, word: u32, set: bool) {
        if set {
            self.write_output_set(word);
        } else {
            self.write_output_clear(word);
        }
    }

    fn write_output_set(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs().out_w1ts().write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1_w1ts().write(|w| unsafe { w.bits(word) }),
        };
    }

    fn write_output_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs().out_w1tc().write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1_w1tc().write(|w| unsafe { w.bits(word) }),
        };
    }
}

/// Any GPIO pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyPin<'lt> {
    pub(crate) pin: u8,
    pub(crate) _lifetime: core::marker::PhantomData<&'lt mut ()>,
}

/// General Purpose Input/Output driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Io<'d> {
    _io_mux: IO_MUX<'d>,
}

impl<'d> Io<'d> {
    /// Initialize the I/O driver.
    #[instability::unstable]
    pub fn new(_io_mux: IO_MUX<'d>) -> Self {
        Io { _io_mux }
    }

    /// Set the interrupt priority for GPIO interrupts.
    ///
    /// # Panics
    ///
    /// Panics if passed interrupt handler is invalid (e.g. has priority
    /// `None`)
    #[instability::unstable]
    pub fn set_interrupt_priority(&self, prio: Priority) {
        interrupt::set_interrupt_priority(Interrupt::GPIO, prio);
    }

    #[cfg_attr(
        not(multi_core),
        doc = "Registers an interrupt handler for all GPIO pins."
    )]
    #[cfg_attr(
        multi_core,
        doc = "Registers an interrupt handler for all GPIO pins on the current core."
    )]
    #[doc = ""]
    /// Note that when using interrupt handlers registered by this function, or
    /// by defining a `#[no_mangle] unsafe extern "C" fn GPIO()` function, we do
    /// **not** clear the interrupt status register or the interrupt enable
    /// setting for you. Based on your use case, you need to do one of this
    /// yourself:
    ///
    /// - Disabling the interrupt enable setting for the GPIO pin allows you to handle an event once
    ///   per call to [`listen()`]. Using this method, the [`is_interrupt_set()`] method will return
    ///   `true` if the interrupt is set even after your handler has finished running.
    /// - Clearing the interrupt status register allows you to handle an event repeatedly after
    ///   [`listen()`] is called. Using this method, [`is_interrupt_set()`] will return `false`
    ///   after your handler has finished running.
    ///
    /// [`listen()`]: Input::listen
    /// [`is_interrupt_set()`]: Input::is_interrupt_set
    ///
    /// # Panics
    ///
    /// Panics if passed interrupt handler is invalid (e.g. has priority
    /// `None`)
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::GPIO);
        }
        self.set_interrupt_priority(handler.priority());
        unsafe {
            crate::interrupt::bind_interrupt(
                Interrupt::GPIO,
                crate::interrupt::IsrCallback::new(user_gpio_interrupt_handler),
            )
        };
        USER_INTERRUPT_HANDLER.store(handler.handler().aligned_ptr());
    }
}

impl crate::private::Sealed for Io<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Io<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

for_each_analog_function! {
    (($_ch:ident, ADCn_CHm, $_n:literal, $_m:literal), $gpio:ident) => {
        #[instability::unstable]
        impl $crate::gpio::AnalogPin for crate::peripherals::$gpio<'_> {
            #[cfg(riscv)]
            fn set_analog(&self, _: private::Internal) {
                io_mux_reg(self.number()).modify(|_, w| unsafe {
                    w.mcu_sel().bits(1);
                    w.fun_ie().clear_bit();
                    w.fun_wpu().clear_bit();
                    w.fun_wpd().clear_bit()
                });

                GPIO::regs()
                    .enable_w1tc()
                    .write(|w| unsafe { w.bits(1 << self.number()) });
            }

            #[cfg(not(riscv))]
            fn set_analog(&self, _: private::Internal) {
                self.set_analog_impl();
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
    /// logical [`Level`]s.
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
/// This struct is used to configure the drive mode, drive strength, and pull
/// direction of an output pin. By default, the configuration is set to:
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

impl<'d> Output<'d> {
    #[procmacros::doc_replace]
    /// Creates a new GPIO output driver.
    ///
    /// The `initial_level` parameter sets the initial output level of the pin.
    /// The `config` parameter sets the drive mode, drive strength, and pull
    /// direction of the pin.
    ///
    /// ## Example
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
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::OutputSignal::freeze).
    ///
    /// ## Example
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
    /// Change the configuration.
    ///
    /// ## Example
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
    /// Set the output as high.
    ///
    /// ## Example
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
    /// Set the output as low.
    ///
    /// ## Example
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
    /// Set the output level.ç
    ///
    /// ## Example
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
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    ///
    /// ## Example
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
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    ///
    /// ## Example
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
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    ///
    /// ## Example
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
    /// ## Example
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

    /// Converts the pin driver into a [`Flex`] driver.
    #[inline]
    #[instability::unstable]
    pub fn into_flex(self) -> Flex<'d> {
        self.pin
    }
}

/// Input pin configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct InputConfig {
    /// Initial pull of the pin.
    pull: Pull,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self { pull: Pull::None }
    }
}

/// Digital input.
///
/// This driver configures the GPIO pin to be an input. Input drivers read the
/// voltage of their pins and convert it to a logical [`Level`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl private::Sealed for Input<'_> {}

impl<'d> Input<'d> {
    #[procmacros::doc_replace]
    /// Creates a new GPIO input.
    ///
    /// The `pull` parameter configures internal pull-up or pull-down
    /// resistors.
    ///
    /// ## Example
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
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
    ///
    /// ## Example
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
    /// Get whether the pin input level is high.
    ///
    /// ## Example
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
    /// Get whether the pin input level is low.
    ///
    /// ## Example
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
    /// Get the current pin input level.
    ///
    /// ## Example
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
    /// Change the configuration.
    ///
    /// ## Example
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
    /// Listen for interrupts.
    ///
    /// The interrupts will be handled by the handler set using
    /// [`Io::set_interrupt_handler`]. All GPIO pins share the same
    /// interrupt handler.
    ///
    /// Note that [`Event::LowLevel`] and [`Event::HighLevel`] are fired
    /// continuously when the pin is low or high, respectively. You must use
    /// a custom interrupt handler to stop listening for these events,
    /// otherwise your program will be stuck in a loop as long as the pin is
    /// reading the corresponding level.
    ///
    /// ## Examples
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
    ///     // but usually you'd probably use `FallingEdge`.
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
    /// #[handler]
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

    /// Stop listening for interrupts
    #[inline]
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        self.pin.unlisten();
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    #[instability::unstable]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt();
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.is_interrupt_set()
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    ///
    /// # Error
    /// Configuring pin to wake up from light sleep on an edge
    /// trigger is currently not supported, corresponding variant of
    /// [`WakeConfigError`] will be returned.
    #[instability::unstable]
    #[inline]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) -> Result<(), WakeConfigError> {
        self.pin.wakeup_enable(enable, event)
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
/// Disabling the input or output stages don't forget their configuration.
/// Disabling the output stage will not change the output level, but it will
/// disable the driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct Flex<'d> {
    pin: AnyPin<'d>,
}

impl private::Sealed for Flex<'_> {}

impl<'d> Flex<'d> {
    /// Create flexible pin driver for a [Pin].
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
    /// This function does not set the pin as input (i.e. it does not enable the
    /// input buffer). Note that the pull direction is common between the
    /// input and output configuration.
    #[inline]
    #[instability::unstable]
    pub fn apply_input_config(&mut self, config: &InputConfig) {
        self.pin.apply_input_config(config);
    }

    /// Enable or disable the GPIO pin input buffer.
    #[inline]
    #[instability::unstable]
    pub fn set_input_enable(&mut self, enable_input: bool) {
        self.pin.set_input_enable(enable_input);
    }

    /// Get whether the pin input level is high.
    #[inline]
    #[instability::unstable]
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    #[instability::unstable]
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    #[instability::unstable]
    pub fn level(&self) -> Level {
        self.pin.is_input_high().into()
    }

    /// Listen for interrupts.
    ///
    /// See [`Input::listen`] for more information and an example.
    #[inline]
    #[instability::unstable]
    pub fn listen(&mut self, event: Event) {
        // Unwrap can't fail currently as listen_with_options is only supposed to return
        // an error if wake_up_from_light_sleep is true.
        unwrap!(self.pin.listen_with_options(event, true, false, false));
    }

    /// Stop listening for interrupts.
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

    /// Check if the pin is listening for interrupts.
    #[inline]
    #[instability::unstable]
    pub fn is_listening(&self) -> bool {
        is_int_enabled(self.pin.number())
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    #[instability::unstable]
    pub fn clear_interrupt(&mut self) {
        self.pin
            .bank()
            .write_interrupt_status_clear(self.pin.mask());
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.bank().read_interrupt_status() & self.pin.mask() != 0
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    ///
    /// # Error
    /// Configuring pin to wake up from light sleep on an edge
    /// trigger is currently not supported, corresponding variant of
    /// [`WakeConfigError`] will be returned.
    #[inline]
    #[instability::unstable]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) -> Result<(), WakeConfigError> {
        self.pin
            .listen_with_options(event.into(), false, false, enable)
    }

    // Output functions

    /// Applies the given output configuration to the pin.
    ///
    /// This function does not set the pin to output (i.e. it does not enable
    /// the output driver). Note that the pull direction is common between
    /// the input and output configuration.
    #[inline]
    #[instability::unstable]
    pub fn apply_output_config(&mut self, config: &OutputConfig) {
        self.pin.apply_output_config(config);
    }

    /// Enable or disable the GPIO pin output driver.
    ///
    /// The output level will be set to the last value. Use [`Self::set_high`],
    /// [`Self::set_low`] or [`Self::set_level`] to set the output level before
    /// enabling the output.
    ///
    /// This function does not disable the input buffer.
    #[inline]
    #[instability::unstable]
    pub fn set_output_enable(&mut self, enable_output: bool) {
        self.pin.set_output_enable(enable_output);
    }

    /// Set the output as high.
    #[inline]
    #[instability::unstable]
    pub fn set_high(&mut self) {
        self.set_level(Level::High)
    }

    /// Set the output as low.
    #[inline]
    #[instability::unstable]
    pub fn set_low(&mut self) {
        self.set_level(Level::Low)
    }

    /// Set the output level.
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

    /// What level output is set to
    #[inline]
    #[instability::unstable]
    pub fn output_level(&self) -> Level {
        self.pin.is_set_high().into()
    }

    /// Toggle pin output
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
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
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
    /// Split the pin into an input and output signal pair.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    ///
    /// Note that the signals returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
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

    /// Split the pin into an [Input] and an [Output] driver pair.
    ///
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::InputSignal::freeze). On the other hand,
    /// the pin driver is free to change settings.
    ///
    /// This function allows you to configure an input-output pin, then keep
    /// working with the output half. This is mainly intended for testing,
    /// allowing you to drive a peripheral from a signal generated by
    /// software.
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
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::OutputSignal::freeze).
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
        #[cfg(gpio_has_bank_1)]
        if self.number() >= 32 {
            return GpioBank::_1;
        }

        GpioBank::_0
    }

    pub(crate) fn disable_usb_pads(&self) {
        #[cfg(soc_has_usb_device)]
        {
            /// Workaround to make D+ and D- work when the pin is assigned to
            /// the `USB_SERIAL_JTAG` peripheral by default.
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

            macro_rules! disable_usb_pads {
                ($gpio:ident) => {
                    if self.number() == crate::peripherals::$gpio::NUMBER {
                        disable_usb_pads(crate::peripherals::$gpio::NUMBER);
                    }
                };
            }

            for_each_analog_function! {
                (USB_DM, $gpio:ident) => { disable_usb_pads!($gpio) };
                (USB_DP, $gpio:ident) => { disable_usb_pads!($gpio) };
            }
        }
    }

    #[inline]
    /// Resets the GPIO to a known state.
    ///
    /// This function needs to be called before using the GPIO pin:
    /// - Before converting it into signals
    /// - Before using it as an input or output
    pub(crate) fn init_gpio(&self) {
        self.set_output_enable(false);
        self.disable_usb_pads();

        GPIO::regs()
            .func_out_sel_cfg(self.number() as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as _) });

        // Use RMW to not overwrite sleep configuration
        io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(AlternateFunction::GPIO as u8);
            w.fun_ie().clear_bit();
            w.slp_sel().clear_bit()
        });
    }

    #[procmacros::doc_replace]
    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// Creating an input signal enables the pin's input buffer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that peripheral drivers don't configure the same
    /// GPIO at the same time in multiple places. This includes clones of the
    /// `InputSignal` struct, as well as the `OutputSignal` struct.
    ///
    /// # Panics
    ///
    /// This function panics if the pin is not an output pin.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::gpio::{AnyPin, Pin};
    /// let pin1 = peripherals.GPIO1.degrade();
    /// let (input, output) = unsafe { pin1.split() };
    /// # {after_snippet}
    /// ```
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

    /// Convert the pin into an input signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// Creating an input signal enables the pin's input buffer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that peripheral drivers don't configure the same
    /// GPIO at the same time in multiple places. This includes clones of the
    /// `InputSignal` struct.
    #[inline]
    #[instability::unstable]
    pub unsafe fn into_input_signal(self) -> interconnect::InputSignal<'lt> {
        // Before each use, reset the GPIO to a known state.
        self.init_gpio();
        self.set_input_enable(true);

        let (input, _) = unsafe { self.split_no_init() };

        input
    }

    /// Convert the pin into an output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// # Panics
    ///
    /// This function panics if the pin is not an output pin.
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

    /// Enable or disable the GPIO pin output buffer.
    #[inline]
    pub(crate) fn set_output_enable(&self, enable: bool) {
        assert!(self.is_output() || !enable);
        self.bank().write_out_en(self.mask(), enable);
    }

    /// Enable input for the pin
    #[inline]
    pub(crate) fn set_input_enable(&self, on: bool) {
        io_mux_reg(self.number()).modify(|_, w| w.fun_ie().bit(on));
    }

    #[inline]
    pub(crate) fn apply_input_config(&self, config: &InputConfig) {
        let pull_up = config.pull == Pull::Up;
        let pull_down = config.pull == Pull::Down;

        #[cfg(esp32)]
        crate::soc::gpio::errata36(unsafe { self.clone_unchecked() }, pull_up, pull_down);

        io_mux_reg(self.number()).modify(|_, w| {
            w.fun_wpd().bit(pull_down);
            w.fun_wpu().bit(pull_up)
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

    fn listen_with_options(
        &self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    ) -> Result<(), WakeConfigError> {
        /// Assembles a valid value for the int_ena pin register field.
        fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    match crate::system::Cpu::current() {
                        crate::system::Cpu::AppCpu => int_enable as u8 | ((nmi_enable as u8) << 1),
                        crate::system::Cpu::ProCpu => ((int_enable as u8) << 2) | ((nmi_enable as u8) << 3),
                    }
                } else {
                    // ESP32 and ESP32-C3 have separate bits for maskable and NMI interrupts.
                    int_enable as u8 | ((nmi_enable as u8) << 1)
                }
            }
        }

        if wake_up_from_light_sleep {
            match event {
                Event::AnyEdge | Event::RisingEdge | Event::FallingEdge => {
                    return Err(WakeConfigError::EdgeTriggeringNotSupported);
                }
                _ => {}
            }
        }

        self.with_gpio_lock(|| {
            // Clear the interrupt status bit for this Pin, just in case the user forgot.
            // Since we disabled the interrupt in the handler, it's not possible to
            // trigger a new interrupt before we re-enable it here.
            self.clear_interrupt();

            set_int_enable(
                self.number(),
                Some(gpio_intr_enable(int_enable, nmi_enable)),
                event as u8,
                wake_up_from_light_sleep,
            );
        });
        Ok(())
    }

    #[inline]
    fn apply_output_config(&self, config: &OutputConfig) {
        let pull_up = config.pull == Pull::Up;
        let pull_down = config.pull == Pull::Down;

        #[cfg(esp32)]
        crate::soc::gpio::errata36(unsafe { self.clone_unchecked() }, pull_up, pull_down);

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

    /// The current state of the input
    #[inline]
    pub(crate) fn is_input_high(&self) -> bool {
        self.bank().read_input() & self.mask() != 0
    }

    /// Set the pin's level to high or low
    #[inline]
    pub(crate) fn set_output_high(&self, high: bool) {
        self.bank().write_output(self.mask(), high);
    }

    /// Is the output set to high
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
    /// ## Example
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
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a pin is in use at one time.
    ///
    /// # Panics
    ///
    /// Panics if the pin with the given number does not exist.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// #
    /// use esp_hal::gpio::AnyPin;
    /// let pin = unsafe { AnyPin::steal(1) };
    /// #
    /// # {after_snippet}
    /// ```
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
    /// # Safety
    ///
    /// Ensure that only one instance of a pin is in use at one time.
    ///
    /// ## Example
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
    pub unsafe fn clone_unchecked(&self) -> Self {
        Self {
            pin: self.pin,
            _lifetime: core::marker::PhantomData,
        }
    }

    #[procmacros::doc_replace]
    /// Create a new AnyPin object that is limited to the lifetime of the
    /// passed reference.
    ///
    /// ## Example
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

#[cold]
#[allow(unused)]
fn pin_does_not_support_function(pin: u8, function: &str) {
    panic!("Pin {} is not an {}", pin, function)
}

#[cfg(not(esp32h2))]
macro_rules! for_each_rtcio_pin {
    (@impl $ident:ident, $target:ident, $gpio:ident, $code:tt) => {
        if $ident.number() == $crate::peripherals::$gpio::NUMBER {
            #[allow(unused_mut)]
            let mut $target = unsafe { $crate::peripherals::$gpio::steal() };
            return $code;
        }
    };

    (($ident:ident, $target:ident) => $code:tt;) => {
        for_each_lp_function! {
            (($_sig:ident, RTC_GPIOn, $_n:literal), $gpio:ident) => {
                for_each_rtcio_pin!(@impl $ident, $target, $gpio, $code)
            };
            (($_sig:ident, LP_GPIOn, $_n:literal), $gpio:ident) => {
                for_each_rtcio_pin!(@impl $ident, $target, $gpio, $code)
            };
        }
        unreachable!();
    };
}

#[cfg(not(esp32h2))]
macro_rules! for_each_rtcio_output_pin {
    (@impl $ident:ident, $target:ident, $gpio:ident, $code:tt, $kind:literal) => {
        if $ident.number() == $crate::peripherals::$gpio::NUMBER {
            for_each_gpio! {
                // If the pin is an output pin, generate $code
                ($n:tt, $gpio $in_afs:tt $out_afs:tt ($input:tt [Output])) => {
                    #[allow(unused_mut)]
                    let mut $target = unsafe { $crate::peripherals::$gpio::steal() };
                    return $code;
                };
                // If the pin is not an output pin, generate a panic
                ($n:tt, $gpio $in_afs:tt $out_afs:tt ($input:tt [])) => {
                    pin_does_not_support_function($crate::peripherals::$gpio::NUMBER, $kind)
                };
            }
        }
    };

    (($ident:ident, $target:ident) => $code:tt;) => {
        for_each_lp_function! {
            (($_sig:ident, RTC_GPIOn, $_n:literal), $gpio:ident) => {
                for_each_rtcio_output_pin!(@impl $ident, $target, $gpio, $code, "RTC_IO output")
            };
            (($_sig:ident, LP_GPIOn, $_n:literal), $gpio:ident) => {
                for_each_rtcio_output_pin!(@impl $ident, $target, $gpio, $code, "LP_IO output")
            };
        }
        unreachable!();
    };
}

#[cfg(not(esp32h2))]
impl RtcPin for AnyPin<'_> {
    #[cfg(xtensa)]
    fn rtc_number(&self) -> u8 {
        for_each_rtcio_pin! {
            (self, target) => { RtcPin::rtc_number(&target) };
        }
    }

    #[cfg(any(xtensa, esp32c6))]
    fn rtc_set_config(&self, input_enable: bool, mux: bool, func: RtcFunction) {
        for_each_rtcio_pin! {
            (self, target) => { RtcPin::rtc_set_config(&target, input_enable, mux, func) };
        }
    }

    fn rtcio_pad_hold(&self, enable: bool) {
        for_each_rtcio_pin! {
            (self, target) => { RtcPin::rtcio_pad_hold(&target, enable) };
        }
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6))]
    unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
        for_each_rtcio_pin! {
            (self, target) => { unsafe { RtcPin::apply_wakeup(&target, wakeup, level) } };
        }
    }
}

#[cfg(not(esp32h2))]
impl RtcPinWithResistors for AnyPin<'_> {
    fn rtcio_pullup(&self, enable: bool) {
        for_each_rtcio_output_pin! {
            (self, target) => { RtcPinWithResistors::rtcio_pullup(&target, enable) };
        }
    }

    fn rtcio_pulldown(&self, enable: bool) {
        for_each_rtcio_output_pin! {
            (self, target) => { RtcPinWithResistors::rtcio_pulldown(&target, enable) };
        }
    }
}

/// Set GPIO event listening.
///
/// - `gpio_num`: the pin to configure
/// - `int_ena`: maskable and non-maskable CPU interrupt bits. None to leave unchanged.
/// - `int_type`: interrupt type, see [Event] (or 0 to disable)
/// - `wake_up_from_light_sleep`: whether to wake up from light sleep
fn set_int_enable(gpio_num: u8, int_ena: Option<u8>, int_type: u8, wake_up_from_light_sleep: bool) {
    GPIO::regs().pin(gpio_num as usize).modify(|_, w| unsafe {
        if let Some(int_ena) = int_ena {
            w.int_ena().bits(int_ena);
        }
        w.int_type().bits(int_type);
        w.wakeup_enable().bit(wake_up_from_light_sleep)
    });
}

fn is_int_enabled(gpio_num: u8) -> bool {
    GPIO::regs().pin(gpio_num as usize).read().int_ena().bits() != 0
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
            /// Split the pin into an input and output signal.
            ///
            /// Peripheral signals allow connecting peripherals together without using
            /// external hardware.
            ///
            /// # Safety
            ///
            /// The caller must ensure that peripheral drivers don't configure the same
            /// GPIO at the same time in multiple places. This includes clones of the
            /// `InputSignal` struct, as well as the `OutputSignal` struct.
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
