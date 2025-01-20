//! # General Purpose Input/Output (GPIO)
//!
//! ## Overview
//!
//! Each pin can be used as a general-purpose I/O, or be connected to one or
//! more internal peripheral signals.
#![cfg_attr(
    soc_etm,
    doc = "The GPIO pins also provide tasks and events via the ETM interconnect system. For more information, see the [etm] module."
)]
#![doc = ""]
//! ## Working with pins
//!
//! After initializing the HAL, you can access the individual pins using the
//! [`crate::Peripherals`] struct. These pins can then be used as general
//! purpose digital IO using pin drivers, or they can be passed to peripherals
//! (such as SPI, UART, I2C, etc.), or can be [`GpioPin::split`]
//! into peripheral signals for advanced use.
//!
//! Pin drivers can be created using [`Flex::new`], [`Input::new`],
//! [`Output::new`] and [`OutputOpenDrain::new`].
//!
//! Each pin is a different type initially. Internally, `esp-hal` will often
//! erase their types automatically, but they can also be converted into
//! [`AnyPin`] manually by calling [`Pin::degrade`].
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
//! To obtain peripheral signals, use the [`GpioPin::split`] method to split a
//! pin into an input and output signal. Alternatively, you may use
//! [`Flex::split`], [`Flex::into_peripheral_output`],
//! [`Flex::peripheral_input`], and similar methods to split a pin driver into
//! an input and output signal. You can then pass these signals to the
//! peripheral drivers similar to how you would pass a pin.
//!
//! ### GPIO interconnect example
//!
//! See the [Inverting TX and RX Pins] example of the UART documentation.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/index.html
//! [Inverting TX and RX Pins]: crate::uart#inverting-rx-and-tx-pins

use core::fmt::Display;

use portable_atomic::{AtomicPtr, AtomicU32, Ordering};
use procmacros::ram;
use strum::EnumCount;

#[cfg(any(lp_io, rtc_cntl))]
use crate::peripherals::gpio::{handle_rtcio, handle_rtcio_with_resistors};
pub use crate::soc::gpio::*;
use crate::{
    interrupt::{
        self,
        InterruptConfigurable,
        InterruptHandler,
        Priority,
        DEFAULT_INTERRUPT_HANDLER,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        gpio::{handle_gpio_input, handle_gpio_output},
        Interrupt,
        GPIO,
        IO_MUX,
    },
    private::{self, Sealed},
};

mod placeholder;

pub use placeholder::NoPin;

crate::unstable_module! {
    pub mod interconnect;

    #[cfg(soc_etm)]
    pub mod etm;

    #[cfg(lp_io)]
    pub mod lp_io;

    #[cfg(all(rtc_io, not(esp32)))]
    pub mod rtc_io;
}

/// Convenience constant for `Option::None` pin
static USER_INTERRUPT_HANDLER: CFnPtr = CFnPtr::new();

struct CFnPtr(AtomicPtr<()>);
impl CFnPtr {
    pub const fn new() -> Self {
        Self(AtomicPtr::new(core::ptr::null_mut()))
    }

    pub fn store(&self, f: extern "C" fn()) {
        self.0.store(f as *mut (), Ordering::Relaxed);
    }

    pub fn call(&self) {
        let ptr = self.0.load(Ordering::Relaxed);
        if !ptr.is_null() {
            unsafe { (core::mem::transmute::<*mut (), extern "C" fn()>(ptr))() };
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

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {}

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

/// Pull setting for an input.
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

impl TryFrom<usize> for AlternateFunction {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AlternateFunction::_0),
            1 => Ok(AlternateFunction::_1),
            2 => Ok(AlternateFunction::_2),
            3 => Ok(AlternateFunction::_3),
            4 => Ok(AlternateFunction::_4),
            5 => Ok(AlternateFunction::_5),
            _ => Err(()),
        }
    }
}

/// RTC function
#[instability::unstable]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcFunction {
    /// RTC mode.
    Rtc     = 0,
    /// Digital mode.
    Digital = 1,
}

/// Trait implemented by RTC pins
#[instability::unstable]
pub trait RtcPin: Pin {
    /// RTC number of the pin
    #[cfg(xtensa)]
    fn rtc_number(&self) -> u8;

    /// Configure the pin
    #[cfg(any(xtensa, esp32c6))]
    fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: RtcFunction);

    /// Enable or disable PAD_HOLD
    fn rtcio_pad_hold(&mut self, enable: bool);

    /// # Safety
    ///
    /// The `level` argument needs to be a valid setting for the
    /// `rtc_cntl.gpio_wakeup.gpio_pinX_int_type`.
    #[cfg(any(esp32c3, esp32c2, esp32c6))]
    unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8);
}

/// Trait implemented by RTC pins which supporting internal pull-up / pull-down
/// resistors.
#[instability::unstable]
#[cfg(any(lp_io, rtc_cntl))]
pub trait RtcPinWithResistors: RtcPin {
    /// Enable/disable the internal pull-up resistor
    fn rtcio_pullup(&mut self, enable: bool);
    /// Enable/disable the internal pull-down resistor
    fn rtcio_pulldown(&mut self, enable: bool);
}

/// Common trait implemented by pins
pub trait Pin: Sealed {
    /// GPIO number
    fn number(&self) -> u8;

    /// Type-erase (degrade) this pin into an [`AnyPin`].
    ///
    /// This converts pin singletons (`GpioPin<0>`, …), which are all different
    /// types, into the same type. It is useful for creating arrays of pins,
    /// or avoiding generics.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::gpio::{AnyPin, Pin, Output, OutputConfig, Level};
    /// use esp_hal::delay::Delay;
    ///
    /// fn toggle_pins(pins: [AnyPin; 2], delay: &mut Delay) {
    ///     let [red, blue] = pins;
    ///     let mut red = Output::new(
    ///         red,
    ///         OutputConfig::default().with_level(Level::High)
    ///     ).unwrap();
    ///     let mut blue = Output::new(
    ///         blue,
    ///         OutputConfig::default().with_level(Level::Low)
    ///     ).unwrap();
    ///
    ///     loop {
    ///         red.toggle();
    ///         blue.toggle();
    ///         delay.delay_millis(500);
    ///     }
    /// }
    ///
    /// let pins: [AnyPin; 2] = [
    ///    peripherals.GPIO5.degrade(),
    ///    peripherals.GPIO6.degrade(),
    /// ];
    ///
    /// let mut delay = Delay::new();
    /// toggle_pins(pins, &mut delay);
    /// # }
    /// ```
    fn degrade(self) -> AnyPin
    where
        Self: Sized,
    {
        unsafe { AnyPin::steal(self.number()) }
    }

    #[doc(hidden)]
    fn output_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, OutputSignal)];

    #[doc(hidden)]
    fn input_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, InputSignal)];
}

/// Trait implemented by pins which can be used as inputs.
pub trait InputPin: Pin + Into<AnyPin> + 'static {}

/// Trait implemented by pins which can be used as outputs.
pub trait OutputPin: Pin + Into<AnyPin> + 'static {}

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
pub enum GpioRegisterAccess {
    Bank0,
    #[cfg(gpio_bank_1)]
    Bank1,
}

impl From<usize> for GpioRegisterAccess {
    fn from(_gpio_num: usize) -> Self {
        #[cfg(gpio_bank_1)]
        if _gpio_num >= 32 {
            return GpioRegisterAccess::Bank1;
        }

        GpioRegisterAccess::Bank0
    }
}

impl GpioRegisterAccess {
    fn async_operations(self) -> &'static AtomicU32 {
        static FLAGS: [AtomicU32; GpioRegisterAccess::COUNT] =
            [const { AtomicU32::new(0) }; GpioRegisterAccess::COUNT];

        &FLAGS[self as usize]
    }

    fn offset(self) -> u8 {
        match self {
            Self::Bank0 => 0,
            #[cfg(gpio_bank_1)]
            Self::Bank1 => 32,
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
            Self::Bank0 => Bank0GpioRegisterAccess::write_out_en_clear(word),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::write_out_en_clear(word),
        }
    }

    fn write_out_en_set(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_out_en_set(word),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::write_out_en_set(word),
        }
    }

    fn read_input(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_input(),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::read_input(),
        }
    }

    fn read_output(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_output(),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::read_output(),
        }
    }

    fn read_interrupt_status(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_interrupt_status(),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::read_interrupt_status(),
        }
    }

    fn write_interrupt_status_clear(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_interrupt_status_clear(word),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::write_interrupt_status_clear(word),
        }
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
            Self::Bank0 => Bank0GpioRegisterAccess::write_output_set(word),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::write_output_set(word),
        }
    }

    fn write_output_clear(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_output_clear(word),
            #[cfg(gpio_bank_1)]
            Self::Bank1 => Bank1GpioRegisterAccess::write_output_clear(word),
        }
    }
}

struct Bank0GpioRegisterAccess;

impl Bank0GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        GPIO::regs().in_().read().bits()
    }

    fn read_output() -> u32 {
        GPIO::regs().out().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        GPIO::regs().status().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        GPIO::regs()
            .status_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        GPIO::regs().out_w1ts().write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        GPIO::regs().out_w1tc().write(|w| unsafe { w.bits(word) });
    }
}

#[cfg(gpio_bank_1)]
struct Bank1GpioRegisterAccess;

#[cfg(gpio_bank_1)]
impl Bank1GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        GPIO::regs()
            .enable1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        GPIO::regs()
            .enable1_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        GPIO::regs().in1().read().bits()
    }

    fn read_output() -> u32 {
        GPIO::regs().out1().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        GPIO::regs().status1().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        GPIO::regs()
            .status1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        GPIO::regs().out1_w1ts().write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        GPIO::regs().out1_w1tc().write(|w| unsafe { w.bits(word) });
    }
}

/// GPIO pin
#[non_exhaustive]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GpioPin<const GPIONUM: u8>;

/// Type-erased GPIO pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyPin(pub(crate) u8);

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: Pin,
{
    /// Create a pin out of thin air.
    ///
    /// # Safety
    ///
    /// Ensure that only one instance of a pin exists at one time.
    pub unsafe fn steal() -> Self {
        Self
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        // FIXME: we should implement this in the gpio macro for output pins, but we
        // should also have an input-only alternative for pins that can't be used as
        // outputs.
        self.degrade().split()
    }
}

/// Workaround to make D+ and D- work on the ESP32-C3 and ESP32-S3, which by
/// default are assigned to the `USB_SERIAL_JTAG` peripheral.
#[cfg(usb_device)]
fn disable_usb_pads(gpionum: u8) {
    cfg_if::cfg_if! {
        if #[cfg(esp32c3)] {
            let pins = [18, 19];
        } else if #[cfg(esp32c6)] {
            let pins = [12, 13];
        } else if #[cfg(esp32h2)] {
            let pins = [26, 27];
        } else if #[cfg(esp32s3)] {
            let pins = [19, 20];
        } else {
            compile_error!("Please define USB pins for this chip");
        }
    }

    if pins.contains(&gpionum) {
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
}

impl<const GPIONUM: u8> Peripheral for GpioPin<GPIONUM>
where
    Self: Pin,
{
    type P = GpioPin<GPIONUM>;

    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

impl<const GPIONUM: u8> private::Sealed for GpioPin<GPIONUM> {}

pub(crate) fn bind_default_interrupt_handler() {
    // We first check if a handler is set in the vector table.
    if let Some(handler) = interrupt::bound_handler(Interrupt::GPIO) {
        let handler = handler as *const unsafe extern "C" fn();

        // We only allow binding the default handler if nothing else is bound.
        // This prevents silently overwriting RTIC's interrupt handler, if using GPIO.
        if !core::ptr::eq(handler, DEFAULT_INTERRUPT_HANDLER.handler() as _) {
            // The user has configured an interrupt handler they wish to use.
            info!("Not using default GPIO interrupt handler: already bound in vector table");
            return;
        }
    }
    // The vector table doesn't contain a custom entry.Still, the
    // peripheral interrupt may already be bound to something else.
    if interrupt::bound_cpu_interrupt_for(crate::Cpu::current(), Interrupt::GPIO).is_some() {
        info!("Not using default GPIO interrupt handler: peripheral interrupt already in use");
        return;
    }

    unsafe { interrupt::bind_interrupt(Interrupt::GPIO, default_gpio_interrupt_handler) };
    // By default, we use lowest priority
    unwrap!(interrupt::enable(Interrupt::GPIO, Priority::min()));
}

/// General Purpose Input/Output driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Io {
    _io_mux: IO_MUX,
}

impl Io {
    /// Initialize the I/O driver.
    pub fn new(_io_mux: IO_MUX) -> Self {
        Io { _io_mux }
    }

    /// Set the interrupt priority for GPIO interrupts.
    pub fn set_interrupt_priority(&self, prio: Priority) {
        unwrap!(interrupt::enable(Interrupt::GPIO, prio));
    }
}

impl crate::private::Sealed for Io {}

impl InterruptConfigurable for Io {
    /// Install the given interrupt handler replacing any previously set
    /// handler.
    ///
    /// Note that when using interrupt handlers registered by this function,
    /// we clear the interrupt status register for you. This is NOT the case
    /// if you register the interrupt handler directly, by defining a
    /// `#[no_mangle] unsafe extern "C" fn GPIO()` function.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::GPIO);
        }
        self.set_interrupt_priority(handler.priority());
        unsafe { interrupt::bind_interrupt(Interrupt::GPIO, user_gpio_interrupt_handler) };
        USER_INTERRUPT_HANDLER.store(handler.handler());
    }
}

#[ram]
extern "C" fn user_gpio_interrupt_handler() {
    handle_pin_interrupts(|| USER_INTERRUPT_HANDLER.call());
}

#[ram]
extern "C" fn default_gpio_interrupt_handler() {
    handle_pin_interrupts(|| ());
}

#[ram]
fn handle_pin_interrupts(user_handler: fn()) {
    let intrs_bank0 = InterruptStatusRegisterAccess::Bank0.interrupt_status_read();

    #[cfg(gpio_bank_1)]
    let intrs_bank1 = InterruptStatusRegisterAccess::Bank1.interrupt_status_read();

    user_handler();

    let banks = [
        (GpioRegisterAccess::Bank0, intrs_bank0),
        #[cfg(gpio_bank_1)]
        (GpioRegisterAccess::Bank1, intrs_bank1),
    ];

    for (bank, intrs) in banks {
        // Get the mask of active async pins and also unmark them in the same go.
        let async_pins = bank.async_operations().fetch_and(!intrs, Ordering::Relaxed);

        // Wake up the tasks
        let mut intr_bits = intrs & async_pins;
        while intr_bits != 0 {
            let pin_pos = intr_bits.trailing_zeros();
            intr_bits -= 1 << pin_pos;

            let pin_nr = pin_pos as u8 + bank.offset();

            crate::interrupt::free(|| {
                asynch::PIN_WAKERS[pin_nr as usize].wake();
                set_int_enable(pin_nr, Some(0), 0, false);
            });
        }

        bank.write_interrupt_status_clear(intrs);
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! if_output_pin {
    // Base case: not an Output pin, substitute the else branch
    ({ $($then:tt)* } else { $($else:tt)* }) => { $($else)* };

    // First is an Output pin, skip checking and substitute the then branch
    (Output $(, $other:ident)* { $($then:tt)* } else { $($else:tt)* }) => { $($then)* };

    // First is not an Output pin, check the rest
    ($not:ident $(, $other:ident)* { $($then:tt)* } else { $($else:tt)* }) => {
        $crate::if_output_pin!($($other),* { $($then)* } else { $($else)* })
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! if_rtcio_pin {
    // Base case: not an RtcIo pin, substitute the else branch
    ({ $($then:tt)* } else { $($else:tt)* }) => { $($else)* };

    // First is an RtcIo pin, skip checking and substitute the then branch
    (RtcIo $(, $other:ident)* { $($then:tt)* } else { $($else:tt)* }) => { $($then)* };
    (RtcIoInput $(, $other:ident)* { $($then:tt)* } else { $($else:tt)* }) => { $($then)* };

    // First is not an RtcIo pin, check the rest
    ($not:ident $(, $other:ident)* { $($then:tt)* } else { $($else:tt)* }) => {
        $crate::if_rtcio_pin!($($other),* { $($then)* } else { $($else)* })
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! io_type {
    (Input, $gpionum:literal) => {
        impl $crate::gpio::InputPin for $crate::gpio::GpioPin<$gpionum> {}
    };
    (Output, $gpionum:literal) => {
        impl $crate::gpio::OutputPin for $crate::gpio::GpioPin<$gpionum> {}
    };
    (Analog, $gpionum:literal) => {
        // FIXME: the implementation shouldn't be in the GPIO module
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
        #[cfg(any(doc, feature = "unstable"))]
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl $crate::gpio::AnalogPin for $crate::gpio::GpioPin<$gpionum> {
            /// Configures the pin for analog mode.
            fn set_analog(&self, _: $crate::private::Internal) {
                use $crate::peripherals::GPIO;

                $crate::gpio::io_mux_reg($gpionum).modify(|_, w| unsafe {
                    w.mcu_sel().bits(1);
                    w.fun_ie().clear_bit();
                    w.fun_wpu().clear_bit();
                    w.fun_wpd().clear_bit()
                });

                GPIO::regs()
                    .enable_w1tc()
                    .write(|w| unsafe { w.bits(1 << $gpionum) });
            }
        }
    };
    ($other:ident, $gpionum:literal) => {
        // TODO
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! gpio {
    (
        $(
            ($gpionum:literal, [$($type:tt),*]
                $(
                    ( $( $af_input_num:literal => $af_input_signal:ident )* )
                    ( $( $af_output_num:literal => $af_output_signal:ident )* )
                )?
            )
        )+
    ) => {
        paste::paste! {
            $(
                $(
                    $crate::io_type!($type, $gpionum);
                )*

                impl $crate::gpio::Pin for $crate::gpio::GpioPin<$gpionum> {
                    #[inline(always)]
                    fn number(&self) -> u8 {
                        $gpionum
                    }

                    fn output_signals(&self, _: $crate::private::Internal) -> &'static [($crate::gpio::AlternateFunction, $crate::gpio::OutputSignal)] {
                        &[
                            $(
                                $(
                                    (
                                        $crate::gpio::AlternateFunction::[< _ $af_output_num >],
                                        $crate::gpio::OutputSignal::$af_output_signal
                                    ),
                                )*
                            )?
                        ]
                    }

                    fn input_signals(&self, _: $crate::private::Internal) -> &'static [($crate::gpio::AlternateFunction, $crate::gpio::InputSignal)] {
                        &[
                            $(
                                $(
                                    (
                                        $crate::gpio::AlternateFunction::[< _ $af_input_num >],
                                        $crate::gpio::InputSignal::$af_input_signal
                                    ),
                                )*
                            )?
                        ]
                    }
                }

                impl From<$crate::gpio::GpioPin<$gpionum>> for $crate::gpio::AnyPin {
                    fn from(pin: $crate::gpio::GpioPin<$gpionum>) -> Self {
                        $crate::gpio::Pin::degrade(pin)
                    }
                }
            )+

            impl $crate::peripheral::Peripheral for $crate::gpio::AnyPin {
                type P = $crate::gpio::AnyPin;
                unsafe fn clone_unchecked(&self) ->  Self {
                    Self(self.0)
                }
            }

            impl $crate::gpio::AnyPin {
                /// Conjure a new GPIO pin out of thin air.
                ///
                /// # Safety
                ///
                /// The caller must ensure that only one instance of a pin is in use at one time.
                ///
                /// # Panics
                ///
                /// Panics if the pin with the given number does not exist.
                pub unsafe fn steal(pin: u8) ->  Self {
                    const PINS: &[u8] = &[$($gpionum),*];
                    assert!(PINS.contains(&pin), "Pin {} does not exist", pin);
                    Self(pin)
                }

                pub(crate) fn is_output(&self) -> bool {
                    match self.0 {
                        $(
                            $gpionum => $crate::if_output_pin!($($type),* { true } else { false }),
                        )+
                        _ => false,
                    }
                }
            }

            // These macros call the code block on the actually contained GPIO pin.

            #[doc(hidden)]
            macro_rules! handle_gpio_output {
                ($this:expr, $inner:ident, $code:tt) => {
                    match $this.number() {
                        $(
                            $gpionum => $crate::if_output_pin!($($type),* {{
                                #[allow(unused_mut)]
                                let mut $inner = unsafe { GpioPin::<$gpionum>::steal() };
                                $code
                            }} else {{
                                panic!("Unsupported")
                            }}),
                        )+
                        _ => unreachable!(),
                    }
                }
            }

            #[doc(hidden)]
            macro_rules! handle_gpio_input {
                ($this:expr, $inner:ident, $code:tt) => {
                    match $this.number() {
                        $(
                            $gpionum => {{
                                #[allow(unused_mut)]
                                let mut $inner = unsafe { GpioPin::<$gpionum>::steal() };
                                $code
                            }},
                        )+
                        _ => unreachable!(),
                    }
                }
            }

            pub(crate) use handle_gpio_output;
            pub(crate) use handle_gpio_input;

            cfg_if::cfg_if! {
                if #[cfg(any(lp_io, rtc_cntl))] {
                    #[doc(hidden)]
                    macro_rules! handle_rtcio {
                        ($this:expr, $inner:ident, $code:tt) => {
                            match $this.number() {
                                $(
                                    $gpionum => $crate::if_rtcio_pin!($($type),* {{
                                        #[allow(unused_mut)]
                                        let mut $inner = unsafe { GpioPin::<$gpionum>::steal() };
                                        $code
                                    }} else {{
                                        panic!("Unsupported")
                                    }}),
                                )+
                                _ => unreachable!(),
                            }
                        }
                    }

                    #[doc(hidden)]
                    macro_rules! handle_rtcio_with_resistors {
                        ($this:expr, $inner:ident, $code:tt) => {
                            match $this.number() {
                                $(
                                    $gpionum => $crate::if_rtcio_pin!($($type),* {
                                        $crate::if_output_pin!($($type),* {{
                                            #[allow(unused_mut)]
                                            let mut $inner = unsafe { GpioPin::<$gpionum>::steal() };
                                            $code
                                        }} else {{
                                            panic!("Unsupported")
                                        }})
                                    } else {{
                                        panic!("Unsupported")
                                    }}),
                                )+
                                _ => unreachable!(),
                            }
                        }
                    }

                    pub(crate) use handle_rtcio;
                    pub(crate) use handle_rtcio_with_resistors;
                }
            }
        }
    };
}

/// Output pin configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct OutputConfig {
    /// Initial output level of the pin.
    pub level: Level,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self { level: Level::Low }
    }
}

/// Push-pull digital output.
///
/// This driver configures the GPIO pin to be a push-pull output driver.
/// Push-pull means that the driver actively sets the output voltage level
/// for both high and low logical [`Level`]s.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl private::Sealed for Output<'_> {}

impl<'d> Peripheral for Output<'d> {
    type P = Flex<'d>;
    unsafe fn clone_unchecked(&self) -> Self::P {
        self.pin.clone_unchecked()
    }
}

impl<'d> Output<'d> {
    /// Creates a new GPIO output driver.
    ///
    /// The `initial_output` parameter sets the initial output level of the pin.
    ///
    /// ## Example
    ///
    /// The following example configures `GPIO5` to pulse a LED once. The
    /// example assumes that the LED is connected such that it is on when
    /// the pin is low.
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::gpio::{Level, Output, OutputConfig};
    /// use esp_hal::delay::Delay;
    ///
    /// fn blink_once(led: &mut Output<'_>, delay: &mut Delay) {
    ///     led.set_low();
    ///     delay.delay_millis(500);
    ///     led.set_high();
    /// }
    ///
    /// let config = OutputConfig::default().with_level(Level::High);
    /// let mut led = Output::new(peripherals.GPIO5, config).unwrap();
    /// let mut delay = Delay::new();
    ///
    /// blink_once(&mut led, &mut delay);
    /// # }
    /// ```
    #[inline]
    pub fn new(
        pin: impl Peripheral<P = impl OutputPin> + 'd,
        config: OutputConfig,
    ) -> Result<Self, ConfigError> {
        let mut pin = Flex::new(pin);

        pin.set_level(config.level);
        pin.set_as_output();

        Ok(Self { pin })
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    #[inline]
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
    }

    /// Change the configuration.
    pub fn apply_config(&mut self, config: &OutputConfig) -> Result<(), ConfigError> {
        self.set_level(config.level);
        Ok(())
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.set_level(Level::High)
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.set_level(Level::Low)
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// Configure the [DriveStrength] of the pin
    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength);
    }
}

/// Input pin configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct InputConfig {
    /// Initial pull of the pin.
    pub pull: Pull,
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

impl<'d> Peripheral for Input<'d> {
    type P = Flex<'d>;
    unsafe fn clone_unchecked(&self) -> Self::P {
        self.pin.clone_unchecked()
    }
}

impl<'d> Input<'d> {
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
    #[doc = crate::before_snippet!()]
    /// use esp_hal::gpio::{Level, Input, InputConfig, Pull};
    /// use esp_hal::delay::Delay;
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
    /// let mut button = Input::new(peripherals.GPIO5, config).unwrap();
    /// let mut delay = Delay::new();
    ///
    /// print_when_pressed(&mut button, &mut delay);
    /// # }
    /// ```
    #[inline]
    pub fn new(
        pin: impl Peripheral<P = impl InputPin> + 'd,
        config: InputConfig,
    ) -> Result<Self, ConfigError> {
        let mut pin = Flex::new(pin);

        pin.set_as_input(config.pull);

        Ok(Self { pin })
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    pub fn level(&self) -> Level {
        self.pin.level()
    }

    /// Change the configuration.
    pub fn apply_config(&mut self, config: &InputConfig) -> Result<(), ConfigError> {
        self.pin.set_as_input(config.pull);
        Ok(())
    }

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
    /// ## Example: print something when a button is pressed.
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::gpio::{Event, Input, InputConfig, Pull, Io};
    ///
    /// let mut io = Io::new(peripherals.IO_MUX);
    /// io.set_interrupt_handler(handler);
    ///
    /// // Set up the input and store it in the static variable.
    /// // This example uses a push button that is high when not
    /// // pressed and low when pressed.
    /// let config = InputConfig::default().with_pull(Pull::Up);
    /// let mut button = Input::new(peripherals.GPIO5, config).unwrap();
    ///
    /// critical_section::with(|cs| {
    ///     // Here we are listening for a low level to demonstrate
    ///     // that you need to stop listening for level interrupts,
    ///     // but usually you'd probably use `FallingEdge`.
    ///     button.listen(Event::LowLevel);
    ///     BUTTON.borrow_ref_mut(cs).replace(button);
    /// });
    /// # }
    ///
    /// // Outside of your `main` function:
    ///
    /// # use esp_hal::gpio::Input;
    /// use core::cell::RefCell;
    /// use critical_section::Mutex;
    ///
    /// // You will need to store the `Input` object in a static variable so
    /// // that the interrupt handler can access it.
    /// static BUTTON: Mutex<RefCell<Option<Input>>> =
    ///     Mutex::new(RefCell::new(None));
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
    #[instability::unstable]
    #[inline]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) -> Result<(), WakeConfigError> {
        self.pin.wakeup_enable(enable, event)
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    #[inline]
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
    }
}

/// Open-drain output pin configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[non_exhaustive]
pub struct OutputOpenDrainConfig {
    /// Initial output level of the pin.
    pub level: Level,
    /// Initial pull of the pin.
    pub pull: Pull,
}

impl Default for OutputOpenDrainConfig {
    fn default() -> Self {
        Self {
            level: Level::Low,
            pull: Pull::None,
        }
    }
}

/// Open drain digital output.
///
/// This driver configures the GPIO pin to be an open drain output driver.
/// Open drain means that the driver actively pulls the output voltage level low
/// for the low logical [`Level`], but leaves the high level floating, which is
/// then determined by external hardware, or internal pull-up/pull-down
/// resistors.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputOpenDrain<'d> {
    pin: Flex<'d>,
}

impl private::Sealed for OutputOpenDrain<'_> {}

impl<'d> Peripheral for OutputOpenDrain<'d> {
    type P = Flex<'d>;
    unsafe fn clone_unchecked(&self) -> Self::P {
        self.pin.clone_unchecked()
    }
}

impl<'d> OutputOpenDrain<'d> {
    /// Creates a new GPIO output driver.
    ///
    /// The `initial_output` parameter sets the initial output level of the pin.
    /// The `pull` parameter configures internal pull-up or pull-down
    /// resistors.
    ///
    /// ## Example
    ///
    /// The following example configures `GPIO5` to pulse a LED once. The
    /// example assumes that the LED is connected such that it is on when
    /// the pin is low.
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::gpio::{
    ///     Level, OutputOpenDrain, OutputOpenDrainConfig, Pull,
    /// };
    /// use esp_hal::delay::Delay;
    ///
    /// fn blink_once(led: &mut OutputOpenDrain<'_>, delay: &mut Delay) {
    ///     led.set_low();
    ///     delay.delay_millis(500);
    ///     led.set_high();
    /// }
    ///
    /// let config = OutputOpenDrainConfig::default()
    ///     .with_level(Level::High)
    ///     .with_pull(Pull::Up);
    /// let mut led = OutputOpenDrain::new(peripherals.GPIO5, config).unwrap();
    /// let mut delay = Delay::new();
    ///
    /// blink_once(&mut led, &mut delay);
    /// # }
    /// ```
    #[inline]
    pub fn new(
        pin: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        config: OutputOpenDrainConfig,
    ) -> Result<Self, ConfigError> {
        let mut pin = Flex::new(pin);

        pin.set_level(config.level);
        pin.set_as_open_drain(config.pull);

        Ok(Self { pin })
    }

    /// Change the configuration.
    pub fn apply_config(&mut self, config: &OutputOpenDrainConfig) -> Result<(), ConfigError> {
        self.set_level(config.level);
        self.pin.set_as_open_drain(config.pull);
        Ok(())
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    #[inline]
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    pub fn level(&self) -> Level {
        self.pin.level()
    }

    /// Listen for interrupts.
    ///
    /// See [`Input::listen`] for more information and an example.
    #[inline]
    #[instability::unstable]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }

    /// Stop listening for interrupts.
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

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.set_level(Level::High);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.set_level(Level::Low);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    /// Configure the [DriveStrength] of the pin
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength);
    }
}

/// Flexible pin driver.
///
/// This driver allows changing the pin mode between input and output.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Flex<'d> {
    pin: PeripheralRef<'d, AnyPin>,
}

impl private::Sealed for Flex<'_> {}

impl Peripheral for Flex<'_> {
    type P = Self;
    unsafe fn clone_unchecked(&self) -> Self::P {
        Self {
            pin: PeripheralRef::new(core::ptr::read(&*self.pin as *const AnyPin)),
        }
    }
}

impl<'d> Flex<'d> {
    /// Create flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    #[instability::unstable]
    pub fn new(pin: impl Peripheral<P = impl Into<AnyPin>> + 'd) -> Self {
        crate::into_mapped_ref!(pin);
        Self { pin }
    }

    fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        unsafe { AnyPin::steal(self.number()) }.split().0
    }

    /// Set the GPIO to input mode.
    #[inline]
    #[instability::unstable]
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin.init_input(pull);
        self.pin.enable_output(false);
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

    fn listen_with_options(
        &self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    ) -> Result<(), WakeConfigError> {
        if wake_up_from_light_sleep {
            match event {
                Event::AnyEdge | Event::RisingEdge | Event::FallingEdge => {
                    return Err(WakeConfigError::EdgeTriggeringNotSupported);
                }
                _ => {}
            }
        }

        set_int_enable(
            self.pin.number(),
            Some(gpio_intr_enable(int_enable, nmi_enable)),
            event as u8,
            wake_up_from_light_sleep,
        );

        Ok(())
    }

    /// Listen for interrupts.
    ///
    /// See [`Input::listen`] for more information and an example.
    #[inline]
    #[instability::unstable]
    pub fn listen(&mut self, event: Event) {
        // Unwrap can't fail currently as listen_with_options is only supposed to return
        // an error if wake_up_from_light_sleep is true.
        unwrap!(self.listen_with_options(event, true, false, false));
    }

    /// Stop listening for interrupts.
    #[inline]
    #[instability::unstable]
    pub fn unlisten(&mut self) {
        set_int_enable(self.pin.number(), Some(0), 0, false);
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
        GpioRegisterAccess::from(self.pin.number() as usize)
            .write_interrupt_status_clear(1 << (self.pin.number() % 32));
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    #[instability::unstable]
    pub fn is_interrupt_set(&self) -> bool {
        GpioRegisterAccess::from(self.pin.number() as usize).read_interrupt_status()
            & (1 << (self.pin.number() % 32))
            != 0
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    #[inline]
    #[instability::unstable]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) -> Result<(), WakeConfigError> {
        self.listen_with_options(event.into(), false, false, enable)
    }

    /// Set the GPIO to output mode.
    #[inline]
    #[instability::unstable]
    pub fn set_as_output(&mut self) {
        self.pin.set_to_push_pull_output();
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

    /// Configure the [DriveStrength] of the pin
    #[inline]
    #[instability::unstable]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength);
    }

    /// Set the GPIO to open-drain mode.
    #[inline]
    #[instability::unstable]
    pub fn set_as_open_drain(&mut self, pull: Pull) {
        self.pin.set_to_open_drain_output();
        self.pin.pull_direction(pull);
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    #[inline]
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        assert!(self.pin.is_output());
        unsafe { AnyPin::steal(self.number()) }.split()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.split().1
    }
}

impl private::Sealed for AnyPin {}

impl AnyPin {
    /// Init as input with the given pull-up/pull-down
    #[inline]
    pub(crate) fn init_input(&self, pull: Pull) {
        self.pull_direction(pull);

        #[cfg(usb_device)]
        disable_usb_pads(self.number());

        io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(GPIO_FUNCTION as u8);
            w.fun_ie().set_bit();
            w.slp_sel().clear_bit()
        });
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    #[inline]
    #[instability::unstable]
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        assert!(self.is_output());
        (
            interconnect::InputSignal::new(Self(self.0)),
            interconnect::OutputSignal::new(Self(self.0)),
        )
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
    pub(crate) fn enable_output(&self, enable: bool) {
        assert!(self.is_output() || !enable);
        GpioRegisterAccess::from(self.number() as usize).write_out_en(self.mask(), enable);
    }

    /// Enable input for the pin
    #[inline]
    pub(crate) fn enable_input(&self, on: bool) {
        io_mux_reg(self.number()).modify(|_, w| w.fun_ie().bit(on));
    }

    #[inline]
    pub(crate) fn pull_direction(&self, pull: Pull) {
        let pull_up = pull == Pull::Up;
        let pull_down = pull == Pull::Down;

        #[cfg(esp32)]
        crate::soc::gpio::errata36(Self(self.0), pull_up, pull_down);

        io_mux_reg(self.number()).modify(|_, w| {
            w.fun_wpd().bit(pull_down);
            w.fun_wpu().bit(pull_up)
        });
    }

    #[inline]
    fn mask(&self) -> u32 {
        1 << (self.number() % 32)
    }

    /// The current state of the input
    #[inline]
    pub(crate) fn is_input_high(&self) -> bool {
        GpioRegisterAccess::from(self.number() as usize).read_input() & self.mask() != 0
    }

    /// Set up as output
    #[inline]
    pub(crate) fn init_output(
        &mut self,
        alternate: AlternateFunction,
        open_drain: bool,
        input_enable: Option<bool>,
    ) {
        self.enable_output(true);

        let gpio = GPIO::regs();

        gpio.pin(self.number() as usize)
            .modify(|_, w| w.pad_driver().bit(open_drain));

        gpio.func_out_sel_cfg(self.number() as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        #[cfg(usb_device)]
        disable_usb_pads(self.number());

        io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(alternate as u8);
            if let Some(input_enable) = input_enable {
                w.fun_ie().bit(input_enable);
            }
            w.fun_drv().bits(DriveStrength::_20mA as u8);
            w.slp_sel().clear_bit()
        });
    }

    /// Configure open-drain mode
    #[inline]
    pub(crate) fn set_to_open_drain_output(&mut self) {
        self.init_output(GPIO_FUNCTION, true, Some(true));
    }

    /// Configure output mode
    #[inline]
    pub(crate) fn set_to_push_pull_output(&mut self) {
        self.init_output(GPIO_FUNCTION, false, None);
    }

    /// Set the pin's level to high or low
    #[inline]
    pub(crate) fn set_output_high(&mut self, high: bool) {
        GpioRegisterAccess::from(self.number() as usize).write_output(self.mask(), high);
    }

    /// Configure the [DriveStrength] of the pin
    #[inline]
    pub(crate) fn set_drive_strength(&mut self, strength: DriveStrength) {
        io_mux_reg(self.number()).modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
    }

    /// Enable/disable open-drain mode
    #[inline]
    pub(crate) fn enable_open_drain(&mut self, on: bool) {
        GPIO::regs()
            .pin(self.number() as usize)
            .modify(|_, w| w.pad_driver().bit(on));
    }

    /// Is the output set to high
    #[inline]
    pub(crate) fn is_set_high(&self) -> bool {
        GpioRegisterAccess::from(self.number() as usize).read_output() & self.mask() != 0
    }
}

impl Pin for AnyPin {
    #[inline(always)]
    fn number(&self) -> u8 {
        self.0
    }

    fn output_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, OutputSignal)] {
        handle_gpio_output!(self, target, {
            Pin::output_signals(&target, private::Internal)
        })
    }

    fn input_signals(&self, _: private::Internal) -> &'static [(AlternateFunction, InputSignal)] {
        handle_gpio_input!(self, target, {
            Pin::input_signals(&target, private::Internal)
        })
    }
}

impl InputPin for AnyPin {}
impl OutputPin for AnyPin {}

#[cfg(any(lp_io, rtc_cntl))]
impl RtcPin for AnyPin {
    #[cfg(xtensa)]
    #[allow(unused_braces)] // False positive :/
    fn rtc_number(&self) -> u8 {
        handle_rtcio!(self, target, { RtcPin::rtc_number(&target) })
    }

    #[cfg(any(xtensa, esp32c6))]
    fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: RtcFunction) {
        handle_rtcio!(self, target, {
            RtcPin::rtc_set_config(&mut target, input_enable, mux, func)
        })
    }

    fn rtcio_pad_hold(&mut self, enable: bool) {
        handle_rtcio!(self, target, {
            RtcPin::rtcio_pad_hold(&mut target, enable)
        })
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6))]
    unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
        handle_rtcio!(self, target, {
            RtcPin::apply_wakeup(&mut target, wakeup, level)
        })
    }
}

#[cfg(any(lp_io, rtc_cntl))]
impl RtcPinWithResistors for AnyPin {
    fn rtcio_pullup(&mut self, enable: bool) {
        handle_rtcio_with_resistors!(self, target, {
            RtcPinWithResistors::rtcio_pullup(&mut target, enable)
        })
    }

    fn rtcio_pulldown(&mut self, enable: bool) {
        handle_rtcio_with_resistors!(self, target, {
            RtcPinWithResistors::rtcio_pulldown(&mut target, enable)
        })
    }
}

/// Set GPIO event listening.
///
/// - `gpio_num`: the pin to configure
/// - `int_ena`: maskable and non-maskable CPU interrupt bits. None to leave
///   unchanged.
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

mod asynch {
    use core::{
        future::poll_fn,
        task::{Context, Poll},
    };

    use super::*;
    use crate::asynch::AtomicWaker;

    #[ram]
    pub(super) static PIN_WAKERS: [AtomicWaker; NUM_PINS] =
        [const { AtomicWaker::new() }; NUM_PINS];

    impl Flex<'_> {
        /// Wait until the pin experiences a particular [`Event`].
        ///
        /// The GPIO driver will disable listening for the event once it occurs,
        /// or if the `Future` is dropped.
        ///
        /// Note that calling this function will overwrite previous
        /// [`listen`][Self::listen] operations for this pin.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for(&mut self, event: Event) {
            let mut future = PinFuture {
                pin: unsafe { self.clone_unchecked() },
            };

            // Make sure this pin is not being processed by an interrupt handler.
            if future.pin.is_listening() {
                set_int_enable(
                    future.pin.number(),
                    None, // Do not disable handling pending interrupts.
                    0,    // Disable generating new events
                    false,
                );
                poll_fn(|cx| {
                    if future.pin.is_interrupt_set() {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                })
                .await;
            }

            // At this point the pin is no longer listening, we can safely
            // do our setup.

            // Mark pin as async.
            GpioRegisterAccess::from(future.pin.number() as usize)
                .async_operations()
                .fetch_or(future.pin_mask(), Ordering::Relaxed);

            future.pin.listen(event);

            future.await
        }

        /// Wait until the pin is high.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for_high(&mut self) {
            self.wait_for(Event::HighLevel).await
        }

        /// Wait until the pin is low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for_low(&mut self) {
            self.wait_for(Event::LowLevel).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for_rising_edge(&mut self) {
            self.wait_for(Event::RisingEdge).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for_falling_edge(&mut self) {
            self.wait_for(Event::FallingEdge).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        #[instability::unstable]
        pub async fn wait_for_any_edge(&mut self) {
            self.wait_for(Event::AnyEdge).await
        }
    }

    impl Input<'_> {
        /// Wait until the pin experiences a particular [`Event`].
        ///
        /// The GPIO driver will disable listening for the event once it occurs,
        /// or if the `Future` is dropped.
        ///
        /// Note that calling this function will overwrite previous
        /// [`listen`][Self::listen] operations for this pin.
        #[inline]
        pub async fn wait_for(&mut self, event: Event) {
            self.pin.wait_for(event).await
        }

        /// Wait until the pin is high.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        pub async fn wait_for_high(&mut self) {
            self.pin.wait_for_high().await
        }

        /// Wait until the pin is low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        pub async fn wait_for_low(&mut self) {
            self.pin.wait_for_low().await
        }

        /// Wait for the pin to undergo a transition from low to high.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        pub async fn wait_for_rising_edge(&mut self) {
            self.pin.wait_for_rising_edge().await
        }

        /// Wait for the pin to undergo a transition from high to low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        pub async fn wait_for_falling_edge(&mut self) {
            self.pin.wait_for_falling_edge().await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        ///
        /// See [Self::wait_for] for more information.
        #[inline]
        pub async fn wait_for_any_edge(&mut self) {
            self.pin.wait_for_any_edge().await
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    struct PinFuture<'d> {
        pin: Flex<'d>,
    }

    impl PinFuture<'_> {
        fn pin_mask(&self) -> u32 {
            let bank = GpioRegisterAccess::from(self.pin.number() as usize);
            1 << (self.pin.number() - bank.offset())
        }

        fn is_done(&self) -> bool {
            // Only the interrupt handler should clear the async bit, and only if the
            // specific pin is handling an interrupt.
            GpioRegisterAccess::from(self.pin.number() as usize)
                .async_operations()
                .load(Ordering::Acquire)
                & self.pin_mask()
                == 0
        }
    }

    impl core::future::Future for PinFuture<'_> {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PIN_WAKERS[self.pin.number() as usize].register(cx.waker());

            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for PinFuture<'_> {
        fn drop(&mut self) {
            // If the pin isn't listening, the future has either been dropped before setup,
            // or the interrupt has already been handled.
            if self.pin.is_listening() {
                // Make sure the future isn't dropped while the interrupt is being handled.
                // This prevents tricky drop-and-relisten scenarios.

                set_int_enable(
                    self.pin.number(),
                    None, // Do not disable handling pending interrupts.
                    0,    // Disable generating new events
                    false,
                );

                while self.pin.is_interrupt_set() {}

                // Unmark pin as async
                GpioRegisterAccess::from(self.pin.number() as usize)
                    .async_operations()
                    .fetch_and(!self.pin_mask(), Ordering::Relaxed);
            }
        }
    }
}

mod embedded_hal_impls {
    use embedded_hal::digital;

    use super::*;

    impl digital::ErrorType for Input<'_> {
        type Error = core::convert::Infallible;
    }

    impl digital::InputPin for Input<'_> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl digital::ErrorType for Output<'_> {
        type Error = core::convert::Infallible;
    }

    impl digital::OutputPin for Output<'_> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    impl digital::StatefulOutputPin for Output<'_> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl digital::InputPin for OutputOpenDrain<'_> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl digital::ErrorType for OutputOpenDrain<'_> {
        type Error = core::convert::Infallible;
    }

    impl digital::OutputPin for OutputOpenDrain<'_> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    impl digital::StatefulOutputPin for OutputOpenDrain<'_> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    #[instability::unstable]
    impl digital::InputPin for Flex<'_> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    #[instability::unstable]
    impl digital::ErrorType for Flex<'_> {
        type Error = core::convert::Infallible;
    }

    #[instability::unstable]
    impl digital::OutputPin for Flex<'_> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    #[instability::unstable]
    impl digital::StatefulOutputPin for Flex<'_> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }
}

mod embedded_hal_async_impls {
    use embedded_hal_async::digital::Wait;

    use super::*;

    #[instability::unstable]
    impl Wait for Flex<'_> {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_high(self).await;
            Ok(())
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_low(self).await;
            Ok(())
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_rising_edge(self).await;
            Ok(())
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_falling_edge(self).await;
            Ok(())
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_any_edge(self).await;
            Ok(())
        }
    }

    impl Wait for Input<'_> {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_high(self).await;
            Ok(())
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_low(self).await;
            Ok(())
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_rising_edge(self).await;
            Ok(())
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_falling_edge(self).await;
            Ok(())
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            Self::wait_for_any_edge(self).await;
            Ok(())
        }
    }
}
