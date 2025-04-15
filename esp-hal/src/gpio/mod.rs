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

use core::fmt::Display;

use portable_atomic::{AtomicU32, Ordering};
use procmacros::ram;
use strum::EnumCount;

#[cfg(any(lp_io, rtc_cntl))]
use crate::peripherals::{handle_rtcio, handle_rtcio_with_resistors};
pub use crate::soc::gpio::*;
use crate::{
    interrupt::{self, DEFAULT_INTERRUPT_HANDLER, InterruptHandler, Priority},
    peripherals::{GPIO, IO_MUX, Interrupt, handle_gpio_input, handle_gpio_output},
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

mod user_irq {
    use portable_atomic::{AtomicPtr, Ordering};
    use procmacros::ram;

    /// Convenience constant for `Option::None` pin
    pub(super) static USER_INTERRUPT_HANDLER: CFnPtr = CFnPtr::new();

    pub(super) struct CFnPtr(AtomicPtr<()>);
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

    #[ram]
    pub(super) extern "C" fn user_gpio_interrupt_handler() {
        super::handle_pin_interrupts(|| USER_INTERRUPT_HANDLER.call());
    }
}

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
#[cfg(any(lp_io, rtc_cntl))]
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

    /// Type-erase this pin into an [`AnyPin`].
    ///
    /// This function converts pin singletons (`GPIO0<'_>`, …), which are all
    /// different types, into the same type. It is useful for creating
    /// arrays of pins, or avoiding generics.
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
    ///         Level::High,
    ///         OutputConfig::default(),
    ///     );
    ///     let mut blue = Output::new(
    ///         blue,
    ///         Level::Low,
    ///         OutputConfig::default(),
    ///     );
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
    /// # Ok(())
    /// # }
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
pub trait InputPin: Pin {}

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
    #[cfg(gpio_bank_1)]
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
            #[cfg(gpio_bank_1)]
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
            #[cfg(gpio_bank_1)]
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
            #[cfg(gpio_bank_1)]
            Self::_1 => GPIO::regs()
                .enable1_w1ts()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    fn read_input(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().in_().read().bits(),
            #[cfg(gpio_bank_1)]
            Self::_1 => GPIO::regs().in1().read().bits(),
        }
    }

    fn read_output(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().out().read().bits(),
            #[cfg(gpio_bank_1)]
            Self::_1 => GPIO::regs().out1().read().bits(),
        }
    }

    fn read_interrupt_status(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().status().read().bits(),
            #[cfg(gpio_bank_1)]
            Self::_1 => GPIO::regs().status1().read().bits(),
        }
    }

    fn write_interrupt_status_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .status_w1tc()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_bank_1)]
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
            #[cfg(gpio_bank_1)]
            Self::_1 => GPIO::regs().out1_w1ts().write(|w| unsafe { w.bits(word) }),
        };
    }

    fn write_output_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs().out_w1tc().write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_bank_1)]
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
    if interrupt::bound_cpu_interrupt_for(crate::system::Cpu::current(), Interrupt::GPIO).is_some()
    {
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
        unwrap!(interrupt::enable(Interrupt::GPIO, prio));
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
    /// Note that when using interrupt handlers registered by this function,
    /// we clear the interrupt status register for you. This is NOT the case
    /// if you register the interrupt handler directly, by defining a
    /// `#[no_mangle] unsafe extern "C" fn GPIO()` function.
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
            interrupt::bind_interrupt(Interrupt::GPIO, user_irq::user_gpio_interrupt_handler)
        };
        user_irq::USER_INTERRUPT_HANDLER.store(handler.handler());
    }
}

impl crate::private::Sealed for Io<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Io<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
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
        (GpioBank::_0, intrs_bank0),
        #[cfg(gpio_bank_1)]
        (GpioBank::_1, intrs_bank1),
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
        impl $crate::gpio::InputPin for paste::paste!( [<GPIO $gpionum>]<'_> ) {}
    };
    (Output, $gpionum:literal) => {
        impl $crate::gpio::OutputPin for paste::paste!( [<GPIO $gpionum>]<'_> ) {}
    };
    (Analog, $gpionum:literal) => {
        // FIXME: the implementation shouldn't be in the GPIO module
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
        #[cfg(any(doc, feature = "unstable"))]
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl $crate::gpio::AnalogPin for paste::paste!( [<GPIO $gpionum>]<'_> ) {
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
                impl<'d> [< GPIO $gpionum >]<'d> {
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
                    #[doc = $crate::before_snippet!()]
                    /// let (rx, tx) = unsafe { peripherals.GPIO2.split() };
                    /// // rx and tx can then be passed to different peripherals to connect them.
                    /// # Ok(())
                    /// # }
                    /// ```
                    #[instability::unstable]
                    pub unsafe fn split(self) -> ($crate::gpio::interconnect::InputSignal<'d>, $crate::gpio::interconnect::OutputSignal<'d>) {
                        use $crate::gpio::Pin;
                        // FIXME: we should implement this in the gpio macro for output pins, but we
                        // should also have an input-only alternative for pins that can't be used as
                        // outputs.
                        unsafe { self.degrade().split() }
                    }
                }

                $(
                    $crate::io_type!($type, $gpionum);
                )*

                impl $crate::gpio::Pin for [<GPIO $gpionum>]<'_> {
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

                impl<'lt> From<[<GPIO $gpionum>]<'lt>> for $crate::gpio::AnyPin<'lt> {
                    fn from(pin: [<GPIO $gpionum>]<'lt>) -> Self {
                        $crate::gpio::Pin::degrade(pin)
                    }
                }
            )+

            impl $crate::gpio::AnyPin<'_> {
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
                    Self { pin, _lifetime: core::marker::PhantomData }
                }

                /// Unsafely clone the pin.
                ///
                /// # Safety
                ///
                /// Ensure that only one instance of a pin is in use at one time.
                pub unsafe fn clone_unchecked(&self) -> Self {
                    Self {
                        pin: self.pin,
                        _lifetime: core::marker::PhantomData,
                    }
                }

                /// Create a new AnyPin object that is limited to the lifetime of the
                /// passed reference.
                pub fn reborrow(&mut self) -> $crate::gpio::AnyPin<'_> {
                    unsafe { self.clone_unchecked() }
                }

                pub(crate) fn is_output(&self) -> bool {
                    match self.pin {
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
                                let mut $inner = unsafe { $crate::peripherals::[<GPIO $gpionum>]::steal() };
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
                                let mut $inner = unsafe { $crate::peripherals::[<GPIO $gpionum>]::steal() };
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
                                        #[allow(unused_unsafe)]
                                        let mut $inner = unsafe { $crate::peripherals::[<GPIO $gpionum>]::steal() };
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
                                            let mut $inner = unsafe { $crate::peripherals::[<GPIO $gpionum>]::steal() };
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
    /// let config = OutputConfig::default();
    /// let mut led = Output::new(peripherals.GPIO5, Level::High, config);
    /// let mut delay = Delay::new();
    ///
    /// blink_once(&mut led, &mut delay);
    /// # Ok(())
    /// # }
    /// ```
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
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
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::{Output, OutputConfig, Level};
    /// # let config = OutputConfig::default();
    /// let pin1_gpio = Output::new(peripherals.GPIO1, Level::High, config);
    /// let output = pin1_gpio.into_peripheral_output();
    /// # Ok(())
    /// # }
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal<'d> {
        self.pin.into_peripheral_output()
    }

    /// Change the configuration.
    #[inline]
    pub fn apply_config(&mut self, config: &OutputConfig) {
        self.pin.apply_output_config(config)
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

    /// Returns whether the pin is set to high level.
    ///
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.output_level() == Level::High
    }

    /// Returns whether the pin is set to low level.
    ///
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.output_level() == Level::Low
    }

    /// Returns which level the pin is set to.
    ///
    /// This function reads back the value set using `set_level`, `set_high` or
    /// `set_low`. It does not need the input stage to be enabled.
    #[inline]
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }

    /// Toggles the pin output.
    ///
    /// If the pin was previously set to high, it will be set to low, and vice
    /// versa.
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
    /// let mut button = Input::new(peripherals.GPIO5, config);
    /// let mut delay = Delay::new();
    ///
    /// print_when_pressed(&mut button, &mut delay);
    /// # Ok(())
    /// # }
    /// ```
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
    #[inline]
    pub fn new(pin: impl InputPin + 'd, config: InputConfig) -> Self {
        let mut pin = Flex::new(pin);

        pin.set_output_enable(false);
        pin.set_input_enable(true);
        pin.apply_input_config(&config);

        Self { pin }
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    ///
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::{Input, InputConfig, Pull};
    /// let config = InputConfig::default().with_pull(Pull::Up);
    /// let pin1_gpio = Input::new(peripherals.GPIO1, config);
    /// let pin1 = pin1_gpio.peripheral_input();
    /// # Ok(())
    /// # }
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal<'d> {
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
    pub fn apply_config(&mut self, config: &InputConfig) {
        self.pin.apply_input_config(config)
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
    /// ## Examples
    ///
    /// ### Print something when a button is pressed.
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
    /// let mut button = Input::new(peripherals.GPIO5, config);
    ///
    /// critical_section::with(|cs| {
    ///     // Here we are listening for a low level to demonstrate
    ///     // that you need to stop listening for level interrupts,
    ///     // but usually you'd probably use `FallingEdge`.
    ///     button.listen(Event::LowLevel);
    ///     BUTTON.borrow_ref_mut(cs).replace(button);
    /// });
    /// # Ok(())
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

        pin.init_gpio();

        Self { pin }
    }

    fn number(&self) -> u8 {
        self.pin.number()
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
        self.listen_with_options(event.into(), false, false, enable)
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

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    ///
    /// Note that the signal returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::Flex;
    /// let pin1_gpio = Flex::new(peripherals.GPIO1);
    /// // Can be passed as an input.
    /// let pin1 = pin1_gpio.peripheral_input();
    /// // You can keep using the Flex, as well as connect the pin to a
    /// // peripheral input.
    /// # Ok(())
    /// # }
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn peripheral_input(&self) -> interconnect::InputSignal<'d> {
        let mut input = unsafe {
            // Safety: the signal is frozen by this function.
            self.pin.clone_unchecked().split_no_init().0
        };
        input.frozen = true;
        input
    }

    /// Split the pin into an input and output signal pair.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    ///
    /// Note that the signals returned by this function is
    /// [frozen](interconnect::InputSignal::freeze).
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::Flex;
    /// let pin1 = Flex::new(peripherals.GPIO1);
    /// let (input, output) = pin1.split();
    /// # Ok(())
    /// # }
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
        let input = Input {
            pin: Flex {
                pin: unsafe { self.pin.clone_unchecked() },
            },
        };
        let output = Output { pin: self };

        (input, output)
    }

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
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::Flex;
    /// let pin1_gpio = Flex::new(peripherals.GPIO1);
    /// // Can be passed as an output.
    /// let pin1 = pin1_gpio.into_peripheral_output();
    /// # Ok(())
    /// # }
    /// ```
    #[inline]
    #[instability::unstable]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal<'d> {
        let mut output = unsafe {
            // Safety: the signals are frozen by this function.
            self.pin.split_no_init().1
        };
        output.frozen = true;
        output
    }
}

impl private::Sealed for AnyPin<'_> {}

impl<'lt> AnyPin<'lt> {
    fn bank(&self) -> GpioBank {
        #[cfg(gpio_bank_1)]
        if self.number() >= 32 {
            return GpioBank::_1;
        }

        GpioBank::_0
    }

    #[inline]
    pub(crate) fn init_gpio(&self) {
        #[cfg(usb_device)]
        disable_usb_pads(self.number());

        self.set_output_enable(false);

        GPIO::regs()
            .func_out_sel_cfg(self.number() as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        // Use RMW to not overwrite sleep configuration
        io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(GPIO_FUNCTION as u8);
            w.fun_ie().clear_bit();
            w.slp_sel().clear_bit()
        });
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
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
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::gpio::{AnyPin, Pin};
    /// let pin1 = peripherals.GPIO1.degrade();
    /// let (input, output) = unsafe { pin1.split() };
    /// # Ok(())
    /// # }
    /// ```
    #[inline]
    #[instability::unstable]
    pub unsafe fn split(
        self,
    ) -> (
        interconnect::InputSignal<'lt>,
        interconnect::OutputSignal<'lt>,
    ) {
        self.init_gpio();
        unsafe { self.split_no_init() }
    }

    unsafe fn split_no_init(
        self,
    ) -> (
        interconnect::InputSignal<'lt>,
        interconnect::OutputSignal<'lt>,
    ) {
        let input = unsafe { self.clone_unchecked().into_input_signal() };
        let mut output = self.into_output_signal();

        // We don't know if the input signal(s) will support bypassing the GPIO matrix.
        // Since the bypass option is common between input and output halves of
        // a single GPIO, we can't assume anything about the output, either.
        output.force_gpio_matrix = true;

        (input, output)
    }

    /// Convert the pin into an input signal.
    ///
    /// Peripheral signals allow connecting peripherals together without
    /// using external hardware.
    ///
    /// # Safety
    ///
    /// The caller must ensure that peripheral drivers don't configure the same
    /// GPIO at the same time in multiple places. This includes clones of the
    /// `InputSignal` struct.
    #[inline]
    #[instability::unstable]
    pub unsafe fn into_input_signal(self) -> interconnect::InputSignal<'lt> {
        let mut input = interconnect::InputSignal::new(Self {
            pin: self.pin,
            _lifetime: self._lifetime,
        });

        // Since InputSignal can be cloned, we have no way of knowing how many signals
        // end up being configured, and in what order. If multiple signals are
        // passed to peripherals, and one of them would allow GPIO alternate
        // function configurations, it would mean that the GPIO MCU_SEL bit's
        // final value would depend on the order of operations.
        input.force_gpio_matrix = true;

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

        // AnyPin is used as output only, we can allow bypassing the GPIO matrix.
        interconnect::OutputSignal::new(Self {
            pin: self.pin,
            _lifetime: self._lifetime,
        })
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

        let gpio = GPIO::regs();

        gpio.pin(self.number() as usize).modify(|_, w| {
            w.pad_driver()
                .bit(config.drive_mode == DriveMode::OpenDrain)
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

impl InputPin for AnyPin<'_> {}
impl OutputPin for AnyPin<'_> {}

#[cfg(any(lp_io, rtc_cntl))]
impl RtcPin for AnyPin<'_> {
    #[cfg(xtensa)]
    #[allow(unused_braces, reason = "False positive")]
    fn rtc_number(&self) -> u8 {
        handle_rtcio!(self, target, { RtcPin::rtc_number(&target) })
    }

    #[cfg(any(xtensa, esp32c6))]
    fn rtc_set_config(&self, input_enable: bool, mux: bool, func: RtcFunction) {
        handle_rtcio!(self, target, {
            RtcPin::rtc_set_config(&target, input_enable, mux, func)
        })
    }

    #[allow(unused_braces, reason = "False positive")]
    fn rtcio_pad_hold(&self, enable: bool) {
        handle_rtcio!(self, target, { RtcPin::rtcio_pad_hold(&target, enable) })
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6))]
    unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
        unsafe {
            handle_rtcio!(self, target, {
                RtcPin::apply_wakeup(&target, wakeup, level)
            })
        }
    }
}

#[cfg(any(lp_io, rtc_cntl))]
impl RtcPinWithResistors for AnyPin<'_> {
    fn rtcio_pullup(&self, enable: bool) {
        handle_rtcio_with_resistors!(self, target, {
            RtcPinWithResistors::rtcio_pullup(&target, enable)
        })
    }

    fn rtcio_pulldown(&self, enable: bool) {
        handle_rtcio_with_resistors!(self, target, {
            RtcPinWithResistors::rtcio_pulldown(&target, enable)
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
            // We construct the Future first, because its `Drop` implementation
            // is load-bearing if `wait_for` is dropped during the initialization.
            let future = PinFuture { pin: self };

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
            future
                .bank()
                .async_operations()
                .fetch_or(future.mask(), Ordering::Relaxed);

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
    struct PinFuture<'f, 'd> {
        pin: &'f mut Flex<'d>,
    }

    impl PinFuture<'_, '_> {
        fn number(&self) -> u8 {
            self.pin.number()
        }

        fn bank(&self) -> GpioBank {
            self.pin.pin.bank()
        }

        fn mask(&self) -> u32 {
            self.pin.pin.mask()
        }

        fn is_done(&self) -> bool {
            // Only the interrupt handler should clear the async bit, and only if the
            // specific pin is handling an interrupt.
            self.bank().async_operations().load(Ordering::Acquire) & self.mask() == 0
        }
    }

    impl core::future::Future for PinFuture<'_, '_> {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PIN_WAKERS[self.number() as usize].register(cx.waker());

            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for PinFuture<'_, '_> {
        fn drop(&mut self) {
            // If the pin isn't listening, the future has either been dropped before setup,
            // or the interrupt has already been handled.
            if self.pin.is_listening() {
                // Make sure the future isn't dropped while the interrupt is being handled.
                // This prevents tricky drop-and-relisten scenarios.

                set_int_enable(
                    self.number(),
                    None, // Do not disable handling pending interrupts.
                    0,    // Disable generating new events
                    false,
                );

                while self.pin.is_interrupt_set() {}

                // Unmark pin as async
                self.bank()
                    .async_operations()
                    .fetch_and(!self.mask(), Ordering::Relaxed);
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
