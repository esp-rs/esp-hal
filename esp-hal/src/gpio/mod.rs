//! # General Purpose I/Os (GPIO)
//!
//! ## Overview
//!
//! Each pin can be used as a general-purpose I/O, or be connected to one or
//! more internal peripheral signals.
//!
//! ## Configuration
//!
//! This driver supports various operations on GPIO pins, including setting the
//! pin mode, direction, and manipulating the pin state (setting high/low,
//! toggling). It provides an interface to interact with GPIO pins on ESP chips,
//! allowing developers to control and read the state of the pins.
//!
//! ## Usage
//!
//! This module also implements a number of traits from [embedded-hal] to
//! provide a common interface for GPIO pins.
//!
//! To get access to the pins, you first need to convert them into a HAL
//! designed struct from the pac struct `GPIO` and `IO_MUX` using [`Io::new`].
//!
//! ### Pin Types
//!
//! - [Input] pins can be used as digital inputs.
//! - [Output] and [OutputOpenDrain] pins can be used as digital outputs.
//! - [Flex] pin is a pin that can be used as an input and output pin.
//! - [AnyPin] is a type-erased GPIO pin with support for inverted signalling.
//! - [NoPin] is a useful for cases where peripheral driver requires a pin, but
//!   real pin cannot be used.
//!
//! ### GPIO interconnect
//!
//! Each GPIO can be connected to one output signal and any number of input
//! signals. This allows connections inside of the MCU without allocating and
//! connecting multiple pins for loopback functionality.
//!
//! ## Examples
//!
//! ### Set up a GPIO as an Output
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::{Io, Level, Output};
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = Output::new(io.pins.gpio5, Level::High);
//! # }
//! ```
//! 
//! ### Blink an LED
//!
//! See the [Blinky] section of the crate documentation.
//!
//! ### Inverting a signal using `AnyPin`
//! See the [Inverting TX and RX Pins] example of the UART documentation.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [Blinky]: ../index.html#blinky
//! [Inverting TX and RX Pins]: ../uart/index.html#inverting-tx-and-rx-pins

use portable_atomic::{AtomicPtr, Ordering};
use procmacros::ram;

pub use crate::soc::gpio::*;
use crate::{
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{GPIO, IO_MUX},
    private::{self, Sealed},
    InterruptConfigurable,
};

pub mod interconnect;
mod placeholder;

pub use placeholder::NoPin;

#[cfg(soc_etm)]
pub mod etm;
#[cfg(lp_io)]
pub mod lp_io;
#[cfg(all(rtc_io, not(esp32)))]
pub mod rtc_io;

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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
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

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
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
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    /// Drive strength of approximately 5mA.
    I5mA  = 0,
    /// Drive strength of approximately 10mA.
    I10mA = 1,
    /// Drive strength of approximately 20mA.
    I20mA = 2,
    /// Drive strength of approximately 40mA.
    I40mA = 3,
}

/// Alternate functions
///
/// GPIO pins can be configured for various functions, such as GPIO
/// or being directly connected to a peripheral's signal like UART, SPI, etc.
/// The `AlternateFunction` enum allows to select one of several functions that
/// a pin can perform, rather than using it as a general-purpose input or
/// output.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlternateFunction {
    /// Alternate function 0.
    Function0 = 0,
    /// Alternate function 1.
    Function1 = 1,
    /// Alternate function 2.
    Function2 = 2,
    /// Alternate function 3.
    Function3 = 3,
    /// Alternate function 4.
    Function4 = 4,
    /// Alternate function 5.
    Function5 = 5,
}

impl TryFrom<usize> for AlternateFunction {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AlternateFunction::Function0),
            1 => Ok(AlternateFunction::Function1),
            2 => Ok(AlternateFunction::Function2),
            3 => Ok(AlternateFunction::Function3),
            4 => Ok(AlternateFunction::Function4),
            5 => Ok(AlternateFunction::Function5),
            _ => Err(()),
        }
    }
}

/// RTC function
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcFunction {
    /// RTC mode.
    Rtc     = 0,
    /// Digital mode.
    Digital = 1,
}

/// Trait implemented by RTC pins
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

    #[doc(hidden)]
    fn mask(&self) -> u32 {
        1 << (self.number() % 32)
    }

    /// Type-erase (degrade) this pin into an AnyPin.
    ///
    /// This converts pin singletons (`GpioPin<0>`, …), which are all different
    /// types, into the same type. It is useful for creating arrays of pins,
    /// or avoiding generics.
    fn degrade(self) -> AnyPin
    where
        Self: Sized,
    {
        self.degrade_pin(private::Internal)
    }

    #[doc(hidden)]
    fn degrade_pin(&self, _: private::Internal) -> AnyPin;

    /// Enable/disable sleep-mode
    #[doc(hidden)]
    fn sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.slp_sel().bit(on));
    }

    /// Configure the alternate function
    #[doc(hidden)]
    fn set_alternate_function(&mut self, alternate: AlternateFunction, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
    }

    /// Enable or disable the GPIO pin output buffer.
    #[doc(hidden)]
    fn enable_output(&self, enable: bool, _: private::Internal) {
        self.gpio_bank(private::Internal)
            .write_out_en(self.mask(), enable);
    }

    /// Enable input for the pin
    #[doc(hidden)]
    fn enable_input(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.fun_ie().bit(on));
    }

    #[doc(hidden)]
    fn output_signals(&self, _: private::Internal) -> &[(AlternateFunction, OutputSignal)];

    #[doc(hidden)]
    fn input_signals(&self, _: private::Internal) -> &[(AlternateFunction, InputSignal)];

    #[doc(hidden)]
    fn gpio_bank(&self, _: private::Internal) -> GpioRegisterAccess;

    #[doc(hidden)]
    fn pull_direction(&self, pull: Pull, _: private::Internal) {
        let pull_up = pull == Pull::Up;
        let pull_down = pull == Pull::Down;

        #[cfg(esp32)]
        crate::soc::gpio::errata36(self.degrade_pin(private::Internal), pull_up, pull_down);

        get_io_mux_reg(self.number()).modify(|_, w| {
            w.fun_wpd().bit(pull_down);
            w.fun_wpu().bit(pull_up)
        });
    }
}

/// Trait implemented by pins which can be used as inputs.
pub trait InputPin: Pin + Into<AnyPin> + 'static {
    /// Init as input with the given pull-up/pull-down
    #[doc(hidden)]
    fn init_input(&self, pull: Pull, _: private::Internal) {
        self.pull_direction(pull, private::Internal);

        #[cfg(usb_device)]
        disable_usb_pads(self.number());

        get_io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(GPIO_FUNCTION as u8);
            w.fun_ie().set_bit();
            w.slp_sel().clear_bit()
        });
    }

    /// Enable input in sleep mode for the pin
    #[doc(hidden)]
    fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.mcu_ie().bit(on));
    }

    /// The current state of the input
    #[doc(hidden)]
    fn is_input_high(&self, _: private::Internal) -> bool {
        self.gpio_bank(private::Internal).read_input() & self.mask() != 0
    }
}

/// Trait implemented by pins which can be used as outputs.
pub trait OutputPin: Pin + Into<AnyPin> + 'static {
    /// Set up as output
    #[doc(hidden)]
    fn init_output(
        &mut self,
        alternate: AlternateFunction,
        open_drain: bool,
        _: private::Internal,
    ) {
        self.enable_output(true, private::Internal);

        let gpio = unsafe { GPIO::steal() };

        gpio.pin(self.number() as usize)
            .modify(|_, w| w.pad_driver().bit(open_drain));

        gpio.func_out_sel_cfg(self.number() as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        #[cfg(any(esp32c3, esp32s3))]
        disable_usb_pads(self.number());

        get_io_mux_reg(self.number()).modify(|_, w| unsafe {
            w.mcu_sel().bits(alternate as u8);
            w.fun_ie().bit(open_drain);
            w.fun_drv().bits(DriveStrength::I20mA as u8);
            w.slp_sel().clear_bit()
        });
    }

    /// Configure open-drain mode
    #[doc(hidden)]
    fn set_to_open_drain_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, true, private::Internal);
    }

    /// Configure output mode
    #[doc(hidden)]
    fn set_to_push_pull_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, false, private::Internal);
    }

    /// Set the pin's level to high or low
    #[doc(hidden)]
    fn set_output_high(&mut self, high: bool, _: private::Internal) {
        self.gpio_bank(private::Internal)
            .write_output(self.mask(), high);
    }

    /// Configure the [DriveStrength] of the pin
    #[doc(hidden)]
    fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
    }

    /// Enable/disable open-drain mode
    #[doc(hidden)]
    fn enable_open_drain(&mut self, on: bool, _: private::Internal) {
        unsafe { GPIO::steal() }
            .pin(self.number() as usize)
            .modify(|_, w| w.pad_driver().bit(on));
    }

    /// Enable/disable output in sleep mode
    #[doc(hidden)]
    fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.mcu_oe().bit(on));
    }

    /// Configure internal pull-up resistor in sleep mode
    #[doc(hidden)]
    fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.mcu_wpu().bit(on));
    }

    /// Configure internal pull-down resistor in sleep mode
    #[doc(hidden)]
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(self.number()).modify(|_, w| w.mcu_wpd().bit(on));
    }

    /// Is the output set to high
    #[doc(hidden)]
    fn is_set_high(&self, _: private::Internal) -> bool {
        self.gpio_bank(private::Internal).read_output() & self.mask() != 0
    }
}

/// Trait implemented by pins which can be used as analog pins
pub trait AnalogPin: Pin {
    /// Configure the pin for analog operation
    #[doc(hidden)]
    fn set_analog(&self, _: private::Internal);
}

/// Trait implemented by pins which can be used as Touchpad pins
pub trait TouchPin: Pin {
    /// Configure the pin for analog operation
    #[doc(hidden)]
    fn set_touch(&self, _: private::Internal);

    /// Reads the pin's touch measurement register
    #[doc(hidden)]
    fn get_touch_measurement(&self, _: private::Internal) -> u16;

    /// Maps the pin nr to the touch pad nr
    #[doc(hidden)]
    fn get_touch_nr(&self, _: private::Internal) -> u8;

    /// Set a pins touch threshold for interrupts.
    #[doc(hidden)]
    fn set_threshold(&self, threshold: u16, _: private::Internal);
}

#[doc(hidden)]
#[derive(Clone, Copy)]
pub enum GpioRegisterAccess {
    Bank0,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    Bank1,
}

impl From<usize> for GpioRegisterAccess {
    fn from(_gpio_num: usize) -> Self {
        #[cfg(any(esp32, esp32s2, esp32s3))]
        if _gpio_num >= 32 {
            return GpioRegisterAccess::Bank1;
        }

        GpioRegisterAccess::Bank0
    }
}

impl GpioRegisterAccess {
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
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::write_out_en_clear(word),
        }
    }

    fn write_out_en_set(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_out_en_set(word),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::write_out_en_set(word),
        }
    }

    fn read_input(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_input(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::read_input(),
        }
    }

    fn read_output(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_output(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::read_output(),
        }
    }

    fn read_interrupt_status(self) -> u32 {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::read_interrupt_status(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::read_interrupt_status(),
        }
    }

    fn write_interrupt_status_clear(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_interrupt_status_clear(word),
            #[cfg(any(esp32, esp32s2, esp32s3))]
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
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::write_output_set(word),
        }
    }

    fn write_output_clear(self, word: u32) {
        match self {
            Self::Bank0 => Bank0GpioRegisterAccess::write_output_clear(word),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            Self::Bank1 => Bank1GpioRegisterAccess::write_output_clear(word),
        }
    }
}

struct Bank0GpioRegisterAccess;

impl Bank0GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        unsafe { GPIO::steal() }
            .enable_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { GPIO::steal() }
            .enable_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { GPIO::steal() }.in_().read().bits()
    }

    fn read_output() -> u32 {
        unsafe { GPIO::steal() }.out().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        unsafe { GPIO::steal() }.status().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { GPIO::steal() }
            .status_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { GPIO::steal() }
            .out_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { GPIO::steal() }
            .out_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
struct Bank1GpioRegisterAccess;

#[cfg(any(esp32, esp32s2, esp32s3))]
impl Bank1GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        unsafe { GPIO::steal() }
            .enable1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { GPIO::steal() }
            .enable1_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { GPIO::steal() }.in1().read().bits()
    }

    fn read_output() -> u32 {
        unsafe { GPIO::steal() }.out1().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        unsafe { GPIO::steal() }.status1().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { GPIO::steal() }
            .status1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { GPIO::steal() }
            .out1_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { GPIO::steal() }
            .out1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }
}

/// GPIO pin
pub struct GpioPin<const GPIONUM: u8>;

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
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        (
            interconnect::InputSignal::new(self.degrade_pin(private::Internal)),
            interconnect::OutputSignal::new(self.degrade_pin(private::Internal)),
        )
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
        unsafe { crate::peripherals::USB_DEVICE::steal() }
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

/// General Purpose Input/Output driver
pub struct Io {
    _io_mux: IO_MUX,
    /// The pins available on this chip
    pub pins: Pins,
}

impl Io {
    /// Initialize the I/O driver.
    pub fn new(gpio: GPIO, io_mux: IO_MUX) -> Self {
        Self::new_with_priority(gpio, io_mux, crate::interrupt::Priority::min())
    }

    /// Initialize the I/O driver with a interrupt priority.
    ///
    /// This decides the priority for the interrupt when using async.
    pub fn new_with_priority(
        mut gpio: GPIO,
        io_mux: IO_MUX,
        prio: crate::interrupt::Priority,
    ) -> Self {
        gpio.bind_gpio_interrupt(gpio_interrupt_handler);
        crate::interrupt::enable(crate::peripherals::Interrupt::GPIO, prio).unwrap();

        Self::new_no_bind_interrupt(gpio, io_mux)
    }

    /// Initialize the I/O driver without enabling the GPIO interrupt or
    /// binding an interrupt handler to it.
    ///
    /// *Note:* You probably don't want to use this, it is intended to be used
    /// in very specific use cases. Async GPIO functionality will not work
    /// when instantiating `Io` using this constructor.
    pub fn new_no_bind_interrupt(_gpio: GPIO, _io_mux: IO_MUX) -> Self {
        Io {
            _io_mux,
            pins: unsafe { Pins::steal() },
        }
    }
}

impl crate::private::Sealed for Io {}

impl InterruptConfigurable for Io {
    /// Install the given interrupt handler replacing any previously set
    /// handler.
    ///
    /// ⚠️ Be careful when using this together with the async API:
    ///
    /// - The async driver will disable any interrupts whose status is not
    ///   cleared by the user handler.
    /// - Clearing the interrupt status in the user handler will prevent the
    ///   async driver from detecting the interrupt, silently disabling the
    ///   corresponding pin's async API.
    /// - You will not be notified if you make a mistake.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        crate::interrupt::enable(crate::peripherals::Interrupt::GPIO, handler.priority()).unwrap();
        USER_INTERRUPT_HANDLER.store(handler.handler());
    }
}

#[ram]
extern "C" fn gpio_interrupt_handler() {
    USER_INTERRUPT_HANDLER.call();

    handle_pin_interrupts(on_pin_irq);
}

#[ram]
fn on_pin_irq(pin_nr: u8) {
    // FIXME: async handlers signal completion by disabling the interrupt, but this
    // conflicts with user handlers.
    set_int_enable(pin_nr, 0, 0, false);
    asynch::PIN_WAKERS[pin_nr as usize].wake(); // wake task
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
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
    };
    (Output, $gpionum:literal) => {
        impl $crate::gpio::OutputPin for GpioPin<$gpionum> {}
    };
    (Analog, $gpionum:literal) => {
        // FIXME: the implementation shouldn't be in the GPIO module
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
        impl $crate::gpio::AnalogPin for GpioPin<$gpionum> {
            /// Configures the pin for analog mode.
            fn set_analog(&self, _: $crate::private::Internal) {
                use $crate::peripherals::GPIO;

                get_io_mux_reg($gpionum).modify(|_, w| unsafe {
                    w.mcu_sel().bits(1);
                    w.fun_ie().clear_bit();
                    w.fun_wpu().clear_bit();
                    w.fun_wpd().clear_bit()
                });

                unsafe { GPIO::steal() }
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
            /// Pins available on this chip
            pub struct Pins {
                $(
                    #[doc = concat!("GPIO pin number ", $gpionum, ".")]
                    pub [< gpio $gpionum >] : GpioPin<$gpionum>,
                )+
            }

            impl Pins {
                /// Unsafely create GPIO pins.
                ///
                /// # Safety
                ///
                /// The caller must ensure that only one instance of a pin is in use at one time.
                pub unsafe fn steal() -> Self {
                    Self {
                        $(
                            [< gpio $gpionum >]: GpioPin::steal(),
                        )+
                    }
                }
            }

            $(
                $(
                    $crate::io_type!($type, $gpionum);
                )*

                impl $crate::gpio::Pin for GpioPin<$gpionum> {
                    fn number(&self) -> u8 {
                        $gpionum
                    }

                    fn degrade_pin(&self, _: $crate::private::Internal) -> AnyPin {
                        AnyPin($crate::gpio::AnyPinInner::[< Gpio $gpionum >](unsafe { Self::steal() }))
                    }

                    fn gpio_bank(&self, _: $crate::private::Internal) -> $crate::gpio::GpioRegisterAccess {
                        $crate::gpio::GpioRegisterAccess::from($gpionum)
                    }

                    fn output_signals(&self, _: $crate::private::Internal) -> &[(AlternateFunction, OutputSignal)] {
                        &[
                            $(
                                $(
                                    (AlternateFunction::[< Function $af_output_num >], OutputSignal::$af_output_signal ),
                                )*
                            )?
                        ]
                    }

                    fn input_signals(&self, _: $crate::private::Internal) -> &[(AlternateFunction, InputSignal)] {
                        &[
                            $(
                                $(
                                    (AlternateFunction::[< Function $af_input_num >], InputSignal::$af_input_signal ),
                                )*
                            )?
                        ]
                    }
                }

                impl From<GpioPin<$gpionum>> for AnyPin {
                    fn from(pin: GpioPin<$gpionum>) -> Self {
                        use $crate::gpio::Pin;
                        pin.degrade()
                    }
                }
            )+

            pub(crate) enum AnyPinInner {
                $(
                    [<Gpio $gpionum >](GpioPin<$gpionum>),
                )+
            }

            /// Type-erased GPIO pin
            pub struct AnyPin(pub(crate) AnyPinInner);

            impl $crate::peripheral::Peripheral for AnyPin {
                type P = AnyPin;
                unsafe fn clone_unchecked(&self) ->  Self {
                    match self.0 {
                        $(AnyPinInner::[<Gpio $gpionum >](_) => {
                            Self(AnyPinInner::[< Gpio $gpionum >](unsafe { GpioPin::steal() }))
                        })+
                    }
                }
            }

            // These macros call the code block on the actually contained GPIO pin.

            #[doc(hidden)]
            #[macro_export]
            macro_rules! handle_gpio_output {
                ($this:expr, $inner:ident, $code:tt) => {
                    match $this {
                        $(
                            AnyPinInner::[<Gpio $gpionum >]($inner) => $crate::if_output_pin!($($type),* {
                                $code
                            } else {{
                                let _ = $inner;
                                panic!("Unsupported")
                            }}),
                        )+
                    }
                }
            }

            #[doc(hidden)]
            #[macro_export]
            macro_rules! handle_gpio_input {
                ($this:expr, $inner:ident, $code:tt) => {
                    match $this {
                        $(
                            AnyPinInner::[<Gpio $gpionum >]($inner) => $code
                        )+
                    }
                }
            }

            pub(crate) use handle_gpio_output;
            pub(crate) use handle_gpio_input;

            cfg_if::cfg_if! {
                if #[cfg(any(lp_io, rtc_cntl))] {
                    #[doc(hidden)]
                    #[macro_export]
                    macro_rules! handle_rtcio {
                        ($this:expr, $inner:ident, $code:tt) => {
                            match $this {
                                $(
                                    AnyPinInner::[<Gpio $gpionum >]($inner) => $crate::if_rtcio_pin!($($type),* {
                                        $code
                                    } else {{
                                        let _ = $inner;
                                        panic!("Unsupported")
                                    }}),
                                )+
                            }
                        }
                    }

                    #[doc(hidden)]
                    #[macro_export]
                    macro_rules! handle_rtcio_with_resistors {
                        ($this:expr, $inner:ident, $code:tt) => {
                            match $this {
                                $(
                                    AnyPinInner::[<Gpio $gpionum >]($inner) => $crate::if_rtcio_pin!($($type),* {
                                        $crate::if_output_pin!($($type),* {
                                            $code
                                        } else {{
                                            let _ = $inner;
                                            panic!("Unsupported")
                                        }})
                                    } else {{
                                        let _ = $inner;
                                        panic!("Unsupported")
                                    }}),
                                )+
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

/// GPIO output driver.
pub struct Output<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for Output<'_, P> {}

impl<P> Peripheral for Output<'_, P> {
    type P = P;
    unsafe fn clone_unchecked(&self) -> P {
        self.pin.clone_unchecked()
    }
}

impl<'d> Output<'d> {
    /// Create GPIO open-drain output driver for a [Pin] with the provided
    /// initial output-level and [Pull] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl OutputPin> + 'd, initial_output: Level) -> Self {
        Self::new_typed(pin.map_into(), initial_output)
    }
}

impl<'d, P> Output<'d, P>
where
    P: OutputPin,
{
    /// Create GPIO output driver for a [GpioPin] with the provided level
    #[inline]
    pub fn new_typed(pin: impl Peripheral<P = P> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new_typed(pin);

        pin.set_level(initial_output);
        pin.set_as_output();

        Self { pin }
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
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
        self.get_output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.get_output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
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

/// GPIO input driver.
pub struct Input<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for Input<'_, P> {}

impl<P> Peripheral for Input<'_, P> {
    type P = P;
    unsafe fn clone_unchecked(&self) -> P {
        self.pin.clone_unchecked()
    }
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull]
    /// configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl InputPin> + 'd, pull: Pull) -> Self {
        Self::new_typed(pin.map_into(), pull)
    }
}

impl<'d, P> Input<'d, P>
where
    P: InputPin,
{
    /// Create GPIO input driver for a [Pin] with the provided [Pull]
    /// configuration.
    #[inline]
    pub fn new_typed(pin: impl Peripheral<P = P> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new_typed(pin);

        pin.set_as_input(pull);

        Self { pin }
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.get_level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.get_level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }

    /// Stop listening for interrupts
    #[inline]
    pub fn unlisten(&mut self) {
        self.pin.unlisten();
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt();
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.is_interrupt_set()
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    #[inline]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.pin.wakeup_enable(enable, event);
    }
}

impl<P> Input<'_, P>
where
    P: InputPin + OutputPin,
{
    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
    }
}

/// GPIO open-drain output driver.
pub struct OutputOpenDrain<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for OutputOpenDrain<'_, P> {}

impl<P> Peripheral for OutputOpenDrain<'_, P> {
    type P = P;
    unsafe fn clone_unchecked(&self) -> P {
        self.pin.clone_unchecked()
    }
}

impl<'d> OutputOpenDrain<'d> {
    /// Create GPIO open-drain output driver for a [Pin] with the provided
    /// initial output-level and [Pull] configuration.
    #[inline]
    pub fn new(
        pin: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        initial_output: Level,
        pull: Pull,
    ) -> Self {
        Self::new_typed(pin.map_into(), initial_output, pull)
    }
}

impl<'d, P> OutputOpenDrain<'d, P>
where
    P: InputPin + OutputPin,
{
    /// Create GPIO open-drain output driver for a [Pin] with the provided
    /// initial output-level and [Pull] configuration.
    #[inline]
    pub fn new_typed(pin: impl Peripheral<P = P> + 'd, initial_output: Level, pull: Pull) -> Self {
        let mut pin = Flex::new_typed(pin);

        pin.set_level(initial_output);
        pin.set_as_open_drain(pull);

        Self { pin }
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.split()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.pin.into_peripheral_output()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.get_level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.get_level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
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
        self.get_output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.get_output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
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
pub struct Flex<'d, P = AnyPin> {
    pin: PeripheralRef<'d, P>,
}

impl<P> private::Sealed for Flex<'_, P> {}

impl<P> Peripheral for Flex<'_, P> {
    type P = P;
    unsafe fn clone_unchecked(&self) -> P {
        core::ptr::read(&*self.pin as *const _)
    }
}

impl<'d> Flex<'d> {
    /// Create flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Into<AnyPin>> + 'd) -> Self {
        Self::new_typed(pin.map_into())
    }
}

impl<'d, P> Flex<'d, P>
where
    P: Pin,
{
    /// Create flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    pub fn new_typed(pin: impl Peripheral<P = P> + 'd) -> Self {
        crate::into_ref!(pin);
        Self { pin }
    }

    /// Split the pin into an input and output signal.
    ///
    /// Peripheral signals allow connecting peripherals together without using
    /// external hardware.
    pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
        self.pin.degrade_pin(private::Internal).split()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.degrade_pin(private::Internal).split().0
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        self.split().1
    }
}

impl<P> Flex<'_, P>
where
    P: InputPin,
{
    /// Set the GPIO to input mode.
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin.init_input(pull, private::Internal);
        self.pin.enable_output(false, private::Internal);
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.get_level() == Level::High
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.get_level() == Level::Low
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.is_input_high(private::Internal).into()
    }

    fn listen_with_options(
        &self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    ) {
        if wake_up_from_light_sleep {
            match event {
                Event::AnyEdge | Event::RisingEdge | Event::FallingEdge => {
                    panic!("Edge triggering is not supported for wake-up from light sleep");
                }
                _ => {}
            }
        }

        set_int_enable(
            self.pin.number(),
            gpio_intr_enable(int_enable, nmi_enable),
            event as u8,
            wake_up_from_light_sleep,
        )
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.listen_with_options(event, true, false, false)
    }

    /// Stop listening for interrupts
    pub fn unlisten(&mut self) {
        set_int_enable(self.pin.number(), 0, 0, false);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin
            .gpio_bank(private::Internal)
            .write_interrupt_status_clear(1 << (self.pin.number() % 32));
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin
            .gpio_bank(private::Internal)
            .read_interrupt_status()
            & 1 << (self.pin.number() % 32)
            != 0
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    #[inline]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.listen_with_options(event.into(), false, false, enable);
    }
}

impl<P> Flex<'_, P>
where
    P: OutputPin,
{
    /// Set the GPIO to output mode.
    pub fn set_as_output(&mut self) {
        self.pin.set_to_push_pull_output(private::Internal);
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
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.get_output_level() == Level::High
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.get_output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let level = !self.get_output_level();
        self.set_level(level);
    }

    /// Configure the [DriveStrength] of the pin
    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength, private::Internal);
    }
}

impl<P> Flex<'_, P>
where
    P: InputPin + OutputPin,
{
    /// Set the GPIO to open-drain mode.
    pub fn set_as_open_drain(&mut self, pull: Pull) {
        self.pin.set_to_open_drain_output(private::Internal);
        self.pin.pull_direction(pull, private::Internal);
    }
}

pub(crate) mod internal {
    use super::*;

    impl private::Sealed for AnyPin {}

    impl AnyPin {
        /// Split the pin into an input and output signal.
        ///
        /// Peripheral signals allow connecting peripherals together without
        /// using external hardware.
        #[inline]
        pub fn split(self) -> (interconnect::InputSignal, interconnect::OutputSignal) {
            handle_gpio_input!(self.0, target, { target.split() })
        }
    }

    impl Pin for AnyPin {
        fn number(&self) -> u8 {
            handle_gpio_input!(&self.0, target, { Pin::number(target) })
        }

        fn degrade_pin(&self, _: private::Internal) -> AnyPin {
            unsafe { self.clone_unchecked() }
        }

        fn sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                Pin::sleep_mode(target, on, private::Internal)
            })
        }

        fn set_alternate_function(&mut self, alternate: AlternateFunction, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                Pin::set_alternate_function(target, alternate, private::Internal)
            })
        }

        fn output_signals(&self, _: private::Internal) -> &[(AlternateFunction, OutputSignal)] {
            handle_gpio_output!(&self.0, target, {
                Pin::output_signals(target, private::Internal)
            })
        }

        fn input_signals(&self, _: private::Internal) -> &[(AlternateFunction, InputSignal)] {
            handle_gpio_input!(&self.0, target, {
                Pin::input_signals(target, private::Internal)
            })
        }

        fn gpio_bank(&self, _: private::Internal) -> GpioRegisterAccess {
            handle_gpio_input!(&self.0, target, {
                Pin::gpio_bank(target, private::Internal)
            })
        }
    }

    impl InputPin for AnyPin {}

    // Need to forward these one by one because not all pins support all functions.
    impl OutputPin for AnyPin {
        fn init_output(
            &mut self,
            alternate: AlternateFunction,
            open_drain: bool,
            _: private::Internal,
        ) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::init_output(target, alternate, open_drain, private::Internal)
            })
        }

        fn set_to_open_drain_output(&mut self, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::set_to_open_drain_output(target, private::Internal)
            })
        }

        fn set_to_push_pull_output(&mut self, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::set_to_push_pull_output(target, private::Internal)
            })
        }

        fn set_output_high(&mut self, high: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::set_output_high(target, high, private::Internal)
            })
        }

        fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::set_drive_strength(target, strength, private::Internal)
            })
        }

        fn enable_open_drain(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::enable_open_drain(target, on, private::Internal)
            })
        }

        fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::enable_output_in_sleep_mode(target, on, private::Internal)
            })
        }

        fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::internal_pull_up_in_sleep_mode(target, on, private::Internal)
            })
        }

        fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                OutputPin::internal_pull_down_in_sleep_mode(target, on, private::Internal)
            })
        }

        fn is_set_high(&self, _: private::Internal) -> bool {
            handle_gpio_output!(&self.0, target, {
                OutputPin::is_set_high(target, private::Internal)
            })
        }
    }

    #[cfg(any(xtensa, esp32c2, esp32c3, esp32c6))]
    impl RtcPin for AnyPin {
        #[cfg(xtensa)]
        #[allow(unused_braces)] // False positive :/
        fn rtc_number(&self) -> u8 {
            handle_rtcio!(&self.0, target, { RtcPin::rtc_number(target) })
        }

        #[cfg(any(xtensa, esp32c6))]
        fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: RtcFunction) {
            handle_rtcio!(&mut self.0, target, {
                RtcPin::rtc_set_config(target, input_enable, mux, func)
            })
        }

        fn rtcio_pad_hold(&mut self, enable: bool) {
            handle_rtcio!(&mut self.0, target, {
                RtcPin::rtcio_pad_hold(target, enable)
            })
        }

        #[cfg(any(esp32c2, esp32c3, esp32c6))]
        unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
            handle_rtcio!(&mut self.0, target, {
                RtcPin::apply_wakeup(target, wakeup, level)
            })
        }
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, xtensa))]
    impl RtcPinWithResistors for AnyPin {
        fn rtcio_pullup(&mut self, enable: bool) {
            handle_rtcio_with_resistors!(&mut self.0, target, {
                RtcPinWithResistors::rtcio_pullup(target, enable)
            })
        }

        fn rtcio_pulldown(&mut self, enable: bool) {
            handle_rtcio_with_resistors!(&mut self.0, target, {
                RtcPinWithResistors::rtcio_pulldown(target, enable)
            })
        }
    }
}

fn is_listening(pin_num: u8) -> bool {
    let bits = unsafe { GPIO::steal() }
        .pin(pin_num as usize)
        .read()
        .int_ena()
        .bits();
    bits != 0
}

fn set_int_enable(gpio_num: u8, int_ena: u8, int_type: u8, wake_up_from_light_sleep: bool) {
    unsafe { GPIO::steal() }
        .pin(gpio_num as usize)
        .modify(|_, w| unsafe {
            w.int_ena().bits(int_ena);
            w.int_type().bits(int_type);
            w.wakeup_enable().bit(wake_up_from_light_sleep)
        });
}

#[ram]
fn handle_pin_interrupts(handle: impl Fn(u8)) {
    let intrs_bank0 = InterruptStatusRegisterAccess::Bank0.interrupt_status_read();

    #[cfg(any(esp32, esp32s2, esp32s3))]
    let intrs_bank1 = InterruptStatusRegisterAccess::Bank1.interrupt_status_read();

    let mut intr_bits = intrs_bank0;
    while intr_bits != 0 {
        let pin_nr = intr_bits.trailing_zeros();
        handle(pin_nr as u8);
        intr_bits -= 1 << pin_nr;
    }

    // clear interrupt bits
    Bank0GpioRegisterAccess::write_interrupt_status_clear(intrs_bank0);

    #[cfg(any(esp32, esp32s2, esp32s3))]
    {
        let mut intr_bits = intrs_bank1;
        while intr_bits != 0 {
            let pin_nr = intr_bits.trailing_zeros();
            handle(pin_nr as u8 + 32);
            intr_bits -= 1 << pin_nr;
        }
        Bank1GpioRegisterAccess::write_interrupt_status_clear(intrs_bank1);
    }
}

mod asynch {
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    #[ram]
    pub(super) static PIN_WAKERS: [AtomicWaker; NUM_PINS] =
        [const { AtomicWaker::new() }; NUM_PINS];

    impl<P> Flex<'_, P>
    where
        P: InputPin,
    {
        async fn wait_for(&mut self, event: Event) {
            self.listen(event);
            PinFuture::new(self.pin.number()).await
        }

        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            self.wait_for(Event::HighLevel).await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            self.wait_for(Event::LowLevel).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            self.wait_for(Event::RisingEdge).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            self.wait_for(Event::FallingEdge).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            self.wait_for(Event::AnyEdge).await
        }
    }

    impl<P> Input<'_, P>
    where
        P: InputPin,
    {
        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            self.pin.wait_for_high().await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            self.pin.wait_for_low().await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            self.pin.wait_for_rising_edge().await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            self.pin.wait_for_falling_edge().await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            self.pin.wait_for_any_edge().await
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    struct PinFuture {
        pin_num: u8,
    }

    impl PinFuture {
        fn new(pin_num: u8) -> Self {
            Self { pin_num }
        }
    }

    impl core::future::Future for PinFuture {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PIN_WAKERS[self.pin_num as usize].register(cx.waker());

            // if pin is no longer listening its been triggered
            // therefore the future has resolved
            if !is_listening(self.pin_num) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}

mod embedded_hal_02_impls {
    use embedded_hal_02::digital::v2 as digital;

    use super::*;

    impl<P> digital::InputPin for Input<'_, P>
    where
        P: InputPin,
    {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_high())
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_low())
        }
    }

    impl<P> digital::OutputPin for Output<'_, P>
    where
        P: OutputPin,
    {
        type Error = core::convert::Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.pin.set_high();
            Ok(())
        }
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.pin.set_low();
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for Output<'_, P>
    where
        P: OutputPin,
    {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<P> digital::ToggleableOutputPin for Output<'_, P>
    where
        P: OutputPin,
    {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<P> digital::InputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_high())
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_low())
        }
    }

    impl<P> digital::OutputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<P> digital::ToggleableOutputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<P> digital::InputPin for Flex<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<P> digital::OutputPin for Flex<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.pin.set_output_high(true, private::Internal);
            Ok(())
        }
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.pin.set_output_high(false, private::Internal);
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for Flex<'_, P>
    where
        P: InputPin + OutputPin,
    {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<P> digital::ToggleableOutputPin for Flex<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }
}

mod embedded_hal_impls {
    use embedded_hal::digital;

    use super::*;

    impl<P> digital::ErrorType for Input<'_, P>
    where
        P: InputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<P> digital::InputPin for Input<'_, P>
    where
        P: InputPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<P> digital::ErrorType for Output<'_, P>
    where
        P: OutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<P> digital::OutputPin for Output<'_, P>
    where
        P: OutputPin,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for Output<'_, P>
    where
        P: OutputPin,
    {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl<P> digital::InputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<P> digital::ErrorType for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<P> digital::OutputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for OutputOpenDrain<'_, P>
    where
        P: InputPin + OutputPin,
    {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl<P> digital::InputPin for Flex<'_, P>
    where
        P: InputPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<P> digital::ErrorType for Flex<'_, P> {
        type Error = core::convert::Infallible;
    }

    impl<P> digital::OutputPin for Flex<'_, P>
    where
        P: OutputPin,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Self::set_low(self);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Self::set_high(self);
            Ok(())
        }
    }

    impl<P> digital::StatefulOutputPin for Flex<'_, P>
    where
        P: OutputPin,
    {
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

    impl<P> Wait for Flex<'_, P>
    where
        P: InputPin,
    {
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

    impl<P> Wait for Input<'_, P>
    where
        P: InputPin,
    {
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
