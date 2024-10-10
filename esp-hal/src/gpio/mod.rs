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

#[cfg(any(adc, dac))]
pub(crate) use crate::analog;
pub(crate) use crate::gpio;
#[cfg(any(xtensa, esp32c3, esp32c2))]
pub(crate) use crate::rtc_pins;
pub use crate::soc::gpio::*;
#[cfg(touch)]
pub(crate) use crate::touch;
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
#[derive(PartialEq)]
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
#[derive(PartialEq)]
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
    fn output_enable(&self, enable: bool, _: private::Internal) {
        let bank = self.gpio_bank(private::Internal);
        let mask = 1 << (self.number() % 32);
        if enable {
            bank.write_out_en_set(mask);
        } else {
            bank.write_out_en_clear(mask);
        }
    }

    #[doc(hidden)]
    fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6];

    #[doc(hidden)]
    fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6];

    #[doc(hidden)]
    fn gpio_bank(&self, _: private::Internal) -> GpioRegisterAccess;
}

/// Common trait implemented by signals which can be used as peripheral inputs
/// or outputs.
pub trait PeripheralSignal: Sealed {
    /// Configure the pullup and pulldown resistors
    #[doc(hidden)]
    fn pull_direction(&self, pull: Pull, _: private::Internal);
}

/// Trait implemented by pins which can be used as peripheral inputs.
pub trait PeripheralInput: PeripheralSignal {
    /// Init as input with the given pull-up/pull-down
    #[doc(hidden)]
    fn init_input(&self, pull: Pull, _: private::Internal);

    /// Enable input for the pin
    #[doc(hidden)]
    fn enable_input(&mut self, on: bool, _: private::Internal);

    /// Enable input in sleep mode for the pin
    #[doc(hidden)]
    fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// The current state of the input
    #[doc(hidden)]
    fn is_input_high(&self, _: private::Internal) -> bool;

    /// Returns the list of input signals that can be connected to this pin
    /// using IO_MUX.
    #[doc(hidden)]
    fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6];

    /// Connect the pin to a peripheral.
    #[doc(hidden)]
    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal);

    /// Disconnect the pin from a peripheral.
    #[doc(hidden)]
    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal);
}

/// Trait implemented by pins which can be used as peripheral outputs.
pub trait PeripheralOutput: PeripheralSignal {
    /// Configure open-drain mode
    #[doc(hidden)]
    fn set_to_open_drain_output(&mut self, _: private::Internal);

    /// Configure output mode
    #[doc(hidden)]
    fn set_to_push_pull_output(&mut self, _: private::Internal);

    /// Enable/disable the pin as output
    #[doc(hidden)]
    fn enable_output(&mut self, on: bool, _: private::Internal);

    /// Set the pin's level to high or low
    #[doc(hidden)]
    fn set_output_high(&mut self, on: bool, _: private::Internal);

    /// Configure the [DriveStrength] of the pin
    #[doc(hidden)]
    fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal);

    /// Enable/disable open-drain mode
    #[doc(hidden)]
    fn enable_open_drain(&mut self, on: bool, _: private::Internal);

    /// Enable/disable output in sleep mode
    #[doc(hidden)]
    fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Configure internal pull-up resistor in sleep mode
    #[doc(hidden)]
    fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Configure internal pull-down resistor in sleep mode
    #[doc(hidden)]
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Is the output set to high
    #[doc(hidden)]
    fn is_set_high(&self, _: private::Internal) -> bool;

    /// Returns the list of output signals that can be connected to this pin
    /// using IO_MUX.
    #[doc(hidden)]
    fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6];

    /// Connect the pin to a peripheral.
    #[doc(hidden)]
    fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal);

    /// Disconnect the pin from a peripheral.
    #[doc(hidden)]
    fn disconnect_from_peripheral_output(&mut self, signal: OutputSignal, _: private::Internal);
}

/// Trait implemented by pins which can be used as inputs.
pub trait InputPin: Pin + PeripheralInput + Into<AnyPin> + 'static {
    /// Listen for interrupts
    #[doc(hidden)]
    fn listen(&mut self, event: Event, _: private::Internal) {
        self.listen_with_options(event, true, false, false, private::Internal)
    }

    /// Checks if listening for interrupts is enabled for this Pin
    #[doc(hidden)]
    fn is_listening(&self, _: private::Internal) -> bool {
        is_listening(self.number())
    }

    /// Listen for interrupts
    #[doc(hidden)]
    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
        _: private::Internal,
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
            self.number(),
            gpio_intr_enable(int_enable, nmi_enable),
            event as u8,
            wake_up_from_light_sleep,
        )
    }

    /// Stop listening for interrupts
    #[doc(hidden)]
    fn unlisten(&mut self, _: private::Internal) {
        unsafe {
            (*GPIO::PTR)
                .pin(self.number() as usize)
                .modify(|_, w| w.int_ena().bits(0).int_type().bits(0).int_ena().bits(0));
        }
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[doc(hidden)]
    fn is_interrupt_set(&self, _: private::Internal) -> bool {
        self.gpio_bank(private::Internal).read_interrupt_status() & 1 << (self.number() % 32) != 0
    }

    /// Clear the interrupt status bit for this Pin
    #[doc(hidden)]
    fn clear_interrupt(&mut self, _: private::Internal) {
        self.gpio_bank(private::Internal)
            .write_interrupt_status_clear(1 << (self.number() % 32));
    }

    /// Enable this pin as a wake up source
    #[doc(hidden)]
    fn wakeup_enable(&mut self, enable: bool, event: WakeEvent, _: private::Internal) {
        self.listen_with_options(event.into(), false, false, enable, private::Internal);
    }
}

/// Trait implemented by pins which can be used as outputs.
pub trait OutputPin: Pin + PeripheralOutput + Into<AnyPin> + 'static {}

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

impl GpioRegisterAccess {
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
        unsafe { &*GPIO::PTR }
            .enable_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { &*GPIO::PTR }.in_().read().bits()
    }

    fn read_output() -> u32 {
        unsafe { &*GPIO::PTR }.out().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        unsafe { &*GPIO::PTR }.status().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .status_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .out_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .out_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
struct Bank1GpioRegisterAccess;

#[cfg(any(esp32, esp32s2, esp32s3))]
impl Bank1GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable1_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { &*GPIO::PTR }.in1().read().bits()
    }

    fn read_output() -> u32 {
        unsafe { &*GPIO::PTR }.out1().read().bits()
    }

    fn read_interrupt_status() -> u32 {
        unsafe { &*GPIO::PTR }.status1().read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .status1_w1tc()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .out1_w1ts()
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { &*GPIO::PTR }
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
    pub(crate) fn new() -> Self {
        Self
    }

    /// Create a pin out of thin air.
    ///
    /// # Safety
    ///
    /// Ensure that only one instance of a pin exists at one time.
    pub unsafe fn steal() -> Self {
        Self::new()
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        interconnect::InputSignal::new(self.degrade_pin(private::Internal))
    }
}

/// Workaround to make D+ and D- work on the ESP32-C3 and ESP32-S3, which by
/// default are assigned to the `USB_SERIAL_JTAG` peripheral.
#[cfg(any(esp32c3, esp32s3))]
fn disable_usb_pads(gpionum: u8) {
    cfg_if::cfg_if! {
        if #[cfg(esp32c3)] {
            let pins = [18, 19];
        } else if #[cfg(esp32s3)] {
            let pins = [19, 20];
        }
    }

    if pins.contains(&gpionum) {
        unsafe { &*crate::peripherals::USB_DEVICE::PTR }
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

impl<const GPIONUM: u8> PeripheralSignal for GpioPin<GPIONUM>
where
    Self: Pin,
{
    fn pull_direction(&self, pull: Pull, _: private::Internal) {
        let pull_up = pull == Pull::Up;
        let pull_down = pull == Pull::Down;

        #[cfg(esp32)]
        crate::soc::gpio::errata36(self.degrade_pin(private::Internal), pull_up, pull_down);

        get_io_mux_reg(GPIONUM).modify(|_, w| {
            w.fun_wpd().bit(pull_down);
            w.fun_wpu().bit(pull_up)
        });
    }
}

impl<const GPIONUM: u8> PeripheralInput for GpioPin<GPIONUM>
where
    Self: Pin,
{
    fn init_input(&self, pull: Pull, _: private::Internal) {
        self.pull_direction(pull, private::Internal);

        #[cfg(any(esp32c3, esp32s3))]
        disable_usb_pads(GPIONUM);

        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe {
            w.mcu_sel().bits(GPIO_FUNCTION as u8);
            w.fun_ie().set_bit();
            w.slp_sel().clear_bit()
        });
    }

    fn enable_input(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_ie().bit(on));
    }

    fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_ie().bit(on));
    }

    fn is_input_high(&self, _: private::Internal) -> bool {
        self.gpio_bank(private::Internal).read_input() & (1 << (GPIONUM % 32)) != 0
    }

    fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6] {
        <Self as Pin>::input_signals(self, private::Internal)
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        self.peripheral_input()
            .connect_input_to_peripheral(signal, private::Internal);
    }

    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        self.peripheral_input()
            .disconnect_input_from_peripheral(signal, private::Internal);
    }
}

impl<const GPIONUM: u8> PeripheralOutput for GpioPin<GPIONUM>
where
    Self: OutputPin + Pin,
{
    fn set_to_open_drain_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, true);
    }

    fn set_to_push_pull_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, false);
    }

    fn enable_output(&mut self, on: bool, _: private::Internal) {
        self.output_enable(on, private::Internal);
    }

    fn set_output_high(&mut self, high: bool, _: private::Internal) {
        let bank = self.gpio_bank(private::Internal);
        let mask = 1 << (GPIONUM % 32);
        if high {
            bank.write_output_set(mask);
        } else {
            bank.write_output_clear(mask);
        }
    }

    fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
    }

    fn enable_open_drain(&mut self, on: bool, _: private::Internal) {
        unsafe { &*GPIO::PTR }
            .pin(GPIONUM as usize)
            .modify(|_, w| w.pad_driver().bit(on));
    }

    fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_wpu().bit(on));
    }

    fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_wpd().bit(on));
    }

    fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_oe().bit(on));
    }

    fn is_set_high(&self, _: private::Internal) -> bool {
        self.gpio_bank(private::Internal).read_output() & (1 << (GPIONUM % 32)) != 0
    }

    fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6] {
        <Self as Pin>::output_signals(self, private::Internal)
    }

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal) {
        self.degrade_pin(private::Internal)
            .connect_peripheral_to_output(signal, private::Internal);
    }

    fn disconnect_from_peripheral_output(&mut self, signal: OutputSignal, _: private::Internal) {
        self.degrade_pin(private::Internal)
            .disconnect_from_peripheral_output(signal, private::Internal);
    }
}

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: OutputPin,
{
    fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
        self.output_enable(true, private::Internal);

        let gpio = unsafe { &*GPIO::PTR };

        gpio.pin(GPIONUM as usize)
            .modify(|_, w| w.pad_driver().bit(open_drain));

        gpio.func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        #[cfg(any(esp32c3, esp32s3))]
        disable_usb_pads(GPIONUM);

        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe {
            w.mcu_sel().bits(alternate as u8);
            w.fun_ie().bit(open_drain);
            w.fun_drv().bits(DriveStrength::I20mA as u8);
            w.slp_sel().clear_bit()
        });
    }

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        interconnect::OutputSignal::new(self.degrade_pin(private::Internal))
    }
}

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
    pub fn new_no_bind_interrupt(gpio: GPIO, _io_mux: IO_MUX) -> Self {
        Io {
            _io_mux,
            pins: gpio.pins(),
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
    (InputOutputAnalog, { $($then:tt)* } else { $($else:tt)* } ) => { $($then)* };
    (InputOutputAnalogTouch, { $($then:tt)* } else { $($else:tt)* } ) => { $($then)* };
    (InputOutput, { $($then:tt)* } else { $($else:tt)* } ) => { $($then)* };
    ($other:ident, { $($then:tt)* } else { $($else:tt)* } ) => { $($else)* };
}
pub(crate) use if_output_pin;

#[doc(hidden)]
#[macro_export]
macro_rules! io_types {
    (InputOnly, $gpionum:literal) => {
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
    };
    (InputOnlyAnalog, $gpionum:literal) => {
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
    };
    (InputOutput, $gpionum:literal) => {
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
        impl $crate::gpio::OutputPin for GpioPin<$gpionum> {}
    };
    (InputOutputAnalog, $gpionum:literal) => {
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
        impl $crate::gpio::OutputPin for GpioPin<$gpionum> {}
    };
    (InputOutputAnalogTouch, $gpionum:literal) => {
        impl $crate::gpio::InputPin for GpioPin<$gpionum> {}
        impl $crate::gpio::OutputPin for GpioPin<$gpionum> {}
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! gpio {
    (
        $(
            ($gpionum:literal, $bank:literal, $type:ident
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

            impl GPIO {
                pub(crate) fn pins(self) -> Pins {
                    Pins {
                        $(
                            [< gpio $gpionum >]: GpioPin::new(),
                        )+
                    }
                }
            }

            $(
                $crate::io_types!($type, $gpionum);

                impl $crate::gpio::Pin for GpioPin<$gpionum> {
                    fn number(&self) -> u8 {
                        $gpionum
                    }

                    fn degrade_pin(&self, _: $crate::private::Internal) -> AnyPin {
                        AnyPin($crate::gpio::AnyPinInner::[< Gpio $gpionum >](unsafe { Self::steal() }))
                    }

                    fn gpio_bank(&self, _: $crate::private::Internal) -> $crate::gpio::GpioRegisterAccess {
                        $crate::gpio::GpioRegisterAccess::[<Bank $bank >]
                    }

                    fn output_signals(&self, _: $crate::private::Internal) -> [Option<OutputSignal>; 6]{
                        #[allow(unused_mut)]
                        let mut output_signals = [None; 6];

                        $(
                            $(
                                output_signals[ $af_output_num ] = Some( OutputSignal::$af_output_signal );
                            )*
                        )?

                        output_signals
                    }
                    fn input_signals(&self, _: $crate::private::Internal) -> [Option<InputSignal>; 6] {
                        #[allow(unused_mut)]
                        let mut input_signals = [None; 6];

                        $(
                            $(
                                input_signals[ $af_input_num ] = Some( InputSignal::$af_input_signal );
                            )*
                        )?

                        input_signals
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
                            AnyPinInner::[<Gpio $gpionum >]($inner) => if_output_pin!($type, {
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
        }
    };
}

#[cfg(xtensa)]
#[doc(hidden)]
#[macro_export]
macro_rules! rtc_pins {
    ( @ignore $rue:literal ) => {};

    (
        $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:literal)?
    ) => {
        impl $crate::gpio::RtcPin for GpioPin<$pin_num>
        {
            fn rtc_number(&self) -> u8 {
                $rtc_pin
            }

            /// Set the RTC properties of the pin. If `mux` is true then then pin is
            /// routed to RTC, when false it is routed to IO_MUX.
            fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                use $crate::peripherals::RTC_IO;

                let rtcio = unsafe{ &*RTC_IO::ptr() };

                $crate::gpio::enable_iomux_clk_gate();

                // disable input
                paste::paste!{
                    rtcio.$pin_reg.modify(|_,w| unsafe {
                        w.[<$prefix fun_ie>]().bit(input_enable);
                        w.[<$prefix mux_sel>]().bit(mux);
                        w.[<$prefix fun_sel>]().bits(func as u8)
                    });
                }
            }

            fn rtcio_pad_hold(&mut self, enable: bool) {
                let rtc_ctrl = unsafe { &*$crate::peripherals::LPWR::PTR };

                cfg_if::cfg_if! {
                    if #[cfg(esp32)] {
                        let pad_hold = rtc_ctrl.hold_force();
                    } else {
                        let pad_hold = rtc_ctrl.pad_hold();
                    }
                };

                pad_hold.modify(|_, w| w.$hold().bit(enable));
            }
        }

        $(
            // FIXME: replace with $(ignore($rue)) once stable
            $crate::rtc_pins!(@ignore $rue);
            impl $crate::gpio::RtcPinWithResistors for GpioPin<$pin_num>
            {
                fn rtcio_pullup(&mut self, enable: bool) {
                    let rtcio = unsafe { &*$crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.[< $prefix rue >]().bit([< enable >]));
                    }
                }

                fn rtcio_pulldown(&mut self, enable: bool) {
                    let rtcio = unsafe { &*$crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.[< $prefix rde >]().bit([< enable >]));
                    }
                }
            }
        )?
    };

    (
        $( ( $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:literal )? ) )+
    ) => {
        $(
            $crate::gpio::rtc_pins!($pin_num, $rtc_pin, $pin_reg, $prefix, $hold $(, $rue )?);
        )+

        #[doc(hidden)]
        #[macro_export]
        macro_rules! handle_rtcio {
            ($this:expr, $inner:ident, $code:tt) => {
                match $this {
                    $(
                        paste::paste! { AnyPinInner::[<Gpio $pin_num >]($inner) } => {
                            $code
                        },
                    )+

                    _ => panic!("Unsupported")
                }
            }
        }

        #[doc(hidden)]
        #[macro_export]
        macro_rules! handle_rtcio_with_resistors {
            (@ignore $a:tt) => {};
            ($this:expr, $inner:ident, $code:tt) => {
                match $this {
                    $(
                        $(
                            paste::paste! { AnyPinInner::[<Gpio $pin_num >]($inner) } => {
                                // FIXME: replace with $(ignore($rue)) once stable
                                handle_rtcio_with_resistors!(@ignore $rue);
                                $code
                            },
                        )?
                    )+

                    _ => panic!("Unsupported")
                }
            }
        }
        pub(crate) use handle_rtcio;
        pub(crate) use handle_rtcio_with_resistors;
    };
}

#[cfg(any(esp32c2, esp32c3))]
#[doc(hidden)]
#[macro_export]
macro_rules! rtc_pins {
    ( $pin_num:expr ) => {
        impl $crate::gpio::RtcPin for GpioPin<$pin_num> {
            unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
                let rtc_cntl = unsafe { &*$crate::peripherals::RTC_CNTL::ptr() };
                cfg_if::cfg_if! {
                    if #[cfg(esp32c2)] {
                        let gpio_wakeup = rtc_cntl.cntl_gpio_wakeup();
                    } else {
                        let gpio_wakeup = rtc_cntl.gpio_wakeup();
                    }
                }

                paste::paste! {
                    gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _wakeup_enable >]().bit(wakeup));
                    gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _int_type >]().bits(level));
                }
            }

            fn rtcio_pad_hold(&mut self, enable: bool) {
                let rtc_cntl = unsafe { &*$crate::peripherals::RTC_CNTL::ptr() };
                paste::paste! {
                    rtc_cntl.pad_hold().modify(|_, w| w.[< gpio_pin $pin_num _hold >]().bit(enable));
                }
            }
        }

        impl $crate::gpio::RtcPinWithResistors for GpioPin<$pin_num> {
            fn rtcio_pullup(&mut self, enable: bool) {
                let io_mux = unsafe { &*$crate::peripherals::IO_MUX::ptr() };
                io_mux.gpio($pin_num).modify(|_, w| w.fun_wpu().bit(enable));
            }

            fn rtcio_pulldown(&mut self, enable: bool) {
                let io_mux = unsafe { &*$crate::peripherals::IO_MUX::ptr() };
                io_mux.gpio($pin_num).modify(|_, w| w.fun_wpd().bit(enable));
            }
        }
    };

    ( $( $pin_num:expr )+ ) => {
        $( $crate::gpio::rtc_pins!($pin_num); )+

        #[doc(hidden)]
        #[macro_export]
        macro_rules! handle_rtcio {
            ($this:expr, $inner:ident, $code:tt) => {
                match $this {
                    $(
                        paste::paste! { AnyPinInner::[<Gpio $pin_num >]($inner) } => {
                            $code
                        },
                    )+

                    _ => panic!("Unsupported")
                }
            }
        }

        pub(crate) use handle_rtcio;
        pub(crate) use handle_rtcio as handle_rtcio_with_resistors;
    };
}

#[doc(hidden)]
pub fn enable_iomux_clk_gate() {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            let sensors = unsafe { &*crate::peripherals::SENS::ptr() };
            sensors
                .sar_io_mux_conf()
                .modify(|_, w| w.iomux_clk_gate_en().set_bit());
        } else if #[cfg(esp32s3)] {
            let sensors = unsafe { &*crate::peripherals::SENS::ptr() };
            sensors
                .sar_peri_clk_gate_conf()
                .modify(|_,w| w.iomux_clk_en().set_bit());
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
#[doc(hidden)]
#[macro_export]
macro_rules! analog {
    (@ignore $rue:literal) => {};
    (
        $(
            (
                $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat $(, $rue:literal)?
            )
        )+
    ) => {
        $(
            #[cfg(any(adc, dac))]
            impl $crate::gpio::AnalogPin for GpioPin<$pin_num> {
                /// Configures the pin for analog mode.
                fn set_analog(&self, _: $crate::private::Internal) {
                    let rtcio = unsafe{ &*$crate::peripherals::RTC_IO::ptr() };

                    #[cfg(esp32s2)]
                    $crate::gpio::enable_iomux_clk_gate();

                    // We need `paste` (and a [< >] in it) to rewrite the token stream to
                    // handle indexed pins.
                    paste::paste! {
                        // disable input
                        rtcio.$pin_reg.modify(|_,w| w.[<$prefix fun_ie>]().bit(false));

                        // disable output
                        rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });

                        // disable open drain
                        rtcio.pin($rtc_pin).modify(|_,w| w.pad_driver().bit(false));

                        rtcio.$pin_reg.modify(|_,w| {
                            w.[<$prefix fun_ie>]().clear_bit();

                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.[<$prefix mux_sel>]().set_bit();

                            // Select function "RTC function 1" (GPIO) for analog use
                            unsafe { w.[<$prefix fun_sel>]().bits(0b00) };

                            // Disable pull-up and pull-down resistors on the pin, if it has them
                            $(
                                // FIXME: replace with $(ignore($rue)) once stable
                                $crate::analog!( @ignore $rue );
                                w.[<$prefix rue>]().bit(false);
                                w.[<$prefix rde>]().bit(false);
                            )?

                            w
                        });
                    }
                }
            }
        )+
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
#[doc(hidden)]
#[macro_export]
macro_rules! analog {
    (
        $($pin_num:literal)+
    ) => {
        $(
            #[cfg(any(adc, dac))]
            impl $crate::gpio::AnalogPin for GpioPin<$pin_num> {
                /// Configures the pin for analog mode.
                fn set_analog(&self, _: $crate::private::Internal) {
                    use $crate::peripherals::{GPIO};

                    get_io_mux_reg($pin_num).modify(|_,w| unsafe {
                        w.mcu_sel().bits(1);
                        w.fun_ie().clear_bit();
                        w.fun_wpu().clear_bit();
                        w.fun_wpd().clear_bit()
                    });

                    unsafe{ &*GPIO::PTR }.enable_w1tc().write(|w| unsafe { w.bits(1 << $pin_num) });
                }
            }
        )+
    }
}

/// Common functionality for all touch pads
#[doc(hidden)]
#[macro_export]
macro_rules! touch {
    (@pin_specific $touch_num:expr, true) => {
        paste::paste! {
            unsafe { &*RTC_IO::ptr() }.[< touch_pad $touch_num >]().write(|w| unsafe {
                w.xpd().set_bit();
                // clear input_enable
                w.fun_ie().clear_bit();
                // Connect pin to analog / RTC module instead of standard GPIO
                w.mux_sel().set_bit();
                // Disable pull-up and pull-down resistors on the pin
                w.rue().clear_bit();
                w.rde().clear_bit();
                w.tie_opt().clear_bit();
                // Select function "RTC function 1" (GPIO) for analog use
                w.fun_sel().bits(0b00)
            });
        }
    };

    (@pin_specific $touch_num:expr, false) => {
        paste::paste! {
            unsafe { &*RTC_IO::ptr() }.[< touch_pad $touch_num >]().write(|w| {
                w.xpd().set_bit();
                w.tie_opt().clear_bit()
            });
        }
    };

    (
        $(
            (
                $touch_num:literal, $pin_num:literal, $rtc_pin:literal, $touch_out_reg:expr, $meas_field: expr, $touch_thres_reg:expr, $touch_thres_field:expr, $normal_pin:literal
            )
        )+
    ) => {
        $(
        impl $crate::gpio::TouchPin for GpioPin<$pin_num> {
            fn set_touch(&self, _: $crate::private::Internal) {
                use $crate::peripherals::{GPIO, RTC_IO, SENS};

                let gpio = unsafe { &*GPIO::ptr() };
                let rtcio = unsafe { &*RTC_IO::ptr() };
                let sens = unsafe { &*SENS::ptr() };

                // Pad to normal mode (not open-drain)
                gpio.pin($rtc_pin).write(|w| w.pad_driver().clear_bit());

                // clear output
                rtcio
                    .enable_w1tc()
                    .write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });
                paste::paste! {
                    sens . $touch_thres_reg ()
                        .write(|w| unsafe {
                            w. $touch_thres_field ().bits(
                                0b0 // Default: 0 for esp32 gets overridden later anyway.
                            )
                        });

                    $crate::touch!( @pin_specific $touch_num, $normal_pin );

                    // enable the pin
                    sens.sar_touch_enable().modify(|r, w| unsafe {
                        w.touch_pad_worken().bits(
                            r.touch_pad_worken().bits() | ( 1 << [< $touch_num >] )
                        )
                    });
                }
            }

            fn get_touch_measurement(&self, _: $crate::private::Internal) -> u16 {
                paste::paste! {
                    unsafe { &* $crate::peripherals::SENS::ptr() }
                        . $touch_out_reg ()
                        .read()
                        . $meas_field ()
                        .bits()
                }
            }

            fn get_touch_nr(&self, _: $crate::private::Internal) -> u8 {
                $touch_num
            }

            fn set_threshold(&self, threshold: u16, _: $crate::private::Internal) {
                paste::paste! {
                    unsafe { &* $crate::peripherals::SENS::ptr() }
                        . $touch_thres_reg ()
                        .write(|w| unsafe {
                            w. $touch_thres_field ().bits(threshold)
                        });
                }
            }
        })+
    };
}

/// GPIO output driver.
pub struct Output<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for Output<'_, P> {}

impl<'d, P> Peripheral for Output<'d, P> {
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
        let pin = Flex::new_typed(pin);

        Self::new_inner(pin, initial_output)
    }

    fn new_inner(mut pin: Flex<'d, P>, initial_output: Level) -> Self {
        pin.pin
            .set_output_high(initial_output.into(), private::Internal);

        pin.set_as_output();

        Self { pin }
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

/// GPIO input driver.
pub struct Input<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for Input<'_, P> {}

impl<'d, P> Peripheral for Input<'d, P> {
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

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        self.pin.peripheral_input()
    }
}

/// GPIO open-drain output driver.
pub struct OutputOpenDrain<'d, P = AnyPin> {
    pin: Flex<'d, P>,
}

impl<P> private::Sealed for OutputOpenDrain<'_, P> {}

impl<'d, P> Peripheral for OutputOpenDrain<'d, P> {
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

/// Flexible pin driver.
pub struct Flex<'d, P = AnyPin> {
    pin: PeripheralRef<'d, P>,
}

impl<P> private::Sealed for Flex<'_, P> {}

impl<'d, P> Peripheral for Flex<'d, P> {
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
}

impl<'d, P> Flex<'d, P>
where
    P: InputPin,
{
    /// Set the GPIO to input mode.
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin.init_input(pull, private::Internal);
        self.pin.output_enable(false, private::Internal);
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

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Stop listening for interrupts
    pub fn unlisten(&mut self) {
        self.pin.unlisten(private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }

    /// Checks if the interrupt status bit for this Pin is set
    #[inline]
    pub fn is_interrupt_set(&self) -> bool {
        self.pin.is_interrupt_set(private::Internal)
    }

    /// Enable as a wake-up source.
    ///
    /// This will unlisten for interrupts
    #[inline]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.pin.wakeup_enable(enable, event, private::Internal);
    }

    /// Returns a peripheral [input][interconnect::InputSignal] connected to
    /// this pin.
    ///
    /// The input signal can be passed to peripherals in place of an input pin.
    #[inline]
    pub fn peripheral_input(&self) -> interconnect::InputSignal {
        interconnect::InputSignal::new(self.pin.degrade_pin(private::Internal))
    }
}

impl<'d, P> Flex<'d, P>
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

    /// Turns the pin object into a peripheral
    /// [output][interconnect::OutputSignal].
    ///
    /// The output signal can be passed to peripherals in place of an output
    /// pin.
    #[inline]
    pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
        interconnect::OutputSignal::new(self.pin.degrade_pin(private::Internal))
    }
}

impl<'d, P> Flex<'d, P>
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
        /// Returns a peripheral [input][interconnect::InputSignal] connected to
        /// this pin.
        ///
        /// The input signal can be passed to peripherals in place of an input
        /// pin.
        #[inline]
        pub fn peripheral_input(&self) -> interconnect::InputSignal {
            handle_gpio_input!(&self.0, target, { target.peripheral_input() })
        }

        /// Turns the pin object into a peripheral
        /// [output][interconnect::OutputSignal].
        ///
        /// The output signal can be passed to peripherals in place of an output
        /// pin.
        #[inline]
        pub fn into_peripheral_output(self) -> interconnect::OutputSignal {
            handle_gpio_output!(self.0, target, { target.into_peripheral_output() })
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

        fn output_enable(&self, enable: bool, _: private::Internal) {
            handle_gpio_input!(&self.0, target, {
                Pin::output_enable(target, enable, private::Internal)
            })
        }

        fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6] {
            handle_gpio_input!(&self.0, target, {
                Pin::output_signals(target, private::Internal)
            })
        }

        fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6] {
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

    impl PeripheralSignal for AnyPin {
        fn pull_direction(&self, pull: Pull, _: private::Internal) {
            handle_gpio_input!(&self.0, target, {
                PeripheralSignal::pull_direction(target, pull, private::Internal)
            })
        }
    }

    impl PeripheralInput for AnyPin {
        fn init_input(&self, pull: Pull, _: private::Internal) {
            handle_gpio_input!(&self.0, target, {
                PeripheralInput::init_input(target, pull, private::Internal)
            });
        }

        fn enable_input(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                PeripheralInput::enable_input(target, on, private::Internal)
            });
        }

        fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                PeripheralInput::enable_input_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn is_input_high(&self, _: private::Internal) -> bool {
            handle_gpio_input!(&self.0, target, {
                PeripheralInput::is_input_high(target, private::Internal)
            })
        }

        fn input_signals(&self, _: private::Internal) -> [Option<InputSignal>; 6] {
            handle_gpio_input!(&self.0, target, {
                PeripheralInput::input_signals(target, private::Internal)
            })
        }

        fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
            interconnect::InputSignal::new(self.degrade_pin(private::Internal))
                .connect_input_to_peripheral(signal, private::Internal);
        }

        fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
            interconnect::InputSignal::new(self.degrade_pin(private::Internal))
                .disconnect_input_from_peripheral(signal, private::Internal);
        }
    }

    impl InputPin for AnyPin {
        fn listen_with_options(
            &mut self,
            event: Event,
            int_enable: bool,
            nmi_enable: bool,
            wake_up_from_light_sleep: bool,
            _: private::Internal,
        ) {
            handle_gpio_input!(&mut self.0, target, {
                InputPin::listen_with_options(
                    target,
                    event,
                    int_enable,
                    nmi_enable,
                    wake_up_from_light_sleep,
                    private::Internal,
                )
            })
        }

        fn unlisten(&mut self, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                InputPin::unlisten(target, private::Internal)
            })
        }

        fn is_interrupt_set(&self, _: private::Internal) -> bool {
            handle_gpio_input!(&self.0, target, {
                InputPin::is_interrupt_set(target, private::Internal)
            })
        }

        fn clear_interrupt(&mut self, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                InputPin::clear_interrupt(target, private::Internal)
            })
        }

        fn listen(&mut self, event: Event, _: private::Internal) {
            handle_gpio_input!(&mut self.0, target, {
                InputPin::listen(target, event, private::Internal)
            })
        }
    }

    impl PeripheralOutput for AnyPin {
        fn set_to_open_drain_output(&mut self, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::set_to_open_drain_output(target, private::Internal)
            });
        }

        fn set_to_push_pull_output(&mut self, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::set_to_push_pull_output(target, private::Internal)
            });
        }

        fn enable_output(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::enable_output(target, on, private::Internal)
            });
        }

        fn set_output_high(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::set_output_high(target, on, private::Internal)
            });
        }

        fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::set_drive_strength(target, strength, private::Internal)
            });
        }

        fn enable_open_drain(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::enable_open_drain(target, on, private::Internal)
            });
        }

        fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::enable_output_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::internal_pull_up_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(&mut self.0, target, {
                PeripheralOutput::internal_pull_down_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn is_set_high(&self, _: private::Internal) -> bool {
            handle_gpio_output!(&self.0, target, {
                PeripheralOutput::is_set_high(target, private::Internal)
            })
        }

        fn output_signals(&self, _: private::Internal) -> [Option<OutputSignal>; 6] {
            handle_gpio_output!(&self.0, target, {
                PeripheralOutput::output_signals(target, private::Internal)
            })
        }

        fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal) {
            interconnect::OutputSignal::new(self.degrade_pin(private::Internal))
                .connect_peripheral_to_output(signal, private::Internal);
        }

        fn disconnect_from_peripheral_output(
            &mut self,
            signal: OutputSignal,
            _: private::Internal,
        ) {
            interconnect::OutputSignal::new(self.degrade_pin(private::Internal))
                .disconnect_from_peripheral_output(signal, private::Internal);
        }
    }

    impl OutputPin for AnyPin {}

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
    let bits = unsafe { &*GPIO::PTR }
        .pin(pin_num as usize)
        .read()
        .int_ena()
        .bits();
    bits != 0
}

fn set_int_enable(gpio_num: u8, int_ena: u8, int_type: u8, wake_up_from_light_sleep: bool) {
    let gpio = unsafe { &*crate::peripherals::GPIO::PTR };
    gpio.pin(gpio_num as usize).modify(|_, w| unsafe {
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

    impl<'d, P> Flex<'d, P>
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

    impl<'d, P> Input<'d, P>
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

    impl<'d, P> digital::InputPin for Input<'d, P>
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

    impl<'d, P> digital::OutputPin for Output<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for Output<'d, P>
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

    impl<'d, P> digital::ToggleableOutputPin for Output<'d, P>
    where
        P: OutputPin,
    {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d, P> digital::InputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::OutputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::ToggleableOutputPin for OutputOpenDrain<'d, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d, P> digital::InputPin for Flex<'d, P>
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

    impl<'d, P> digital::OutputPin for Flex<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for Flex<'d, P>
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

    impl<'d, P> digital::ToggleableOutputPin for Flex<'d, P>
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

    impl<'d, P> digital::ErrorType for Input<'d, P>
    where
        P: InputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<'d, P> digital::InputPin for Input<'d, P>
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

    impl<'d, P> digital::ErrorType for Output<'d, P>
    where
        P: OutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<'d, P> digital::OutputPin for Output<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for Output<'d, P>
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

    impl<'d, P> digital::InputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::ErrorType for OutputOpenDrain<'d, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<'d, P> digital::OutputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for OutputOpenDrain<'d, P>
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

    impl<'d, P> digital::InputPin for Flex<'d, P>
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

    impl<'d, P> digital::ErrorType for Flex<'d, P> {
        type Error = core::convert::Infallible;
    }

    impl<'d, P> digital::OutputPin for Flex<'d, P>
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

    impl<'d, P> digital::StatefulOutputPin for Flex<'d, P>
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

    impl<'d, P> Wait for Flex<'d, P>
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

    impl<'d, P> Wait for Input<'d, P>
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
