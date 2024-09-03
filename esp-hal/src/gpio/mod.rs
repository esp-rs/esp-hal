//! # General Purpose I/Os (GPIO)
//!
//! ## Overview
//!
//! Each pin can be used as a general-purpose I/O, or be connected to an
//! internal peripheral signal.
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
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `Io::new`.
//!
//! ### Pin Types
//!
//! - [Input] pins can be used as digital inputs.
//! - [Output] and [OutputOpenDrain] pins can be used as digital outputs.
//! - [Flex] pin is a pin that can be used as an input and output pin.
//! - [AnyPin] is a type-erased GPIO pin with support for inverted signalling.
//! - [DummyPin] is a useful for cases where peripheral driver requires a pin,
//!   but real pin cannot be used.
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
//! See the [Commonly Used Setup] section of the crate documentation.
//!
//! ### Inverting a signal using `AnyPin`
//! See the [Inverting TX and RX Pins] example of the UART documentation.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [Commonly Used Setup]: ../index.html#commonly-used-setup
//! [Inverting TX and RX Pins]: ../uart/index.html#inverting-tx-and-rx-pins

use core::{cell::Cell, marker::PhantomData};

use critical_section::Mutex;
use procmacros::ram;

#[cfg(any(adc, dac))]
pub(crate) use crate::analog;
pub(crate) use crate::gpio;
#[cfg(any(xtensa, esp32c3))]
pub(crate) use crate::rtc_pins;
pub use crate::soc::gpio::*;
use crate::{
    interrupt::InterruptHandler,
    peripheral::PeripheralRef,
    peripherals::{GPIO, IO_MUX},
    private,
    InterruptConfigurable,
};
#[cfg(touch)]
pub(crate) use crate::{touch_common, touch_into};

mod any_pin;
mod dummy_pin;

pub use any_pin::AnyPin;
pub use dummy_pin::DummyPin;

#[cfg(soc_etm)]
pub mod etm;
#[cfg(lp_io)]
pub mod lp_io;
#[cfg(all(rtc_io, not(esp32)))]
pub mod rtc_io;

/// Convenience constant for `Option::None` pin
pub const NO_PIN: Option<DummyPin> = None;

static USER_INTERRUPT_HANDLER: Mutex<Cell<Option<InterruptHandler>>> = Mutex::new(Cell::new(None));

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
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// Low
    Low,
    /// High
    High,
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

/// RTC input pin mode
pub struct RtcInput;

/// RTC output pin mode
pub struct RtcOutput;

/// Analog mode
pub struct Analog;

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
    #[cfg(any(esp32c3, esp32c6))]
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

/// Marker for RTC pins which support input mode
pub trait RtcInputPin: RtcPin {}
/// Marker for RTC pins which support output mode
pub trait RtcOutputPin: RtcPin {}

/// Common trait implemented by pins
pub trait Pin: private::Sealed {
    /// GPIO number
    fn number(&self, _: private::Internal) -> u8;

    /// Enable/disable sleep-mode
    fn sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Configure the alternate function
    fn set_alternate_function(&mut self, alternate: AlternateFunction, _: private::Internal);

    /// Listen for interrupts
    fn listen(&mut self, event: Event, _: private::Internal) {
        self.listen_with_options(event, true, false, false, private::Internal)
    }

    /// Checks if listening for interrupts is enabled for this Pin
    fn is_listening(&self, _: private::Internal) -> bool;

    /// Listen for interrupts
    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
        _: private::Internal,
    );

    /// Stop listening for interrupts
    fn unlisten(&mut self, _: private::Internal);

    /// Checks if the interrupt status bit for this Pin is set
    fn is_interrupt_set(&self, _: private::Internal) -> bool;

    /// Clear the interrupt status bit for this Pin
    fn clear_interrupt(&mut self, _: private::Internal);

    /// Enable this pin as a wake up source
    fn wakeup_enable(&mut self, enable: bool, event: WakeEvent, _: private::Internal);
}

/// Trait implemented by pins which can be used as inputs
pub trait InputPin: Pin {
    /// Init as input with the given pull-up/pull-down
    fn init_input(&self, pull_down: bool, pull_up: bool, _: private::Internal);

    /// Set the pin to input mode without internal pull-up / pull-down resistors
    fn set_to_input(&mut self, _: private::Internal);

    /// Enable input for the pin
    fn enable_input(&mut self, on: bool, _: private::Internal);

    /// Enable input in sleep mode for the pin
    fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// The current state of the input
    fn is_input_high(&self, _: private::Internal) -> bool;

    /// Connect the pin to a peripheral input signal
    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal);

    /// Connect the pin to a peripheral input signal.
    ///
    /// Optionally invert the signal. When `force_via_gpio_mux` is true it will
    /// won't use the alternate function even if it matches
    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
        _: private::Internal,
    );

    /// Remove a connected `signal` from this input pin.
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal);
}

/// Trait implemented by pins which can be used as outputs
pub trait OutputPin: Pin {
    /// Configure open-drain mode
    fn set_to_open_drain_output(&mut self, _: private::Internal);

    /// Configure output mode
    fn set_to_push_pull_output(&mut self, _: private::Internal);

    /// Enable/disable the pin as output
    fn enable_output(&mut self, on: bool, _: private::Internal);

    /// Set the pin's level to high or low
    fn set_output_high(&mut self, on: bool, _: private::Internal);

    /// Configure the [DriveStrength] of the pin
    fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal);

    /// Enable/disable open-drain mode
    fn enable_open_drain(&mut self, on: bool, _: private::Internal);

    /// Enable/disable output in sleep mode
    fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Configure internal pull-up resistor in sleep mode
    fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Configure internal pull-down resistor in sleep mode
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal);

    /// Enable/disable internal pull-up resistor for normal operation
    fn internal_pull_up(&mut self, on: bool, _: private::Internal);

    /// Enable/disable internal pull-down resistor for normal operation
    fn internal_pull_down(&mut self, on: bool, _: private::Internal);

    /// Connect the pin to a peripheral output signal
    fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal);

    /// Connect the pin to a peripheral output signal.
    ///
    /// invert: Configures whether or not to invert the output value
    ///
    /// invert_enable: Configures whether or not to invert the output enable
    /// signal
    ///
    /// enable_from_gpio: Configures to select the source of output enable
    /// signal.
    /// - false =  Use output enable signal from peripheral
    /// - true = Force the output enable signal to be sourced from bit n of
    ///   GPIO_ENABLE_REG
    ///
    /// force_via_gpio_mux: if true don't use the alternate function even if it
    /// matches
    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
        _: private::Internal,
    );

    /// Remove this output pin from a connected [signal](`InputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`InputSignal`). Any other
    /// outputs connected to the signal remain intact.
    fn disconnect_peripheral_from_output(&mut self, _: private::Internal);

    /// Is the output set to high
    fn is_set_high(&self, _: private::Internal) -> bool;
}

/// Trait implemented by pins which can be used as analog pins
pub trait AnalogPin: Pin {
    /// Configure the pin for analog operation
    fn set_analog(&self, _: private::Internal);
}

/// Trait implemented by pins which can be used as Touchpad pins
pub trait TouchPin: Pin {
    /// Configure the pin for analog operation
    fn set_touch(&self, _: private::Internal);

    /// Reads the pin's touch measurement register
    fn get_touch_measurement(&self, _: private::Internal) -> u16;

    /// Maps the pin nr to the touch pad nr
    fn get_touch_nr(&self, _: private::Internal) -> u8;

    /// Set a pins touch threshold for interrupts.
    fn set_threshold(&self, threshold: u16, _: private::Internal);
}

#[doc(hidden)]
pub trait CreateErasedPin: Pin {
    fn erased_pin(&self, _: private::Internal) -> ErasedPin;
}

#[doc(hidden)]
pub trait InterruptStatusRegisterAccess {
    fn pro_cpu_interrupt_status_read() -> u32;

    fn pro_cpu_nmi_status_read() -> u32;

    fn app_cpu_interrupt_status_read() -> u32 {
        Self::pro_cpu_interrupt_status_read()
    }

    fn app_cpu_nmi_status_read() -> u32 {
        Self::pro_cpu_nmi_status_read()
    }

    fn interrupt_status_read() -> u32 {
        match crate::get_core() {
            crate::Cpu::ProCpu => Self::pro_cpu_interrupt_status_read(),
            #[cfg(multi_core)]
            crate::Cpu::AppCpu => Self::app_cpu_interrupt_status_read(),
        }
    }

    fn nmi_status_read() -> u32 {
        match crate::get_core() {
            crate::Cpu::ProCpu => Self::pro_cpu_nmi_status_read(),
            #[cfg(multi_core)]
            crate::Cpu::AppCpu => Self::app_cpu_nmi_status_read(),
        }
    }
}

#[doc(hidden)]
pub struct InterruptStatusRegisterAccessBank0;

#[doc(hidden)]
pub struct InterruptStatusRegisterAccessBank1;

#[doc(hidden)]
pub trait InterruptStatusRegisters<RegisterAccess>
where
    RegisterAccess: InterruptStatusRegisterAccess,
{
    fn pro_cpu_interrupt_status_read() -> u32 {
        RegisterAccess::pro_cpu_interrupt_status_read()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        RegisterAccess::pro_cpu_nmi_status_read()
    }

    fn app_cpu_interrupt_status_read() -> u32 {
        RegisterAccess::app_cpu_interrupt_status_read()
    }

    fn app_cpu_nmi_status_read() -> u32 {
        RegisterAccess::app_cpu_nmi_status_read()
    }
}

#[doc(hidden)]
pub trait GpioSignal {
    fn output_signals() -> [Option<OutputSignal>; 6];
    fn input_signals() -> [Option<InputSignal>; 6];
}

#[doc(hidden)]
pub struct Bank0GpioRegisterAccess;

#[doc(hidden)]
pub struct Bank1GpioRegisterAccess;

#[doc(hidden)]
pub trait BankGpioRegisterAccess {
    fn write_out_en_clear(word: u32);

    fn write_out_en_set(word: u32);

    fn read_input() -> u32;

    fn read_output() -> u32;

    fn read_interrupt_status() -> u32;

    fn write_interrupt_status_clear(word: u32);

    fn write_output_set(word: u32);

    fn write_output_clear(word: u32);
}

impl BankGpioRegisterAccess for Bank0GpioRegisterAccess {
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

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
impl BankGpioRegisterAccess for Bank1GpioRegisterAccess {
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

/// Connect an always-low signal to the peripheral input signal
pub fn connect_low_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .modify(|_, w| unsafe {
            w.sel()
                .set_bit()
                .in_inv_sel()
                .bit(false)
                .in_sel()
                .bits(ZERO_INPUT)
        });
}

/// Connect an always-high signal to the peripheral input signal
pub fn connect_high_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }
        .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
        .modify(|_, w| unsafe {
            w.sel()
                .set_bit()
                .in_inv_sel()
                .bit(false)
                .in_sel()
                .bits(ONE_INPUT)
        });
}

#[doc(hidden)]
pub trait PinType {}

#[doc(hidden)]
pub trait IsOutputPin: PinType {}

#[doc(hidden)]
pub trait IsInputPin: PinType {}

#[doc(hidden)]
pub trait IsAnalogPin: PinType {}

#[doc(hidden)]
pub trait IsTouchPin: PinType {}

#[doc(hidden)]
pub struct InputOutputPinType;

#[doc(hidden)]
pub struct InputOnlyPinType;

#[doc(hidden)]
pub struct InputOutputAnalogPinType;

#[doc(hidden)]
pub struct InputOnlyAnalogPinType;

#[doc(hidden)]
pub struct InputOutputAnalogTouchPinType;

impl PinType for InputOutputPinType {}
impl IsOutputPin for InputOutputPinType {}
impl IsInputPin for InputOutputPinType {}

impl PinType for InputOnlyPinType {}
impl IsInputPin for InputOnlyPinType {}

impl PinType for InputOutputAnalogPinType {}
impl IsOutputPin for InputOutputAnalogPinType {}
impl IsInputPin for InputOutputAnalogPinType {}
impl IsAnalogPin for InputOutputAnalogPinType {}

impl PinType for InputOnlyAnalogPinType {}
impl IsInputPin for InputOnlyAnalogPinType {}
impl IsAnalogPin for InputOnlyAnalogPinType {}

impl PinType for InputOutputAnalogTouchPinType {}
impl IsOutputPin for InputOutputAnalogTouchPinType {}
impl IsInputPin for InputOutputAnalogTouchPinType {}
impl IsAnalogPin for InputOutputAnalogTouchPinType {}
impl IsTouchPin for InputOutputAnalogTouchPinType {}

/// GPIO pin
pub struct GpioPin<const GPIONUM: u8>;

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    /// Is the input pin high?
    #[inline]
    pub fn is_high(&self) -> bool {
        <Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0
    }

    /// Is the input pin low?
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
}

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
{
    pub(crate) fn new() -> Self {
        Self
    }
}

impl<const GPIONUM: u8> InputPin for GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
{
    fn init_input(&self, pull_down: bool, pull_up: bool, _: private::Internal) {
        let gpio = unsafe { &*GPIO::PTR };

        <Self as GpioProperties>::Bank::write_out_en_clear(1 << (GPIONUM % 32));
        gpio.func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, Some(pull_up), Some(pull_down));

        // NOTE: Workaround to make GPIO18 and GPIO19 work on the ESP32-C3, which by
        //       default are assigned to the `USB_SERIAL_JTAG` peripheral.
        #[cfg(esp32c3)]
        if GPIONUM == 18 || GPIONUM == 19 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0()
                .modify(|_, w| {
                    w.usb_pad_enable()
                        .clear_bit()
                        .dm_pullup()
                        .clear_bit()
                        .dm_pulldown()
                        .clear_bit()
                        .dp_pullup()
                        .clear_bit()
                        .dp_pulldown()
                        .clear_bit()
                });
        }

        // Same workaround as above for ESP32-S3
        #[cfg(esp32s3)]
        if GPIONUM == 19 || GPIONUM == 20 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0()
                .modify(|_, w| {
                    w.usb_pad_enable()
                        .clear_bit()
                        .dm_pullup()
                        .clear_bit()
                        .dm_pulldown()
                        .clear_bit()
                        .dp_pullup()
                        .clear_bit()
                        .dp_pulldown()
                        .clear_bit()
                });
        }

        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe {
            w.mcu_sel()
                .bits(GPIO_FUNCTION as u8)
                .fun_ie()
                .set_bit()
                .fun_wpd()
                .bit(pull_down)
                .fun_wpu()
                .bit(pull_up)
                .slp_sel()
                .clear_bit()
        });
    }

    fn set_to_input(&mut self, _: private::Internal) {
        self.init_input(false, false, private::Internal);
    }

    fn enable_input(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_ie().bit(on));
    }

    fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_ie().bit(on));
    }

    fn is_input_high(&self, _: private::Internal) -> bool {
        <Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        self.connect_input_to_peripheral_with_options(signal, false, false, private::Internal);
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
        _: private::Internal,
    ) {
        let af = if force_via_gpio_mux {
            GPIO_FUNCTION
        } else {
            let mut res = GPIO_FUNCTION;
            for (i, input_signal) in <Self as GpioProperties>::Signals::input_signals()
                .iter()
                .enumerate()
            {
                if let Some(input_signal) = input_signal {
                    if *input_signal == signal {
                        res = match i {
                            0 => AlternateFunction::Function0,
                            1 => AlternateFunction::Function1,
                            2 => AlternateFunction::Function2,
                            3 => AlternateFunction::Function3,
                            4 => AlternateFunction::Function4,
                            5 => AlternateFunction::Function5,
                            _ => unreachable!(),
                        };
                        break;
                    }
                }
            }
            res
        };
        if af == GPIO_FUNCTION && signal as usize > INPUT_SIGNAL_MAX as usize {
            panic!("Cannot connect GPIO to this peripheral");
        }
        self.set_alternate_function(af, private::Internal);
        if (signal as usize) <= INPUT_SIGNAL_MAX as usize {
            unsafe { &*GPIO::PTR }
                .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
                .modify(|_, w| unsafe {
                    w.sel()
                        .set_bit()
                        .in_inv_sel()
                        .bit(invert)
                        .in_sel()
                        .bits(GPIONUM)
                });
        }
    }

    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
        self.set_alternate_function(GPIO_FUNCTION, private::Internal);

        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| w.sel().clear_bit());
    }
}

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: GpioProperties,
{
    /// Create a pin out of thin air.
    ///
    /// # Safety
    ///
    /// Ensure that only one instance of a pin exists at one time.
    pub unsafe fn steal() -> Self {
        Self
    }
}

impl<const GPIONUM: u8> Pin for GpioPin<GPIONUM>
where
    Self: GpioProperties,
{
    fn number(&self, _: private::Internal) -> u8 {
        GPIONUM
    }

    fn sleep_mode(&mut self, on: bool, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.slp_sel().bit(on));
    }

    fn set_alternate_function(&mut self, alternate: AlternateFunction, _: private::Internal) {
        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
    }

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

        unsafe {
            (*GPIO::PTR).pin(GPIONUM as usize).modify(|_, w| {
                w.int_ena()
                    .bits(gpio_intr_enable(int_enable, nmi_enable))
                    .int_type()
                    .bits(event as u8)
                    .wakeup_enable()
                    .bit(wake_up_from_light_sleep)
            });
        }
    }

    fn is_listening(&self, _: private::Internal) -> bool {
        let bits = unsafe { &*GPIO::PTR }
            .pin(GPIONUM as usize)
            .read()
            .int_ena()
            .bits();
        bits != 0
    }

    fn unlisten(&mut self, _: private::Internal) {
        unsafe {
            (*GPIO::PTR)
                .pin(GPIONUM as usize)
                .modify(|_, w| w.int_ena().bits(0).int_type().bits(0).int_ena().bits(0));
        }
    }

    fn is_interrupt_set(&self, _: private::Internal) -> bool {
        <Self as GpioProperties>::Bank::read_interrupt_status() & 1 << (GPIONUM % 32) != 0
    }

    fn clear_interrupt(&mut self, _: private::Internal) {
        <Self as GpioProperties>::Bank::write_interrupt_status_clear(1 << (GPIONUM % 32));
    }

    fn wakeup_enable(&mut self, enable: bool, event: WakeEvent, _: private::Internal) {
        self.listen_with_options(event.into(), false, false, enable, private::Internal);
    }
}

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    /// Drives the pin high.
    #[inline]
    pub fn set_high(&mut self) {
        <Self as GpioProperties>::Bank::write_output_set(1 << (GPIONUM % 32));
    }

    /// Drives the pin low.
    #[inline]
    pub fn set_low(&mut self) {
        <Self as GpioProperties>::Bank::write_output_clear(1 << (GPIONUM % 32));
    }

    /// Drives the pin high or low depending on the provided value.
    #[inline]
    pub fn set_state(&mut self, state: bool) {
        match state {
            true => self.set_high(),
            false => self.set_low(),
        }
    }

    /// Is the pin in drive high mode?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        <Self as GpioProperties>::Bank::read_output() & (1 << (GPIONUM % 32)) != 0
    }

    /// Is the pin in drive low mode?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// Toggle pin output.
    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

impl<const GPIONUM: u8> crate::peripheral::Peripheral for GpioPin<GPIONUM>
where
    Self: GpioProperties,
{
    type P = GpioPin<GPIONUM>;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

impl<const GPIONUM: u8> private::Sealed for GpioPin<GPIONUM> where Self: GpioProperties {}

impl<const GPIONUM: u8> GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn init_output(&self, alternate: AlternateFunction, open_drain: bool, _: private::Internal) {
        let gpio = unsafe { &*GPIO::PTR };

        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, Some(false), Some(false));

        <Self as GpioProperties>::Bank::write_out_en_set(1 << (GPIONUM % 32));
        gpio.pin(GPIONUM as usize)
            .modify(|_, w| w.pad_driver().bit(open_drain));

        gpio.func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        // NOTE: Workaround to make GPIO18 and GPIO19 work on the ESP32-C3, which by
        //       default are assigned to the `USB_SERIAL_JTAG` peripheral.
        #[cfg(esp32c3)]
        if GPIONUM == 18 || GPIONUM == 19 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0()
                .modify(|_, w| w.usb_pad_enable().clear_bit());
        }

        // Same workaround as above for ESP32-S3
        #[cfg(esp32s3)]
        if GPIONUM == 19 || GPIONUM == 20 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0()
                .modify(|_, w| w.usb_pad_enable().clear_bit());
        }

        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe {
            w.mcu_sel()
                .bits(alternate as u8)
                .fun_ie()
                .bit(open_drain)
                .fun_wpd()
                .clear_bit()
                .fun_wpu()
                .clear_bit()
                .fun_drv()
                .bits(DriveStrength::I20mA as u8)
                .slp_sel()
                .clear_bit()
        });
    }
}

impl<const GPIONUM: u8> OutputPin for GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn set_to_open_drain_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, true, private::Internal);
    }

    fn set_to_push_pull_output(&mut self, _: private::Internal) {
        self.init_output(GPIO_FUNCTION, false, private::Internal);
    }

    fn enable_output(&mut self, on: bool, _: private::Internal) {
        if on {
            <Self as GpioProperties>::Bank::write_out_en_set(1 << (GPIONUM % 32));
        } else {
            <Self as GpioProperties>::Bank::write_out_en_clear(1 << (GPIONUM % 32));
        }
    }

    fn set_output_high(&mut self, high: bool, _: private::Internal) {
        if high {
            <Self as GpioProperties>::Bank::write_output_set(1 << (GPIONUM % 32));
        } else {
            <Self as GpioProperties>::Bank::write_output_clear(1 << (GPIONUM % 32));
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

    fn internal_pull_up(&mut self, on: bool, _: private::Internal) {
        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, Some(on), None);

        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpu().bit(on));
    }
    fn internal_pull_down(&mut self, on: bool, _: private::Internal) {
        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, None, Some(on));

        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpd().bit(on));
    }

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal) {
        self.connect_peripheral_to_output_with_options(
            signal,
            false,
            false,
            false,
            false,
            private::Internal,
        )
    }

    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
        _: private::Internal,
    ) {
        let af = if force_via_gpio_mux {
            GPIO_FUNCTION
        } else {
            let mut res = GPIO_FUNCTION;
            for (i, output_signal) in <Self as GpioProperties>::Signals::output_signals()
                .iter()
                .enumerate()
            {
                if let Some(output_signal) = output_signal {
                    if *output_signal == signal {
                        res = match i {
                            0 => AlternateFunction::Function0,
                            1 => AlternateFunction::Function1,
                            2 => AlternateFunction::Function2,
                            3 => AlternateFunction::Function3,
                            4 => AlternateFunction::Function4,
                            5 => AlternateFunction::Function5,
                            _ => unreachable!(),
                        };
                        break;
                    }
                }
            }
            res
        };
        if af == GPIO_FUNCTION && signal as usize > OUTPUT_SIGNAL_MAX as usize {
            panic!("Cannot connect this peripheral to GPIO");
        }
        self.set_alternate_function(af, private::Internal);
        let clipped_signal = if signal as usize <= OUTPUT_SIGNAL_MAX as usize {
            signal as OutputSignalType
        } else {
            OUTPUT_SIGNAL_MAX
        };
        unsafe { &*GPIO::PTR }
            .func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe {
                w.out_sel()
                    .bits(clipped_signal)
                    .inv_sel()
                    .bit(invert)
                    .oen_sel()
                    .bit(enable_from_gpio)
                    .oen_inv_sel()
                    .bit(invert_enable)
            });
    }

    fn disconnect_peripheral_from_output(&mut self, _: private::Internal) {
        self.set_alternate_function(GPIO_FUNCTION, private::Internal);
        unsafe { &*GPIO::PTR }
            .func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });
    }

    fn is_set_high(&self, _: private::Internal) -> bool {
        <Self as GpioProperties>::Bank::read_output() & (1 << (GPIONUM % 32)) != 0
    }
}

#[cfg(any(adc, dac))]
impl<const GPIONUM: u8> AnalogPin for GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsAnalogPin,
{
    /// Configures the pin for analog mode.
    fn set_analog(&self, _: private::Internal) {
        crate::soc::gpio::internal_into_analog(GPIONUM);
    }
}

#[cfg(touch)]
impl<const GPIONUM: u8> TouchPin for GpioPin<GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsTouchPin,
{
    fn set_touch(&self, _: private::Internal) {
        crate::soc::gpio::internal_into_touch(GPIONUM);
    }

    fn get_touch_measurement(&self, _: private::Internal) -> u16 {
        crate::soc::gpio::internal_get_touch_measurement(GPIONUM)
    }

    fn get_touch_nr(&self, _: private::Internal) -> u8 {
        crate::soc::gpio::internal_get_touch_nr(GPIONUM)
    }

    fn set_threshold(&self, threshold: u16, _: private::Internal) {
        crate::soc::gpio::internal_set_threshold(GPIONUM, threshold)
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

        let pins = gpio.pins();

        Io {
            _io_mux: io_mux,
            pins,
        }
    }

    /// Initialize the I/O driver without enabling the GPIO interrupt or
    /// binding an interrupt handler to it.
    ///
    /// *Note:* You probably don't want to use this, it is intended to be used
    /// in very specific use cases. Async GPIO functionality will not work
    /// when instantiating `Io` using this constructor.
    pub fn new_no_bind_interrupt(gpio: GPIO, io_mux: IO_MUX) -> Self {
        let pins = gpio.pins();

        Io {
            _io_mux: io_mux,
            pins,
        }
    }
}

impl crate::private::Sealed for Io {}

impl InterruptConfigurable for Io {
    /// Install the given interrupt handler replacing any previously set
    /// handler.
    ///
    /// When the async feature is enabled the handler will be called first and
    /// the internal async handler will run after. In that case it's
    /// important to not reset the interrupt status when mixing sync and
    /// async (i.e. using async wait) interrupt handling.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        critical_section::with(|cs| {
            crate::interrupt::enable(crate::peripherals::Interrupt::GPIO, handler.priority())
                .unwrap();
            USER_INTERRUPT_HANDLER.borrow(cs).set(Some(handler));
        });
    }
}

#[ram]
extern "C" fn gpio_interrupt_handler() {
    if let Some(user_handler) = critical_section::with(|cs| USER_INTERRUPT_HANDLER.borrow(cs).get())
    {
        user_handler.call();
    }

    asynch::handle_gpio_interrupt();
}

#[doc(hidden)]
pub trait GpioProperties {
    type Bank: BankGpioRegisterAccess;
    type InterruptStatus: InterruptStatusRegisterAccess;
    type Signals: GpioSignal;
    type PinType: PinType;
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
            impl GPIO {
                pub(crate) fn pins(self) -> Pins {
                    Pins {
                        $(
                            [< gpio $gpionum >]: {
                                GpioPin::new()
                            },
                        )+
                    }
                }
            }

            $(
                impl $crate::gpio::GpioProperties for GpioPin<$gpionum> {
                    type Bank = $crate::gpio::[< Bank $bank GpioRegisterAccess >];
                    type InterruptStatus = $crate::gpio::[< InterruptStatusRegisterAccessBank $bank >];
                    type Signals = [< Gpio $gpionum Signals >];
                    type PinType = $crate::gpio::[<$type PinType>];
                }

                #[doc(hidden)]
                pub struct [<Gpio $gpionum Signals>];

                impl $crate::gpio::GpioSignal for [<Gpio $gpionum Signals>] {
                    fn output_signals() -> [Option<OutputSignal>; 6]{
                        #[allow(unused_mut)]
                        let mut output_signals = [None, None, None, None, None, None];

                        $(
                            $(
                                output_signals[ $af_output_num ] = Some( OutputSignal::$af_output_signal );
                            )*
                        )?

                        output_signals
                    }
                    fn input_signals() -> [Option<InputSignal>; 6] {
                        #[allow(unused_mut)]
                        let mut input_signals = [None, None, None, None, None, None];

                        $(
                            $(
                                input_signals[ $af_input_num ] = Some( InputSignal::$af_input_signal );
                            )*
                        )?

                        input_signals
                    }
                }

                impl $crate::gpio::CreateErasedPin for GpioPin<$gpionum> {
                    fn erased_pin(&self, _: $crate::private::Internal) -> ErasedPin {
                        $crate::gpio::ErasedPin::[< Gpio $gpionum >](unsafe { Self::steal() })
                    }
                }
            )+

            /// Pins available on this chip
            pub struct Pins {
                $(
                    /// GPIO pin number `$gpionum`.
                    pub [< gpio $gpionum >] : GpioPin<$gpionum>,
                )+
            }

            #[doc(hidden)]
            pub enum ErasedPin {
                $(
                    [<Gpio $gpionum >](GpioPin<$gpionum>),
                )+
            }

            impl ErasedPin {
                pub(crate) unsafe fn clone_unchecked(&mut self) ->  Self {
                    match self {
                        $(
                        ErasedPin::[<Gpio $gpionum >](_) => {
                            $crate::gpio::ErasedPin::[< Gpio $gpionum >](unsafe { GpioPin::steal() })
                        }
                        )+
                    }
                }
            }

            procmacros::make_gpio_enum_dispatch_macro!(
                handle_gpio_output
                { InputOutputAnalogTouch, InputOutputAnalog, InputOutput, }
                {
                    $(
                        $type,$gpionum
                    )+
                }
            );

            procmacros::make_gpio_enum_dispatch_macro!(
                handle_gpio_input
                { InputOutputAnalogTouch, InputOutputAnalog, InputOutput, InputOnlyAnalog }
                {
                    $(
                        $type,$gpionum
                    )+
                }
            );
        }
    };
}

#[cfg(xtensa)]
#[doc(hidden)]
#[macro_export]
macro_rules! rtc_pins {
    (
        $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:ident, $rde:ident)?
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

                #[cfg(esp32s3)]
                unsafe { $crate::peripherals::SENS::steal() }.sar_peri_clk_gate_conf().modify(|_,w| w.iomux_clk_en().set_bit());
                #[cfg(esp32s2)]
                unsafe { $crate::peripherals::SENS::steal() }.sar_io_mux_conf().modify(|_,w| w.iomux_clk_gate_en().set_bit());

                // disable input
                paste::paste!{
                    rtcio.$pin_reg.modify(|_,w| unsafe {w
                        .[<$prefix fun_ie>]().bit(input_enable)
                        .[<$prefix mux_sel>]().bit(mux)
                        .[<$prefix fun_sel>]().bits(func as u8)
                    });
                }
            }

            fn rtcio_pad_hold(&mut self, enable: bool) {
                let rtc_ctrl = unsafe { &*$crate::peripherals::LPWR::PTR };

                #[cfg(esp32)]
                rtc_ctrl.hold_force().modify(|_, w| w.$hold().bit(enable));

                #[cfg(not(esp32))]
                rtc_ctrl.pad_hold().modify(|_, w| w.$hold().bit(enable));
            }
        }

        $(
            impl $crate::gpio::RtcPinWithResistors for GpioPin<$pin_num>
            {
                fn rtcio_pullup(&mut self, enable: bool) {
                    let rtcio = unsafe { &*$crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.$rue().bit([< enable >]));
                    }
                }

                fn rtcio_pulldown(&mut self, enable: bool) {
                    let rtcio = unsafe { &*$crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.$rde().bit([< enable >]));
                    }
                }
            }
        )?
    };

    (
        $( ( $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:ident, $rde:ident)? ) )+
    ) => {
        $(
            $crate::gpio::rtc_pins!($pin_num, $rtc_pin, $pin_reg, $prefix, $hold $(, $rue, $rde)?);
        )+
    };
}

#[cfg(esp32c3)]
#[doc(hidden)]
#[macro_export]
macro_rules! rtc_pins {
    (
        $pin_num:expr
    ) => {
        impl $crate::gpio::RtcPin for GpioPin<$pin_num> {
            unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
                let rtc_cntl = unsafe { &*$crate::peripherals::RTC_CNTL::ptr() };
                paste::paste! {
                    rtc_cntl.gpio_wakeup().modify(|_, w| w.[< gpio_pin $pin_num _wakeup_enable >]().bit(wakeup));
                    rtc_cntl.gpio_wakeup().modify(|_, w| w.[< gpio_pin $pin_num _int_type >]().bits(level));
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

    ( $( $pin_num:expr )+ ) => { $( $crate::gpio::rtc_pins!($pin_num); )+ };
}

// Following code enables `into_analog`

#[doc(hidden)]
pub fn enable_iomux_clk_gate() {
    #[cfg(esp32s2)]
    {
        use crate::peripherals::SENS;
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
#[doc(hidden)]
#[macro_export]
macro_rules! analog {
    (
        $(
            (
                $pin_num:expr, $rtc_pin:expr, $pin_reg:expr,
                $mux_sel:ident, $fun_sel:ident, $fun_ie:ident $(, $rue:ident, $rde:ident)?
            )
        )+
    ) => {
        pub(crate) fn internal_into_analog(pin: u8) {
            use $crate::peripherals::RTC_IO;

            let rtcio = unsafe{ &*RTC_IO::ptr() };
            $crate::gpio::enable_iomux_clk_gate();

            match pin {
                $(
                    $pin_num => {
                        // We need `paste` (and a [< >] in it) to rewrite the token stream to
                        // handle indexed pins.
                        paste::paste! {
                            // disable input
                            rtcio.$pin_reg.modify(|_,w| w.$fun_ie().bit([< false >]));

                            // disable output
                            rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });

                            // disable open drain
                            rtcio.pin($rtc_pin).modify(|_,w| w.pad_driver().bit(false));

                            rtcio.$pin_reg.modify(|_,w| {
                                w.$fun_ie().clear_bit();

                                // Connect pin to analog / RTC module instead of standard GPIO
                                w.$mux_sel().set_bit();

                                // Select function "RTC function 1" (GPIO) for analog use
                                unsafe { w.$fun_sel().bits(0b00) }
                            });

                            // Disable pull-up and pull-down resistors on the pin, if it has them
                            $(
                                rtcio.$pin_reg.modify(|_,w| {
                                    w
                                    .$rue().bit(false)
                                    .$rde().bit(false)
                                });
                            )?
                        }
                    }
                )+
                    _ => unreachable!(),
            }
        }
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
#[doc(hidden)]
#[macro_export]
macro_rules! analog {
    (
        $($pin_num:literal)+
    ) => {
        pub(crate) fn internal_into_analog(pin: u8) {
            use $crate::peripherals::IO_MUX;
            use $crate::peripherals::GPIO;

            let io_mux = unsafe{ &*IO_MUX::PTR };
            let gpio = unsafe{ &*GPIO::PTR };

            match pin {
                $(
                    $pin_num => {
                        io_mux.gpio($pin_num).modify(|_,w| unsafe {
                            w.mcu_sel().bits(1)
                                .fun_ie().clear_bit()
                                .fun_wpu().clear_bit()
                                .fun_wpd().clear_bit()
                        });

                        gpio.enable_w1tc().write(|w| unsafe { w.bits(1 << $pin_num) });
                    }
                )+
                _ => unreachable!()
            }

        }
    }
}

/// normal touch pin initialization. This is separate from touch_common, as we
/// need to handle some touch_pads differently
#[doc(hidden)]
#[macro_export]
macro_rules! touch_into {
    (
        $( ( $touch_num:expr, $pin_num:expr, $rtc_pin:expr, $touch_thres_reg:expr, $touch_thres_field:expr , true ) )+
        ---
        $( ( $touch_numx:expr, $pin_numx:expr, $rtc_pinx:expr, $touch_thres_regx:expr , $touch_thres_fieldx:expr , false ) )*
    ) => {
        pub(crate) fn internal_into_touch(pin: u8) {
            use $crate::peripherals::{GPIO, RTC_IO, SENS};

            let rtcio = unsafe { &*RTC_IO::ptr() };
            let sens = unsafe { &*SENS::ptr() };
            let gpio = unsafe { &*GPIO::ptr() };

            match pin {
                $(
                    $pin_num => {
                        paste::paste! {
                            // Pad to normal mode (not open-drain)
                            gpio.pin($rtc_pin).write(|w| w.pad_driver().clear_bit());

                            // clear output
                            rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });
                            sens
                                . $touch_thres_reg ()
                                .write(|w| unsafe {
                                    w. $touch_thres_field ().bits(
                                        0b0 // Default: 0 for esp32 gets overridden later anyway.
                                    )
                                });

                            rtcio.[< touch_pad $touch_num >]().write(|w| unsafe {
                                w
                                .xpd().set_bit()
                                // clear input_enable
                                .fun_ie().clear_bit()
                                // Connect pin to analog / RTC module instead of standard GPIO
                                .mux_sel().set_bit()
                                // Disable pull-up and pull-down resistors on the pin
                                .rue().clear_bit()
                                .rde().clear_bit()
                                .tie_opt().clear_bit()
                                // Select function "RTC function 1" (GPIO) for analog use
                                .fun_sel().bits(0b00)
                            });

                            sens.sar_touch_enable().modify(|r, w| unsafe {
                                w
                                // enable the pin
                                .touch_pad_worken().bits(
                                   r.touch_pad_worken().bits() | ( 1 << [< $touch_num >] )
                                )
                            });
                        }
                    }
                )+

                $(
                    $pin_numx => {
                        paste::paste! {
                            // Pad to normal mode (not open-drain)
                            gpio.pin($rtc_pinx).write(|w| w.pad_driver().clear_bit());

                            // clear output
                            rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pinx) });
                            sens
                                . $touch_thres_regx ()
                                .write(|w| unsafe {
                                    w. $touch_thres_fieldx ().bits(
                                        0b0 // Default: 0 for esp32 gets overridden later anyway.
                                    )
                                });

                            rtcio.[< touch_pad $touch_numx >]().write(|w|
                                w
                                .xpd().set_bit()
                                .tie_opt().clear_bit()
                            );

                            sens.sar_touch_enable().modify(|r, w| unsafe {
                                w
                                // enable the pin
                                .touch_pad_worken().bits(
                                   r.touch_pad_worken().bits() | ( 1 << [< $touch_numx >] )
                                )
                            });
                        }
                    }
                )*
                _ => unreachable!(),
            }
        }
    };
}

/// Common functionality for all touch pads
#[doc(hidden)]
#[macro_export]
macro_rules! touch_common {
    (
        $(
            (
                $touch_num:expr, $pin_num:expr, $touch_out_reg:expr, $meas_field: expr, $touch_thres_reg:expr, $touch_thres_field:expr
            )
        )+
    ) => {
        pub(crate) fn internal_get_touch_measurement(pin: u8) -> u16 {
            match pin {
                $(
                    $pin_num => {
                        paste::paste! {
                        unsafe { &* $crate::peripherals::SENS::ptr() }
                            . $touch_out_reg ()
                            .read()
                            . $meas_field ()
                            .bits()
                        }
                    },
                )+
                _ => unreachable!(),
            }
        }
        pub(crate) fn internal_get_touch_nr(pin: u8) -> u8 {
            match pin {
                $($pin_num => $touch_num,)+
                _ => unreachable!(),
            }
        }
        pub(crate) fn internal_set_threshold(pin: u8, threshold: u16) {
            match pin {
                $(
                    $pin_num => {
                        paste::paste! {
                            unsafe { &* $crate::peripherals::SENS::ptr() }
                                . $touch_thres_reg ()
                                .write(|w| unsafe {
                                    w. $touch_thres_field ().bits(threshold)
                                });
                        }
                    },
                )+
                _ => unreachable!(),
            }
        }
    };
}

/// GPIO output driver.
pub struct Output<'d, P> {
    pin: PeripheralRef<'d, P>,
}

impl<'d, P> Output<'d, P>
where
    P: OutputPin,
{
    /// Create GPIO output driver for a [GpioPin] with the provided level
    #[inline]
    pub fn new(pin: impl crate::peripheral::Peripheral<P = P> + 'd, initial_output: Level) -> Self {
        crate::into_ref!(pin);

        pin.set_output_high(initial_output.into(), private::Internal);
        pin.set_to_push_pull_output(private::Internal);

        Self { pin }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let level = !self.pin.is_set_high(private::Internal);
        self.pin.set_output_high(level, private::Internal);
    }

    /// Configure the [DriveStrength] of the pin
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength, private::Internal);
    }
}

/// GPIO input driver.
pub struct Input<'d, P> {
    pin: PeripheralRef<'d, P>,
}

impl<'d, P> Input<'d, P>
where
    P: InputPin,
{
    /// Create GPIO input driver for a [Pin] with the provided [Pull]
    /// configuration.
    #[inline]
    pub fn new(pin: impl crate::peripheral::Peripheral<P = P> + 'd, pull: Pull) -> Self {
        crate::into_ref!(pin);
        pin.init_input(pull == Pull::Down, pull == Pull::Up, private::Internal);
        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
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
}

/// GPIO open-drain output driver.
pub struct OutputOpenDrain<'d, P> {
    pin: PeripheralRef<'d, P>,
}

impl<'d, P> OutputOpenDrain<'d, P>
where
    P: InputPin + OutputPin,
{
    /// Create GPIO open-drain output driver for a [Pin] with the provided
    /// initial output-level and [Pull] configuration.
    #[inline]
    pub fn new(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
        initial_output: Level,
        pull: Pull,
    ) -> Self {
        crate::into_ref!(pin);
        pin.set_output_high(initial_output.into(), private::Internal);
        pin.set_to_open_drain_output(private::Internal);
        pin.internal_pull_down(pull == Pull::Down, private::Internal);
        pin.internal_pull_up(pull == Pull::Up, private::Internal);

        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let level = !self.pin.is_set_high(private::Internal);
        self.pin.set_output_high(level, private::Internal);
    }

    /// Configure the [DriveStrength] of the pin
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength, private::Internal);
    }
}

/// GPIO flexible pin driver.
pub struct Flex<'d, P> {
    pin: PeripheralRef<'d, P>,
}

impl<'d, P> Flex<'d, P>
where
    P: InputPin + OutputPin,
{
    /// Create GPIO flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    pub fn new(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self {
        crate::into_ref!(pin);
        Self { pin }
    }

    /// Set the GPIO to open-drain mode.
    pub fn set_as_open_drain(&mut self, pull: Pull) {
        self.pin.set_to_open_drain_output(private::Internal);
        self.pin
            .internal_pull_down(pull == Pull::Down, private::Internal);
        self.pin
            .internal_pull_up(pull == Pull::Up, private::Internal);
    }

    /// Set the GPIO to input mode.
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin
            .init_input(pull == Pull::Down, pull == Pull::Up, private::Internal);
    }

    /// Set the GPIO to output mode.
    pub fn set_as_output(&mut self) {
        self.pin.set_to_push_pull_output(private::Internal);
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let level = !self.pin.is_set_high(private::Internal);
        self.pin.set_output_high(level, private::Internal);
    }

    /// Configure the [DriveStrength] of the pin
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength, private::Internal);
    }
}

/// Generic GPIO output driver.
pub struct AnyOutput<'d> {
    pin: ErasedPin,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyOutput<'d> {
    /// Create GPIO output driver for a [GpioPin] with the provided level
    #[inline]
    pub fn new<P: OutputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
        initial_output: Level,
    ) -> Self {
        crate::into_ref!(pin);

        pin.set_output_high(initial_output.into(), private::Internal);
        pin.set_to_push_pull_output(private::Internal);

        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let pin = &mut self.pin;
        pin.set_output_high(!pin.is_set_high(private::Internal), private::Internal);
    }
}

/// Generic GPIO input driver.
pub struct AnyInput<'d> {
    pin: ErasedPin,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyInput<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull]
    /// configuration.
    #[inline]
    pub fn new<P: InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
        pull: Pull,
    ) -> Self {
        crate::into_ref!(pin);
        pin.init_input(pull == Pull::Down, pull == Pull::Up, private::Internal);

        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }
}

/// Generic GPIO open-drain output driver.
pub struct AnyOutputOpenDrain<'d> {
    pin: ErasedPin,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyOutputOpenDrain<'d> {
    /// Create GPIO open-drain output driver for a [Pin] with the provided
    /// initial output-level and [Pull] configuration.
    #[inline]
    pub fn new<P: OutputPin + InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
        initial_output: Level,
        pull: Pull,
    ) -> Self {
        crate::into_ref!(pin);
        pin.set_output_high(initial_output.into(), private::Internal);
        pin.internal_pull_down(pull == Pull::Down, private::Internal);
        pin.internal_pull_up(pull == Pull::Up, private::Internal);
        pin.set_to_open_drain_output(private::Internal);

        let pin = pin.erased_pin(private::Internal);

        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let pin = &mut self.pin;
        pin.set_output_high(!pin.is_set_high(private::Internal), private::Internal);
    }
}

/// Generic GPIO flexible pin driver.
pub struct AnyFlex<'d> {
    pin: ErasedPin,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> AnyFlex<'d> {
    /// Create GPIO flexible pin driver for a [Pin].
    /// No mode change happens.
    #[inline]
    pub fn new<P: OutputPin + InputPin + CreateErasedPin>(
        pin: impl crate::peripheral::Peripheral<P = P> + 'd,
    ) -> Self {
        crate::into_ref!(pin);
        let pin = pin.erased_pin(private::Internal);
        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Set the GPIO to open-drain mode.
    pub fn set_as_open_drain(&mut self, pull: Pull) {
        self.pin.set_to_open_drain_output(private::Internal);
        self.pin
            .internal_pull_down(pull == Pull::Down, private::Internal);
        self.pin
            .internal_pull_up(pull == Pull::Up, private::Internal);
    }

    /// Set the GPIO to input mode.
    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin
            .init_input(pull == Pull::Down, pull == Pull::Up, private::Internal);
    }

    /// Set the GPIO to output mode.
    pub fn set_as_output(&mut self) {
        self.pin.set_to_push_pull_output(private::Internal);
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_input_high(private::Internal)
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Listen for interrupts
    #[inline]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event, private::Internal);
    }

    /// Clear the interrupt status bit for this Pin
    #[inline]
    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt(private::Internal);
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_output_high(true, private::Internal);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_output_high(false, private::Internal);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_high(level.into(), private::Internal);
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high(private::Internal)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.pin.is_set_high(private::Internal)
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.is_set_high(private::Internal).into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        let pin = &mut self.pin;
        pin.set_output_high(!pin.is_set_high(private::Internal), private::Internal);
    }
}

pub(crate) mod internal {
    use super::*;

    impl private::Sealed for ErasedPin {}

    impl Pin for ErasedPin {
        fn number(&self, _: private::Internal) -> u8 {
            handle_gpio_input!(self, target, { Pin::number(target, private::Internal) })
        }

        fn sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(self, target, {
                Pin::sleep_mode(target, on, private::Internal)
            })
        }

        fn set_alternate_function(&mut self, alternate: AlternateFunction, _: private::Internal) {
            handle_gpio_input!(self, target, {
                Pin::set_alternate_function(target, alternate, private::Internal)
            })
        }

        fn is_listening(&self, _: private::Internal) -> bool {
            handle_gpio_input!(self, target, {
                Pin::is_listening(target, private::Internal)
            })
        }

        fn listen_with_options(
            &mut self,
            event: Event,
            int_enable: bool,
            nmi_enable: bool,
            wake_up_from_light_sleep: bool,
            _: private::Internal,
        ) {
            handle_gpio_input!(self, target, {
                Pin::listen_with_options(
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
            handle_gpio_input!(self, target, { Pin::unlisten(target, private::Internal) })
        }

        fn is_interrupt_set(&self, _: private::Internal) -> bool {
            handle_gpio_input!(self, target, {
                Pin::is_interrupt_set(target, private::Internal)
            })
        }

        fn clear_interrupt(&mut self, _: private::Internal) {
            handle_gpio_input!(self, target, {
                Pin::clear_interrupt(target, private::Internal)
            })
        }

        fn wakeup_enable(&mut self, enable: bool, event: WakeEvent, _: private::Internal) {
            handle_gpio_input!(self, target, {
                Pin::wakeup_enable(target, enable, event, private::Internal)
            })
        }
    }

    impl InputPin for ErasedPin {
        fn init_input(&self, pull_down: bool, pull_up: bool, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::init_input(target, pull_down, pull_up, private::Internal)
            })
        }

        fn set_to_input(&mut self, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::set_to_input(target, private::Internal);
            });
        }

        fn enable_input(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::enable_input(target, on, private::Internal)
            });
        }

        fn enable_input_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::enable_input_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn is_input_high(&self, _: private::Internal) -> bool {
            handle_gpio_input!(self, target, {
                InputPin::is_input_high(target, private::Internal)
            })
        }

        fn connect_input_to_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::connect_input_to_peripheral(target, signal, private::Internal)
            });
        }

        fn connect_input_to_peripheral_with_options(
            &mut self,
            signal: InputSignal,
            invert: bool,
            force_via_gpio_mux: bool,
            _: private::Internal,
        ) {
            handle_gpio_input!(self, target, {
                InputPin::connect_input_to_peripheral_with_options(
                    target,
                    signal,
                    invert,
                    force_via_gpio_mux,
                    private::Internal,
                )
            });
        }

        fn disconnect_input_from_peripheral(&mut self, signal: InputSignal, _: private::Internal) {
            handle_gpio_input!(self, target, {
                InputPin::disconnect_input_from_peripheral(target, signal, private::Internal)
            });
        }
    }

    impl OutputPin for ErasedPin {
        fn set_to_open_drain_output(&mut self, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::set_to_open_drain_output(target, private::Internal)
            });
        }

        fn set_to_push_pull_output(&mut self, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::set_to_push_pull_output(target, private::Internal)
            });
        }

        fn enable_output(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::enable_output(target, on, private::Internal)
            });
        }

        fn set_output_high(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::set_output_high(target, on, private::Internal)
            });
        }

        fn set_drive_strength(&mut self, strength: DriveStrength, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::set_drive_strength(target, strength, private::Internal)
            });
        }

        fn enable_open_drain(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::enable_open_drain(target, on, private::Internal)
            });
        }

        fn enable_output_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::enable_output_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn internal_pull_up_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::internal_pull_up_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn internal_pull_down_in_sleep_mode(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::internal_pull_down_in_sleep_mode(target, on, private::Internal)
            });
        }

        fn internal_pull_up(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::internal_pull_up(target, on, private::Internal)
            });
        }

        fn internal_pull_down(&mut self, on: bool, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::internal_pull_down(target, on, private::Internal)
            });
        }

        fn connect_peripheral_to_output(&mut self, signal: OutputSignal, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::connect_peripheral_to_output(target, signal, private::Internal)
            });
        }

        fn connect_peripheral_to_output_with_options(
            &mut self,
            signal: OutputSignal,
            invert: bool,
            invert_enable: bool,
            enable_from_gpio: bool,
            force_via_gpio_mux: bool,
            _: private::Internal,
        ) {
            handle_gpio_output!(self, target, {
                OutputPin::connect_peripheral_to_output_with_options(
                    target,
                    signal,
                    invert,
                    invert_enable,
                    enable_from_gpio,
                    force_via_gpio_mux,
                    private::Internal,
                )
            });
        }

        fn disconnect_peripheral_from_output(&mut self, _: private::Internal) {
            handle_gpio_output!(self, target, {
                OutputPin::disconnect_peripheral_from_output(target, private::Internal)
            });
        }

        fn is_set_high(&self, _: private::Internal) -> bool {
            handle_gpio_output!(self, target, {
                OutputPin::is_set_high(target, private::Internal)
            })
        }
    }
}

mod asynch {
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_AW: AtomicWaker = AtomicWaker::new();
    static PIN_WAKERS: [AtomicWaker; NUM_PINS] = [NEW_AW; NUM_PINS];

    impl<'d, P> Input<'d, P>
    where
        P: InputPin,
    {
        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            self.listen(Event::HighLevel);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            self.listen(Event::LowLevel);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            self.listen(Event::RisingEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            self.listen(Event::FallingEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            self.listen(Event::AnyEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }
    }

    impl<'d> AnyInput<'d> {
        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            self.listen(Event::HighLevel);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            self.listen(Event::LowLevel);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            self.listen(Event::RisingEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            self.listen(Event::FallingEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            self.listen(Event::AnyEdge);
            PinFuture::new(self.pin.number(private::Internal)).await
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct PinFuture {
        pin_num: u8,
    }

    impl PinFuture {
        pub fn new(pin_num: u8) -> Self {
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

    pub(crate) fn is_listening(gpio_num: u8) -> bool {
        let bits = unsafe { &*GPIO::PTR }
            .pin(gpio_num as usize)
            .read()
            .int_ena()
            .bits();
        bits != 0
    }

    pub(crate) fn set_int_enable(
        gpio_num: u8,
        int_ena: u8,
        int_type: u8,
        wake_up_from_light_sleep: bool,
    ) {
        let gpio = unsafe { &*crate::peripherals::GPIO::PTR };
        gpio.pin(gpio_num as usize).modify(|_, w| unsafe {
            w.int_ena()
                .bits(int_ena)
                .int_type()
                .bits(int_type)
                .wakeup_enable()
                .bit(wake_up_from_light_sleep)
        });
    }

    #[ram]
    pub(super) fn handle_gpio_interrupt() {
        let intrs_bank0 = InterruptStatusRegisterAccessBank0::interrupt_status_read();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let intrs_bank1 = InterruptStatusRegisterAccessBank1::interrupt_status_read();

        let mut intr_bits = intrs_bank0;
        while intr_bits != 0 {
            let pin_nr = intr_bits.trailing_zeros();
            set_int_enable(pin_nr as u8, 0, 0, false);
            PIN_WAKERS[pin_nr as usize].wake(); // wake task
            intr_bits -= 1 << pin_nr;
        }

        // clear interrupt bits
        Bank0GpioRegisterAccess::write_interrupt_status_clear(intrs_bank0);

        #[cfg(any(esp32, esp32s2, esp32s3))]
        {
            let mut intr_bits = intrs_bank1;
            while intr_bits != 0 {
                let pin_nr = intr_bits.trailing_zeros();
                set_int_enable(pin_nr as u8 + 32, 0, 0, false);
                PIN_WAKERS[pin_nr as usize + 32].wake(); // wake task
                intr_bits -= 1 << pin_nr;
            }
            Bank1GpioRegisterAccess::write_interrupt_status_clear(intrs_bank1);
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
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
        }
    }

    impl<'d, P> digital::OutputPin for Output<'d, P>
    where
        P: OutputPin,
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
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
        }
    }

    impl<'d, P> digital::OutputPin for OutputOpenDrain<'d, P>
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
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
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

    impl<'d> digital::InputPin for AnyInput<'d> {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
        }
    }

    impl<'d> digital::OutputPin for AnyOutput<'d> {
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

    impl<'d> digital::StatefulOutputPin for AnyOutput<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> digital::ToggleableOutputPin for AnyOutput<'d> {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d> digital::InputPin for AnyOutputOpenDrain<'d> {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
        }
    }

    impl<'d> digital::OutputPin for AnyOutputOpenDrain<'d> {
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

    impl<'d> digital::StatefulOutputPin for AnyOutputOpenDrain<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> digital::ToggleableOutputPin for AnyOutputOpenDrain<'d> {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d> digital::InputPin for AnyFlex<'d> {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_input_high(private::Internal))
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_input_high(private::Internal))
        }
    }

    impl<'d> digital::OutputPin for AnyFlex<'d> {
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

    impl<'d> digital::StatefulOutputPin for AnyFlex<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> digital::ToggleableOutputPin for AnyFlex<'d> {
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
            Ok(Input::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Input::is_low(self))
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
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
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
            Ok(OutputOpenDrain::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(OutputOpenDrain::is_low(self))
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
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
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
        P: InputPin + OutputPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Flex::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Flex::is_low(self))
        }
    }

    impl<'d, P> digital::ErrorType for Flex<'d, P>
    where
        P: InputPin + OutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<'d, P> digital::OutputPin for Flex<'d, P>
    where
        P: InputPin + OutputPin,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<'d, P> digital::StatefulOutputPin for Flex<'d, P>
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

    impl<'d> digital::ErrorType for AnyInput<'d> {
        type Error = core::convert::Infallible;
    }

    impl<'d> digital::InputPin for AnyInput<'d> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(AnyInput::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(AnyInput::is_low(self))
        }
    }

    impl<'d> digital::ErrorType for AnyOutput<'d> {
        type Error = core::convert::Infallible;
    }

    impl<'d> digital::OutputPin for AnyOutput<'d> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<'d> digital::StatefulOutputPin for AnyOutput<'d> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl<'d> digital::ErrorType for AnyOutputOpenDrain<'d> {
        type Error = core::convert::Infallible;
    }

    impl<'d> digital::OutputPin for AnyOutputOpenDrain<'d> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<'d> digital::StatefulOutputPin for AnyOutputOpenDrain<'d> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl<'d> digital::ErrorType for AnyFlex<'d> {
        type Error = core::convert::Infallible;
    }

    impl<'d> digital::OutputPin for AnyFlex<'d> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<'d> digital::StatefulOutputPin for AnyFlex<'d> {
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

    impl<'d, P> Wait for Input<'d, P>
    where
        P: InputPin,
    {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            self.wait_for_high().await;
            Ok(())
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            self.wait_for_low().await;
            Ok(())
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_rising_edge().await;
            Ok(())
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_falling_edge().await;
            Ok(())
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_any_edge().await;
            Ok(())
        }
    }

    impl<'d> Wait for AnyInput<'d> {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            self.wait_for_high().await;
            Ok(())
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            self.wait_for_low().await;
            Ok(())
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_rising_edge().await;
            Ok(())
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_falling_edge().await;
            Ok(())
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            self.wait_for_any_edge().await;
            Ok(())
        }
    }
}
