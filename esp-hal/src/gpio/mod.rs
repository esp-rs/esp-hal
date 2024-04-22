//! # General Purpose I/Os
//!
//! ## Overview
//! The GPIO peripheral provides access to General Purpose Input/Output pins on
//! ESP chips.
//!
//! This driver supports various operations on GPIO pins, including setting the
//! pin mode, direction, and manipulating the pin state (setting high/low,
//! toggling). It provides an interface to interact with GPIO pins on ESP chips,
//! allowing developers to control and read the state of the pins. This module
//! also implements a number of traits from [embedded-hal] to provide a common
//! interface for GPIO pins.
//!
//! To get access to the pins, you first need to convert them into a HAL
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `Io::new`.
//!
//! ## Example
//! ```no_run
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio5.into_push_pull_output();
//! ```
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
#![warn(missing_docs)]

use core::{cell::Cell, marker::PhantomData};

use critical_section::Mutex;

#[cfg(any(adc, dac))]
pub(crate) use crate::analog;
pub(crate) use crate::gpio;
#[cfg(any(xtensa, esp32c3))]
pub(crate) use crate::rtc_pins;
pub use crate::soc::gpio::*;
use crate::{
    interrupt::InterruptHandler,
    peripherals::{GPIO, IO_MUX},
};

#[cfg(soc_etm)]
pub mod etm;
#[cfg(lp_io)]
pub mod lp_io;
#[cfg(all(rtc_io, not(esp32)))]
pub mod rtc_io;

/// Convenience type-alias for a no-pin / don't care - pin
pub type NoPinType = Gpio0<Unknown>;

/// Convenience constant for `Option::None` pin
pub const NO_PIN: Option<NoPinType> = None;

static USER_INTERRUPT_HANDLER: Mutex<Cell<Option<InterruptHandler>>> = Mutex::new(Cell::new(None));

/// Event used to trigger interrupts.
#[derive(Copy, Clone)]
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

/// Unknown pin mode
pub struct Unknown {}

/// Input pin mode
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Inverted input pin mode
pub struct InvertedInput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Used to decide if the pin is inverted or not when the pin gets connected to
/// a peripheral
trait InputMode {
    const PIN_IS_INVERTED: bool;
}

impl<MODE> InputMode for Input<MODE> {
    const PIN_IS_INVERTED: bool = false;
}

impl<MODE> InputMode for InvertedInput<MODE> {
    const PIN_IS_INVERTED: bool = true;
}

impl InputMode for Unknown {
    const PIN_IS_INVERTED: bool = false;
}

/// RTC input pin mode
pub struct RtcInput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating mode
pub struct Floating;

/// Pull-down mode
pub struct PullDown;

/// Pull-up mode
pub struct PullUp;

/// Output pin mode
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Inverted output pin mode
pub struct InvertedOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Used to decide if the pin is inverted or not when the pin gets connected to
/// a peripheral
trait OutputMode {
    const PIN_IS_INVERTED: bool;
}

impl<MODE> OutputMode for Output<MODE> {
    const PIN_IS_INVERTED: bool = false;
}

impl<MODE> OutputMode for InvertedOutput<MODE> {
    const PIN_IS_INVERTED: bool = true;
}

impl OutputMode for Unknown {
    const PIN_IS_INVERTED: bool = false;
}

/// RTC output pin mode
pub struct RtcOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Open-drain mode
pub struct OpenDrain;

/// Push-pull mode
pub struct PushPull;

/// Analog mode
pub struct Analog;

/// Drive strength (values are approximates)
#[allow(missing_docs)]
pub enum DriveStrength {
    I5mA  = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

/// Alternate functions
#[derive(PartialEq)]
#[allow(missing_docs)]
pub enum AlternateFunction {
    Function0 = 0,
    Function1 = 1,
    Function2 = 2,
    Function3 = 3,
    Function4 = 4,
    Function5 = 5,
}

/// RTC function
#[derive(PartialEq)]
#[allow(missing_docs)]
pub enum RtcFunction {
    Rtc     = 0,
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

/// Marker for pins which support analog mode
pub trait AnalogPin {}

/// Common trait implemented by pins
pub trait Pin: crate::private::Sealed {
    /// GPIO number
    fn number(&self) -> u8;

    /// Enable/disable sleep-mode
    fn sleep_mode(&mut self, on: bool);

    /// Configure the alternate function
    fn set_alternate_function(&mut self, alternate: AlternateFunction);

    /// Listen for interrupts
    fn listen(&mut self, event: Event) {
        self.listen_with_options(event, true, false, false)
    }

    /// Checks if listening for interrupts is enabled for this Pin
    fn is_listening(&self) -> bool;

    /// Listen for interrupts
    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    );

    /// Stop listening for interrupts
    fn unlisten(&mut self);

    /// Checks if the interrupt status bit for this Pin is set
    fn is_interrupt_set(&self) -> bool;

    /// Clear the interrupt status bit for this Pin
    fn clear_interrupt(&mut self);
}

/// Trait implemented by pins which can be used as inputs
pub trait InputPin: Pin {
    /// Set the pin to input mode without internal pull-up / pull-down resistors
    fn set_to_input(&mut self) -> &mut Self;

    /// Enable input for the pin
    fn enable_input(&mut self, on: bool) -> &mut Self;

    /// Enable input in sleep mode for the pin
    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// The current state of the input
    fn is_input_high(&self) -> bool;

    /// Connect the pin to a peripheral input signal
    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self;

    /// Connect the pin to a peripheral input signal.
    ///
    /// Optionally invert the signal. When `force_via_gpio_mux` is true it will
    /// won't use the alternate function even if it matches
    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;

    /// Remove a connected `signal` from this input pin.
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal) -> &mut Self;
}

/// Trait implemented by pins which can be used as outputs
pub trait OutputPin: Pin {
    /// Configure open-drain mode
    fn set_to_open_drain_output(&mut self) -> &mut Self;

    /// Configure output mode
    fn set_to_push_pull_output(&mut self) -> &mut Self;

    /// Enable/disable the pin as output
    fn enable_output(&mut self, on: bool) -> &mut Self;

    /// Set the pin's level to high or low
    fn set_output_high(&mut self, on: bool) -> &mut Self;

    /// Configure the [DriveStrength] of the pin
    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self;

    /// Enable/disable open-drain mode
    fn enable_open_drain(&mut self, on: bool) -> &mut Self;

    /// Enable/disable output in sleep mode
    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Configure internal pull-up resistor in sleep mode
    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Configure internal pull-down resistor in sleep mode
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Enable/disable internal pull-up resistor for normal operation
    fn internal_pull_up(&mut self, on: bool) -> &mut Self;

    /// Enable/disable internal pull-down resistor for normal operation
    fn internal_pull_down(&mut self, on: bool) -> &mut Self;

    /// Connect the pin to a peripheral output signal
    fn connect_peripheral_to_output(&mut self, signal: OutputSignal) -> &mut Self;

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
    ) -> &mut Self;

    /// Remove this output pin from a connected [signal](`InputSignal`).
    ///
    /// Clears the entry in the GPIO matrix / Io mux that associates this output
    /// pin with a previously connected [signal](`InputSignal`). Any other
    /// outputs connected to the signal remain intact.
    fn disconnect_peripheral_from_output(&mut self) -> &mut Self;
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
pub struct InputOutputPinType;

#[doc(hidden)]
pub struct InputOnlyPinType;

#[doc(hidden)]
pub struct InputOutputAnalogPinType;

#[doc(hidden)]
pub struct InputOnlyAnalogPinType;

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

/// GPIO pin
pub struct GpioPin<MODE, const GPIONUM: u8> {
    _mode: PhantomData<MODE>,
}

impl<MODE, const GPIONUM: u8> GpioPin<Input<MODE>, GPIONUM>
where
    Self: GpioProperties,
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

impl<const GPIONUM: u8> GpioPin<Output<OpenDrain>, GPIONUM>
where
    Self: GpioProperties,
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

impl<MODE, const GPIONUM: u8> GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
{
    pub(crate) fn new() -> Self {
        Self { _mode: PhantomData }
    }

    fn init_input(&self, pull_down: bool, pull_up: bool) {
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

    /// Configures the pin to operate as a floating input pin
    pub fn into_floating_input(self) -> GpioPin<Input<Floating>, GPIONUM> {
        self.init_input(false, false);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as a pulled up input pin
    pub fn into_pull_up_input(self) -> GpioPin<Input<PullUp>, GPIONUM> {
        self.init_input(false, true);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as a pulled down input pin
    pub fn into_pull_down_input(self) -> GpioPin<Input<PullDown>, GPIONUM> {
        self.init_input(true, false);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an inverted floating input pin.
    /// Only suitable to be passed into a peripheral driver.
    pub fn into_inverted_floating_input(self) -> GpioPin<InvertedInput<Floating>, GPIONUM> {
        self.init_input(false, false);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an inverted pulled up input pin.
    /// Only suitable to be passed into a peripheral driver.
    pub fn into_inverted_pull_up_input(self) -> GpioPin<InvertedInput<PullUp>, GPIONUM> {
        self.init_input(false, true);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an inverted pulled down input pin.
    /// Only suitable to be passed into a peripheral driver.
    pub fn into_inverted_pull_down_input(self) -> GpioPin<InvertedInput<PullDown>, GPIONUM> {
        self.init_input(true, false);
        GpioPin { _mode: PhantomData }
    }
}

impl<MODE, const GPIONUM: u8> InputPin for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    MODE: InputMode,
{
    fn set_to_input(&mut self) -> &mut Self {
        self.init_input(false, false);
        self
    }
    fn enable_input(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_ie().bit(on));
        self
    }
    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_ie().bit(on));
        self
    }
    fn is_input_high(&self) -> bool {
        <Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        self.connect_input_to_peripheral_with_options(
            signal,
            MODE::PIN_IS_INVERTED,
            MODE::PIN_IS_INVERTED,
        )
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self {
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
        self.set_alternate_function(af);
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
        self
    }

    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        self.set_alternate_function(GPIO_FUNCTION);

        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(signal as usize - FUNC_IN_SEL_OFFSET)
            .modify(|_, w| w.sel().clear_bit());
        self
    }
}

impl<const GPIONUM: u8> GpioPin<Unknown, GPIONUM>
where
    Self: GpioProperties,
{
    /// Create a pin out of thin air.
    ///
    /// # Safety
    ///
    /// Ensure that only one instance of a pin exists at one time.
    pub unsafe fn steal() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<MODE, const GPIONUM: u8> Pin for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
{
    fn number(&self) -> u8 {
        GPIONUM
    }

    fn sleep_mode(&mut self, on: bool) {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.slp_sel().bit(on));
    }

    fn set_alternate_function(&mut self, alternate: AlternateFunction) {
        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
    }

    fn listen_with_options(
        &mut self,
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

    fn is_listening(&self) -> bool {
        let bits = unsafe { &*GPIO::PTR }
            .pin(GPIONUM as usize)
            .read()
            .int_ena()
            .bits();
        bits != 0
    }

    fn unlisten(&mut self) {
        unsafe {
            (*GPIO::PTR)
                .pin(GPIONUM as usize)
                .modify(|_, w| w.int_ena().bits(0).int_type().bits(0).int_ena().bits(0));
        }
    }

    fn is_interrupt_set(&self) -> bool {
        <Self as GpioProperties>::Bank::read_interrupt_status() & 1 << (GPIONUM % 32) != 0
    }

    fn clear_interrupt(&mut self) {
        <Self as GpioProperties>::Bank::write_interrupt_status_clear(1 << (GPIONUM % 32));
    }
}

impl<MODE, const GPIONUM: u8> GpioPin<Output<MODE>, GPIONUM>
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

impl<MODE, const GPIONUM: u8> crate::peripheral::Peripheral for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
{
    type P = GpioPin<MODE, GPIONUM>;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

impl<MODE, const GPIONUM: u8> crate::private::Sealed for GpioPin<MODE, GPIONUM> where
    Self: GpioProperties
{
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Input<Floating>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_floating_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>>
    for GpioPin<InvertedInput<Floating>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_inverted_floating_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Input<PullUp>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_pull_up_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<InvertedInput<PullUp>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_inverted_pull_up_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Input<PullDown>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_pull_down_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>>
    for GpioPin<InvertedInput<PullDown>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_inverted_pull_down_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Output<PushPull>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_push_pull_output()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>>
    for GpioPin<InvertedOutput<PushPull>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_inverted_push_pull_output()
    }
}

#[cfg(any(adc, dac))]
impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Analog, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsAnalogPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsAnalogPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_analog()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Output<OpenDrain>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_open_drain_output()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>>
    for GpioPin<InvertedOutput<OpenDrain>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_inverted_open_drain_output()
    }
}

impl<MODE, const GPIONUM: u8> GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
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

    /// Configures the pin to operate as an push pull output pin
    pub fn into_push_pull_output(self) -> GpioPin<Output<PushPull>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, false);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an open drain output pin
    pub fn into_open_drain_output(self) -> GpioPin<Output<OpenDrain>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, true);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an inverted push pull output pin.
    /// Only suitable to be passed into an peripheral driver
    pub fn into_inverted_push_pull_output(self) -> GpioPin<InvertedOutput<PushPull>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, false);
        GpioPin { _mode: PhantomData }
    }

    /// Configures the pin to operate as an open drain output pin.
    /// Only suitable to be passed into an peripheral driver
    pub fn into_inverted_open_drain_output(self) -> GpioPin<InvertedOutput<OpenDrain>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, true);
        GpioPin { _mode: PhantomData }
    }
}

impl<MODE, const GPIONUM: u8> OutputPin for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    MODE: OutputMode,
{
    fn set_to_open_drain_output(&mut self) -> &mut Self {
        self.init_output(GPIO_FUNCTION, true);
        self
    }

    fn set_to_push_pull_output(&mut self) -> &mut Self {
        self.init_output(GPIO_FUNCTION, false);
        self
    }

    fn enable_output(&mut self, on: bool) -> &mut Self {
        if on {
            <Self as GpioProperties>::Bank::write_out_en_set(1 << (GPIONUM % 32));
        } else {
            <Self as GpioProperties>::Bank::write_out_en_clear(1 << (GPIONUM % 32));
        }
        self
    }

    fn set_output_high(&mut self, high: bool) -> &mut Self {
        if high {
            <Self as GpioProperties>::Bank::write_output_set(1 << (GPIONUM % 32));
        } else {
            <Self as GpioProperties>::Bank::write_output_clear(1 << (GPIONUM % 32));
        }
        self
    }

    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });

        self
    }

    fn enable_open_drain(&mut self, on: bool) -> &mut Self {
        unsafe { &*GPIO::PTR }
            .pin(GPIONUM as usize)
            .modify(|_, w| w.pad_driver().bit(on));
        self
    }

    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_wpu().bit(on));
        self
    }
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_wpd().bit(on));
        self
    }
    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.mcu_oe().bit(on));
        self
    }

    fn internal_pull_up(&mut self, on: bool) -> &mut Self {
        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, Some(on), None);

        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpu().bit(on));
        self
    }
    fn internal_pull_down(&mut self, on: bool) -> &mut Self {
        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, None, Some(on));

        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpd().bit(on));
        self
    }

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal) -> &mut Self {
        self.connect_peripheral_to_output_with_options(
            signal,
            MODE::PIN_IS_INVERTED,
            false,
            false,
            MODE::PIN_IS_INVERTED,
        )
    }

    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self {
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
        self.set_alternate_function(af);
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
        self
    }

    fn disconnect_peripheral_from_output(&mut self) -> &mut Self {
        self.set_alternate_function(GPIO_FUNCTION);
        unsafe { &*GPIO::PTR }
            .func_out_sel_cfg(GPIONUM as usize)
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });
        self
    }
}

#[cfg(any(adc, dac))]
impl<MODE, const GPIONUM: u8> GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsAnalogPin,
{
    /// Configures the pin into a an [Analog] pin.
    pub fn into_analog(self) -> GpioPin<Analog, GPIONUM> {
        crate::soc::gpio::internal_into_analog(GPIONUM);

        GpioPin { _mode: PhantomData }
    }
}

impl<MODE, TYPE> From<AnyPin<MODE, TYPE>> for AnyPin<MODE>
where
    TYPE: PinType,
{
    fn from(pin: AnyPin<MODE, TYPE>) -> Self {
        Self {
            inner: pin.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE, TYPE> crate::peripheral::Peripheral for AnyPin<MODE, TYPE>
where
    TYPE: PinType,
{
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        let inner = &mut self.inner;
        let this: AnyPin<MODE> = handle_gpio_input!(inner, target, {
            crate::peripheral::Peripheral::clone_unchecked(target).into()
        });
        Self {
            inner: this.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE, TYPE> crate::private::Sealed for AnyPin<MODE, TYPE> where TYPE: PinType {}

impl<MODE, TYPE> AnyPin<MODE, TYPE>
where
    TYPE: PinType,
{
    /// Degrade the pin to remove the pin number generics.
    pub fn degrade(self) -> AnyPin<MODE> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE> AnyPin<MODE, InputOutputPinType> {
    /// Convert the pin
    pub fn into_input_type(self) -> AnyPin<MODE, InputOnlyPinType> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE> AnyPin<MODE, InputOutputAnalogPinType> {
    /// Convert the pin
    pub fn into_input_type(self) -> AnyPin<MODE, InputOnlyPinType> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }

    /// Convert the pin
    pub fn into_input_output_type(self) -> AnyPin<MODE, InputOutputPinType> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }

    /// Convert the pin
    pub fn into_input_only_analog_type(self) -> AnyPin<MODE, InputOnlyAnalogPinType> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE> AnyPin<MODE, InputOnlyAnalogPinType> {
    /// Convert the pin
    pub fn into_input_type(self) -> AnyPin<MODE, InputOnlyPinType> {
        AnyPin {
            inner: self.inner,
            _type: core::marker::PhantomData,
        }
    }
}

impl<MODE, TYPE> Pin for AnyPin<MODE, TYPE>
where
    TYPE: PinType,
{
    fn number(&self) -> u8 {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { Pin::number(target) })
    }

    fn sleep_mode(&mut self, on: bool) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { Pin::sleep_mode(target, on) })
    }

    fn set_alternate_function(&mut self, alternate: AlternateFunction) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            Pin::set_alternate_function(target, alternate)
        })
    }

    fn is_listening(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { Pin::is_listening(target) })
    }

    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    ) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            Pin::listen_with_options(
                target,
                event,
                int_enable,
                nmi_enable,
                wake_up_from_light_sleep,
            )
        })
    }

    fn listen(&mut self, event: Event) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { Pin::listen(target, event) })
    }

    fn unlisten(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { Pin::unlisten(target) })
    }

    fn is_interrupt_set(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { Pin::is_interrupt_set(target) })
    }

    fn clear_interrupt(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { Pin::clear_interrupt(target) })
    }
}

impl<MODE, TYPE> InputPin for AnyPin<MODE, TYPE>
where
    MODE: InputMode,
    TYPE: IsInputPin,
{
    fn set_to_input(&mut self) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::set_to_input(target);
        });
        self
    }

    fn enable_input(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::enable_input(target, on);
        });
        self
    }

    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::enable_input_in_sleep_mode(target, on);
        });
        self
    }

    fn is_input_high(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { InputPin::is_input_high(target) })
    }

    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::connect_input_to_peripheral(target, signal);
        });
        self
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::connect_input_to_peripheral_with_options(
                target,
                signal,
                invert,
                force_via_gpio_mux,
            );
        });
        self
    }

    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, {
            InputPin::disconnect_input_from_peripheral(target, signal);
        });
        self
    }
}

impl<MODE, TYPE> OutputPin for AnyPin<MODE, TYPE>
where
    MODE: OutputMode,
    TYPE: IsOutputPin,
{
    fn set_to_open_drain_output(&mut self) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::set_to_open_drain_output(target);
        });
        self
    }

    fn set_to_push_pull_output(&mut self) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::set_to_push_pull_output(target);
        });
        self
    }

    fn enable_output(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::enable_output(target, on);
        });
        self
    }

    fn set_output_high(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::set_output_high(target, on);
        });
        self
    }

    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::set_drive_strength(target, strength);
        });
        self
    }

    fn enable_open_drain(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::enable_open_drain(target, on);
        });
        self
    }

    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::enable_output_in_sleep_mode(target, on);
        });
        self
    }

    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::internal_pull_up_in_sleep_mode(target, on);
        });
        self
    }

    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::internal_pull_down_in_sleep_mode(target, on);
        });
        self
    }

    fn internal_pull_up(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::internal_pull_up(target, on);
        });
        self
    }

    fn internal_pull_down(&mut self, on: bool) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::internal_pull_down(target, on);
        });
        self
    }

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::connect_peripheral_to_output(target, signal);
        });
        self
    }

    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::connect_peripheral_to_output_with_options(
                target,
                signal,
                invert,
                invert_enable,
                enable_from_gpio,
                force_via_gpio_mux,
            );
        });
        self
    }

    fn disconnect_peripheral_from_output(&mut self) -> &mut Self {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, {
            OutputPin::disconnect_peripheral_from_output(target);
        });
        self
    }
}

impl<MODE, TYPE> AnyPin<Input<MODE>, TYPE> {
    /// Is the input pin high?
    #[inline]
    pub fn is_high(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_high() })
    }

    /// Is the input pin low?
    #[inline]
    pub fn is_low(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_low() })
    }
}

impl<MODE, TYPE> AnyPin<Output<MODE>, TYPE> {
    /// Drives the pin low.
    #[inline]
    pub fn set_low(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_low() })
    }

    /// Drives the pin high.
    #[inline]
    pub fn set_high(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_high() })
    }

    /// Drives the pin high or low depending on the provided value.
    #[inline]
    pub fn set_state(&mut self, state: bool) {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_state(state) })
    }

    /// Is the pin in drive high mode?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_high() })
    }

    /// Is the pin in drive low mode?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_low() })
    }

    /// Toggle pin output.
    #[inline]
    pub fn toggle(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.toggle() })
    }
}

#[cfg(feature = "async")]
impl<MODE, TYPE> AnyPin<Input<MODE>, TYPE> {
    /// Wait until the pin is high. If it is already high, return immediately.
    pub async fn wait_for_high(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_high().await })
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    pub async fn wait_for_low(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_low().await })
    }

    /// Wait for the pin to undergo a transition from low to high.
    pub async fn wait_for_rising_edge(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_rising_edge().await })
    }

    /// Wait for the pin to undergo a transition from high to low.
    pub async fn wait_for_falling_edge(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_falling_edge().await })
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to
    /// low.
    pub async fn wait_for_any_edge(&mut self) {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_any_edge().await })
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

    /// Install the given interrupt handler replacing any previously set
    /// handler.
    ///
    /// When the async feature is enabled the handler will be called first and
    /// the internal async handler will run after. In that case it's
    /// important to not reset the interrupt status when mixing sync and
    /// async (i.e. using async wait) interrupt handling.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        critical_section::with(|cs| {
            crate::interrupt::enable(crate::peripherals::Interrupt::GPIO, handler.priority())
                .unwrap();
            USER_INTERRUPT_HANDLER.borrow(cs).set(Some(handler));
        });
    }
}

extern "C" fn gpio_interrupt_handler() {
    if let Some(user_handler) = critical_section::with(|cs| USER_INTERRUPT_HANDLER.borrow(cs).get())
    {
        user_handler.call();
    }

    #[cfg(feature = "async")]
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
                pub(crate) fn pins(&self) -> Pins {
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
                impl<MODE> $crate::gpio::GpioProperties for GpioPin<MODE,  $gpionum> {
                    type Bank = $crate::gpio::[< Bank $bank GpioRegisterAccess >];
                    type InterruptStatus = $crate::gpio::[< InterruptStatusRegisterAccessBank $bank >];
                    type Signals = [< Gpio $gpionum Signals >];
                    type PinType = $crate::gpio::[<$type PinType>];
                }

                #[doc(hidden)]
                pub struct [<Gpio $gpionum Signals>] {}

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
            )+

            /// Pins available on this chip
            #[allow(missing_docs)]
            pub struct Pins {
                $(
                    pub [< gpio $gpionum >] : GpioPin<Unknown, $gpionum>,
                )+
            }

            $(
                #[doc = concat!("Alias for GpioPin<MODE, ", $gpionum, ">")]
                pub type [<Gpio $gpionum >]<MODE> = GpioPin<MODE, $gpionum>;
            )+

            pub(crate) enum ErasedPin<MODE> {
                $(
                    [<Gpio $gpionum >]([<Gpio $gpionum >]<MODE>),
                )+
            }

            /// Generic pin
            ///
            /// This is useful e.g. if you need an array of pins.
            pub struct AnyPin<MODE, TYPE = ()> {
                pub(crate) inner: ErasedPin<MODE>,
                pub(crate) _type: core::marker::PhantomData<TYPE>,
            }

            $(
            impl<MODE> From< [<Gpio $gpionum >]<MODE> > for AnyPin<MODE, $crate::gpio::[<$type PinType>]> {
                fn from(value: [<Gpio $gpionum >]<MODE>) -> Self {
                    AnyPin {
                        inner: ErasedPin::[<Gpio $gpionum >](value),
                        _type: core::marker::PhantomData,
                    }
                }
            }

            impl<MODE> From< [<Gpio $gpionum >]<MODE> > for AnyPin<MODE> {
                fn from(value: [<Gpio $gpionum >]<MODE>) -> Self {
                    AnyPin {
                        inner: ErasedPin::[<Gpio $gpionum >](value),
                        _type: core::marker::PhantomData,
                    }
                }
            }

            impl<MODE> [<Gpio $gpionum >]<MODE> {
                pub fn degrade(self) -> AnyPin<MODE, $crate::gpio::[<$type PinType>]> {
                    AnyPin {
                        inner: ErasedPin::[<Gpio $gpionum >](self),
                        _type: core::marker::PhantomData,
                    }
                }
            }

            impl<MODE, TYPE> TryInto<[<Gpio $gpionum >]<MODE>> for AnyPin<MODE, TYPE> {
                type Error = ();

                fn try_into(self) -> Result<[<Gpio $gpionum >]<MODE>, Self::Error> {
                    match self.inner {
                        ErasedPin::[<Gpio $gpionum >](gpio) => Ok(gpio),
                        _ => Err(()),
                    }
                }
            }
            )+

            procmacros::make_gpio_enum_dispatch_macro!(
                handle_gpio_output
                { InputOutputAnalog, InputOutput, }
                {
                    $(
                        $type,$gpionum
                    )+
                }
            );

            procmacros::make_gpio_enum_dispatch_macro!(
                handle_gpio_input
                { InputOutputAnalog, InputOutput, InputOnlyAnalog }
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
        impl<MODE> $crate::gpio::RtcPin for GpioPin<MODE, $pin_num>
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
            impl<MODE> $crate::gpio::RtcPinWithResistors for GpioPin<MODE, $pin_num>
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

            #[cfg(not(esp32))]
            paste::paste!{
                impl<MODE> $crate::gpio::rtc_io::IntoLowPowerPin<$pin_num> for GpioPin<MODE, $pin_num> {
                    fn into_low_power(mut self) -> $crate::gpio::rtc_io::LowPowerPin<Unknown, $pin_num> {
                        use $crate::gpio::RtcPin;

                        self.rtc_set_config(false, true, $crate::gpio::RtcFunction::Rtc);

                        $crate::gpio::rtc_io::LowPowerPin {
                            private: core::marker::PhantomData,
                        }
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
        impl<MODE> $crate::gpio::RtcPin for GpioPin<MODE, $pin_num> {
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

        impl<MODE> $crate::gpio::RtcPinWithResistors for GpioPin<MODE, $pin_num> {
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

#[cfg(feature = "async")]
mod asynch {
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_AW: AtomicWaker = AtomicWaker::new();
    static PIN_WAKERS: [AtomicWaker; NUM_PINS] = [NEW_AW; NUM_PINS];

    impl<MODE, const GPIONUM: u8> GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsInputPin,
    {
        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            PinFuture::new(self, Event::HighLevel).await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            PinFuture::new(self, Event::LowLevel).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            PinFuture::new(self, Event::RisingEdge).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            PinFuture::new(self, Event::FallingEdge).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            PinFuture::new(self, Event::AnyEdge).await
        }
    }

    impl<const GPIONUM: u8> GpioPin<Output<OpenDrain>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsInputPin + IsOutputPin,
    {
        /// Wait until the pin is high. If it is already high, return
        /// immediately.
        pub async fn wait_for_high(&mut self) {
            PinFuture::new(self, Event::HighLevel).await
        }

        /// Wait until the pin is low. If it is already low, return immediately.
        pub async fn wait_for_low(&mut self) {
            PinFuture::new(self, Event::LowLevel).await
        }

        /// Wait for the pin to undergo a transition from low to high.
        pub async fn wait_for_rising_edge(&mut self) {
            PinFuture::new(self, Event::RisingEdge).await
        }

        /// Wait for the pin to undergo a transition from high to low.
        pub async fn wait_for_falling_edge(&mut self) {
            PinFuture::new(self, Event::FallingEdge).await
        }

        /// Wait for the pin to undergo any transition, i.e low to high OR high
        /// to low.
        pub async fn wait_for_any_edge(&mut self) {
            PinFuture::new(self, Event::AnyEdge).await
        }
    }

    pub struct PinFuture<'a, P> {
        pin: &'a mut P,
    }

    impl<'a, P> PinFuture<'a, P>
    where
        P: crate::gpio::Pin,
    {
        pub fn new(pin: &'a mut P, event: Event) -> Self {
            pin.listen(event);
            Self { pin }
        }
    }

    impl<'a, P> core::future::Future for PinFuture<'a, P>
    where
        P: crate::gpio::Pin,
    {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PIN_WAKERS[self.pin.number() as usize].register(cx.waker());

            // if pin is no longer listening its been triggered
            // therefore the future has resolved
            if !self.pin.is_listening() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
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

#[cfg(feature = "embedded-hal-02")]
mod embedded_hal_02_impls {
    use embedded_hal_02::digital::v2 as digital;

    use super::*;

    impl<MODE, const GPIONUM: u8> digital::InputPin for GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
    {
        type Error = core::convert::Infallible;
        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<const GPIONUM: u8> digital::InputPin for GpioPin<Output<OpenDrain>, GPIONUM>
    where
        Self: GpioProperties,
    {
        type Error = core::convert::Infallible;
        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(<Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0)
        }
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<MODE, const GPIONUM: u8> digital::OutputPin for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
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

    impl<MODE, const GPIONUM: u8> digital::StatefulOutputPin for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
    {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<MODE, const GPIONUM: u8> digital::ToggleableOutputPin for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
    {
        type Error = core::convert::Infallible;
        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<MODE, TYPE> digital::InputPin for AnyPin<Input<MODE>, TYPE> {
        type Error = core::convert::Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<MODE, TYPE> digital::OutputPin for AnyPin<Output<MODE>, TYPE> {
        type Error = core::convert::Infallible;

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<MODE, TYPE> digital::StatefulOutputPin for AnyPin<Output<MODE>, TYPE> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<MODE, TYPE> digital::ToggleableOutputPin for AnyPin<Output<MODE>, TYPE> {
        type Error = core::convert::Infallible;

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }
}

#[cfg(feature = "embedded-hal")]
mod embedded_hal_impls {
    use embedded_hal::digital;

    use super::*;

    impl<MODE, const GPIONUM: u8> digital::ErrorType for GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
    {
        type Error = core::convert::Infallible;
    }

    impl<MODE, const GPIONUM: u8> digital::InputPin for GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }
        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<const GPIONUM: u8> digital::InputPin for GpioPin<Output<OpenDrain>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }
        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<MODE, const GPIONUM: u8> digital::ErrorType for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
    {
        type Error = core::convert::Infallible;
    }

    impl<MODE, const GPIONUM: u8> digital::OutputPin for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
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

    impl<MODE, const GPIONUM: u8> digital::StatefulOutputPin for GpioPin<Output<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsOutputPin,
    {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }
        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }

    impl<MODE, TYPE> digital::ErrorType for AnyPin<Input<MODE>, TYPE> {
        type Error = core::convert::Infallible;
    }

    impl<MODE, TYPE> digital::InputPin for AnyPin<Input<MODE>, TYPE> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_high(self))
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_low(self))
        }
    }

    impl<MODE, TYPE> digital::ErrorType for AnyPin<Output<MODE>, TYPE> {
        type Error = core::convert::Infallible;
    }

    impl<MODE, TYPE> digital::OutputPin for AnyPin<Output<MODE>, TYPE> {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }
    }

    impl<MODE, TYPE> digital::StatefulOutputPin for AnyPin<Output<MODE>, TYPE> {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_high(self))
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(Self::is_set_low(self))
        }
    }
}

#[cfg(feature = "embedded-hal")]
#[cfg(feature = "async")]
mod embedded_hal_async_impls {
    use embedded_hal_async::digital::Wait;

    use super::*;

    impl<MODE, const GPIONUM: u8> Wait for GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsInputPin,
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

    impl<const GPIONUM: u8> Wait for GpioPin<Output<OpenDrain>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsInputPin + IsOutputPin,
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

    impl<MODE, TYPE> embedded_hal_async::digital::Wait for AnyPin<Input<MODE>, TYPE> {
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
