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
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `IO::new`.
//!
//! ## Example
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio5.into_push_pull_output();
//! ```
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

use core::{convert::Infallible, marker::PhantomData};

use crate::peripherals::{GPIO, IO_MUX};
#[cfg(any(xtensa, esp32c3))]
pub(crate) use crate::rtc_pins;
pub use crate::soc::gpio::*;
pub(crate) use crate::{analog, gpio};

/// Convenience type-alias for a no-pin / don't care - pin
pub type NoPinType = Gpio0<Unknown>;

/// Convenience constant for `Option::None` pin
pub const NO_PIN: Option<NoPinType> = None;

#[derive(Copy, Clone)]
pub enum Event {
    RisingEdge  = 1,
    FallingEdge = 2,
    AnyEdge     = 3,
    LowLevel    = 4,
    HighLevel   = 5,
}

pub struct Unknown {}

pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct RTCInput<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct Floating;

pub struct PullDown;

pub struct PullUp;

pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct RTCOutput<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct OpenDrain;

pub struct PushPull;

pub struct Analog;

pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}

#[doc(hidden)]
pub struct AF0;

#[doc(hidden)]
pub struct AF1;

#[doc(hidden)]
pub struct AF2;

pub enum DriveStrength {
    I5mA  = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

#[derive(PartialEq)]
pub enum AlternateFunction {
    Function0 = 0,
    Function1 = 1,
    Function2 = 2,
    Function3 = 3,
    Function4 = 4,
    Function5 = 5,
}

#[derive(PartialEq)]
pub enum RtcFunction {
    Rtc     = 0,
    Digital = 1,
}

pub trait RTCPin: Pin {
    #[cfg(xtensa)]
    fn rtc_number(&self) -> u8;
    #[cfg(xtensa)]
    fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: RtcFunction);

    fn rtcio_pad_hold(&mut self, enable: bool);

    // Unsafe because `level` needs to be a valid setting for the
    // rtc_cntl.gpio_wakeup.gpio_pinX_int_type
    #[cfg(esp32c3)]
    unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8);
}

pub trait RTCPinWithResistors: RTCPin {
    fn rtcio_pullup(&mut self, enable: bool);
    fn rtcio_pulldown(&mut self, enable: bool);
}

pub trait RTCInputPin: RTCPin {}
pub trait RTCOutputPin: RTCPin {}

pub trait AnalogPin {}

pub trait Pin {
    fn number(&self) -> u8;

    fn sleep_mode(&mut self, on: bool);

    fn set_alternate_function(&mut self, alternate: AlternateFunction);

    fn listen(&mut self, event: Event) {
        self.listen_with_options(event, true, false, false)
    }

    fn is_listening(&self) -> bool;

    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    );

    fn unlisten(&mut self);

    fn clear_interrupt(&mut self);
}

pub trait InputPin: Pin {
    fn set_to_input(&mut self) -> &mut Self;

    fn enable_input(&mut self, on: bool) -> &mut Self;

    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn is_input_high(&self) -> bool;

    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        self.connect_input_to_peripheral_with_options(signal, false, false)
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;

    /// Remove a connected `signal` from this input pin.
    ///
    /// Clears the entry in the GPIO matrix / IO mux that associates this input
    /// pin with the given [input `signal`](`InputSignal`). Any other
    /// connected signals remain intact.
    fn disconnect_input_from_peripheral(&mut self, signal: InputSignal) -> &mut Self;
}

pub trait OutputPin: Pin {
    fn set_to_open_drain_output(&mut self) -> &mut Self;

    fn set_to_push_pull_output(&mut self) -> &mut Self;

    fn enable_output(&mut self, on: bool) -> &mut Self;

    fn set_output_high(&mut self, on: bool) -> &mut Self;

    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self;

    fn enable_open_drain(&mut self, on: bool) -> &mut Self;

    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn connect_peripheral_to_output(&mut self, signal: OutputSignal) -> &mut Self {
        self.connect_peripheral_to_output_with_options(signal, false, false, false, false)
    }

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
    /// Clears the entry in the GPIO matrix / IO mux that associates this output
    /// pin with a previously connected [signal](`InputSignal`). Any other
    /// outputs connected to the signal remain intact.
    fn disconnect_peripheral_from_output(&mut self) -> &mut Self;

    fn internal_pull_up(&mut self, on: bool) -> &mut Self;

    fn internal_pull_down(&mut self, on: bool) -> &mut Self;
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

    fn write_interrupt_status_clear(word: u32);

    fn write_output_set(word: u32);

    fn write_output_clear(word: u32);
}

impl BankGpioRegisterAccess for Bank0GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable_w1tc
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable_w1ts
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { &*GPIO::PTR }.in_.read().bits()
    }

    fn read_output() -> u32 {
        unsafe { &*GPIO::PTR }.out.read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .status_w1tc
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .out_w1ts
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .out_w1tc
            .write(|w| unsafe { w.bits(word) });
    }
}

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
impl BankGpioRegisterAccess for Bank1GpioRegisterAccess {
    fn write_out_en_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable1_w1tc
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_out_en_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .enable1_w1ts
            .write(|w| unsafe { w.bits(word) });
    }

    fn read_input() -> u32 {
        unsafe { &*GPIO::PTR }.in1.read().bits()
    }

    fn read_output() -> u32 {
        unsafe { &*GPIO::PTR }.out1.read().bits()
    }

    fn write_interrupt_status_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .status1_w1tc
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_set(word: u32) {
        unsafe { &*GPIO::PTR }
            .out1_w1ts
            .write(|w| unsafe { w.bits(word) });
    }

    fn write_output_clear(word: u32) {
        unsafe { &*GPIO::PTR }
            .out1_w1tc
            .write(|w| unsafe { w.bits(word) });
    }
}

pub fn connect_low_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sel()
            .set_bit()
            .in_inv_sel()
            .bit(false)
            .in_sel()
            .bits(ZERO_INPUT)
    });
}

pub fn connect_high_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
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

pub struct GpioPin<MODE, const GPIONUM: u8> {
    _mode: PhantomData<MODE>,
}

impl<MODE, const GPIONUM: u8> embedded_hal::digital::v2::InputPin for GpioPin<Input<MODE>, GPIONUM>
where
    Self: GpioProperties,
{
    type Error = Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(<Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0)
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

impl<const GPIONUM: u8> embedded_hal::digital::v2::InputPin for GpioPin<Output<OpenDrain>, GPIONUM>
where
    Self: GpioProperties,
{
    type Error = Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(<Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0)
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::ErrorType for GpioPin<Input<MODE>, GPIONUM>
where
    Self: GpioProperties,
{
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::InputPin for GpioPin<Input<MODE>, GPIONUM>
where
    Self: GpioProperties,
{
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(<Self as GpioProperties>::Bank::read_input() & (1 << (GPIONUM % 32)) != 0)
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
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
        gpio.func_out_sel_cfg[GPIONUM as usize]
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        #[cfg(esp32)]
        crate::soc::gpio::errata36(GPIONUM, pull_up, pull_down);

        // NOTE: Workaround to make GPIO18 and GPIO19 work on the ESP32-C3, which by
        //       default are assigned to the `USB_SERIAL_JTAG` peripheral.
        #[cfg(esp32c3)]
        if GPIONUM == 18 || GPIONUM == 19 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0
                .modify(|_, w| w.usb_pad_enable().clear_bit());
        }

        // Same workaround as above for ESP32-S3
        #[cfg(esp32s3)]
        if GPIONUM == 19 || GPIONUM == 20 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0
                .modify(|_, w| w.usb_pad_enable().clear_bit());
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

    pub fn into_floating_input(self) -> GpioPin<Input<Floating>, GPIONUM> {
        self.init_input(false, false);
        GpioPin { _mode: PhantomData }
    }

    pub fn into_pull_up_input(self) -> GpioPin<Input<PullUp>, GPIONUM> {
        self.init_input(false, true);
        GpioPin { _mode: PhantomData }
    }

    pub fn into_pull_down_input(self) -> GpioPin<Input<PullDown>, GPIONUM> {
        self.init_input(true, false);
        GpioPin { _mode: PhantomData }
    }
}

impl<MODE, const GPIONUM: u8> InputPin for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
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
            unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
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

        unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| w.sel().clear_bit());
        self
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
            (&*GPIO::PTR).pin[GPIONUM as usize].modify(|_, w| {
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
        let bits = unsafe { &*GPIO::PTR }.pin[GPIONUM as usize]
            .read()
            .int_ena()
            .bits();
        bits != 0
    }

    fn unlisten(&mut self) {
        unsafe {
            (&*GPIO::PTR).pin[GPIONUM as usize]
                .modify(|_, w| w.int_ena().bits(0).int_type().bits(0).int_ena().bits(0));
        }
    }

    fn clear_interrupt(&mut self) {
        <Self as GpioProperties>::Bank::write_interrupt_status_clear(1 << (GPIONUM % 32));
    }
}

impl<MODE, const GPIONUM: u8> embedded_hal::digital::v2::OutputPin
    for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    type Error = Infallible;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        <Self as GpioProperties>::Bank::write_output_set(1 << (GPIONUM % 32));
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        <Self as GpioProperties>::Bank::write_output_clear(1 << (GPIONUM % 32));
        Ok(())
    }
}

impl<MODE, const GPIONUM: u8> embedded_hal::digital::v2::StatefulOutputPin
    for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(<Self as GpioProperties>::Bank::read_output() & (1 << (GPIONUM % 32)) != 0)
    }
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

impl<MODE, const GPIONUM: u8> embedded_hal::digital::v2::ToggleableOutputPin
    for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    type Error = Infallible;
    fn toggle(&mut self) -> Result<(), Self::Error> {
        use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};
        if self.is_set_high()? {
            Ok(self.set_low()?)
        } else {
            Ok(self.set_high()?)
        }
    }
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::ErrorType for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::OutputPin for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        <Self as GpioProperties>::Bank::write_output_clear(1 << (GPIONUM % 32));
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        <Self as GpioProperties>::Bank::write_output_set(1 << (GPIONUM % 32));
        Ok(())
    }
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::StatefulOutputPin
    for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(<Self as GpioProperties>::Bank::read_output() & (1 << (GPIONUM % 32)) != 0)
    }
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

#[cfg(feature = "eh1")]
impl<MODE, const GPIONUM: u8> embedded_hal_1::digital::ToggleableOutputPin
    for GpioPin<Output<MODE>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn toggle(&mut self) -> Result<(), Self::Error> {
        use embedded_hal_1::digital::{OutputPin as _, StatefulOutputPin as _};
        if self.is_set_high()? {
            Ok(self.set_low()?)
        } else {
            Ok(self.set_high()?)
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

impl<MODE, const GPIONUM: u8> crate::peripheral::sealed::Sealed for GpioPin<MODE, GPIONUM> where
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

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Alternate<AF1>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_alternate_1()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Alternate<AF2>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> Self {
        pin.into_alternate_2()
    }
}

impl<MODE, const GPIONUM: u8> GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
{
    fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
        let gpio = unsafe { &*GPIO::PTR };

        <Self as GpioProperties>::Bank::write_out_en_set(1 << (GPIONUM % 32));
        gpio.pin[GPIONUM as usize].modify(|_, w| w.pad_driver().bit(open_drain));

        gpio.func_out_sel_cfg[GPIONUM as usize]
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

        // NOTE: Workaround to make GPIO18 and GPIO19 work on the ESP32-C3, which by
        //       default are assigned to the `USB_SERIAL_JTAG` peripheral.
        #[cfg(esp32c3)]
        if GPIONUM == 18 || GPIONUM == 19 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0
                .modify(|_, w| w.usb_pad_enable().clear_bit());
        }

        // Same workaround as above for ESP32-S3
        #[cfg(esp32s3)]
        if GPIONUM == 19 || GPIONUM == 20 {
            unsafe { &*crate::peripherals::USB_DEVICE::PTR }
                .conf0
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

    pub fn into_push_pull_output(self) -> GpioPin<Output<PushPull>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, false);
        GpioPin { _mode: PhantomData }
    }

    pub fn into_open_drain_output(self) -> GpioPin<Output<OpenDrain>, GPIONUM> {
        self.init_output(GPIO_FUNCTION, true);
        GpioPin { _mode: PhantomData }
    }

    pub fn into_alternate_1(self) -> GpioPin<Alternate<AF1>, GPIONUM> {
        self.init_output(AlternateFunction::Function1, false);
        GpioPin { _mode: PhantomData }
    }

    pub fn into_alternate_2(self) -> GpioPin<Alternate<AF2>, GPIONUM> {
        self.init_output(AlternateFunction::Function2, false);
        GpioPin { _mode: PhantomData }
    }
}

impl<MODE, const GPIONUM: u8> OutputPin for GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
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
        unsafe { &*GPIO::PTR }.pin[GPIONUM as usize].modify(|_, w| w.pad_driver().bit(on));
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
        unsafe { &*GPIO::PTR }.func_out_sel_cfg[GPIONUM as usize].modify(|_, w| unsafe {
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
        unsafe { &*GPIO::PTR }.func_out_sel_cfg[GPIONUM as usize]
            .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });
        self
    }

    fn internal_pull_up(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpu().bit(on));
        self
    }
    fn internal_pull_down(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.fun_wpd().bit(on));
        self
    }
}

impl<MODE, const GPIONUM: u8> GpioPin<MODE, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsAnalogPin,
{
    pub fn into_analog(self) -> GpioPin<Analog, GPIONUM> {
        crate::soc::gpio::internal_into_analog(GPIONUM);

        GpioPin { _mode: PhantomData }
    }
}

impl<MODE> embedded_hal::digital::v2::InputPin for AnyPin<Input<MODE>> {
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_high() })
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_low() })
    }
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::ErrorType for AnyPin<Input<MODE>> {
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::InputPin for AnyPin<Input<MODE>> {
    fn is_high(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_high() })
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_input!(inner, target, { target.is_low() })
    }
}

impl<MODE> embedded_hal::digital::v2::OutputPin for AnyPin<Output<MODE>> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_low() })
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_high() })
    }
}

impl<MODE> embedded_hal::digital::v2::StatefulOutputPin for AnyPin<Output<MODE>> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_high() })
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_low() })
    }
}

impl<MODE> embedded_hal::digital::v2::ToggleableOutputPin for AnyPin<Output<MODE>> {
    type Error = Infallible;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.toggle() })
    }
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::ErrorType for AnyPin<Output<MODE>> {
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::OutputPin for AnyPin<Output<MODE>> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_low() })
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.set_high() })
    }
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::StatefulOutputPin for AnyPin<Output<MODE>> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_high() })
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        let inner = &self.inner;
        handle_gpio_output!(inner, target, { target.is_set_low() })
    }
}

#[cfg(feature = "eh1")]
impl<MODE> embedded_hal_1::digital::ToggleableOutputPin for AnyPin<Output<MODE>> {
    fn toggle(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_output!(inner, target, { target.toggle() })
    }
}

#[cfg(feature = "async")]
impl<MODE> embedded_hal_async::digital::Wait for AnyPin<Input<MODE>> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_high().await })
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_low().await })
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_rising_edge().await })
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_falling_edge().await })
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        let inner = &mut self.inner;
        handle_gpio_input!(inner, target, { target.wait_for_any_edge().await })
    }
}

/// General Purpose Input/Output driver
pub struct IO {
    _io_mux: IO_MUX,
    pub pins: Pins,
}

impl IO {
    pub fn new(gpio: GPIO, io_mux: IO_MUX) -> Self {
        let pins = gpio.split();
        let io = IO {
            _io_mux: io_mux,
            pins,
        };
        io
    }
}

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
        #[doc(hidden)]
        pub trait GpioExt {
            type Parts;
            fn split(self) -> Self::Parts;
        }

        paste::paste! {
            impl GpioExt for GPIO {
                type Parts = Pins;
                fn split(self) -> Self::Parts {
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
                impl<MODE> crate::gpio::GpioProperties for GpioPin<MODE,  $gpionum> {
                    type Bank = crate::gpio::[< Bank $bank GpioRegisterAccess >];
                    type InterruptStatus = crate::gpio::[< InterruptStatusRegisterAccessBank $bank >];
                    type Signals = [< Gpio $gpionum Signals >];
                    type PinType = crate::gpio::[<$type PinType>];
                }

                pub struct [<Gpio $gpionum Signals>] {}

                impl crate::gpio::GpioSignal for [<Gpio $gpionum Signals>] {
                    fn output_signals() -> [Option<OutputSignal>; 6]{
                        #[allow(unused_mut)]
                        let mut output_signals = [None,None,None,None,None,None];

                        $(
                            $(
                                output_signals[ $af_output_num ] = Some( OutputSignal::$af_output_signal );
                            )*
                        )?

                        output_signals
                    }
                    fn input_signals() -> [Option<InputSignal>; 6] {
                        #[allow(unused_mut)]
                        let mut input_signals = [None,None,None,None,None,None];

                        $(
                            $(
                                input_signals[ $af_input_num ] = Some( InputSignal::$af_input_signal );
                            )*
                        )?

                        input_signals
                    }
                }
            )+

            pub struct Pins {
                $(
                    pub [< gpio $gpionum >] : GpioPin<Unknown, $gpionum>,
                )+
            }

            $(
                pub type [<Gpio $gpionum >]<MODE> = GpioPin<MODE, $gpionum>;
            )+

            pub(crate) enum ErasedPin<MODE> {
                $(
                    [<Gpio $gpionum >]([<Gpio $gpionum >]<MODE>),
                )+
            }

            pub struct AnyPin<MODE> {
                pub(crate) inner: ErasedPin<MODE>
            }

            $(
            impl<MODE> From< [<Gpio $gpionum >]<MODE> > for AnyPin<MODE> {
                fn from(value: [<Gpio $gpionum >]<MODE>) -> Self {
                    AnyPin {
                        inner: ErasedPin::[<Gpio $gpionum >](value)
                    }
                }
            }

            impl<MODE> [<Gpio $gpionum >]<MODE> {
                pub fn degrade(self) -> AnyPin<MODE> {
                    AnyPin {
                        inner: ErasedPin::[<Gpio $gpionum >](self)
                    }
                }
            }

            impl<MODE> TryInto<[<Gpio $gpionum >]<MODE>> for AnyPin<MODE> {
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
        impl<MODE> crate::gpio::RTCPin for GpioPin<MODE, $pin_num>
        {
            fn rtc_number(&self) -> u8 {
                $rtc_pin
            }

            /// Set the RTC properties of the pin. If `mux` is true then then pin is
            /// routed to RTC, when false it is routed to IO_MUX.
            fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: crate::gpio::RtcFunction) {
                use crate::peripherals::RTC_IO;
                let rtcio = unsafe{ &*RTC_IO::ptr() };

                #[cfg(esp32s3)]
                unsafe { crate::peripherals::SENS::steal() }.sar_peri_clk_gate_conf.modify(|_,w| w.iomux_clk_en().set_bit());
                #[cfg(esp32s2)]
                unsafe { crate::peripherals::SENS::steal() }.sar_io_mux_conf.modify(|_,w| w.iomux_clk_gate_en().set_bit());

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
                let rtc_ctrl = unsafe { &*crate::peripherals::RTC_CNTL::PTR };

                #[cfg(esp32)]
                rtc_ctrl.hold_force.modify(|_, w| w.$hold().bit(enable));

                #[cfg(not(esp32))]
                rtc_ctrl.pad_hold.modify(|_, w| w.$hold().bit(enable));
            }
        }

        $(
            impl<MODE> crate::gpio::RTCPinWithResistors for GpioPin<MODE, $pin_num>
            {
                fn rtcio_pullup(&mut self, enable: bool) {
                    let rtcio = unsafe { &*crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.$rue().bit([< enable >]));
                    }
                }

                fn rtcio_pulldown(&mut self, enable: bool) {
                    let rtcio = unsafe { &*crate::peripherals::RTC_IO::PTR };

                    paste::paste! {
                        rtcio.$pin_reg.modify(|_, w| w.$rde().bit([< enable >]));
                    }
                }
            }

            #[cfg(not(esp32))]
            paste::paste!{
                impl<MODE> crate::gpio::rtc_io::IntoLowPowerPin<$pin_num> for GpioPin<MODE, $pin_num> {
                    fn into_low_power(mut self) -> crate::gpio::rtc_io::LowPowerPin<Unknown, $pin_num> {
                        use crate::gpio::RTCPin;

                        self.rtc_set_config(false, true, crate::gpio::RtcFunction::Rtc);

                        crate::gpio::rtc_io::LowPowerPin {
                            private: core::marker::PhantomData::default(),
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
            crate::gpio::rtc_pins!($pin_num, $rtc_pin, $pin_reg, $prefix, $hold $(, $rue, $rde)?);
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
        impl<MODE> crate::gpio::RTCPin for GpioPin<MODE, $pin_num> {
            unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
                let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };
                paste::paste! {
                    rtc_cntl.gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _wakeup_enable >]().bit(wakeup));
                    rtc_cntl.gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _int_type >]().bits(level));
                }
            }

            fn rtcio_pad_hold(&mut self, enable: bool) {
                let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };
                paste::paste! {
                    rtc_cntl.pad_hold.modify(|_, w| w.[< gpio_pin $pin_num _hold >]().bit(enable));
                }
            }
        }

        impl<MODE> crate::gpio::RTCPinWithResistors for GpioPin<MODE, $pin_num> {
            fn rtcio_pullup(&mut self, enable: bool) {
                let io_mux = unsafe { &*crate::peripherals::IO_MUX::ptr() };
                io_mux.gpio[$pin_num].modify(|_, w| w.fun_wpu().bit(enable));
            }

            fn rtcio_pulldown(&mut self, enable: bool) {
                let io_mux = unsafe { &*crate::peripherals::IO_MUX::ptr() };
                io_mux.gpio[$pin_num].modify(|_, w| w.fun_wpd().bit(enable));
            }
        }

    };

    ( $( $pin_num:expr )+ ) => { $( crate::gpio::rtc_pins!($pin_num); )+ };
}

// Following code enables `into_analog`

#[doc(hidden)]
pub fn enable_iomux_clk_gate() {
    #[cfg(esp32s2)]
    {
        use crate::peripherals::SENS;
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_io_mux_conf
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
            use crate::peripherals::RTC_IO;
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
                            rtcio.enable_w1tc.write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });

                            // disable open drain
                            rtcio.pin[$rtc_pin].modify(|_,w| w.pad_driver().bit(false));

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
            use crate::peripherals::IO_MUX;
            use crate::peripherals::GPIO;

            let io_mux = unsafe{ &*IO_MUX::PTR };
            let gpio = unsafe{ &*GPIO::PTR };

            match pin {
                $(
                    $pin_num => {
                        io_mux.gpio[$pin_num].modify(|_,w| unsafe {
                            w.mcu_sel().bits(1)
                                .fun_ie().clear_bit()
                                .fun_wpu().clear_bit()
                                .fun_wpd().clear_bit()
                        });

                        gpio.enable_w1tc.write(|w| unsafe { w.bits(1 << $pin_num) });
                    }
                )+
                _ => unreachable!()
            }

        }
    }
}

#[cfg(soc_etm)]
pub mod etm {
    //! # Event Task Matrix Function
    //!
    //! ## Overview
    //!
    //! GPIO supports ETM function, that is, the ETM task of GPIO can be
    //! triggered by the ETM event of any peripheral, or the ETM task of any
    //! peripheral can be triggered by the ETM event of GPIO.
    //!
    //! The GPIO ETM provides eight task channels. The ETM tasks that each task
    //! channel can receive are:
    //! - SET: GPIO goes high when triggered
    //! - CLEAR: GPIO goes low when triggered
    //! - TOGGLE: GPIO toggle level when triggered.
    //!
    //! GPIO has eight event channels, and the ETM events that each event
    //! channel can generate are:
    //! - RISE_EDGE: Indicates that the output signal of the corresponding GPIO
    //!   has a rising edge
    //! - FALL_EDGE: Indicates that the output signal of the corresponding GPIO
    //!   has a falling edge
    //! - ANY_EDGE: Indicates that the output signal of the corresponding GPIO
    //!   is reversed
    //!
    //! ## Example
    //! ```no_run
    //! let led_task = gpio_ext.channel0_task.toggle(&mut led);
    //! let button_event = gpio_ext.channel0_event.falling_edge(button);
    //! ```

    use crate::peripheral::{Peripheral, PeripheralRef};

    /// All the GPIO ETM channels
    #[non_exhaustive]
    pub struct GpioEtmChannels<'d> {
        _gpio_sd: PeripheralRef<'d, crate::peripherals::GPIO_SD>,
        pub channel0_task: GpioEtmTaskChannel<0>,
        pub channel0_event: GpioEtmEventChannel<0>,
        pub channel1_task: GpioEtmTaskChannel<1>,
        pub channel1_event: GpioEtmEventChannel<1>,
        pub channel2_task: GpioEtmTaskChannel<2>,
        pub channel2_event: GpioEtmEventChannel<2>,
        pub channel3_task: GpioEtmTaskChannel<3>,
        pub channel3_event: GpioEtmEventChannel<3>,
        pub channel4_task: GpioEtmTaskChannel<4>,
        pub channel4_event: GpioEtmEventChannel<4>,
        pub channel5_task: GpioEtmTaskChannel<5>,
        pub channel5_event: GpioEtmEventChannel<5>,
        pub channel6_task: GpioEtmTaskChannel<6>,
        pub channel6_event: GpioEtmEventChannel<6>,
        pub channel7_task: GpioEtmTaskChannel<7>,
        pub channel7_event: GpioEtmEventChannel<7>,
    }

    impl<'d> GpioEtmChannels<'d> {
        pub fn new(peripheral: impl Peripheral<P = crate::peripherals::GPIO_SD> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _gpio_sd: peripheral,
                channel0_task: GpioEtmTaskChannel {},
                channel0_event: GpioEtmEventChannel {},
                channel1_task: GpioEtmTaskChannel {},
                channel1_event: GpioEtmEventChannel {},
                channel2_task: GpioEtmTaskChannel {},
                channel2_event: GpioEtmEventChannel {},
                channel3_task: GpioEtmTaskChannel {},
                channel3_event: GpioEtmEventChannel {},
                channel4_task: GpioEtmTaskChannel {},
                channel4_event: GpioEtmEventChannel {},
                channel5_task: GpioEtmTaskChannel {},
                channel5_event: GpioEtmEventChannel {},
                channel6_task: GpioEtmTaskChannel {},
                channel6_event: GpioEtmEventChannel {},
                channel7_task: GpioEtmTaskChannel {},
                channel7_event: GpioEtmEventChannel {},
            }
        }
    }

    /// An ETM controlled GPIO event
    pub struct GpioEtmEventChannel<const C: u8> {}

    impl<const C: u8> GpioEtmEventChannel<C> {
        /// Trigger at rising edge
        pub fn rising_edge<'d, PIN>(
            self,
            pin: impl Peripheral<P = PIN> + 'd,
        ) -> GpioEtmEventChannelRising<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_event_channel(C, pin.number());
            GpioEtmEventChannelRising { _pin: pin }
        }

        /// Trigger at falling edge
        pub fn falling_edge<'d, PIN>(
            self,
            pin: impl Peripheral<P = PIN> + 'd,
        ) -> GpioEtmEventChannelFalling<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_event_channel(C, pin.number());
            GpioEtmEventChannelFalling { _pin: pin }
        }

        /// Trigger at any edge
        pub fn any_edge<'d, PIN>(
            self,
            pin: impl Peripheral<P = PIN> + 'd,
        ) -> GpioEtmEventChannelAny<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_event_channel(C, pin.number());
            GpioEtmEventChannelAny { _pin: pin }
        }
    }

    /// Event for rising edge
    #[non_exhaustive]
    pub struct GpioEtmEventChannelRising<'d, PIN, const C: u8>
    where
        PIN: super::Pin,
    {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmEventChannelRising<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelRising<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            1 + C
        }
    }

    /// Event for falling edge
    #[non_exhaustive]
    pub struct GpioEtmEventChannelFalling<'d, PIN, const C: u8>
    where
        PIN: super::Pin,
    {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmEventChannelFalling<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelFalling<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            9 + C
        }
    }

    /// Event for any edge
    #[non_exhaustive]
    pub struct GpioEtmEventChannelAny<'d, PIN, const C: u8>
    where
        PIN: super::Pin,
    {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmEventChannelAny<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelAny<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            17 + C
        }
    }

    /// An ETM controlled GPIO task
    pub struct GpioEtmTaskChannel<const C: u8> {}

    impl<const C: u8> GpioEtmTaskChannel<C> {
        // In theory we could have multiple pins assigned to the same task. Not sure how
        // useful that would be. If we want to support it, the easiest would be
        // to offer additional functions like `set2`, `set3` etc. where the
        // number is the pin-count

        /// Task to set a high level
        pub fn set<'d, PIN>(self, pin: impl Peripheral<P = PIN> + 'd) -> GpioEtmTaskSet<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_task_channel(C, pin.number());
            GpioEtmTaskSet { _pin: pin }
        }

        /// Task to set a low level
        pub fn clear<'d, PIN>(
            self,
            pin: impl Peripheral<P = PIN> + 'd,
        ) -> GpioEtmTaskClear<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_task_channel(C, pin.number());
            GpioEtmTaskClear { _pin: pin }
        }

        /// Task to toggle the level
        pub fn toggle<'d, PIN>(
            self,
            pin: impl Peripheral<P = PIN> + 'd,
        ) -> GpioEtmTaskToggle<'d, PIN, C>
        where
            PIN: super::Pin,
        {
            crate::into_ref!(pin);
            enable_task_channel(C, pin.number());
            GpioEtmTaskToggle { _pin: pin }
        }
    }

    /// Task for set operation
    #[non_exhaustive]
    pub struct GpioEtmTaskSet<'d, PIN, const C: u8>
    where
        PIN: super::Pin,
    {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmTaskSet<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskSet<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            1 + C
        }
    }

    /// Task for clear operation
    #[non_exhaustive]
    pub struct GpioEtmTaskClear<'d, PIN, const C: u8> {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmTaskClear<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskClear<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            9 + C
        }
    }

    /// Task for toggle operation
    #[non_exhaustive]
    pub struct GpioEtmTaskToggle<'d, PIN, const C: u8> {
        _pin: PeripheralRef<'d, PIN>,
    }

    impl<'d, PIN, const C: u8> crate::etm::private::Sealed for GpioEtmTaskToggle<'d, PIN, C> where
        PIN: super::Pin
    {
    }

    impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskToggle<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        fn id(&self) -> u8 {
            17 + C
        }
    }

    fn enable_task_channel(channel: u8, pin: u8) {
        let gpio_sd = unsafe { crate::peripherals::GPIO_SD::steal() };
        let ptr = unsafe { gpio_sd.etm_task_p0_cfg.as_ptr().add(pin as usize / 4) };
        let shift = 8 * (pin as usize % 4);
        // bit 0 = en, bit 1-3 = channel
        unsafe {
            ptr.write_volatile(
                ptr.read_volatile() & !(0xf << shift)
                    | 1 << shift
                    | (channel as u32) << (shift + 1),
            );
        }
    }

    fn enable_event_channel(channel: u8, pin: u8) {
        let gpio_sd = unsafe { crate::peripherals::GPIO_SD::steal() };
        gpio_sd.etm_event_ch_cfg[channel as usize].modify(|_, w| w.etm_ch0_event_en().clear_bit());
        gpio_sd.etm_event_ch_cfg[channel as usize]
            .modify(|_, w| w.etm_ch0_event_sel().variant(pin));
        gpio_sd.etm_event_ch_cfg[channel as usize].modify(|_, w| w.etm_ch0_event_en().set_bit());
    }
}

#[cfg(all(rtc_io, not(esp32)))]
pub mod rtc_io {
    //! RTC IO
    //!
    //! # Overview
    //!
    //! The hardware provides a couple of GPIO pins with low power (LP)
    //! capabilities and analog functions. These pins can be controlled by
    //! either IO MUX or RTC IO.
    //!
    //! If controlled by RTC IO, these pins will bypass IO MUX and GPIO
    //! matrix for the use by ULP and peripherals in RTC system.
    //!
    //! When configured as RTC GPIOs, the pins can still be controlled by ULP or
    //! the peripherals in RTC system during chip Deep-sleep, and wake up the
    //! chip from Deep-sleep.
    //!
    //! # Example
    //! ```no_run
    //! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    //! // configure GPIO 1 as ULP output pin
    //! let lp_pin = io.pins.gpio1.into_low_power().into_push_pull_output();
    //! ```

    use core::marker::PhantomData;

    use super::{Floating, Input, Output, PullDown, PullUp, PushPull, Unknown};

    /// A GPIO pin configured for low power operation
    pub struct LowPowerPin<MODE, const PIN: u8> {
        pub(crate) private: PhantomData<MODE>,
    }

    /// Configures a pin for use as a low power pin
    pub trait IntoLowPowerPin<const PIN: u8> {
        fn into_low_power(self) -> LowPowerPin<Unknown, { PIN }>;
    }

    impl<MODE, const PIN: u8> LowPowerPin<MODE, PIN> {
        #[doc(hidden)]
        pub fn output_enable(&self, enable: bool) {
            let rtc_io = unsafe { crate::peripherals::RTC_IO::steal() };
            if enable {
                // TODO align PAC
                #[cfg(esp32s2)]
                rtc_io
                    .rtc_gpio_enable_w1ts
                    .write(|w| w.reg_rtcio_reg_gpio_enable_w1ts().variant(1 << PIN));

                #[cfg(esp32s3)]
                rtc_io
                    .rtc_gpio_enable_w1ts
                    .write(|w| w.rtc_gpio_enable_w1ts().variant(1 << PIN));
            } else {
                rtc_io
                    .enable_w1tc
                    .write(|w| w.enable_w1tc().variant(1 << PIN));
            }
        }

        fn input_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.fun_ie().bit(enable));
        }

        fn pullup_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.rue().bit(enable));
        }

        fn pulldown_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.rde().bit(enable));
        }

        #[doc(hidden)]
        pub fn set_level(&mut self, level: bool) {
            let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };

            // TODO align PACs
            #[cfg(esp32s2)]
            if level {
                rtc_io
                    .rtc_gpio_out_w1ts
                    .write(|w| w.gpio_out_data_w1ts().variant(1 << PIN));
            } else {
                rtc_io
                    .rtc_gpio_out_w1tc
                    .write(|w| w.gpio_out_data_w1tc().variant(1 << PIN));
            }

            #[cfg(esp32s3)]
            if level {
                rtc_io
                    .rtc_gpio_out_w1ts
                    .write(|w| w.rtc_gpio_out_data_w1ts().variant(1 << PIN));
            } else {
                rtc_io
                    .rtc_gpio_out_w1tc
                    .write(|w| w.rtc_gpio_out_data_w1tc().variant(1 << PIN));
            }
        }

        #[doc(hidden)]
        pub fn get_level(&self) -> bool {
            let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };
            (rtc_io.rtc_gpio_in.read().bits() & 1 << PIN) != 0
        }

        /// Configures the pin as an input with the internal pull-up resistor
        /// enabled.
        pub fn into_pull_up_input(self) -> LowPowerPin<Input<PullUp>, PIN> {
            self.input_enable(true);
            self.pullup_enable(true);
            self.pulldown_enable(false);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as an input with the internal pull-down resistor
        /// enabled.
        pub fn into_pull_down_input(self) -> LowPowerPin<Input<PullDown>, PIN> {
            self.input_enable(true);
            self.pullup_enable(false);
            self.pulldown_enable(true);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as a floating input pin.
        pub fn into_floating_input(self) -> LowPowerPin<Input<Floating>, PIN> {
            self.input_enable(true);
            self.pullup_enable(false);
            self.pulldown_enable(false);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as an output pin.
        pub fn into_push_pull_output(self) -> LowPowerPin<Output<PushPull>, PIN> {
            self.output_enable(true);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }
    }

    #[cfg(esp32s3)]
    #[inline(always)]
    fn get_pin_reg(pin: u8) -> &'static crate::peripherals::rtc_io::TOUCH_PAD0 {
        let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };
        unsafe { core::mem::transmute((rtc_io.touch_pad0.as_ptr()).add(pin as usize)) }
    }

    #[cfg(esp32s2)]
    #[inline(always)]
    fn get_pin_reg(pin: u8) -> &'static crate::peripherals::rtc_io::TOUCH_PAD {
        let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };
        unsafe { core::mem::transmute((rtc_io.touch_pad[0].as_ptr()).add(pin as usize)) }
    }
}

#[cfg(lp_io)]
pub mod lp_gpio {
    //! Low Power IO (LP_IO)
    //!
    //! # Overview
    //!
    //! The hardware provides a couple of GPIO pins with low power (LP)
    //! capabilities and analog functions. These pins can be controlled by
    //! either IO MUX or LP IO MUX.
    //!
    //! If controlled by LP IO MUX, these pins will bypass IO MUX and GPIO
    //! matrix for the use by ULP and peripherals in LP system.
    //!
    //! When configured as LP GPIOs, the pins can still be controlled by ULP or
    //! the peripherals in LP system during chip Deep-sleep, and wake up the
    //! chip from Deep-sleep.
    //!
    //! # Example
    //! ```no_run
    //! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    //! // configure GPIO 1 as LP output pin
    //! let lp_pin = io.pins.gpio1.into_low_power().into_push_pull_output();
    //! ```

    use core::marker::PhantomData;

    use super::{Floating, Input, Output, PullDown, PullUp, PushPull, Unknown};

    /// A GPIO pin configured for low power operation
    pub struct LowPowerPin<MODE, const PIN: u8> {
        pub(crate) private: PhantomData<MODE>,
    }

    impl<MODE, const PIN: u8> LowPowerPin<MODE, PIN> {
        #[doc(hidden)]
        pub fn output_enable(&self, enable: bool) {
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            if enable {
                lp_io
                    .out_enable_w1ts
                    .write(|w| w.lp_gpio_enable_w1ts().variant(1 << PIN));
            } else {
                lp_io
                    .out_enable_w1tc
                    .write(|w| w.lp_gpio_enable_w1tc().variant(1 << PIN));
            }
        }

        fn input_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.lp_gpio0_fun_ie().bit(enable));
        }

        fn pullup_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.lp_gpio0_fun_wpu().bit(enable));
        }

        fn pulldown_enable(&self, enable: bool) {
            get_pin_reg(PIN).modify(|_, w| w.lp_gpio0_fun_wpd().bit(enable));
        }

        #[doc(hidden)]
        pub fn set_level(&mut self, level: bool) {
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            if level {
                lp_io
                    .out_data_w1ts
                    .write(|w| w.lp_gpio_out_data_w1ts().variant(1 << PIN));
            } else {
                lp_io
                    .out_data_w1tc
                    .write(|w| w.lp_gpio_out_data_w1tc().variant(1 << PIN));
            }
        }

        #[doc(hidden)]
        pub fn get_level(&self) -> bool {
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            (lp_io.in_.read().lp_gpio_in_data_next().bits() & 1 << PIN) != 0
        }

        /// Configures the pin as an input with the internal pull-up resistor
        /// enabled.
        pub fn into_pull_up_input(self) -> LowPowerPin<Input<PullUp>, PIN> {
            self.input_enable(true);
            self.pullup_enable(true);
            self.pulldown_enable(false);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as an input with the internal pull-down resistor
        /// enabled.
        pub fn into_pull_down_input(self) -> LowPowerPin<Input<PullDown>, PIN> {
            self.input_enable(true);
            self.pullup_enable(false);
            self.pulldown_enable(true);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as a floating input pin.
        pub fn into_floating_input(self) -> LowPowerPin<Input<Floating>, PIN> {
            self.input_enable(true);
            self.pullup_enable(false);
            self.pulldown_enable(false);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }

        /// Configures the pin as an output pin.
        pub fn into_push_pull_output(self) -> LowPowerPin<Output<PushPull>, PIN> {
            self.output_enable(true);
            LowPowerPin {
                private: PhantomData::default(),
            }
        }
    }

    pub(crate) fn init_low_power_pin(pin: u8) {
        let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };

        lp_aon
            .gpio_mux
            .modify(|r, w| w.sel().variant(r.sel().bits() | 1 << pin));

        get_pin_reg(pin).modify(|_, w| w.lp_gpio0_mcu_sel().variant(0));
    }

    #[inline(always)]
    fn get_pin_reg(pin: u8) -> &'static crate::peripherals::lp_io::GPIO0 {
        let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };

        // ideally we should change the SVD and make the GPIOx registers into an
        // array
        unsafe { core::mem::transmute((lp_io.gpio0.as_ptr()).add(pin as usize)) }
    }

    /// Configures a pin for use as a low power pin
    pub trait IntoLowPowerPin<const PIN: u8> {
        fn into_low_power(self) -> LowPowerPin<Unknown, { PIN }>;
    }

    #[doc(hidden)]
    #[macro_export]
    macro_rules! lp_gpio {
        (
            $($gpionum:literal)+
        ) => {
            paste::paste!{
                $(
                    impl<MODE> crate::gpio::lp_gpio::IntoLowPowerPin<$gpionum> for GpioPin<MODE, $gpionum> {
                        fn into_low_power(self) -> crate::gpio::lp_gpio::LowPowerPin<Unknown, $gpionum> {
                            crate::gpio::lp_gpio::init_low_power_pin($gpionum);
                            crate::gpio::lp_gpio::LowPowerPin {
                                private: core::marker::PhantomData::default(),
                            }
                        }
                    }
                )+
            }
        }
    }

    pub(crate) use lp_gpio;
}

#[cfg(feature = "async")]
mod asynch {
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;
    use embedded_hal_async::digital::Wait;

    use super::*;
    use crate::prelude::*;

    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_AW: AtomicWaker = AtomicWaker::new();
    static PIN_WAKERS: [AtomicWaker; NUM_PINS] = [NEW_AW; NUM_PINS];

    impl<MODE, const GPIONUM: u8> Wait for GpioPin<Input<MODE>, GPIONUM>
    where
        Self: GpioProperties,
        <Self as GpioProperties>::PinType: IsInputPin,
    {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            PinFuture::new(self, Event::HighLevel).await
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            PinFuture::new(self, Event::LowLevel).await
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            PinFuture::new(self, Event::RisingEdge).await
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            PinFuture::new(self, Event::FallingEdge).await
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            PinFuture::new(self, Event::AnyEdge).await
        }
    }

    pub struct PinFuture<'a, P> {
        pin: &'a mut P,
    }

    impl<'a, P> PinFuture<'a, P>
    where
        P: crate::gpio::Pin + embedded_hal_1::digital::ErrorType,
    {
        pub fn new(pin: &'a mut P, event: Event) -> Self {
            pin.listen(event);
            Self { pin }
        }
    }

    impl<'a, P> core::future::Future for PinFuture<'a, P>
    where
        P: crate::gpio::Pin + embedded_hal_1::digital::ErrorType,
    {
        type Output = Result<(), P::Error>;

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PIN_WAKERS[self.pin.number() as usize].register(cx.waker());

            // if pin is no longer listening its been triggered
            // therefore the future has resolved
            if !self.pin.is_listening() {
                Poll::Ready(Ok(()))
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
        gpio.pin[gpio_num as usize].modify(|_, w| unsafe {
            w.int_ena()
                .bits(int_ena)
                .int_type()
                .bits(int_type)
                .wakeup_enable()
                .bit(wake_up_from_light_sleep)
        });
    }

    #[interrupt]
    unsafe fn GPIO() {
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
