//! General Purpose I/Os
//!
//! To get access to the pins, you first need to convert them into a HAL
//! designed struct from the pac struct `GPIO` and `IO_MUX` using `IO::new`.
//!
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio5.into_push_pull_output();
//! ```

use core::{convert::Infallible, marker::PhantomData};

use crate::peripherals::{GPIO, IO_MUX};
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

pub trait RTCPin {}

pub trait AnalogPin {}

pub trait Pin {
    fn number(&self) -> u8;

    fn sleep_mode(&mut self, on: bool) -> &mut Self;

    fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self;

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

    fn is_pcore_interrupt_set(&self) -> bool;

    fn is_pcore_non_maskable_interrupt_set(&self) -> bool;

    fn is_acore_interrupt_set(&self) -> bool;

    fn is_acore_non_maskable_interrupt_set(&self) -> bool;

    fn enable_hold(&mut self, on: bool);
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

    fn sleep_mode(&mut self, on: bool) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| w.slp_sel().bit(on));

        self
    }

    fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self {
        get_io_mux_reg(GPIONUM).modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
        self
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

    fn is_pcore_interrupt_set(&self) -> bool {
        (<Self as GpioProperties>::InterruptStatus::pro_cpu_interrupt_status_read()
            & (1 << (GPIONUM % 32)))
            != 0
    }

    fn is_pcore_non_maskable_interrupt_set(&self) -> bool {
        (<Self as GpioProperties>::InterruptStatus::pro_cpu_nmi_status_read()
            & (1 << (GPIONUM % 32)))
            != 0
    }

    fn is_acore_interrupt_set(&self) -> bool {
        (<Self as GpioProperties>::InterruptStatus::app_cpu_interrupt_status_read()
            & (1 << (GPIONUM % 32)))
            != 0
    }

    fn is_acore_non_maskable_interrupt_set(&self) -> bool {
        (<Self as GpioProperties>::InterruptStatus::app_cpu_nmi_status_read()
            & (1 << (GPIONUM % 32)))
            != 0
    }

    fn enable_hold(&mut self, _on: bool) {
        todo!();
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
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Input<Floating>, GPIONUM> {
        pin.into_floating_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Input<PullUp>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Input<PullUp>, GPIONUM> {
        pin.into_pull_up_input()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Input<PullDown>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsInputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Input<PullDown>, GPIONUM> {
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
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Output<PushPull>, GPIONUM> {
        pin.into_push_pull_output()
    }
}

impl<const GPIONUM: u8> From<GpioPin<Unknown, GPIONUM>> for GpioPin<Output<OpenDrain>, GPIONUM>
where
    Self: GpioProperties,
    <Self as GpioProperties>::PinType: IsOutputPin,
    GpioPin<Unknown, GPIONUM>: GpioProperties,
    <GpioPin<Unknown, GPIONUM> as GpioProperties>::PinType: IsOutputPin,
{
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Output<OpenDrain>, GPIONUM> {
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
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Alternate<AF1>, GPIONUM> {
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
    fn from(pin: GpioPin<Unknown, GPIONUM>) -> GpioPin<Alternate<AF2>, GPIONUM> {
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

        paste!{
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

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s2)))]
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
                        // disable input
                        paste! {
                            rtcio.$pin_reg.modify(|_,w| w.$fun_ie().bit(false));

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

#[cfg(esp32s2)]
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

                        paste!{
                            use $crate::gpio::[< esp32s2_get_rtc_pad_ $pin_reg>];
                            let rtc_pad = [< esp32s2_get_rtc_pad_ $pin_reg>]();
                        }

                        // disable input
                        rtc_pad.modify(|_,w| w.$fun_ie().bit(false));

                        // disable output
                        rtcio.enable_w1tc.write(|w| unsafe { w.enable_w1tc().bits(1 << $rtc_pin) });

                        // disable open drain
                        rtcio.pin[$rtc_pin].modify(|_,w| w.pad_driver().bit(false));

                        rtc_pad.modify(|_,w| {
                            w.$fun_ie().clear_bit();

                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.$mux_sel().set_bit();

                            // Select function "RTC function 1" (GPIO) for analog use
                            unsafe { w.$fun_sel().bits(0b00) }
                        });

                        // Disable pull-up and pull-down resistors on the pin, if it has them
                        $(
                            rtc_pad.modify(|_,w| {
                                w
                                .$rue().bit(false)
                                .$rde().bit(false)
                            });
                        )?
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
            pin.clear_interrupt(); // clear stale interrupt flags, when we await we want to know when an event
                                   // occurs from the await call, not if the pin event has happened recently.
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
        let mut intrs = match crate::get_core() {
            crate::Cpu::ProCpu => {
                InterruptStatusRegisterAccessBank0::pro_cpu_interrupt_status_read() as u64
            }
            #[cfg(multi_core)]
            crate::Cpu::AppCpu => {
                InterruptStatusRegisterAccessBank0::app_cpu_interrupt_status_read() as u64
            }
        };

        #[cfg(any(esp32, esp32s2, esp32s3))]
        match crate::get_core() {
            crate::Cpu::ProCpu => {
                intrs |= (InterruptStatusRegisterAccessBank1::pro_cpu_interrupt_status_read()
                    as u64)
                    << 32
            }
            #[cfg(multi_core)]
            crate::Cpu::AppCpu => {
                intrs |= (InterruptStatusRegisterAccessBank1::app_cpu_interrupt_status_read()
                    as u64)
                    << 32
            }
        };

        log::trace!(
            "Handling interrupt on {:?} - {:064b}",
            crate::get_core(),
            intrs
        );

        while intrs != 0 {
            let pin_nr = intrs.trailing_zeros();
            set_int_enable(pin_nr as u8, 0, 0, false);
            PIN_WAKERS[pin_nr as usize].wake(); // wake task
            intrs &= !(1 << pin_nr);
        }

        // clear interrupt bits
        Bank0GpioRegisterAccess::write_interrupt_status_clear(intrs as u32);
        #[cfg(any(esp32, esp32s2, esp32s3))]
        Bank1GpioRegisterAccess::write_interrupt_status_clear((intrs >> 32) as u32);
    }
}
