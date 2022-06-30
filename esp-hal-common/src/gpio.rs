//! GPIO Types
//!
//! Various traits and enums to work with GPIO

use core::marker::PhantomData;

#[doc(hidden)]
pub use paste::paste;

use crate::pac::GPIO;

#[doc(hidden)]
#[cfg_attr(feature = "esp32", path = "gpio/esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "gpio/esp32c3.rs")]
#[cfg_attr(feature = "esp32s2", path = "gpio/esp32s2.rs")]
#[cfg_attr(feature = "esp32s3", path = "gpio/esp32s3.rs")]
pub mod types;

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

pub struct AF0;

pub struct AF1;

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
    type InputSignal;

    fn set_to_input(&mut self) -> &mut Self;

    fn enable_input(&mut self, on: bool) -> &mut Self;

    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn is_input_high(&self) -> bool;

    fn connect_input_to_peripheral(&mut self, signal: Self::InputSignal) -> &mut Self {
        self.connect_input_to_peripheral_with_options(signal, false, false)
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: Self::InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;
}

pub trait OutputPin: Pin {
    type OutputSignal;

    fn set_to_open_drain_output(&mut self) -> &mut Self;

    fn set_to_push_pull_output(&mut self) -> &mut Self;

    fn enable_output(&mut self, on: bool) -> &mut Self;

    fn set_output_high(&mut self, on: bool) -> &mut Self;

    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self;

    fn enable_open_drain(&mut self, on: bool) -> &mut Self;

    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn connect_peripheral_to_output(&mut self, signal: Self::OutputSignal) -> &mut Self {
        self.connect_peripheral_to_output_with_options(signal, false, false, false, false)
    }

    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: Self::OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;

    fn internal_pull_up(&mut self, on: bool) -> &mut Self;

    fn internal_pull_down(&mut self, on: bool) -> &mut Self;
}

#[doc(hidden)]
pub struct SingleCoreInteruptStatusRegisterAccess {}
#[doc(hidden)]
pub struct DualCoreInteruptStatusRegisterAccess {}

#[doc(hidden)]
pub trait InteruptStatusRegisterAccess {
    fn pro_cpu_interrupt_status_read() -> u32;

    fn pro_cpu_nmi_status_read() -> u32;

    fn app_cpu_interrupt_status_read() -> u32;

    fn app_cpu_nmi_status_read() -> u32;
}

impl InteruptStatusRegisterAccess for SingleCoreInteruptStatusRegisterAccess {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int.read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_nmi_int.read().bits()
    }

    fn app_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int.read().bits()
    }

    fn app_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_nmi_int.read().bits()
    }
}

// ESP32S3 is a dual-core chip however pro cpu and app cpu shares the same
// interrupt enable bit see
// https://github.com/espressif/esp-idf/blob/c04803e88b871a4044da152dfb3699cf47354d18/components/hal/esp32s3/include/hal/gpio_ll.h#L32
// Treating it as SingleCore in the gpio macro makes this work.
#[cfg(not(any(feature = "esp32c3", feature = "esp32s2", feature = "esp32s3")))]
impl InteruptStatusRegisterAccess for DualCoreInteruptStatusRegisterAccess {
    fn pro_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_int.read().bits()
    }

    fn pro_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.pcpu_nmi_int.read().bits()
    }

    fn app_cpu_interrupt_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.acpu_int.read().bits()
    }

    fn app_cpu_nmi_status_read() -> u32 {
        unsafe { &*GPIO::PTR }.acpu_nmi_int.read().bits()
    }
}

#[doc(hidden)]
pub trait InterruptStatusRegisters<RegisterAccess>
where
    RegisterAccess: InteruptStatusRegisterAccess,
{
    fn pro_cpu_interrupt_status_read(&self) -> u32 {
        RegisterAccess::pro_cpu_interrupt_status_read()
    }

    fn pro_cpu_nmi_status_read(&self) -> u32 {
        RegisterAccess::pro_cpu_nmi_status_read()
    }

    fn app_cpu_interrupt_status_read(&self) -> u32 {
        RegisterAccess::app_cpu_interrupt_status_read()
    }

    fn app_cpu_nmi_status_read(&self) -> u32 {
        RegisterAccess::app_cpu_nmi_status_read()
    }
}

#[doc(hidden)]
pub struct Bank0GpioRegisterAccess {}
#[doc(hidden)]
pub struct Bank1GpioRegisterAccess {}

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

#[doc(hidden)]
#[cfg(not(feature = "esp32c3"))]
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

#[doc(hidden)]
pub trait GpioRegisters<RegisterAccess>
where
    RegisterAccess: BankGpioRegisterAccess,
{
    fn write_out_en_clear(&self, word: u32) {
        RegisterAccess::write_out_en_clear(word);
    }

    fn write_out_en_set(&self, word: u32) {
        RegisterAccess::write_out_en_set(word);
    }

    fn read_input(&self) -> u32 {
        RegisterAccess::read_input()
    }

    fn read_output(&self) -> u32 {
        RegisterAccess::read_output()
    }

    fn write_interrupt_status_clear(&self, word: u32) {
        RegisterAccess::write_interrupt_status_clear(word);
    }

    fn write_output_clear(&self, word: u32) {
        RegisterAccess::write_output_clear(word)
    }

    fn write_output_set(&self, word: u32) {
        RegisterAccess::write_output_set(word)
    }
}

#[doc(hidden)]
pub fn connect_low_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sel()
            .set_bit()
            .in_inv_sel()
            .bit(false)
            .in_sel()
            .bits(0x1f)
    });
}

#[doc(hidden)]
pub fn connect_high_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::PTR }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sel()
            .set_bit()
            .in_inv_sel()
            .bit(false)
            .in_sel()
            .bits(0x1e)
    });
}

// Only for ESP32 in order to workaround errata 3.6
#[doc(hidden)]
#[macro_export]
macro_rules! impl_errata36 {
    (None, $pull_down:expr, $pull_up:expr) => {
        // NONE
    };

    (pad_dac1, $pull_down:expr, $pull_up:expr) => {
        use crate::pac::RTCIO;
        let rtcio = unsafe { &*RTCIO::PTR };
        rtcio.pad_dac1.modify(|r, w| unsafe {
            w.bits(r.bits())
                .pdac1_rue()
                .bit($pull_up)
                .pdac1_rde()
                .bit($pull_down)
        });
    };

    (pad_dac2, $pull_down:expr, $pull_up:expr) => {
        use crate::pac::RTCIO;
        let rtcio = unsafe { &*RTCIO::PTR };
        rtcio.pad_dac2.modify(|r, w| unsafe {
            w.bits(r.bits())
                .pdac2_rue()
                .bit($pull_up)
                .pdac2_rde()
                .bit($pull_down)
        });
    };

    (xtal_32k_n, $pull_down:expr, $pull_up:expr) => {
        use crate::pac::RTCIO;
        let rtcio = unsafe { &*RTCIO::PTR };
        rtcio.xtal_32k_pad.modify(|r, w| unsafe {
            w.bits(r.bits())
                .x32n_rue()
                .bit($pull_up)
                .x32n_rde()
                .bit($pull_down)
        });
    };

    (xtal_32k_p, $pull_down:expr, $pull_up:expr) => {
        use crate::pac::RTCIO;
        let rtcio = unsafe { &*RTCIO::PTR };
        rtcio.xtal_32k_pad.modify(|r, w| unsafe {
            w.bits(r.bits())
                .x32p_rue()
                .bit($pull_up)
                .x32p_rde()
                .bit($pull_down)
        });
    };

    ($errata36:ident, $pull_down:expr, $pull_up:expr) => {
        use crate::pac::RTCIO;
        let rtcio = unsafe { &*RTCIO::PTR };
        rtcio
            .$errata36
            .modify(|r, w| unsafe { w.bits(r.bits()).rue().bit($pull_up).rde().bit($pull_down) });
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_input {
    (
        $gpio_function:ident,
        $input_signal:ty,
        $pxi:ident:
        (
            $pin_num:expr, $iomux_reg:expr, $bit:expr,
            $errata36:ident
        )
        $( ,( $( $af_signal:ident : $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::InputPin for $pxi<Input<MODE>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.read_input() & (1 << $bit) != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        impl embedded_hal::digital::v2::InputPin for $pxi<Output<OpenDrain>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.read_input() & (1 << $bit) != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::ErrorType for $pxi<Input<MODE>> {
            type Error = Infallible;
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::blocking::InputPin for $pxi<Input<MODE>> {
            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.read_input() & (1 << $bit) != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        impl<MODE> $pxi<MODE> {
            fn init_input(&self, pull_down: bool, pull_up: bool) {
                let gpio = unsafe { &*GPIO::PTR };
                let iomux = unsafe { &*IO_MUX::PTR };

                self.write_out_en_clear(1 << $bit);

                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

                impl_errata36!($errata36, pull_down, pull_up);

                paste! {
                    iomux.$iomux_reg.modify(|_, w| unsafe {
                        w.mcu_sel()
                            .bits(2)
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
            }

            pub fn into_floating_input(self) -> $pxi<Input<Floating>> {
                self.init_input(false, false);
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> InputPin for $pxi<MODE> {
            type InputSignal = $input_signal;

            fn set_to_input(&mut self) -> &mut Self {
                self.init_input(false, false);
                self
            }

            fn enable_input(&mut self, on: bool) -> &mut Self {
                paste!{
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.fun_ie().bit(on));
                }
                self
            }

            fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_ie().bit(on));
                }
                self
            }

            fn is_input_high(&self) -> bool {
                self.read_input() & (1 << $bit) != 0
            }

            fn connect_input_to_peripheral_with_options(
                &mut self,
                signal: Self::InputSignal,
                invert: bool,
                force_via_gpio_mux: bool,
            ) -> &mut Self {
                let af = if force_via_gpio_mux {
                    AlternateFunction::$gpio_function
                } else {
                    match signal {
                        $( $(
                            Self::InputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::$gpio_function
                    }
                };

                if af == AlternateFunction::$gpio_function && signal as usize > INPUT_SIGNAL_MAX as usize {
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
                            .bits($pin_num)
                    });
                }
                self
            }
        }

        impl<MODE> Pin for $pxi<MODE> {
            fn number(&self) -> u8 {
                $pin_num
            }

            fn sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.slp_sel().bit(on));
                }
                self
            }

            fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
                }
                self
            }

            fn listen_with_options(&mut self, event: Event,
                int_enable: bool, nmi_enable: bool,
                wake_up_from_light_sleep: bool
            ) {
                if wake_up_from_light_sleep {
                    match event {
                        Event::AnyEdge | Event::RisingEdge | Event::FallingEdge => {
                            panic!("Edge triggering is not supported for wake-up from light sleep");
                        },
                        _ => {}
                    }
                }

                // a crate using this macro needs to provide gpio_intr_enable
                unsafe {
                    (&*GPIO::PTR).pin[$pin_num].modify(|_, w|
                        w
                            .pin_int_ena().bits(crate::gpio_intr_enable(int_enable, nmi_enable))
                            .pin_int_type().bits(event as u8)
                            .pin_wakeup_enable().bit(wake_up_from_light_sleep)
                    );
                }
            }

            fn unlisten(&mut self) {
                unsafe { (&*GPIO::PTR).pin[$pin_num].modify(|_, w|
                    w.pin_int_ena().bits(0).pin_int_type().bits(0).pin_int_ena().bits(0) );
                }
            }

            fn clear_interrupt(&mut self) {
                self.write_interrupt_status_clear(1 << $bit);
            }

            fn is_pcore_interrupt_set(&self) -> bool {
                (self.pro_cpu_interrupt_status_read() & (1 << $bit)) !=0
            }

            fn is_pcore_non_maskable_interrupt_set(&self) -> bool {
                (self.pro_cpu_nmi_status_read() & (1 << $bit)) !=0
            }

            fn is_acore_interrupt_set(&self) -> bool {
                (self.app_cpu_interrupt_status_read() & (1 << $bit)) !=0
            }

            fn is_acore_non_maskable_interrupt_set(&self) -> bool {
                (self.app_cpu_nmi_status_read() & (1 << $bit)) !=0
            }

            fn enable_hold(&mut self, _on: bool) {
                todo!();
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_output {
    (
        $gpio_function:ident,
        $output_signal:ty,
        $pxi:ident:
        (
            $pin_num:expr, $iomux_reg:expr, $bit:expr
        )
        $( ,( $( $af_signal:ident: $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::OutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.write_output_set(1 << $bit);
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.write_output_clear(1 << $bit);
                Ok(())
            }
        }

        impl<MODE> embedded_hal::digital::v2::StatefulOutputPin for $pxi<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.read_output() & (1 << $bit) != 0)
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_set_high()?)
            }
        }

        impl<MODE> embedded_hal::digital::v2::ToggleableOutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                use embedded_hal::digital::v2::{StatefulOutputPin as _, OutputPin as _};

                if self.is_set_high()? {
                    Ok(self.set_low()?)
                } else {
                    Ok(self.set_high()?)
                }
            }
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::ErrorType for $pxi<Output<MODE>> {
            type Error = Infallible;
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::blocking::OutputPin for $pxi<Output<MODE>> {
            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.write_output_clear(1 << $bit);
                Ok(())
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.write_output_set(1 << $bit);
                Ok(())
            }
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::blocking::StatefulOutputPin for $pxi<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.read_output() & (1 << $bit) != 0)
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_set_high()?)
            }
        }

        #[cfg(feature = "eh1")]
        impl<MODE> embedded_hal_1::digital::blocking::ToggleableOutputPin for $pxi<Output<MODE>> {
            fn toggle(&mut self) -> Result<(), Self::Error> {
                use embedded_hal_1::digital::blocking::{StatefulOutputPin as _, OutputPin as _};

                if self.is_set_high()? {
                    Ok(self.set_low()?)
                } else {
                    Ok(self.set_high()?)
                }
            }
        }

        impl<MODE> $pxi<MODE> {
            pub fn into_pull_up_input(self) -> $pxi<Input<PullUp>> {
                self.init_input(false, true);
                $pxi { _mode: PhantomData }
            }

            pub fn into_pull_down_input(self) -> $pxi<Input<PullDown>> {
                self.init_input(true, false);
                $pxi { _mode: PhantomData }
            }

            fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
                let gpio = unsafe { &*GPIO::PTR };
                let iomux = unsafe { &*IO_MUX::PTR };

                self.write_out_en_set(1 << $bit);
                gpio.pin[$pin_num].modify(|_, w| w.pin_pad_driver().bit(open_drain));

                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

                paste! {
                    iomux.$iomux_reg.modify(|_, w| unsafe {
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

            pub fn into_push_pull_output(self) -> $pxi<Output<PushPull>> {
                self.init_output(AlternateFunction::$gpio_function, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_open_drain_output(self) -> $pxi<Output<OpenDrain>> {
                self.init_output(AlternateFunction::$gpio_function, true);
                $pxi { _mode: PhantomData }
            }

            pub fn into_alternate_1(self) -> $pxi<Alternate<AF1>> {
                self.init_output(AlternateFunction::Function1, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_alternate_2(self) -> $pxi<Alternate<AF2>> {
                self.init_output(AlternateFunction::Function2, false);
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> OutputPin for $pxi<MODE> {
            type OutputSignal = $output_signal;

            fn set_to_open_drain_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::$gpio_function, true);
                self
            }

            fn set_to_push_pull_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::$gpio_function, false);
                self
            }

            fn enable_output(&mut self, on: bool) -> &mut Self {
                if on {
                    self.write_out_en_set(1 << $bit);
                } else {
                    self.write_out_en_clear(1 << $bit);
                }
                self
            }

            fn set_output_high(&mut self, high: bool) -> &mut Self {
                if high {
                    self.write_output_set(1 << $bit);
                } else {
                    self.write_output_clear(1 << $bit);
                }
                self
            }

            fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                    .$iomux_reg
                    .modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
                }
                self
            }

            fn enable_open_drain(&mut self, on: bool) -> &mut Self {
                unsafe { &*GPIO::PTR }.pin[$pin_num].modify(|_, w| w.pin_pad_driver().bit(on));
                self
            }

            fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_wpu().bit(on));
                }
                self
            }

            fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste!{
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_wpd().bit(on));
                }
                self
            }

            fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_oe().bit(on));
                }
                self
            }

            fn connect_peripheral_to_output_with_options(
                &mut self,
                signal: Self::OutputSignal,
                invert: bool,
                invert_enable: bool,
                enable_from_gpio: bool,
                force_via_gpio_mux: bool,
            ) -> &mut Self {
                let af = if force_via_gpio_mux {
                    AlternateFunction::$gpio_function
                } else {
                    match signal {
                        $( $(
                            Self::OutputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::$gpio_function
                    }
                };

                if af == AlternateFunction::$gpio_function && signal as usize > OUTPUT_SIGNAL_MAX as usize {
                    panic!("Cannot connect this peripheral to GPIO");
                }

                self.set_alternate_function(af);

                let clipped_signal = if signal as usize <= OUTPUT_SIGNAL_MAX as usize { signal as OutputSignalType } else { OUTPUT_SIGNAL_MAX };

                unsafe { &*GPIO::PTR }.func_out_sel_cfg[$pin_num].modify(|_, w| unsafe {
                    w
                        .out_sel().bits(clipped_signal)
                        .inv_sel().bit(invert)
                        .oen_sel().bit(enable_from_gpio)
                        .oen_inv_sel().bit(invert_enable)
                });

                self
            }

            fn internal_pull_up(&mut self, on: bool) -> &mut Self {
                paste!{
                    unsafe { &*IO_MUX::PTR }.$iomux_reg.modify(|_, w| w.fun_wpu().bit(on));
                }
                self
            }

            fn internal_pull_down(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::PTR }.$iomux_reg.modify(|_, w| w.fun_wpd().bit(on));
                }
                self
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_output_wrap {
    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, IO,
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {
        impl_output!(
            $gpio_function,
            OutputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32
            )
            $( ,( $( $af_output_signal: $af_output ),* ) )?
        );
    };

    // if it's not an output enabled pin just emit no code here
    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident,
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {};
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_gpio_register_access {
    (Bank0, $pxi:ident) => {
        #[doc(hidden)]
        impl<MODE> GpioRegisters<Bank0GpioRegisterAccess> for $pxi<MODE> {}
    };

    (Bank1, $pxi:ident) => {
        #[doc(hidden)]
        impl<MODE> GpioRegisters<Bank1GpioRegisterAccess> for $pxi<MODE> {}
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_interrupt_status_register_access {
    (SingleCore, $pxi:ident) => {
        #[doc(hidden)]
        impl<MODE> InterruptStatusRegisters<SingleCoreInteruptStatusRegisterAccess> for $pxi<MODE> {}
    };

    (DualCore, $pxi:ident) => {
        #[doc(hidden)]
        impl<MODE> InterruptStatusRegisters<DualCoreInteruptStatusRegisterAccess> for $pxi<MODE> {}
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! gpio {
    (
        $gpio_function:ident,
        $cores:ident,
        $(
            $pxi:ident:
            (
                $pname:ident, $pin_num:literal, $iomux_reg:expr, $type:ident,
                $rtc:tt, $bank:ident, $errata36:ident
            ),
            $(
                ( $( $af_input_signal:ident: $af_input:ident ),* ),
                $( ( $( $af_output_signal:ident: $af_output:ident ),* ), )?
            )?
        )+
    ) => {
        use core::{convert::Infallible, marker::PhantomData};

        use crate::pac::{GPIO, IO_MUX};

        pub struct IO {
            io_mux: IO_MUX,
            pub pins: Pins,
        }

        impl IO {
            pub fn new(gpio: GPIO, io_mux: IO_MUX) -> Self {
                let pins = gpio.split();
                let io = IO { io_mux, pins };

                io
            }
        }

        pub trait GpioExt {
            type Parts;

            fn split(self) -> Self::Parts;
        }

        impl GpioExt for GPIO {
            type Parts = Pins;

            fn split(self) -> Self::Parts {
                Pins {
                    $(
                        $pname: $pxi { _mode: PhantomData },
                    )+
                }
            }
        }

        pub struct Pins {
            $(
                pub $pname: $pxi<Unknown>,
            )+
        }

        $(
            pub struct $pxi<MODE> {
                _mode: PhantomData<MODE>,
            }

            impl_gpio_register_access!($bank, $pxi);

            impl_interrupt_status_register_access!($cores, $pxi);

            impl_input!(
                $gpio_function,
                InputSignal,
                $pxi:
                (
                    $pin_num, $iomux_reg, $pin_num % 32,
                    $errata36
                )
                $( ,( $( $af_input_signal: $af_input ),* ) )?
            );

            impl_output_wrap!(
                $gpio_function, $pxi, $pin_num, $iomux_reg, $type,
                $($( ,( $( $af_output_signal: $af_output ),* ) )? )?
            );
        )+
    };
}

#[doc(hidden)]
pub fn enable_iomux_clk_gate() {
    #[cfg(feature = "esp32s2")]
    {
        use crate::pac::SENS;
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_io_mux_conf
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! analog {
    ([
        $($pxi:ident: ($pin_num:expr, $pin_reg:ident, $hold: ident, $mux_sel:ident,
            $fun_sel:ident, $fun_ie:ident, $slp_ie:ident, $slp_sel:ident
            $(, $rue:ident, $rde:ident, $drv:ident, $slp_oe:ident)?),)+
    ]) => {
        $(
            impl<MODE> $pxi<MODE> {
                pub fn into_analog(mut self) -> $pxi<Analog> {
                    $(
                        use crate::pac::RTCIO;
                        let rtcio = unsafe{ &*RTCIO::ptr() };

                        $crate::gpio::enable_iomux_clk_gate();

                        // disable input
                        rtcio.$pin_reg.modify(|_,w| w.$fun_ie().bit(false));

                        // disable output
                        rtcio.enable_w1tc.write(|w| unsafe { w.enable_w1tc().bits(1 << $pin_num) });
                        // ESP32-S2 rtcio.rtc_gpio_enable_w1tc.write(|w| unsafe { w.reg_rtcio_reg_gpio_enable_w1tc().bits(1 << $pin_num) });
                        //rtcio.rtc_gpio_enable_w1tc.write(|w| unsafe { w.rtc_gpio_enable_w1tc().bits(1 << $pin_num) });

                        // disable open drain
                        rtcio.pin[$pin_num].modify(|_,w| w.pin_pad_driver().bit(false));
                        //rtcio.rtc_gpio_pin[$pin_num].modify(|_,w| w.gpio_pin0_pad_driver().bit(false));

                        rtcio.$pin_reg.modify(|_,w| {
                            w.$fun_ie().clear_bit();

                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.$mux_sel().set_bit();

                            // Select function "RTC function 1" (GPIO) for analog use
                            unsafe { w.$fun_sel().bits(0b00) }
                        });

                        // Disable pull-up and pull-down resistors on the pin, if it has them
                        rtcio.$pin_reg.modify(|_,w| {
                            w
                            .$rue().bit(false)
                            .$rde().bit(false)
                        });
                    )?

                    $pxi { _mode: PhantomData }
                }
            }
        )+
    }
}

pub use analog;
pub use gpio;
pub use impl_errata36;
pub use impl_gpio_register_access;
pub use impl_input;
pub use impl_interrupt_status_register_access;
pub use impl_output;
pub use impl_output_wrap;

use self::types::InputSignal;
