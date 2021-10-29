use core::{convert::Infallible, marker::PhantomData};

use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};

use crate::pac::{GPIO, IO_MUX};

mod mux;
pub use mux::*;

pub use crate::prelude::*;

#[allow(dead_code)]
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

pub trait Pin {
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

    fn is_interrupt_set(&mut self) -> bool;

    fn is_non_maskable_interrupt_set(&mut self) -> bool;

    fn enable_hold(&mut self, on: bool);
}

pub trait InputPin: Pin {
    fn set_to_input(&mut self) -> &mut Self;

    fn enable_input(&mut self, on: bool) -> &mut Self;

    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn is_input_high(&mut self) -> bool;

    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        self.connect_input_to_peripheral_with_options(signal, false, false)
    }

    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;
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

    fn internal_pull_up(&mut self, on: bool) -> &mut Self;

    fn internal_pull_down(&mut self, on: bool) -> &mut Self;
}

pub trait RTCPin {}

pub trait AnalogPin {}

#[derive(Copy, Clone)]
pub enum Event {
    RisingEdge = 1,
    FallingEdge = 2,
    AnyEdge    = 3,
    LowLevel   = 4,
    HighLevel  = 5,
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
}

pub fn connect_low_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sig_in_sel()
            .set_bit()
            .func_in_inv_sel()
            .bit(false)
            .func_in_sel()
            .bits(0x1f)
    });
}

pub fn connect_high_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sig_in_sel()
            .set_bit()
            .func_in_inv_sel()
            .bit(false)
            .func_in_sel()
            .bits(0x1e)
    });
}

macro_rules! impl_output {
    (
        $pxi:ident:
        (
            $pin_num:expr, $bit:expr, $out_en_set:ident, $out_en_clear:ident,
            $out_set:ident, $out_clear:ident, $out_reg:ident
        )
        $( ,( $( $af_signal:ident: $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::OutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                unsafe { (*GPIO::ptr()).$out_set.write(|w| w.bits(1 << $bit)) };
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                unsafe { (*GPIO::ptr()).$out_clear.write(|w| w.bits(1 << $bit)) };
                Ok(())
            }
        }

        impl<MODE> embedded_hal::digital::v2::StatefulOutputPin for $pxi<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                unsafe { Ok((*GPIO::ptr()).$out_reg.read().bits() & (1 << $bit) != 0) }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_set_high()?)
            }
        }

        impl<MODE> embedded_hal::digital::v2::ToggleableOutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_set_high()? {
                    Ok(self.set_low()?)
                } else {
                    Ok(self.set_high()?)
                }
            }
        }

        impl<MODE> $pxi<MODE> {
            pub fn into_pull_up_input(self) -> $pxi<Input<PullUp>> {
                self.init_input(false, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_pull_down_input(self) -> $pxi<Input<PullDown>> {
                self.init_input(true, false);
                $pxi { _mode: PhantomData }
            }

            fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
                let gpio = unsafe { &*GPIO::ptr() };
                let iomux = unsafe { &*IO_MUX::ptr() };

                gpio.$out_en_set.write(|w| unsafe { w.bits(1 << $bit) });
                gpio.pin[$pin_num].modify(|_, w| w.pin_pad_driver().bit(open_drain));
                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.func_out_sel().bits(OutputSignal::GPIO as u8) });

                iomux.gpio[$pin_num].modify(|_, w| unsafe {
                    w.mcu_sel()
                        .bits(alternate as u8)
                        .fun_ie()
                        .clear_bit()
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

            pub fn into_push_pull_output(self) -> $pxi<Output<PushPull>> {
                self.init_output(AlternateFunction::Function1, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_open_drain_output(self) -> $pxi<Output<OpenDrain>> {
                self.init_output(AlternateFunction::Function1, true);
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
            fn set_to_open_drain_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::Function1, true);
                self
            }

            fn set_to_push_pull_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::Function1, false);
                self
            }

            fn enable_output(&mut self, on: bool) -> &mut Self {
                if on {
                    unsafe { &*GPIO::ptr() }
                        .$out_en_set
                        .write(|w| unsafe { w.bits(1 << $bit) });
                } else {
                    unsafe { &*GPIO::ptr() }
                        .$out_en_clear
                        .write(|w| unsafe { w.bits(1 << $bit) });
                }
                self
            }

            fn set_output_high(&mut self, high: bool) -> &mut Self {
                if high {
                    unsafe { (*GPIO::ptr()).$out_set.write(|w| w.bits(1 << $bit)) };
                } else {
                    unsafe { (*GPIO::ptr()).$out_clear.write(|w| w.bits(1 << $bit)) };
                }
                self
            }

            fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
                self
            }

            fn enable_open_drain(&mut self, on: bool) -> &mut Self {
                unsafe { &*GPIO::ptr() }.pin[$pin_num].modify(|_, w| w.pin_pad_driver().bit(on));
                self
            }

            fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_wpu().bit(on));
                self
            }

            fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_wpd().bit(on));
                self
            }

            fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_oe().bit(on));
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
                    AlternateFunction::Function1
                } else {
                    match signal {
                        $( $(
                            OutputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::Function1
                    }
                };

                if af == AlternateFunction::Function1 && signal as usize > 128 {
                    panic!("Cannot connect this peripheral to GPIO");
                }

                self.set_alternate_function(af);

                let clipped_signal = if signal as usize <= 128 { signal as u8 } else { 128u8 };

                unsafe { &*GPIO::ptr() }.func_out_sel_cfg[$pin_num].modify(|_, w| unsafe {
                    w
                        .func_out_sel().bits(clipped_signal)
                        .func_out_inv_sel().bit(invert)
                        .func_oen_sel().bit(enable_from_gpio)
                        .func_oen_inv_sel().bit(invert_enable)
                });

                self
            }

            fn internal_pull_up(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }.gpio[$pin_num].modify(|_, w| w.fun_wpu().bit(on));
                self
            }

            fn internal_pull_down(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }.gpio[$pin_num].modify(|_, w| w.fun_wpd().bit(on));
                self
            }
        }
    };
}

macro_rules! impl_input {
    ($pxi:ident:
        ($pin_num:expr, $bit:expr, $out_en_clear:ident, $reg:ident, $reader:ident,
            $status_w1tc:ident, $pcpu_int:ident, $pcpu_nmi:ident
        ) $( ,( $( $af_signal:ident : $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::InputPin for $pxi<Input<MODE>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { &*GPIO::ptr() }.$reg.read().$reader().bits() & (1 << $bit) != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        impl<MODE> $pxi<MODE> {
            fn init_input(&self, pull_down: bool, pull_up: bool) {
                let gpio = unsafe { &*GPIO::ptr() };
                let iomux = unsafe { &*IO_MUX::ptr() };


                gpio.$out_en_clear
                    .write(|w| unsafe { w.bits(1 << $bit) });

                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.func_out_sel().bits(OutputSignal::GPIO as u8) });

                iomux.gpio[$pin_num].modify(|_, w| unsafe {
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

            pub fn into_floating_input(self) -> $pxi<Input<Floating>> {
                self.init_input(false, false);
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> InputPin for $pxi<MODE> {
            fn set_to_input(&mut self) -> &mut Self {
                self.init_input(false, false);
                self
            }

            fn enable_input(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.fun_ie().bit(on));
                self
            }

            fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_ie().bit(on));
                self
            }

            fn is_input_high(&mut self) -> bool {
                unsafe { &*GPIO::ptr() }.$reg.read().$reader().bits() & (1 << $bit) != 0
            }

            fn connect_input_to_peripheral_with_options(
                &mut self,
                signal: InputSignal,
                invert: bool,
                force_via_gpio_mux: bool,
            ) -> &mut Self {

                let af = if force_via_gpio_mux
                {
                    AlternateFunction::Function1
                }
                else {
                    match signal {
                        $( $(
                            InputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::Function1
                    }
                };

                if af == AlternateFunction::Function1 && signal as usize >= 128 {
                    panic!("Cannot connect GPIO to this peripheral");
                }

                self.set_alternate_function(af);

                if (signal as usize) < 128 {
                    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                        w.sig_in_sel()
                            .set_bit()
                            .func_in_inv_sel()
                            .bit(invert)
                            .func_in_sel()
                            .bits($pin_num)
                    });
                }
                self
            }
        }

        impl<MODE> Pin for $pxi<MODE> {
            fn sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.slp_sel().bit(on));
                self
            }

            fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self {

                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
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
                unsafe {
                    (&*GPIO::ptr()).pin[$pin_num].modify(|_, w|
                        w
                            .pin_int_ena().bits(int_enable as u8 | ((nmi_enable as u8) << 1))
                            .pin_int_type().bits(event as u8)
                            .pin_wakeup_enable().bit(wake_up_from_light_sleep)
                    );
                }
            }

            fn unlisten(&mut self) {
                unsafe { (&*GPIO::ptr()).pin[$pin_num].modify(|_, w|
                    w.pin_int_ena().bits(0).pin_int_type().bits(0).pin_int_ena().bits(0) );
                }
            }

            fn clear_interrupt(&mut self) {
                unsafe {&*GPIO::ptr()}.$status_w1tc.write(|w|
                    unsafe {w.bits(1 << $bit)})
            }

            fn is_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_int.read().bits() & (1 << $bit)) !=0
            }

            fn is_non_maskable_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_nmi.read().bits() & (1 << $bit)) !=0
            }

            fn enable_hold(&mut self, _on: bool) {


                todo!();
            }
        }
    };
}

macro_rules! impl_pin_wrap {
    ($pxi:ident, $pin_num:expr, IO
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!($pxi: ($pin_num, $pin_num % 32, enable_w1tc, in_, data_next,
            status_w1tc, pcpu_int, pcpu_nmi_int)
            $( ,( $( $af_input_signal: $af_input ),* ) )? );
    };
}

macro_rules! impl_output_wrap {
    ($pxi:ident, $pin_num:expr, IO
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {
        impl_output!($pxi:
            ($pin_num, $pin_num % 32, enable_w1ts, enable_w1tc, out_w1ts, out_w1tc, out)

            $( ,( $( $af_output_signal: $af_output ),* ) )? );
    };
}

macro_rules! gpio {
    ( $($pxi:ident: ($pname:ident, $pin_num:literal,
        $type:ident, $rtc:tt ),
        $(
            ( $( $af_input_signal:ident: $af_input:ident ),* ),
            $(
            ( $( $af_output_signal:ident: $af_output:ident ),* ),
            )?
        )?
        )+ ) => {

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

            impl_pin_wrap!($pxi, $pin_num, $type
                $( ,( $( $af_input_signal: $af_input ),* ) )? );
            impl_output_wrap!($pxi, $pin_num, $type
                $($( ,( $( $af_output_signal: $af_output ),* ) )? )? );
        )+
    };
}

gpio! {
    Gpio0: (gpio0, 0, IO, RTC),
    Gpio1: (gpio1, 1, IO, RTC),
    Gpio2: (gpio2, 2, IO, RTC), (FSPIQ: Function2), (FSPIQ: Function2),
    Gpio3: (gpio3, 3, IO, RTC),
    Gpio4: (gpio4, 4, IO, RTC), (FSPIHD: Function2), (USB_JTAG_TMS: Function0, FSPIHD: Function2),
    Gpio5: (gpio5, 5, IO, RTC), (FSPIWP: Function2), (USB_JTAG_TDI: Function0, FSPIWP: Function2),
    Gpio6: (gpio6, 6, IO, 0), (FSPICLK: Function2), (USB_JTAG_TCK: Function0, FSPICLK_MUX: Function2),
    Gpio7: (gpio7, 7, IO, 0), (FSPID: Function2), (USB_JTAG_TDO: Function0, FSPID: Function2),
    Gpio8: (gpio8, 8, IO, 0),
    Gpio9: (gpio9, 9, IO, 0),
    Gpio10: (gpio10, 10, IO, 0), (FSPICS0: Function2), (FSPICS0: Function2),
    Gpio11: (gpio11, 11, IO, 0),
    Gpio12: (gpio12, 12, IO, 0), (SPIHD: Function0), (SPIHD: Function0),
    Gpio13: (gpio13, 13, IO, 0), (SPIWP: Function0), (SPIWP: Function0),
    Gpio14: (gpio14, 14, IO, 0), (), (SPICS0: Function0),
    Gpio15: (gpio15, 15, IO, 0), (), (SPICLK_MUX: Function0),
    Gpio16: (gpio16, 16, IO, 0), (SPID: Function0), (SPID: Function0),
    Gpio17: (gpio17, 17, IO, 0), (SPIQ: Function0), (SPIQ: Function0),
    Gpio18: (gpio18, 18, IO, 0),
    Gpio19: (gpio19, 19, IO, 0),
    Gpio20: (gpio20, 20, IO, 0), (U0RXD: Function0), (),
    Gpio21: (gpio21, 21, IO, 0), (), (U0TXD: Function0),
}
