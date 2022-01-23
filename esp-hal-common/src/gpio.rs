//! GPIO driver
//!
//! Defines a series of macros which allow for the definition of each chip's
//! GPIO pins in a generic manner. Implements the various traits defined by
//! [embedded-hal].
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

use core::marker::PhantomData;

pub use paste::paste;

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

    fn is_pcore_interrupt_set(&mut self) -> bool;

    fn is_pcore_non_maskable_interrupt_set(&mut self) -> bool;

    fn is_acore_interrupt_set(&mut self) -> bool;

    fn is_acore_non_maskable_interrupt_set(&mut self) -> bool;

    fn enable_hold(&mut self, on: bool);
}

pub trait InputPin: Pin {
    type InputSignal;

    fn set_to_input(&mut self) -> &mut Self;

    fn enable_input(&mut self, on: bool) -> &mut Self;

    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    fn is_input_high(&mut self) -> bool;

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

#[macro_export]
macro_rules! impl_input {
    (
        $gpio_function:ident,
        $input_signal:ty,
        $pxi:ident:
        (
            $pin_num:expr, $iomux_reg:expr, $bit:expr, $out_en_clear:ident,
            $reg:ident, $reader:ident, $status_w1tc:ident, $pcpu_int:ident,
            $pcpu_nmi:ident, $acpu_int:ident, $acpu_nmi:ident
        )
        $( ,( $( $af_signal:ident : $af:ident ),* ))?
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
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

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
                    unsafe { &*IO_MUX::ptr() }
                        .$iomux_reg
                        .modify(|_, w| w.fun_ie().bit(on));
                }
                self
            }

            fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_ie().bit(on));
                }
                self
            }

            fn is_input_high(&mut self) -> bool {
                unsafe { &*GPIO::ptr() }.$reg.read().$reader().bits() & (1 << $bit) != 0
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
                    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
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
            fn sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }
                        .$iomux_reg
                        .modify(|_, w| w.slp_sel().bit(on));
                }
                self
            }

            fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }
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
                    (&*GPIO::ptr()).pin[$pin_num].modify(|_, w|
                        w
                            .pin_int_ena().bits(crate::gpio_intr_enable(int_enable, nmi_enable))
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

            fn is_pcore_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_int.read().bits() & (1 << $bit)) !=0
            }

            fn is_pcore_non_maskable_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_nmi.read().bits() & (1 << $bit)) !=0
            }

            fn is_acore_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$acpu_int.read().bits() & (1 << $bit)) !=0
            }

            fn is_acore_non_maskable_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$acpu_nmi.read().bits() & (1 << $bit)) !=0
            }

            fn enable_hold(&mut self, _on: bool) {
                todo!();
            }
        }
    };
}

#[macro_export]
macro_rules! impl_input_wrap {
    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident, Bank0, SingleCore
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!(
            $gpio_function,
            InputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable_w1tc, in_, data_next,
                status_w1tc, pcpu_int, pcpu_nmi_int, pcpu_int, pcpu_nmi_int
            )
            $( ,( $( $af_input_signal: $af_input ),* ) )?
        );
    };

    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident, Bank1, SingleCore
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!(
            $gpio_function,
            InputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable1_w1tc, in1, data_next,
                status1_w1tc, pcpu_int1, pcpu_nmi_int1, pcpu_int1, pcpu_nmi_int1
            )
            $( ,( $( $af_input_signal: $af_input ),* ) )?
        );
    };

    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident, Bank0, DualCore
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!(
            $gpio_function,
            InputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable_w1tc, in_, data_next,
                status_w1tc, pcpu_int, pcpu_nmi_int, acpu_int, acpu_nmi_int
            )
            $( ,( $( $af_input_signal: $af_input ),* ) )?
        );
    };

    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident, Bank1, DualCore
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!(
            $gpio_function,
            InputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable1_w1tc, in1, data_next,
                status1_w1tc, pcpu_int1, pcpu_nmi_int1, acpu_int1, acpu_nmi_int1
            )
            $( ,( $( $af_input_signal: $af_input ),* ) )?
        );
    };
}

#[macro_export]
macro_rules! impl_output {
    (
        $gpio_function:ident,
        $output_signal:ty,
        $pxi:ident:
        (
            $pin_num:expr, $iomux_reg:expr, $bit:expr, $out_en_set:ident,
            $out_en_clear:ident, $out_set:ident, $out_clear:ident, $out_reg:ident
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
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

                paste! {
                    iomux.$iomux_reg.modify(|_, w| unsafe {
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
                paste! {
                    unsafe { &*IO_MUX::ptr() }
                    .$iomux_reg
                    .modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
                }
                self
            }

            fn enable_open_drain(&mut self, on: bool) -> &mut Self {
                unsafe { &*GPIO::ptr() }.pin[$pin_num].modify(|_, w| w.pin_pad_driver().bit(on));
                self
            }

            fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_wpu().bit(on));
                }
                self
            }

            fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste!{
                    unsafe { &*IO_MUX::ptr() }
                        .$iomux_reg
                        .modify(|_, w| w.mcu_wpd().bit(on));
                }
                self
            }

            fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }
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

                unsafe { &*GPIO::ptr() }.func_out_sel_cfg[$pin_num].modify(|_, w| unsafe {
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
                    unsafe { &*IO_MUX::ptr() }.$iomux_reg.modify(|_, w| w.fun_wpu().bit(on));
                }
                self
            }

            fn internal_pull_down(&mut self, on: bool) -> &mut Self {
                paste! {
                    unsafe { &*IO_MUX::ptr() }.$iomux_reg.modify(|_, w| w.fun_wpd().bit(on));
                }
                self
            }
        }
    };
}

#[macro_export]
macro_rules! impl_output_wrap {
    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, IO, Bank0
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {
        impl_output!(
            $gpio_function,
            OutputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable_w1ts, enable_w1tc,
                out_w1ts, out_w1tc, out
            )
            $( ,( $( $af_output_signal: $af_output ),* ) )?
        );
    };

    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, IO, Bank1
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {
        impl_output!(
            $gpio_function,
            OutputSignal,
            $pxi:
            (
                $pin_num, $iomux_reg, $pin_num % 32, enable1_w1ts, enable1_w1tc,
                out1_w1ts, out1_w1tc, out1
            )
            $( ,( $( $af_output_signal: $af_output ),* ) )?
        );
    };

    (
        $gpio_function:ident,
        $pxi:ident, $pin_num:expr, $iomux_reg:expr, $type:ident, $bank:ident
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {};
}

#[macro_export]
macro_rules! gpio {
    (
        $gpio_function:ident,
        $cores:ident,
        $(
            $pxi:ident:
            (
                $pname:ident, $pin_num:literal, $iomux_reg:expr, $type:ident,
                $rtc:tt, $bank:ident
            ),
            $(
                ( $( $af_input_signal:ident: $af_input:ident ),* ),
                $( ( $( $af_output_signal:ident: $af_output:ident ),* ), )?
            )?
        )+
    ) => {
        use core::{convert::Infallible, marker::PhantomData};
        use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};
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

        pub fn connect_low_to_peripheral(signal: InputSignal) {
            unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(0x1f)
            });
        }

        pub fn connect_high_to_peripheral(signal: InputSignal) {
            unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(0x1e)
            });
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

            impl_input_wrap!(
                $gpio_function, $pxi, $pin_num, $iomux_reg, $type, $bank, $cores
                $( ,( $( $af_input_signal: $af_input ),* ) )?
            );

            impl_output_wrap!(
                $gpio_function, $pxi, $pin_num, $iomux_reg, $type, $bank
                $($( ,( $( $af_output_signal: $af_output ),* ) )? )?
            );
        )+
    };
}

pub use gpio;
pub use impl_input;
pub use impl_input_wrap;
pub use impl_output;
pub use impl_output_wrap;
