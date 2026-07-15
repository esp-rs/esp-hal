use crate::peripherals::{GPIO, LPWR, RTC_IO};

macro_rules! rtcio_analog {
    ($pin_peri:ident, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident) => {
        paste::paste! {
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl crate::gpio::RtcPin for crate::peripherals::$pin_peri<'_> {
                fn rtc_number(&self) -> u8 {
                    $rtc_pin
                }

                fn rtc_set_config(
                    &self,
                    input_enable: bool,
                    mux: bool,
                    func: crate::gpio::RtcFunction,
                ) {
                    RTC_IO::regs()
                        .$pin_reg
                        .modify(|_, w| unsafe {
                            w.[<$prefix fun_ie>]().bit(input_enable);
                            w.[<$prefix mux_sel>]().bit(mux);
                            w.[<$prefix fun_sel>]().bits(func as u8)
                        });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    LPWR::regs()
                        .hold_force()
                        .modify(|_, w| w.$hold().bit(enable));
                }
            }

            // Only output pins have PU/PD resistors.
            for_each_gpio! {
                ($n:tt, $pin_peri $in_afs:tt $out_afs:tt ($input:tt [Output])) => {
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    impl crate::gpio::RtcPinWithResistors
                        for crate::peripherals::$pin_peri<'_>
                    {
                        fn rtcio_pullup(&self, enable: bool) {
                            pullup_enable($rtc_pin, enable)
                        }

                        fn rtcio_pulldown(&self, enable: bool) {
                            pulldown_enable($rtc_pin, enable)
                        }
                    }
                };
            }

            impl crate::peripherals::$pin_peri<'_> {
                #[cfg(feature = "unstable")]
                pub(crate) fn set_analog_impl(&self) {
                    use crate::gpio::RtcPin;

                    output_enable(self.rtc_number(), false);
                    set_open_drain_output(self.rtc_number(), false);

                    RTC_IO::regs().$pin_reg.modify(|_, w| {
                        w.[<$prefix fun_ie>]().clear_bit();
                        w.[<$prefix mux_sel>]().set_bit();
                        unsafe { w.[<$prefix fun_sel>]().bits(0) };

                        // Only output pins have PU/PD resistors.
                        for_each_gpio! {
                            ($n:tt, $pin_peri $in_afs:tt $out_afs:tt ($input:tt [Output])) => {
                                w.[<$prefix rue>]().bit(false);
                                w.[<$prefix rde>]().bit(false);
                            };
                        }

                        w
                    });
                }
            }
        }
    };

    (
        $(($pin_peri:ident, $rtc_pin:tt, $pin_reg:expr, $prefix:pat, $hold:ident))+
    ) => {
        $(
            rtcio_analog!($pin_peri, $rtc_pin, $pin_reg, $prefix, $hold);
        )+

        macro_rules! set_one_pad_field {
            $(
                ($rtc_pin, $field:ident, $enable:ident) => {{
                    paste::paste! {
                        RTC_IO::regs()
                            .$pin_reg
                            .modify(|_, w|  w.[<$prefix $field>]().bit($enable));
                    }
                }};
            )+
        }
    };
}

rtcio_analog! {
    (GPIO36, 0,  sensor_pads(),    sense1_, sense1    )
    (GPIO37, 1,  sensor_pads(),    sense2_, sense2    )
    (GPIO38, 2,  sensor_pads(),    sense3_, sense3    )
    (GPIO39, 3,  sensor_pads(),    sense4_, sense4    )
    (GPIO34, 4,  adc_pad(),        adc1_,   adc1      )
    (GPIO35, 5,  adc_pad(),        adc2_,   adc2      )
    (GPIO25, 6,  pad_dac1(),       "",      pdac1     )
    (GPIO26, 7,  pad_dac2(),       "",      pdac2     )
    (GPIO33, 8,  xtal_32k_pad(),   x32n_,   x32n      )
    (GPIO32, 9,  xtal_32k_pad(),   x32p_,   x32p      )
    (GPIO4,  10, touch_pad0(),     "",      touch_pad0)
    (GPIO0,  11, touch_pad1(),     "",      touch_pad1)
    (GPIO2,  12, touch_pad2(),     "",      touch_pad2)
    (GPIO15, 13, touch_pad3(),     "",      touch_pad3)
    (GPIO13, 14, touch_pad4(),     "",      touch_pad4)
    (GPIO12, 15, touch_pad5(),     "",      touch_pad5)
    (GPIO14, 16, touch_pad6(),     "",      touch_pad6)
    (GPIO27, 17, touch_pad7(),     "",      touch_pad7)
}

macro_rules! set_pad_field {
    ($pin:ident, $field:ident, $enable:ident) => {{
        match $pin {
            0 => set_one_pad_field!(0, $field, $enable),
            1 => set_one_pad_field!(1, $field, $enable),
            2 => set_one_pad_field!(2, $field, $enable),
            3 => set_one_pad_field!(3, $field, $enable),
            4 => set_one_pad_field!(4, $field, $enable),
            5 => set_one_pad_field!(5, $field, $enable),
            6 => set_one_pad_field!(6, $field, $enable),
            7 => set_one_pad_field!(7, $field, $enable),
            8 => set_one_pad_field!(8, $field, $enable),
            9 => set_one_pad_field!(9, $field, $enable),
            10 => set_one_pad_field!(10, $field, $enable),
            11 => set_one_pad_field!(11, $field, $enable),
            12 => set_one_pad_field!(12, $field, $enable),
            13 => set_one_pad_field!(13, $field, $enable),
            14 => set_one_pad_field!(14, $field, $enable),
            15 => set_one_pad_field!(15, $field, $enable),
            16 => set_one_pad_field!(16, $field, $enable),
            17 => set_one_pad_field!(17, $field, $enable),
            _ => unreachable!(),
        }
    }};
}

macro_rules! set_pull_field {
    ($pin:ident, $field:ident, $enable:ident) => {{
        match $pin {
            0..=5 => warn!("Pin {} does not have PU/PD resistors", $pin),
            6 => set_one_pad_field!(6, $field, $enable),
            7 => set_one_pad_field!(7, $field, $enable),
            8 => set_one_pad_field!(8, $field, $enable),
            9 => set_one_pad_field!(9, $field, $enable),
            10 => set_one_pad_field!(10, $field, $enable),
            11 => set_one_pad_field!(11, $field, $enable),
            12 => set_one_pad_field!(12, $field, $enable),
            13 => set_one_pad_field!(13, $field, $enable),
            14 => set_one_pad_field!(14, $field, $enable),
            15 => set_one_pad_field!(15, $field, $enable),
            16 => set_one_pad_field!(16, $field, $enable),
            17 => set_one_pad_field!(17, $field, $enable),
            _ => unreachable!(),
        }
    }};
}

pub(super) fn init_pin(pin: &impl crate::gpio::RtcPin, input_enable: bool) -> u8 {
    pin.rtc_set_config(input_enable, true, crate::gpio::RtcFunction::Rtc);
    pin.rtc_number()
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    if enable {
        RTC_IO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << pin) });
    } else {
        RTC_IO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << pin) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    set_pad_field!(pin, fun_ie, enable);
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    set_pull_field!(pin, rue, enable);
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    set_pull_field!(pin, rde, enable);
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
