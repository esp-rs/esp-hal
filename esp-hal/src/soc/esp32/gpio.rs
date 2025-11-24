//! # GPIO configuration module (ESP32)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `errata36(pin_num: u8, pull_up: bool, pull_down: bool)`:
//!       * Handles the configuration of pull-up and pull-down resistors for specific GPIO pins
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line represents a pin and its
//!         associated options such as input/output mode, analog capability, and corresponding
//!         functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each line represents a pin
//!         and its associated options such as mux selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input signal is assigned a
//!         specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each output signal is assigned
//!         a specific value.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

macro_rules! rtcio_analog {
    ($pin_peri:ident, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident) => {
        paste::paste! {
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPin for $crate::peripherals::$pin_peri<'_> {
                fn rtc_number(&self) -> u8 {
                    $rtc_pin
                }

                /// Set the RTC properties of the pin. If `mux` is true then then pin is
                /// routed to RTC, when false it is routed to IO_MUX.
                fn rtc_set_config(&self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    // disable input
                    $crate::peripherals::RTC_IO::regs()
                        .$pin_reg.modify(|_,w| unsafe {
                            w.[<$prefix fun_ie>]().bit(input_enable);
                            w.[<$prefix mux_sel>]().bit(mux);
                            w.[<$prefix fun_sel>]().bits(func as u8)
                        });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    $crate::peripherals::LPWR::regs()
                        .hold_force()
                        .modify(|_, w| w.$hold().bit(enable));
                }
            }

            for_each_gpio! {
                // Implement RtcPinWithResistors if $pin_peri is an output pin
                ($n:tt, $pin_peri $in_afs:tt $out_afs:tt ($input:tt [Output])) => {
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    impl $crate::gpio::RtcPinWithResistors for $crate::peripherals::$pin_peri<'_> {
                        fn rtcio_pullup(&self, enable: bool) {
                            $crate::peripherals::RTC_IO::regs()
                                .$pin_reg.modify(|_, w| w.[< $prefix rue >]().bit(enable));
                        }

                        fn rtcio_pulldown(&self, enable: bool) {
                            $crate::peripherals::RTC_IO::regs()
                                .$pin_reg.modify(|_, w| w.[< $prefix rde >]().bit(enable));
                        }
                    }
                };
            }

            impl $crate::peripherals::$pin_peri<'_> {
                /// Configures the pin for analog mode.
                #[cfg(feature = "unstable")]
                pub(crate) fn set_analog_impl(&self) {
                    use $crate::gpio::RtcPin;
                    let rtcio = $crate::peripherals::RTC_IO::regs();

                    // disable output
                    rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                    // disable open drain
                    rtcio.pin(self.rtc_number() as usize).modify(|_,w| w.pad_driver().bit(false));

                    rtcio.$pin_reg.modify(|_,w| {
                        w.[<$prefix fun_ie>]().clear_bit();

                        // Connect pin to analog / RTC module instead of standard GPIO
                        w.[<$prefix mux_sel>]().set_bit();

                        // Select function "RTC function 1" (GPIO) for analog use
                        unsafe { w.[<$prefix fun_sel>]().bits(0b00) };

                        for_each_gpio! {
                            // Disable pull-up and pull-down resistors on the pin, if it has them
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
        $( ( $pin_peri:ident, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:literal )? ) )+
    ) => {
        $(
            rtcio_analog!($pin_peri, $rtc_pin, $pin_reg, $prefix, $hold $(, $rue )?);
        )+
    };
}

#[rustfmt::skip]
macro_rules! touch_out_reg {
    ($regs:expr, 0) => { $regs.sar_touch_out1() };
    ($regs:expr, 1) => { $regs.sar_touch_out1() };
    ($regs:expr, 2) => { $regs.sar_touch_out2() };
    ($regs:expr, 3) => { $regs.sar_touch_out2() };
    ($regs:expr, 4) => { $regs.sar_touch_out3() };
    ($regs:expr, 5) => { $regs.sar_touch_out3() };
    ($regs:expr, 6) => { $regs.sar_touch_out4() };
    ($regs:expr, 7) => { $regs.sar_touch_out4() };
    ($regs:expr, 8) => { $regs.sar_touch_out5() };
    ($regs:expr, 9) => { $regs.sar_touch_out5() };
}

#[rustfmt::skip]
macro_rules! touch_thres_reg {
    ($regs:expr, 0) => { $regs.sar_touch_thres1() };
    ($regs:expr, 1) => { $regs.sar_touch_thres1() };
    ($regs:expr, 2) => { $regs.sar_touch_thres2() };
    ($regs:expr, 3) => { $regs.sar_touch_thres2() };
    ($regs:expr, 4) => { $regs.sar_touch_thres3() };
    ($regs:expr, 5) => { $regs.sar_touch_thres3() };
    ($regs:expr, 6) => { $regs.sar_touch_thres4() };
    ($regs:expr, 7) => { $regs.sar_touch_thres4() };
    ($regs:expr, 8) => { $regs.sar_touch_thres5() };
    ($regs:expr, 9) => { $regs.sar_touch_thres5() };
}

#[rustfmt::skip]
macro_rules! touch_pad_reg {
    ($regs:expr, 0) => { $regs.touch_pad0() };
    ($regs:expr, 1) => { $regs.touch_pad1() };
    ($regs:expr, 2) => { $regs.touch_pad2() };
    ($regs:expr, 3) => { $regs.touch_pad3() };
    ($regs:expr, 4) => { $regs.touch_pad4() };
    ($regs:expr, 5) => { $regs.touch_pad5() };
    ($regs:expr, 6) => { $regs.touch_pad6() };
    ($regs:expr, 7) => { $regs.touch_pad7() };
    ($regs:expr, 8) => { $regs.touch_pad8() };
    ($regs:expr, 9) => { $regs.touch_pad9() };
}

#[rustfmt::skip]
macro_rules! touch_out_th_field {
    ($accessor:expr, 0) => { $accessor.touch_out_th0() };
    ($accessor:expr, 1) => { $accessor.touch_out_th1() };
    ($accessor:expr, 2) => { $accessor.touch_out_th0() };
    ($accessor:expr, 3) => { $accessor.touch_out_th1() };
    ($accessor:expr, 4) => { $accessor.touch_out_th0() };
    ($accessor:expr, 5) => { $accessor.touch_out_th1() };
    ($accessor:expr, 6) => { $accessor.touch_out_th0() };
    ($accessor:expr, 7) => { $accessor.touch_out_th1() };
    ($accessor:expr, 8) => { $accessor.touch_out_th0() };
    ($accessor:expr, 9) => { $accessor.touch_out_th1() };
}

#[rustfmt::skip]
macro_rules! touch_meas_out_field {
    ($accessor:expr, 0) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 1) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 2) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 3) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 4) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 5) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 6) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 7) => { $accessor.touch_meas_out1() };
    ($accessor:expr, 8) => { $accessor.touch_meas_out0() };
    ($accessor:expr, 9) => { $accessor.touch_meas_out1() };
}

/// Common functionality for all touch pads
macro_rules! touch {
    (
        $(
            ($touch_num:tt, $pin_peri:ident $(, $pad_register_has_all_fields:literal)?)
        )+
    ) => {
        $(
        impl $crate::gpio::TouchPin for $crate::peripherals::$pin_peri<'_> {
            fn set_touch(&self, _: $crate::private::Internal) {
                use $crate::peripherals::{GPIO, RTC_IO, SENS};
                use $crate::gpio::RtcPin;

                let gpio = GPIO::regs();
                let rtcio = RTC_IO::regs();
                let sens = SENS::regs();

                // Pad to normal mode (not open-drain)
                gpio.pin(self.rtc_number() as usize).write(|w| w.pad_driver().clear_bit());

                // clear output
                rtcio
                    .enable_w1tc()
                    .write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                touch_thres_reg!(sens, $touch_num).write(|w| unsafe {
                    touch_out_th_field!(w, $touch_num).bits(
                        0b0 // Default: 0 for esp32 gets overridden later anyway.
                    )
                });

                touch_pad_reg!(RTC_IO::regs(), $touch_num).write(
                    #[allow(unused_unsafe)]
                    |w| unsafe {
                        w.xpd().set_bit();
                        w.tie_opt().clear_bit();

                        // touch_pad8 and 9 are missing a few fields
                        $(
                            crate::ignore!($pad_register_has_all_fields);

                            // clear input_enable
                            w.fun_ie().clear_bit();
                            // Connect pin to analog / RTC module instead of standard GPIO
                            w.mux_sel().set_bit();
                            // Disable pull-up and pull-down resistors on the pin
                            w.rue().clear_bit();
                            w.rde().clear_bit();
                            // Select function "RTC function 1" (GPIO) for analog use
                            w.fun_sel().bits(0b00);
                        )?

                        w
                    }
                );

                // enable the pin
                sens.sar_touch_enable().modify(|r, w| unsafe {
                    w.touch_pad_worken().bits(
                        r.touch_pad_worken().bits() | ( 1 << $touch_num )
                    )
                });
            }

            fn touch_measurement(&self, _: $crate::private::Internal) -> u16 {
                let regs = $crate::peripherals::SENS::regs();
                let reg = touch_out_reg!(regs, $touch_num).read();
                touch_meas_out_field!(reg, $touch_num).bits()
            }

            fn touch_nr(&self, _: $crate::private::Internal) -> u8 {
                $touch_num
            }

            fn set_threshold(&self, threshold: u16, _: $crate::private::Internal) {
                let sens = $crate::peripherals::SENS::regs();
                touch_thres_reg!(sens, $touch_num).write(|w| unsafe {
                    touch_out_th_field!(w, $touch_num).bits(threshold)
                });
            }
        })+
    };
}

pub(crate) fn errata36(pin: crate::gpio::AnyPin<'_>, pull_up: bool, pull_down: bool) {
    use crate::gpio::{Pin, RtcPinWithResistors};

    for_each_lp_function! {
        (all_expanded $( (($_sig:ident, RTC_GPIOn, $_n:literal), $gpio:ident) ),* ) => {
            const RTC_IO_PINS: &[u8] = &[ $( $crate::peripherals::$gpio::NUMBER ),* ];
        };
    };

    if RTC_IO_PINS.contains(&pin.number()) && pin.is_output() {
        pin.rtcio_pullup(pull_up);
        pin.rtcio_pulldown(pull_down);
    }
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

touch! {
    // touch_nr, pin_nr, normal_pin
    (0, GPIO4,  true)
    (1, GPIO0,  true)
    (2, GPIO2,  true)
    (3, GPIO15, true)
    (4, GPIO13, true)
    (5, GPIO12, true)
    (6, GPIO14, true)
    (7, GPIO27, true)
    // --- touch_pad8 and 9 are missing a few fields
    (8, GPIO33)
    (9, GPIO32)
}
