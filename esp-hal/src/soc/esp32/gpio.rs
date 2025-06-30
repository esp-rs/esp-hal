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
//!       * Handles the configuration of pull-up and pull-down resistors for
//!         specific GPIO pins
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line
//!         represents a pin and its associated options such as input/output
//!         mode, analog capability, and corresponding functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each
//!         line represents a pin and its associated options such as mux
//!         selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input
//!         signal is assigned a specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each
//!         output signal is assigned a specific value.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

include!(concat!(env!("OUT_DIR"), "/_generated_iomux_signals.rs"));

macro_rules! rtcio_analog {
    (
        $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:literal)?
    ) => {
        paste::paste! {
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPin for $crate::peripherals::[<GPIO $pin_num>]<'_> {
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

            $(
                // FIXME: replace with $(ignore($rue)) once stable
                $crate::ignore!($rue);

                #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                impl $crate::gpio::RtcPinWithResistors for $crate::peripherals::[<GPIO $pin_num>]<'_> {
                    fn rtcio_pullup(&self, enable: bool) {
                        $crate::peripherals::RTC_IO::regs()
                            .$pin_reg.modify(|_, w| w.[< $prefix rue >]().bit(enable));
                    }

                    fn rtcio_pulldown(&self, enable: bool) {
                        $crate::peripherals::RTC_IO::regs()
                            .$pin_reg.modify(|_, w| w.[< $prefix rde >]().bit(enable));
                    }
                }
            )?

            impl $crate::peripherals::[<GPIO $pin_num>]<'_> {
                /// Configures the pin for analog mode.
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

                        // Disable pull-up and pull-down resistors on the pin, if it has them
                        $(
                            // FIXME: replace with $(ignore($rue)) once stable
                            $crate::ignore!($rue);
                            w.[<$prefix rue>]().bit(false);
                            w.[<$prefix rde>]().bit(false);
                        )?

                        w
                    });
                }
            }
        }
    };

    (
        $( ( $pin_num:expr, $rtc_pin:expr, $pin_reg:expr, $prefix:pat, $hold:ident $(, $rue:literal )? ) )+
    ) => {
        $(
            rtcio_analog!($pin_num, $rtc_pin, $pin_reg, $prefix, $hold $(, $rue )?);
        )+

        pub(crate) fn errata36(pin: $crate::gpio::AnyPin<'_>, pull_up: bool, pull_down: bool) {
            use $crate::gpio::{Pin, RtcPinWithResistors};

            let has_pullups = match pin.number() {
                $(
                    $( $pin_num => $rue, )?
                )+
                _ => false,
            };

            if has_pullups {
                pin.rtcio_pullup(pull_up);
                pin.rtcio_pulldown(pull_down);
            }
        }
    };
}

/// Common functionality for all touch pads
macro_rules! touch {
    (@pin_specific $touch_num:expr, true) => {
        paste::paste! {
            RTC_IO::regs().[< touch_pad $touch_num >]().write(|w| unsafe {
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
            RTC_IO::regs().[< touch_pad $touch_num >]().write(|w| {
                w.xpd().set_bit();
                w.tie_opt().clear_bit()
            });
        }
    };

    (
        $(
            (
                $touch_num:literal, $pin_num:literal, $touch_out_reg:expr, $touch_thres_reg:expr, $normal_pin:literal
            )
        )+
    ) => {
        $(
        impl $crate::gpio::TouchPin for paste::paste!($crate::peripherals::[<GPIO $pin_num>]<'_>) {
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
                paste::paste! {
                    sens . $touch_thres_reg ()
                        .write(|w| unsafe {
                            w. [<touch_out_th $touch_num>] ().bits(
                                0b0 // Default: 0 for esp32 gets overridden later anyway.
                            )
                        });

                    touch!( @pin_specific $touch_num, $normal_pin );

                    // enable the pin
                    sens.sar_touch_enable().modify(|r, w| unsafe {
                        w.touch_pad_worken().bits(
                            r.touch_pad_worken().bits() | ( 1 << $touch_num )
                        )
                    });
                }
            }

            fn touch_measurement(&self, _: $crate::private::Internal) -> u16 {
                paste::paste! {
                    $crate::peripherals::SENS::regs() . $touch_out_reg ().read()
                        . [<touch_meas_out $touch_num>] ().bits()
                }
            }

            fn touch_nr(&self, _: $crate::private::Internal) -> u8 {
                $touch_num
            }

            fn set_threshold(&self, threshold: u16, _: $crate::private::Internal) {
                paste::paste! {
                    $crate::peripherals::SENS::regs() . $touch_thres_reg ()
                        .write(|w| unsafe {
                            w. [<touch_out_th $touch_num>] ().bits(threshold)
                        });
                }
            }
        })+
    };
}

rtcio_analog! {
    (36, 0,  sensor_pads(),    sense1_, sense1          )
    (37, 1,  sensor_pads(),    sense2_, sense2          )
    (38, 2,  sensor_pads(),    sense3_, sense3          )
    (39, 3,  sensor_pads(),    sense4_, sense4          )
    (34, 4,  adc_pad(),        adc1_,   adc1            )
    (35, 5,  adc_pad(),        adc2_,   adc2            )
    (25, 6,  pad_dac1(),       "",      pdac1,      true)
    (26, 7,  pad_dac2(),       "",      pdac2,      true)
    (33, 8,  xtal_32k_pad(),   x32n_,   x32n,       true)
    (32, 9,  xtal_32k_pad(),   x32p_,   x32p,       true)
    (4,  10, touch_pad0(),     "",      touch_pad0, true)
    (0,  11, touch_pad1(),     "",      touch_pad1, true)
    (2,  12, touch_pad2(),     "",      touch_pad2, true)
    (15, 13, touch_pad3(),     "",      touch_pad3, true)
    (13, 14, touch_pad4(),     "",      touch_pad4, true)
    (12, 15, touch_pad5(),     "",      touch_pad5, true)
    (14, 16, touch_pad6(),     "",      touch_pad6, true)
    (27, 17, touch_pad7(),     "",      touch_pad7, true)
}

touch! {
    // touch_nr, pin_nr, touch_out_reg, touch_thres_reg, normal_pin
    (0, 4,  sar_touch_out1, sar_touch_thres1, true)
    (1, 0,  sar_touch_out1, sar_touch_thres1, true)
    (2, 2,  sar_touch_out2, sar_touch_thres2, true)
    (3, 15, sar_touch_out2, sar_touch_thres2, true)
    (4, 13, sar_touch_out3, sar_touch_thres3, true)
    (5, 12, sar_touch_out3, sar_touch_thres3, true)
    (6, 14, sar_touch_out4, sar_touch_thres4, true)
    (7, 27, sar_touch_out4, sar_touch_thres4, true)
    // ---
    (8, 33, sar_touch_out5, sar_touch_thres5, false)
    (9, 32, sar_touch_out5, sar_touch_thres5, false)
}
