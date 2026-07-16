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
