//! # GPIO configuration module (ESP32-C2)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-C2` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
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

macro_rules! rtc_pins {
    ( $( $pin_num:expr )+ ) => {
        $(
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPin for paste::paste!($crate::peripherals::[<GPIO $pin_num>]<'_>) {
                unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                    let gpio_wakeup = $crate::peripherals::LPWR::regs().cntl_gpio_wakeup();
                    unsafe {
                        paste::paste! {
                            gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _wakeup_enable >]().bit(wakeup));
                            gpio_wakeup.modify(|_, w| w.[< gpio_pin $pin_num _int_type >]().bits(level));
                        }
                    }
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    paste::paste! {
                        $crate::peripherals::LPWR::regs()
                            .pad_hold().modify(|_, w| w.[< gpio_pin $pin_num _hold >]().bit(enable));
                    }
                }
            }

            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl crate::gpio::RtcPinWithResistors for paste::paste!($crate::peripherals::[<GPIO $pin_num>]<'_>) {
                fn rtcio_pullup(&self, enable: bool) {
                    $crate::peripherals::IO_MUX::regs()
                        .gpio($pin_num)
                        .modify(|_, w| w.fun_wpu().bit(enable));
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    $crate::peripherals::IO_MUX::regs()
                        .gpio($pin_num)
                        .modify(|_, w| w.fun_wpd().bit(enable));
                }
            }
        )+
    };
}

rtc_pins! {
    0
    1
    2
    3
    4
    5
}
