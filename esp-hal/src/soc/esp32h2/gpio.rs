//! # GPIO configuration module (ESP32-H2)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-H2` chip. It allows you to
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

for_each_lp_function! {
    (($_rtc:ident, LP_GPIOn, $n:literal), $gpio:ident) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl $crate::gpio::RtcPin for $crate::peripherals::$gpio<'_> {
            fn rtc_number(&self) -> u8 {
                $n
            }

            fn rtcio_pad_hold(&self, enable: bool) {
                let mask = 1 << $n;
                unsafe {
                    let lp_aon = $crate::peripherals::LP_AON::regs();

                    lp_aon.gpio_hold0().modify(|r, w| {
                        if enable {
                            w.gpio_hold0().bits(r.gpio_hold0().bits() | mask)
                        } else {
                            w.gpio_hold0().bits(r.gpio_hold0().bits() & !mask)
                        }
                    });
                }
            }
        }
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl $crate::gpio::RtcPinWithResistors for $crate::peripherals::$gpio<'_> {}
    };
}
