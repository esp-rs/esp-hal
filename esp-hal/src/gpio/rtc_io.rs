//! RTC IO
//!
//! # Overview
//!
//! The hardware provides a couple of GPIO pins with low power (LP)
//! capabilities and analog functions. These pins can be controlled by
//! either IO MUX or RTC IO.
//!
//! If controlled by RTC IO, these pins will bypass IO MUX and GPIO
//! matrix for the use by ULP and peripherals in RTC system.
//!
//! When configured as RTC GPIOs, the pins can still be controlled by ULP or
//! the peripherals in RTC system during chip Deep-sleep, and wake up the
//! chip from Deep-sleep.
//!
//! # Example
//! ```no_run
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! // configure GPIO 1 as ULP output pin
//! let lp_pin = io.pins.gpio1.into_low_power().into_push_pull_output();
//! ```

use core::marker::PhantomData;

#[cfg(esp32c6)]
use super::OpenDrain;
use super::{Input, Output, Unknown};

/// A GPIO pin configured for low power operation
pub struct LowPowerPin<MODE, const PIN: u8> {
    pub(crate) private: PhantomData<MODE>,
}

/// Configures a pin for use as a low power pin
pub trait IntoLowPowerPin<const PIN: u8> {
    /// Convert into low power pin
    fn into_low_power(self) -> LowPowerPin<Unknown, { PIN }>;
}

impl<MODE, const PIN: u8> LowPowerPin<MODE, PIN> {
    #[doc(hidden)]
    pub fn output_enable(&self, enable: bool) {
        let rtc_io = unsafe { crate::peripherals::RTC_IO::steal() };

        if enable {
            rtc_io
                .rtc_gpio_enable_w1ts()
                .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << PIN) });
        } else {
            rtc_io
                .enable_w1tc()
                .write(|w| unsafe { w.enable_w1tc().bits(1 << PIN) });
        }
    }

    fn input_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.fun_ie().bit(enable));
    }

    fn pullup_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rue().bit(enable));
    }

    fn pulldown_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rde().bit(enable));
    }

    #[doc(hidden)]
    pub fn set_level(&mut self, level: bool) {
        let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };

        if level {
            rtc_io
                .rtc_gpio_out_w1ts()
                .write(|w| unsafe { w.rtc_gpio_out_data_w1ts().bits(1 << PIN) });
        } else {
            rtc_io
                .rtc_gpio_out_w1tc()
                .write(|w| unsafe { w.rtc_gpio_out_data_w1tc().bits(1 << PIN) });
        }
    }

    #[doc(hidden)]
    pub fn get_level(&self) -> bool {
        let rtc_io = unsafe { &*crate::peripherals::RTC_IO::PTR };
        (rtc_io.rtc_gpio_in().read().bits() & 1 << PIN) != 0
    }

    /// Configures the pin as an input with the internal pull-up resistor
    /// enabled.
    pub fn into_pull_up_input(self) -> LowPowerPin<Input, PIN> {
        self.input_enable(true);
        self.pullup_enable(true);
        self.pulldown_enable(false);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as an input with the internal pull-down resistor
    /// enabled.
    pub fn into_pull_down_input(self) -> LowPowerPin<Input, PIN> {
        self.input_enable(true);
        self.pullup_enable(false);
        self.pulldown_enable(true);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as a floating input pin.
    pub fn into_floating_input(self) -> LowPowerPin<Input, PIN> {
        self.input_enable(true);
        self.pullup_enable(false);
        self.pulldown_enable(false);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as an output pin.
    pub fn into_push_pull_output(self) -> LowPowerPin<Output, PIN> {
        self.output_enable(true);
        LowPowerPin {
            private: PhantomData,
        }
    }

    #[cfg(esp32c6)]
    /// Configures the pin as an pullup input and a push pull output pin.
    pub fn into_open_drain_output(self) -> LowPowerPin<OpenDrain, PIN> {
        self.into_pull_up_input();
        self.into_push_pull_output();
        use crate::peripherals::GPIO;

        let gpio = unsafe { &*GPIO::PTR };

        gpio.pin(PIN).modify(|_, w| w.pad_driver().bit(true));
        self.pulldown_enable(false);

        LowPowerPin {
            private: PhantomData,
        }
    }
}

#[cfg(esp32s3)]
#[inline(always)]
fn get_pin_reg(pin: u8) -> &'static crate::peripherals::rtc_io::TOUCH_PAD0 {
    unsafe {
        let rtc_io = &*crate::peripherals::RTC_IO::PTR;
        let pin_ptr = (rtc_io.touch_pad0().as_ptr()).add(pin as usize);

        &*(pin_ptr as *const esp32s3::generic::Reg<esp32s3::rtc_io::touch_pad0::TOUCH_PAD0_SPEC>)
    }
}

#[cfg(esp32s2)]
#[inline(always)]
fn get_pin_reg(pin: u8) -> &'static crate::peripherals::rtc_io::TOUCH_PAD {
    unsafe {
        let rtc_io = &*crate::peripherals::RTC_IO::PTR;
        let pin_ptr = (rtc_io.touch_pad(0).as_ptr()).add(pin as usize);

        &*(pin_ptr as *const esp32s2::generic::Reg<esp32s2::rtc_io::touch_pad::TOUCH_PAD_SPEC>)
    }
}
