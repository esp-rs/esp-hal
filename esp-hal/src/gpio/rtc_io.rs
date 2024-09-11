//! RTC IO
//!
//! # Overview
//!
//! The hardware provides a couple of GPIO pins with low power (LP)
//! capabilities and analog functions.
//!
//! ## Configuration
//!
//! These pins can be controlled by either IO MUX or RTC IO.
//!
//! If controlled by RTC IO, these pins will bypass IO MUX and GPIO
//! matrix for the use by ULP and peripherals in RTC system.
//!
//! When configured as RTC GPIOs, the pins can still be controlled by ULP or
//! the peripherals in RTC system during chip Deep-sleep, and wake up the
//! chip from Deep-sleep.
//!
//! ## Example
//!
//! ### Configure a ULP Pin as Output
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::rtc_io::LowPowerOutput;
//! let io = peripherals.GPIO.pins();
//! // configure GPIO 1 as ULP output pin
//! let lp_pin = LowPowerOutput::<'static, 1>::new(io.gpio1);
//! # }
//! ```

use core::marker::PhantomData;

#[cfg(esp32c6)]
use super::OpenDrain;
use super::RtcPin;
use crate::into_ref;

/// A GPIO output pin configured for low power operation
pub struct LowPowerOutput<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    pub fn new<P>(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
    where
        P: super::OutputPin + RtcPin,
    {
        into_ref!(pin);
        pin.rtc_set_config(false, true, crate::gpio::RtcFunction::Rtc);

        let this = Self {
            phantom: PhantomData,
        };
        this.output_enable(true);

        this
    }

    fn output_enable(&self, enable: bool) {
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
}

/// A GPIO input pin configured for low power operation
pub struct LowPowerInput<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerInput<'d, PIN> {
    /// Create a new input pin for use by the low-power core
    pub fn new<P>(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
    where
        P: super::InputPin + RtcPin,
    {
        into_ref!(pin);
        pin.rtc_set_config(true, true, crate::gpio::RtcFunction::Rtc);

        let this = Self {
            phantom: PhantomData,
        };
        this.input_enable(true);
        this.pullup_enable(false);
        this.pulldown_enable(false);

        this
    }

    fn input_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.fun_ie().bit(enable));
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rue().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rde().bit(enable));
    }
}

/// A GPIO open-drain output pin configured for low power operation
pub struct LowPowerOutputOpenDrain<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerOutputOpenDrain<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    pub fn new<P>(pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
    where
        P: super::InputPin + super::OutputPin + RtcPin,
    {
        into_ref!(pin);
        pin.rtc_set_config(true, true, crate::gpio::RtcFunction::Rtc);

        let this = Self {
            phantom: PhantomData,
        };

        this.set_open_drain_output(true);
        this.input_enable(true);
        this.pullup_enable(true);
        this.pulldown_enable(false);
        this.output_enable(true);

        this
    }

    fn output_enable(&self, enable: bool) {
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

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rue().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.rde().bit(enable));
    }

    fn set_open_drain_output(&self, enable: bool) {
        use crate::peripherals::GPIO;
        let gpio = unsafe { &*GPIO::PTR };

        gpio.pin(PIN as usize)
            .modify(|_, w| w.pad_driver().bit(enable));
    }
}

#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
fn get_pin_reg(pin: u8) -> &'static crate::peripherals::rtc_io::TOUCH_PAD {
    unsafe {
        let rtc_io = &*crate::peripherals::RTC_IO::PTR;
        let pin_ptr = (rtc_io.touch_pad(0).as_ptr()).add(pin as usize);

        &*(pin_ptr as *const crate::peripherals::rtc_io::TOUCH_PAD)
    }
}
