#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! ## Examples
//!
//! ### Configure a ULP Pin as Output
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::gpio::rtc_io::LowPowerOutput;
//! // configure GPIO 1 as ULP output pin
//! let lp_pin = LowPowerOutput::<'static, 1>::new(peripherals.GPIO1);
//! # {after_snippet}
//! ```

use core::marker::PhantomData;

use super::{InputPin, OutputPin, RtcPin};
use crate::{
    gpio::{Pin, RtcFunction, RtcPinWithResistors},
    peripherals::{GPIO, RTC_IO},
};

/// A GPIO output pin configured for low power operation
pub struct LowPowerOutput<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    pub fn new<P>(pin: P) -> Self
    where
        P: OutputPin + RtcPin + 'd,
    {
        pin.rtc_set_config(false, true, RtcFunction::Rtc);

        let this = Self {
            phantom: PhantomData,
        };
        this.output_enable(true);

        this
    }

    fn output_enable(&self, enable: bool) {
        let rtc_io = RTC_IO::regs();

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

/// A GPIO input pin configured for low power operation.
pub struct LowPowerInput<'d, P> {
    pin: P,
    phantom: PhantomData<&'d ()>,
}

impl<'d, P> LowPowerInput<'d, P>
where
    P: InputPin + RtcPinWithResistors + 'd,
{
    /// Create a new input pin for use by the low-power core.
    pub fn new(pin: P) -> Self {
        pin.rtc_set_config(true, true, RtcFunction::Rtc);

        pin.rtcio_pullup(false);
        pin.rtcio_pulldown(false);

        Self {
            pin,
            phantom: PhantomData,
        }
    }

    /// Sets pull-up enable for the pin.
    pub fn pullup_enable(&mut self, enable: bool) {
        self.pin.rtcio_pullup(enable);
    }

    /// Sets pull-down enable for the pin.
    pub fn pulldown_enable(&mut self, enable: bool) {
        self.pin.rtcio_pulldown(enable);
    }
}

/// A GPIO open-drain output pin configured for low power operation.
pub struct LowPowerOutputOpenDrain<'d, P> {
    pin: P,
    phantom: PhantomData<&'d ()>,
}

impl<'d, P> LowPowerOutputOpenDrain<'d, P>
where
    P: InputPin + OutputPin + RtcPinWithResistors + Pin + 'd,
{
    /// Create a new output pin for use by the low-power core.
    pub fn new(pin: P) -> Self {
        // We can now call trait methods directly on the pin.
        pin.rtc_set_config(true, true, RtcFunction::Rtc);

        let mut this = Self {
            pin,
            phantom: PhantomData,
        };

        this.set_open_drain_output(true);
        this.input_enable(true);
        this.pullup_enable(true);
        this.pulldown_enable(false);
        this.output_enable(true);

        this
    }

    fn output_enable(&mut self, enable: bool) {
        let rtc_io = RTC_IO::regs();
        let rtc_pin = self.pin.rtc_number();

        if enable {
            rtc_io
                .rtc_gpio_enable_w1ts()
                .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << rtc_pin) });
        } else {
            rtc_io
                .enable_w1tc()
                .write(|w| unsafe { w.enable_w1tc().bits(1 << rtc_pin) });
        }
    }

    fn input_enable(&mut self, enable: bool) {
        self.pin.rtc_set_config(enable, true, RtcFunction::Rtc);
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&mut self, enable: bool) {
        self.pin.rtcio_pullup(enable);
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&mut self, enable: bool) {
        self.pin.rtcio_pulldown(enable);
    }

    fn set_open_drain_output(&mut self, enable: bool) {
        let pin_number = self.pin.number();
        GPIO::regs()
            .pin(pin_number as usize)
            .modify(|_, w| w.pad_driver().bit(enable));
    }
}
