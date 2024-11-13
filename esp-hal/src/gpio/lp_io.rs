//! Low Power IO (LP_IO)
//!
//! # Overview
//! The hardware provides a couple of GPIO pins with low power (LP)
//! capabilities and analog functions.
//!
//! ## Configuration
//! These pins can be controlled by either IO MUX or LP IO MUX.
//!
//! If controlled by LP IO MUX, these pins will bypass IO MUX and GPIO
//! matrix for the use by ULP and peripherals in LP system.
//!
//! When configured as LP GPIOs, the pins can still be controlled by ULP or
//! the peripherals in LP system during chip Deep-sleep, and wake up the
//! chip from Deep-sleep.
//!
//! # Example
//!
//! ## Configure a LP Pin as Output
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::gpio::lp_io::LowPowerOutput;
//! // configure GPIO 1 as LP output pin
//! let lp_pin: LowPowerOutput<'_, 1> =
//!     LowPowerOutput::new(peripherals.GPIO1);
//! # }
//! ```

use core::marker::PhantomData;

use super::{InputPin, OutputPin, RtcPin};
use crate::{
    peripheral::Peripheral,
    peripherals::{GPIO, LP_AON, LP_IO},
};

/// A GPIO output pin configured for low power operation
pub struct LowPowerOutput<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    pub fn new<P>(_pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: OutputPin + RtcPin,
    {
        init_low_power_pin(PIN);

        let this = Self {
            phantom: PhantomData,
        };
        this.output_enable(true);

        this
    }

    fn output_enable(&self, enable: bool) {
        let lp_io = unsafe { LP_IO::steal() };
        if enable {
            lp_io
                .out_enable_w1ts()
                .write(|w| unsafe { w.enable_w1ts().bits(1 << PIN) });
        } else {
            lp_io
                .out_enable_w1tc()
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
    pub fn new<P>(_pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: InputPin + RtcPin,
    {
        init_low_power_pin(PIN);

        let this = Self {
            phantom: PhantomData,
        };
        this.input_enable(true);
        this.pullup_enable(false);
        this.pulldown_enable(false);

        this
    }

    fn input_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_ie().bit(enable));
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpu().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpd().bit(enable));
    }
}

/// A GPIO open-drain output pin configured for low power operation
pub struct LowPowerOutputOpenDrain<'d, const PIN: u8> {
    phantom: PhantomData<&'d ()>,
}

impl<'d, const PIN: u8> LowPowerOutputOpenDrain<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    pub fn new<P>(_pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: InputPin + OutputPin + RtcPin,
    {
        init_low_power_pin(PIN);

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
        let lp_io = unsafe { LP_IO::steal() };
        if enable {
            lp_io
                .out_enable_w1ts()
                .write(|w| unsafe { w.enable_w1ts().bits(1 << PIN) });
        } else {
            lp_io
                .out_enable_w1tc()
                .write(|w| unsafe { w.enable_w1tc().bits(1 << PIN) });
        }
    }

    fn input_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_ie().bit(enable));
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpu().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        unsafe { LP_IO::steal() }
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpd().bit(enable));
    }

    fn set_open_drain_output(&self, enable: bool) {
        unsafe { GPIO::steal() }
            .pin(PIN as usize)
            .modify(|_, w| w.pad_driver().bit(enable));
    }
}

pub(crate) fn init_low_power_pin(pin: u8) {
    unsafe { LP_AON::steal() }
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << pin) });

    unsafe { LP_IO::steal() }
        .gpio(pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(0) });
}

#[doc(hidden)]
#[macro_export]
macro_rules! lp_gpio {
    (
        $($gpionum:literal)+
    ) => {
        $(
            impl $crate::gpio::RtcPin for GpioPin<$gpionum> {
                unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
                    let lp_io = $crate::peripherals::LP_IO::steal();
                    lp_io.pin($gpionum).modify(|_, w| {
                        w.wakeup_enable().bit(wakeup).int_type().bits(level)
                    });
                }

                fn rtcio_pad_hold(&mut self, enable: bool) {
                    let mask = 1 << $gpionum;
                    unsafe {
                        let lp_aon = $crate::peripherals::LP_AON::steal();

                        lp_aon.gpio_hold0().modify(|r, w| {
                            if enable {
                                w.gpio_hold0().bits(r.gpio_hold0().bits() | mask)
                            } else {
                                w.gpio_hold0().bits(r.gpio_hold0().bits() & !mask)
                            }
                        });
                    }
                }

                /// Set the LP properties of the pin. If `mux` is true then then pin is
                /// routed to LP_IO, when false it is routed to IO_MUX.
                fn rtc_set_config(&mut self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    let mask = 1 << $gpionum;
                    unsafe {
                        // Select LP_IO
                        let lp_aon = $crate::peripherals::LP_AON::steal();
                        lp_aon
                            .gpio_mux()
                            .modify(|r, w| {
                                if mux {
                                    w.sel().bits(r.sel().bits() | mask)
                                } else {
                                    w.sel().bits(r.sel().bits() & !mask)
                                }
                            });

                        // Configure input, function and select normal operation registers
                        let lp_io = $crate::peripherals::LP_IO::steal();
                        lp_io.gpio($gpionum).modify(|_, w| {
                            w.slp_sel().bit(false);
                            w.fun_ie().bit(input_enable);
                            w.mcu_sel().bits(func as u8)
                        });
                    }
                }
            }

            impl $crate::gpio::RtcPinWithResistors for GpioPin<$gpionum> {
                fn rtcio_pullup(&mut self, enable: bool) {
                    let lp_io = unsafe { $crate::peripherals::LP_IO::steal() };
                    lp_io.gpio($gpionum).modify(|_, w| w.fun_wpu().bit(enable));
                }

                fn rtcio_pulldown(&mut self, enable: bool) {
                    let lp_io = unsafe { $crate::peripherals::LP_IO::steal() };
                    lp_io.gpio($gpionum).modify(|_, w| w.fun_wpd().bit(enable));
                }
            }
        )+
    }
}
