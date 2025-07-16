#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! ## Examples
//!
//! ## Configure a LP Pin as Output
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::gpio::lp_io::LowPowerOutput;
//! // configure GPIO 1 as LP output pin
//! let lp_pin: LowPowerOutput<'_, 1> = LowPowerOutput::new(peripherals.GPIO1);
//! # {after_snippet}
//! ```

use core::marker::PhantomData;

use super::{InputPin, OutputPin, RtcPin};
use crate::peripherals::{GPIO, LP_AON, LP_IO};

/// A GPIO output pin configured for low power operation
pub struct LowPowerOutput<'d, const PIN: u8> {
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    #[instability::unstable]
    pub fn new<P>(_pin: P) -> Self
    where
        P: OutputPin + RtcPin + 'd,
    {
        init_low_power_pin(PIN);

        let this = Self {
            phantom: PhantomData,
        };
        this.output_enable(true);

        this
    }

    fn output_enable(&self, enable: bool) {
        let lp_io = LP_IO::regs();
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
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerInput<'d, PIN> {
    /// Create a new input pin for use by the low-power core
    #[instability::unstable]
    pub fn new<P>(_pin: P) -> Self
    where
        P: InputPin + RtcPin + 'd,
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
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_ie().bit(enable));
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpu().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpd().bit(enable));
    }
}

/// A GPIO open-drain output pin configured for low power operation
pub struct LowPowerOutputOpenDrain<'d, const PIN: u8> {
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerOutputOpenDrain<'d, PIN> {
    /// Create a new output pin for use by the low-power core
    #[instability::unstable]
    pub fn new<P>(_pin: P) -> Self
    where
        P: InputPin + OutputPin + RtcPin + 'd,
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
        let lp_io = LP_IO::regs();
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
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_ie().bit(enable));
    }

    /// Sets pull-up enable for the pin
    pub fn pullup_enable(&self, enable: bool) {
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpu().bit(enable));
    }

    /// Sets pull-down enable for the pin
    pub fn pulldown_enable(&self, enable: bool) {
        LP_IO::regs()
            .gpio(PIN as usize)
            .modify(|_, w| w.fun_wpd().bit(enable));
    }

    fn set_open_drain_output(&self, enable: bool) {
        GPIO::regs()
            .pin(PIN as usize)
            .modify(|_, w| w.pad_driver().bit(enable));
    }
}

pub(crate) fn init_low_power_pin(pin: u8) {
    LP_AON::regs()
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | (1 << pin)) });

    LP_IO::regs()
        .gpio(pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(0) });
}

#[doc(hidden)]
macro_rules! lp_gpio {
    (
        $($gpionum:literal)+
    ) => {
        $(
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPin for paste::paste!($crate::peripherals::[<GPIO $gpionum>]<'_>) {
                unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                    let lp_io = $crate::peripherals::LP_IO::regs();
                    lp_io.pin($gpionum).modify(|_, w| {
                        unsafe {
                            w.wakeup_enable().bit(wakeup).int_type().bits(level)
                        }
                    });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    let mask = 1 << $gpionum;
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

                /// Set the LP properties of the pin. If `mux` is true then then pin is
                /// routed to LP_IO, when false it is routed to IO_MUX.
                fn rtc_set_config(&self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    let mask = 1 << $gpionum;
                    unsafe {
                        let lp_aon = $crate::peripherals::LP_AON::regs();
                        // Select LP_IO
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
                        let lp_io = $crate::peripherals::LP_IO::regs();
                        lp_io.gpio($gpionum).modify(|_, w| {
                            w.slp_sel().bit(false);
                            w.fun_ie().bit(input_enable);
                            w.mcu_sel().bits(func as u8)
                        });
                    }
                }
            }

            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl $crate::gpio::RtcPinWithResistors for paste::paste!($crate::peripherals::[<GPIO $gpionum>]<'_>) {
                fn rtcio_pullup(&self, enable: bool) {
                    let lp_io = $crate::peripherals::LP_IO::regs();
                    lp_io.gpio($gpionum).modify(|_, w| w.fun_wpu().bit(enable));
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    let lp_io = $crate::peripherals::LP_IO::regs();
                    lp_io.gpio($gpionum).modify(|_, w| w.fun_wpd().bit(enable));
                }
            }
        )+
    }
}

pub(crate) use lp_gpio;
