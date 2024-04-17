//! Low Power IO (LP_IO)
//!
//! # Overview
//!
//! The hardware provides a couple of GPIO pins with low power (LP)
//! capabilities and analog functions. These pins can be controlled by
//! either IO MUX or LP IO MUX.
//!
//! If controlled by LP IO MUX, these pins will bypass IO MUX and GPIO
//! matrix for the use by ULP and peripherals in LP system.
//!
//! When configured as LP GPIOs, the pins can still be controlled by ULP or
//! the peripherals in LP system during chip Deep-sleep, and wake up the
//! chip from Deep-sleep.
//!
//! # Example
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! // configure GPIO 1 as LP output pin
//! let lp_pin = io.pins.gpio1.into_low_power().into_push_pull_output();
//! ```

use core::marker::PhantomData;

#[cfg(esp32c6)]
use super::OpenDrain;
use super::{Floating, Input, Output, PullDown, PullUp, PushPull, Unknown};

/// A GPIO pin configured for low power operation
pub struct LowPowerPin<MODE, const PIN: u8> {
    pub(crate) private: PhantomData<MODE>,
}

impl<MODE, const PIN: u8> LowPowerPin<MODE, PIN> {
    #[doc(hidden)]
    pub fn output_enable(&self, enable: bool) {
        let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
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
        get_pin_reg(PIN).modify(|_, w| w.fun_ie().bit(enable));
    }

    fn pullup_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.fun_wpu().bit(enable));
    }

    fn pulldown_enable(&self, enable: bool) {
        get_pin_reg(PIN).modify(|_, w| w.fun_wpd().bit(enable));
    }

    #[doc(hidden)]
    pub fn set_level(&mut self, level: bool) {
        let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
        if level {
            lp_io
                .out_data_w1ts()
                .write(|w| unsafe { w.out_data_w1ts().bits(1 << PIN) });
        } else {
            lp_io
                .out_data_w1tc()
                .write(|w| unsafe { w.out_data_w1tc().bits(1 << PIN) });
        }
    }

    #[doc(hidden)]
    pub fn get_level(&self) -> bool {
        let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
        (lp_io.in_().read().data_next().bits() & 1 << PIN) != 0
    }

    /// Configures the pin as an input with the internal pull-up resistor
    /// enabled.
    pub fn into_pull_up_input(self) -> LowPowerPin<Input<PullUp>, PIN> {
        self.input_enable(true);
        self.pullup_enable(true);
        self.pulldown_enable(false);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as an input with the internal pull-down resistor
    /// enabled.
    pub fn into_pull_down_input(self) -> LowPowerPin<Input<PullDown>, PIN> {
        self.input_enable(true);
        self.pullup_enable(false);
        self.pulldown_enable(true);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as a floating input pin.
    pub fn into_floating_input(self) -> LowPowerPin<Input<Floating>, PIN> {
        self.input_enable(true);
        self.pullup_enable(false);
        self.pulldown_enable(false);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as a push-pull output pin.
    pub fn into_push_pull_output(self) -> LowPowerPin<Output<PushPull>, PIN> {
        self.output_enable(true);
        LowPowerPin {
            private: PhantomData,
        }
    }

    /// Configures the pin as an open-drain output pin.
    pub fn into_open_drain_output(self) -> LowPowerPin<OpenDrain, PIN> {
        use crate::peripherals::GPIO;

        let gpio = unsafe { &*GPIO::PTR };

        gpio.pin(PIN as usize)
            .modify(|_, w| w.pad_driver().bit(true));
        self.pulldown_enable(false);
        self.into_pull_up_input().into_push_pull_output();

        LowPowerPin {
            private: PhantomData,
        }
    }
}

pub(crate) fn init_low_power_pin(pin: u8) {
    let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };

    lp_aon
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << pin) });

    get_pin_reg(pin).modify(|_, w| unsafe { w.mcu_sel().bits(0) });
}

#[inline(always)]
fn get_pin_reg(pin: u8) -> &'static crate::peripherals::lp_io::GPIO0 {
    // ideally we should change the SVD and make the GPIOx registers into an
    // array
    unsafe {
        let lp_io = &*crate::peripherals::LP_IO::PTR;
        let pin_ptr = (lp_io.gpio0().as_ptr()).add(pin as usize);

        &*(pin_ptr as *const esp32c6::generic::Reg<esp32c6::lp_io::gpio0::GPIO0_SPEC>)
    }
}

/// Configures a pin for use as a low power pin
pub trait IntoLowPowerPin<const PIN: u8> {
    /// Converts the pin into a low power pin
    fn into_low_power(self) -> LowPowerPin<Unknown, { PIN }>;
}

#[doc(hidden)]
#[macro_export]
macro_rules! lp_gpio {
    (
        $($gpionum:literal)+
    ) => {
        paste::paste!{
            $(
                impl<MODE> $crate::gpio::lp_io::IntoLowPowerPin<$gpionum> for GpioPin<MODE, $gpionum> {
                    fn into_low_power(self) -> $crate::gpio::lp_io::LowPowerPin<Unknown, $gpionum> {
                        $crate::gpio::lp_io::init_low_power_pin($gpionum);
                        $crate::gpio::lp_io::LowPowerPin {
                            private: core::marker::PhantomData,
                        }
                    }
                }

                impl<MODE> $crate::gpio::RTCPin for GpioPin<MODE, $gpionum> {
                    unsafe fn apply_wakeup(&mut self, wakeup: bool, level: u8) {
                        let lp_io = &*$crate::peripherals::LP_IO::ptr();
                        lp_io.[< pin $gpionum >]().modify(|_, w| {
                            w.wakeup_enable().bit(wakeup).int_type().bits(level)
                        });
                    }

                    fn rtcio_pad_hold(&mut self, enable: bool) {
                        let mask = 1 << $gpionum;
                        unsafe {
                            let lp_aon =  &*$crate::peripherals::LP_AON::ptr();

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
                            let lp_aon = &*$crate::peripherals::LP_AON::ptr();
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
                            let lp_io = &*$crate::peripherals::LP_IO::ptr();
                            lp_io.[< gpio $gpionum >]().modify(|_, w| {
                                w
                                    .slp_sel().bit(false)
                                    .fun_ie().bit(input_enable)
                                    .mcu_sel().bits(func as u8)
                            });
                        }
                    }
                }

                impl<MODE> $crate::gpio::RTCPinWithResistors for GpioPin<MODE, $gpionum> {
                    fn rtcio_pullup(&mut self, enable: bool) {
                        let lp_io = unsafe { &*$crate::peripherals::LP_IO::ptr() };
                        lp_io.[< gpio $gpionum >]().modify(|_, w| w.fun_wpu().bit(enable));
                    }

                    fn rtcio_pulldown(&mut self, enable: bool) {
                        let lp_io = unsafe { &*$crate::peripherals::LP_IO::ptr() };
                        lp_io.[< gpio $gpionum >]().modify(|_, w| w.fun_wpd().bit(enable));
                    }
                }
            )+
        }
    }
}

pub(crate) use lp_gpio;
