use crate::{
    gpio::{
        AlternateFunction,
        Level,
        LpPin,
        lp_io::{LpFunction, hold_bit},
    },
    peripherals::{GPIO, IO_MUX, LPWR},
};

for_each_lp_function! {
    (($_lp:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl LpPin for crate::peripherals::$gpio<'_> {
            fn lp_number(&self) -> u8 {
                $pin
            }
        }
    };

    // The wakeup register and the hold register have one named field for each pad, and no index, so
    // these functions select the field by the low-power number.
    (LP_GPIOn $( (($_lp:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) ),*) => {
        paste::paste! {
            pub(crate) fn apply_wakeup(lp: u8, wakeup: bool, level: Level) {
                let trigger = crate::gpio::lp_io::wake_trigger(level);

                let gpio_wakeup = cfg_select! {
                    esp32c2 => LPWR::regs().cntl_gpio_wakeup(),
                    esp32c3 => LPWR::regs().gpio_wakeup(),
                };

                gpio_wakeup.modify(|_, w| unsafe {
                    match lp {
                        $(
                            $pin => {
                                w.[<gpio_pin $pin _wakeup_enable>]().bit(wakeup);
                                w.[<gpio_pin $pin _int_type>]().bits(trigger)
                            }
                        )*
                        _ => unreachable!(),
                    }
                });
            }

            pub(crate) fn pad_hold(lp: u8, enable: bool) {
                LPWR::regs().pad_hold().modify(|_, w| {
                    match lp {
                        $( $pin => w.[<gpio_pin $pin _hold>]().bit(enable), )*
                        _ => unreachable!(),
                    }
                });
            }

            /// Returns the pads that can wake the chip through the per-pin path, as a mask of
            /// low-power numbers.
            #[cfg(sleep_driver_supported)]
            pub(crate) fn wakeup_enabled_mask() -> u32 {
                let gpio_wakeup = cfg_select! {
                    esp32c2 => LPWR::regs().cntl_gpio_wakeup().read(),
                    esp32c3 => LPWR::regs().gpio_wakeup().read(),
                };

                let mut mask = 0;
                $(
                    if gpio_wakeup.[<gpio_pin $pin _wakeup_enable>]().bit_is_set() {
                        mask |= 1 << $pin;
                    }
                )*
                mask
            }
        }
    };
}

/// Reads the hold bit of the pad of `gpio`, and writes it first if `enable` is [`Some`].
///
/// The register numbers the pads of the digital supply the way the digital registers do, and the
/// low-power pads have their own register.
pub(crate) fn digital_pad_hold(gpio: u8, enable: Option<bool>) -> bool {
    hold_bit!(LPWR::regs().dig_pad_hold(), dig_pad_hold, gpio, enable)
}

/// Configures the pad.
///
/// The low-power domain reaches the pad through the digital IO MUX. There is thus no low-power
/// function to select, and the low-power number is the digital pin number.
pub(crate) fn set_config(lp: u8, input_enable: bool, _mux: bool, _func: LpFunction) {
    IO_MUX::regs().gpio(lp as usize).modify(|_, w| unsafe {
        w.slp_sel().bit(false);
        w.mcu_sel().bits(AlternateFunction::GPIO as u8);
        w.fun_ie().bit(input_enable)
    });
}

#[expect(dead_code)]
pub(crate) fn init_pin(lp: u8, enable_input: bool) -> u8 {
    input_enable(lp, enable_input);
    lp
}

#[expect(dead_code)]
pub(crate) fn output_enable(lp: u8, enable: bool) {
    if enable {
        GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << lp) });
    } else {
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << lp) });
    }
}

pub(crate) fn input_enable(lp: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

#[expect(dead_code)]
pub(crate) fn set_open_drain_output(lp: u8, enable: bool) {
    GPIO::regs()
        .pin(lp as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
