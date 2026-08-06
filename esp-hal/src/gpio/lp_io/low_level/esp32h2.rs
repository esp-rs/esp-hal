use crate::{
    gpio::{AlternateFunction, Level, LpPin, LpPinWithResistors, lp_io::LpFunction},
    peripherals::{GPIO, IO_MUX, LP_AON},
};

for_each_lp_function! {
    (($_lp:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl LpPin for crate::peripherals::$gpio<'_> {
            fn lp_number(&self) -> u8 {
                $pin
            }

            fn apply_wakeup(&self, wakeup: bool, level: Level) {
                let mask = 1 << $pin;
                LP_AON::regs().ext_wakeup_cntl().modify(|r, w| unsafe {
                    let select = r.ext_wakeup_sel().bits();
                    let levels = r.ext_wakeup_lv().bits();

                    w.ext_wakeup_filter().set_bit();
                    w.ext_wakeup_sel().bits(if wakeup {
                        select | mask
                    } else {
                        select & !mask
                    });
                    w.ext_wakeup_lv().bits(if level == Level::High {
                        levels | mask
                    } else {
                        levels & !mask
                    })
                });
            }

            fn lp_pad_hold(&self, enable: bool) {
                let mask = 1 << lp_pin_to_gpio($pin);
                LP_AON::regs()
                    .gpio_hold0()
                    .modify(|r, w| unsafe {
                        let bits = r.gpio_hold0().bits();
                        w.gpio_hold0().bits(if enable {
                            bits | mask
                        } else {
                            bits & !mask
                        })
                    });
            }

            // The LP core reaches the pad through the digital IO MUX, so there is no low-power
            // function to select.
            fn lp_set_config(&self, input_enable: bool, _mux: bool, _func: LpFunction) {
                IO_MUX::regs().gpio(lp_pin_to_gpio($pin) as usize)
                    .modify(|_, w| unsafe {
                        w.slp_sel().bit(false);
                        w.mcu_sel().bits(AlternateFunction::GPIO as u8);
                        w.fun_ie().bit(input_enable)
                    });
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl LpPinWithResistors for crate::peripherals::$gpio<'_> {
            fn lp_pullup(&self, enable: bool) {
                pullup_enable($pin, enable);
            }

            fn lp_pulldown(&self, enable: bool) {
                pulldown_enable($pin, enable);
            }
        }
    };
}

#[expect(dead_code)]
pub(super) fn init_pin(pin: &impl LpPin, enable_input: bool) -> u8 {
    let lp_pin = pin.lp_number();
    input_enable(lp_pin, enable_input);
    lp_pin
}

fn lp_pin_to_gpio(pin: u8) -> u8 {
    pin + 7
}

#[expect(dead_code)]
pub(super) fn output_enable(pin: u8, enable: bool) {
    let gpio = lp_pin_to_gpio(pin);
    if enable {
        GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << gpio) });
    } else {
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << gpio) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(pin) as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(pin) as usize)
        .modify(|_, w| w.fun_wpu().bit(enable));
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(pin) as usize)
        .modify(|_, w| w.fun_wpd().bit(enable));
}

#[expect(dead_code)]
pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
