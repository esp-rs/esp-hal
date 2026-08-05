use crate::{
    gpio::{LpPin, LpPinWithResistors, WakeEvent, lp_io::LpFunction},
    peripherals::LP_AON,
};

cfg_select! {
    esp32c6 => {
        use crate::peripherals::{LP_IO as LP_GPIO, LP_IO as LP_IO_MUX};
    }
    any(esp32c5, esp32c61) => {
        use crate::peripherals::{LP_GPIO, LP_IO_MUX};
    }
}

for_each_lp_function! {
    (($_lp:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl LpPin for crate::peripherals::$gpio<'_> {
            fn lp_number(&self) -> u8 {
                $pin
            }

            fn apply_wakeup(&self, wakeup: bool, level: WakeEvent) {
                LP_GPIO::regs().pin($pin).modify(|_, w| unsafe {
                    w.wakeup_enable().bit(wakeup).int_type().bits(level as u8)
                });
            }

            fn lp_pad_hold(&self, enable: bool) {
                let mask = 1 << $pin;
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

            fn lp_set_config(&self, input_enable: bool, mux: bool, func: LpFunction) {
                let mask = 1 << $pin;
                LP_AON::regs()
                    .gpio_mux()
                    .modify(|r, w| unsafe {
                        let bits = r.sel().bits();
                        w.sel().bits(if mux {
                            bits | mask
                        } else {
                            bits & !mask
                        })
                    });

                LP_IO_MUX::regs().gpio($pin).modify(|_, w| unsafe {
                    w.slp_sel().bit(false);
                    w.fun_ie().bit(input_enable);
                    w.mcu_sel().bits(func as u8)
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

pub(super) fn init_pin(pin: &impl LpPin, input_enable: bool) -> u8 {
    pin.lp_set_config(input_enable, true, LpFunction::LP_GPIO);
    pin.number()
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    if enable {
        LP_GPIO::regs()
            .out_enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << pin) });
    } else {
        LP_GPIO::regs()
            .out_enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << pin) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_wpu().bit(enable));
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_wpd().bit(enable));
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    LP_GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}

#[cfg(lp_i2c_master_driver_supported)]
pub(crate) fn reset_pin(pin: u8) {
    output_enable(pin, false);
    set_open_drain_output(pin, false);

    // Resistors, input enable and the pad's LP function all live in this register.
    LP_IO_MUX::regs().gpio(pin as usize).reset();

    // Hand the pad back to the digital IO MUX.
    LP_AON::regs()
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() & !(1 << pin)) });
}
