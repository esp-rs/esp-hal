use crate::{
    gpio::{RtcFunction, RtcPin, RtcPinWithResistors},
    peripherals::{LP_GPIO, LP_IO_MUX},
};

for_each_lp_function! {
    (($lp_pin_name:ident, LP_GPIOn, $lp_pin:literal), $gpio:ident) => {
        impl RtcPin for crate::peripherals::$gpio<'_> {
            unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                LP_GPIO::regs().pin($lp_pin).modify(|_, w| unsafe {
                    w.int_type().bits(level);
                    w.wakeup_enable().bit(wakeup)
                });
            }

            fn rtcio_pad_hold(&self, enable: bool) {
                pad_hold_enable($lp_pin, enable);
            }

            fn rtc_set_config(
                &self,
                input_enable: bool,
                mux: bool,
                func: RtcFunction,
            ) {
                if mux {
                    LP_GPIO::regs()
                        .clk_en()
                        .modify(|_, w| w.reg_clk_en().set_bit());
                    while LP_GPIO::regs().clk_en().read().reg_clk_en().bit_is_clear() {}
                }

                LP_IO_MUX::regs().pad($lp_pin).modify(|_, w| unsafe {
                    w.mux_sel().bit(mux);
                    w.fun_ie().bit(input_enable);
                    w.fun_sel().bits(match func {
                        RtcFunction::Rtc => 1,
                        RtcFunction::Digital => 0,
                    })
                });
            }
        }

        impl RtcPinWithResistors for crate::peripherals::$gpio<'_> {
            fn rtcio_pullup(&self, enable: bool) {
                pullup_enable($lp_pin, enable);
            }

            fn rtcio_pulldown(&self, enable: bool) {
                pulldown_enable($lp_pin, enable);
            }
        }
    };
}

pub(super) fn init_pin(pin: &impl RtcPin, input_enable: bool) -> u8 {
    let lp_pin = pin.number();
    pin.rtc_set_config(input_enable, true, RtcFunction::Rtc);
    lp_pin
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    if enable {
        LP_GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.bits(1 << pin) });
    } else {
        LP_GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.bits(1 << pin) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(pin as usize)
        .modify(|_, w| w.slp_ie().bit(enable));
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(pin as usize)
        .modify(|_, w| w.rue().bit(enable));
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(pin as usize)
        .modify(|_, w| w.rde().bit(enable));
}

pub(super) fn pad_hold_enable(pin: u8, enable: bool) {
    LP_IO_MUX::regs().lp_pad_hold().modify(|r, w| unsafe {
        let bits = r.reg_lp_gpio_hold().bits();
        w.reg_lp_gpio_hold().bits(if enable {
            bits | (1 << pin)
        } else {
            bits & !(1 << pin)
        })
    });
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    crate::peripherals::GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
