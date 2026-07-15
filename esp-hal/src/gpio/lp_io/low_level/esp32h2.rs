use crate::{
    gpio::{RtcPin, RtcPinWithResistors},
    peripherals::{GPIO, IO_MUX, LP_AON},
};

for_each_lp_function! {
    (($_lp:ident, LP_GPIOn, $pin:literal), $gpio:ident) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl RtcPin for crate::peripherals::$gpio<'_> {
            fn rtc_number(&self) -> u8 {
                $pin
            }

            fn rtcio_pad_hold(&self, enable: bool) {
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
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl RtcPinWithResistors for crate::peripherals::$gpio<'_> {}
    };
}

pub(super) fn init_pin(pin: &impl RtcPin, input_enable: bool) -> u8 {
    let lp_pin = pin.rtc_number();
    input_enable_fn(lp_pin, input_enable);
    lp_pin
}

fn lp_pin_to_gpio(pin: u8) -> u8 {
    pin + 7
}

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
    input_enable_fn(pin, enable);
}

fn input_enable_fn(pin: u8, enable: bool) {
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

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
