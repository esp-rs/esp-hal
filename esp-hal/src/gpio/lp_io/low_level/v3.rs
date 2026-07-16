use crate::{
    gpio::{RtcPin, RtcPinWithResistors},
    peripherals::{GPIO, IO_MUX, LPWR},
};

for_each_lp_function! {
    (($_rtc:ident, RTC_GPIOn, $pin:literal), $gpio:ident) => {
        paste::paste! {
            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl RtcPin for crate::peripherals::$gpio<'_> {
                unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                    let gpio_wakeup = cfg_select! {
                        esp32c2 => LPWR::regs().cntl_gpio_wakeup(),
                        esp32c3 => LPWR::regs().gpio_wakeup(),
                    };

                    gpio_wakeup.modify(|_, w| unsafe {
                        w.[<gpio_pin $pin _wakeup_enable>]().bit(wakeup);
                        w.[<gpio_pin $pin _int_type>]().bits(level)
                    });
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    LPWR::regs()
                        .pad_hold()
                        .modify(|_, w| w.[<gpio_pin $pin _hold>]().bit(enable));
                }
            }

            #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
            impl RtcPinWithResistors for crate::peripherals::$gpio<'_> {
                fn rtcio_pullup(&self, enable: bool) {
                    pullup_enable($pin, enable)
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    pulldown_enable($pin, enable)
                }
            }
        }
    };
}

pub(super) fn init_pin(pin: &impl RtcPin, input_enable: bool) -> u8 {
    let pin = pin.number();
    input_enable_fn(pin, input_enable);
    pin
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    if enable {
        GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << pin) });
    } else {
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << pin) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    input_enable_fn(pin, enable);
}

fn input_enable_fn(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_wpu().bit(enable));
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_wpd().bit(enable));
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
