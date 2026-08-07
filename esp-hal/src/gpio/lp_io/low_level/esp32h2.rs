use crate::{
    gpio::{AlternateFunction, LpPin, lp_io::LpFunction},
    peripherals::{GPIO, IO_MUX, LP_AON},
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
}

pub(crate) fn pad_hold(lp: u8, enable: bool) {
    let mask = 1 << lp_pin_to_gpio(lp);
    LP_AON::regs().gpio_hold0().modify(|r, w| unsafe {
        let bits = r.gpio_hold0().bits();
        w.gpio_hold0()
            .bits(if enable { bits | mask } else { bits & !mask })
    });
}

/// Configures the pad.
///
/// The low-power domain reaches the pad through the digital IO MUX. There is thus no low-power
/// function to select.
pub(crate) fn set_config(lp: u8, input_enable: bool, _mux: bool, _func: LpFunction) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(lp) as usize)
        .modify(|_, w| unsafe {
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

fn lp_pin_to_gpio(lp: u8) -> u8 {
    lp + 7
}

#[expect(dead_code)]
pub(crate) fn output_enable(lp: u8, enable: bool) {
    let gpio = lp_pin_to_gpio(lp);
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

pub(crate) fn input_enable(lp: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(lp) as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

#[expect(dead_code)]
pub(crate) fn pullup_enable(lp: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(lp) as usize)
        .modify(|_, w| w.fun_wpu().bit(enable));
}

#[expect(dead_code)]
pub(crate) fn pulldown_enable(lp: u8, enable: bool) {
    IO_MUX::regs()
        .gpio(lp_pin_to_gpio(lp) as usize)
        .modify(|_, w| w.fun_wpd().bit(enable));
}

// The pad driver bit is part of the digital GPIO peripheral, and this chip gives a pad a different
// low-power number, so this function takes the digital number.
#[expect(dead_code)]
pub(crate) fn set_open_drain_output(gpio: u8, enable: bool) {
    GPIO::regs()
        .pin(gpio as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
