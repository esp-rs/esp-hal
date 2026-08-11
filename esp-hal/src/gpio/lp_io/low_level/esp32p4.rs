use crate::{
    gpio::{
        LpPin,
        lp_io::{LpFunction, LpInputSignal, LpOutputSignal, hold_bit},
    },
    peripherals::{HP_SYS, LP_GPIO, LP_IO_MUX},
};

for_each_lp_function! {
    (($_lp_pin_name:ident, LP_GPIOn, $lp_pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl LpPin for crate::peripherals::$gpio<'_> {
            fn lp_number(&self) -> u8 {
                $lp_pin
            }
        }
    };

    // Each pad has its own signal table, so this code selects the table by the low-power number.
    (LP_GPIOn $((
        ($_lp_pin_name:ident, LP_GPIOn, $lp_pin:literal),
        $gpio:ident,
        $_af:ident,
        ($( $lp_in_af:ident => $lp_in_signal:ident )*)
        ($( $lp_out_af:ident => $lp_out_signal:ident )*)
    )),*) => {
        pub(crate) fn input_signals(lp: u8) -> &'static [(LpFunction, LpInputSignal)] {
            match lp {
                $( $lp_pin => &[$( (LpFunction::$lp_in_af, LpInputSignal::$lp_in_signal) ),*], )*
                _ => unreachable!(),
            }
        }

        pub(crate) fn output_signals(lp: u8) -> &'static [(LpFunction, LpOutputSignal)] {
            match lp {
                $( $lp_pin => &[$( (LpFunction::$lp_out_af, LpOutputSignal::$lp_out_signal) ),*], )*
                _ => unreachable!(),
            }
        }
    };
}

pub(crate) fn set_config(lp: u8, input_enable: bool, mux: bool, func: LpFunction) {
    if mux {
        LP_GPIO::regs()
            .clk_en()
            .modify(|_, w| w.reg_clk_en().set_bit());
        while LP_GPIO::regs().clk_en().read().reg_clk_en().bit_is_clear() {}
    }

    LP_IO_MUX::regs().pad(lp as usize).modify(|_, w| unsafe {
        w.mux_sel().bit(mux);
        w.fun_ie().bit(input_enable);
        w.fun_sel().bits(func as u8)
    });
}

pub(crate) fn init_pin(lp: u8, input_enable: bool) -> u8 {
    set_config(lp, input_enable, true, LpFunction::LP_GPIO);
    lp
}

pub(crate) fn output_enable(lp: u8, enable: bool) {
    if enable {
        LP_GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.bits(1 << lp) });
    } else {
        LP_GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.bits(1 << lp) });
    }
}

pub(crate) fn input_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(lp as usize)
        .modify(|_, w| w.slp_ie().bit(enable));
}

pub(crate) fn pullup_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(lp as usize)
        .modify(|_, w| w.rue().bit(enable));
}

pub(crate) fn pulldown_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .pad(lp as usize)
        .modify(|_, w| w.rde().bit(enable));
}

/// Reads the hold bit of the pad, and writes it first if `enable` is [`Some`].
pub(crate) fn pad_hold(lp: u8, enable: Option<bool>) -> bool {
    hold_bit!(
        LP_IO_MUX::regs().lp_pad_hold(),
        reg_lp_gpio_hold,
        lp,
        enable
    )
}

/// Reads the hold bit of the pad of `gpio`, and writes it first if `enable` is [`Some`].
///
/// The registers start at the first pad of the digital supply, which follows the 16 low-power pads.
pub(crate) fn digital_pad_hold(gpio: u8, enable: Option<bool>) -> bool {
    let Some(bit) = gpio.checked_sub(16) else {
        return false;
    };

    if bit < 32 {
        hold_bit!(
            HP_SYS::regs().gpio_o_hold_ctrl0(),
            reg_gpio_0_hold_low,
            bit,
            enable
        )
    } else {
        hold_bit!(
            HP_SYS::regs().gpio_o_hold_ctrl1(),
            reg_gpio_0_hold_high,
            bit - 32,
            enable
        )
    }
}

// The pad driver bit is part of the digital GPIO peripheral, so this function takes the digital
// number.
pub(crate) fn set_open_drain_output(gpio: u8, enable: bool) {
    crate::peripherals::GPIO::regs()
        .pin(gpio as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
