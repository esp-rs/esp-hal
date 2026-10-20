#[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
use crate::gpio::lp_io::{LpInputSignal, LpOutputSignal};
use crate::{
    gpio::{LpPin, lp_io::LpFunction},
    peripherals::{LP_GPIO, LP_IO_MUX, LP_PERI, LP_SYS},
};

/// Number of pads that the low-power registers reach. Digital hold bits start after this count.
const LP_PAD_COUNT: u8 = 8;

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
        #[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
        pub(crate) fn input_signals(lp: u8) -> &'static [(LpFunction, LpInputSignal)] {
            match lp {
                $( $lp_pin => &[$( (LpFunction::$lp_in_af, LpInputSignal::$lp_in_signal) ),*], )*
                _ => unreachable!(),
            }
        }

        #[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
        pub(crate) fn output_signals(lp: u8) -> &'static [(LpFunction, LpOutputSignal)] {
            match lp {
                $( $lp_pin => &[$( (LpFunction::$lp_out_af, LpOutputSignal::$lp_out_signal) ),*], )*
                _ => unreachable!(),
            }
        }
    };
}

fn enable_io_clock() {
    LP_PERI::regs()
        .iomux_ctrl()
        .modify(|_, w| w.lp_iomux_clk_en().set_bit());
    LP_GPIO::regs()
        .clock_gate()
        .modify(|_, w| w.clk_en().set_bit());
    while LP_PERI::regs()
        .iomux_ctrl()
        .read()
        .lp_iomux_clk_en()
        .bit_is_clear()
        || LP_GPIO::regs().clock_gate().read().clk_en().bit_is_clear()
    {}
}

pub(crate) fn set_config(lp: u8, input_enable: bool, mux: bool, func: LpFunction) {
    if mux {
        enable_io_clock();
    }

    LP_SYS::regs().padctrl().modify(|r, w| unsafe {
        let mut sel = r.pad_mux_sel().bits();
        if mux {
            sel |= 1u8 << lp;
        } else {
            sel &= !(1u8 << lp);
        }
        w.pad_mux_sel().bits(sel)
    });

    LP_IO_MUX::regs().gpio(lp as usize).modify(|_, w| unsafe {
        w.slp_sel().bit(false);
        w.fun_ie().bit(input_enable);
        w.mcu_sel().bits(func as u8)
    });
}

#[cfg(lp_io_has_gpio_matrix)]
pub(crate) fn init_pin(lp: u8, input_enable: bool) -> u8 {
    set_config(lp, input_enable, true, LpFunction::LP_GPIO);
    lp
}

#[cfg(lp_io_has_gpio_matrix)]
pub(crate) fn output_enable(lp: u8, enable: bool) {
    if enable {
        LP_GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1u8 << lp) });
    } else {
        LP_GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1u8 << lp) });
    }
}

#[cfg(lp_io_has_gpio_matrix)]
pub(crate) fn input_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(lp as usize)
        .modify(|_, w| w.fun_ie().bit(enable));
}

pub(crate) fn pullup_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(lp as usize)
        .modify(|_, w| w.fun_wpu().bit(enable));
}

pub(crate) fn pulldown_enable(lp: u8, enable: bool) {
    LP_IO_MUX::regs()
        .gpio(lp as usize)
        .modify(|_, w| w.fun_wpd().bit(enable));
}

pub(crate) fn pad_hold(lp: u8, enable: bool) {
    LP_SYS::regs().padctrl().modify(|r, w| unsafe {
        let mut hold = r.pad_hold().bits();
        if enable {
            hold |= 1u8 << lp;
        } else {
            hold &= !(1u8 << lp);
        }
        w.pad_hold().bits(hold)
    });
}

/// Returns whether something holds the pad.
pub(crate) fn is_pad_held(lp: u8) -> bool {
    LP_SYS::regs().padctrl().read().pad_hold().bits() & (1u8 << lp) != 0
}

/// Takes or releases the hold of the pad of `gpio`.
///
/// The digital hold registers start at GPIO8, which is the first pad of the digital supply.
pub(crate) fn digital_pad_hold(gpio: u8, enable: bool) {
    let Some(bit) = gpio.checked_sub(LP_PAD_COUNT) else {
        return;
    };

    if bit < 32 {
        let mask = 1 << bit;
        LP_SYS::regs().hp_gpio_o_hold_ctrl0().modify(|r, w| unsafe {
            let bits = r.hp_gpio_0_hold_ctrl0().bits();
            w.hp_gpio_0_hold_ctrl0()
                .bits(if enable { bits | mask } else { bits & !mask })
        });
    } else {
        let mask = 1 << (bit - 32);
        LP_SYS::regs().hp_gpio_o_hold_ctrl1().modify(|r, w| unsafe {
            let bits = r.hp_gpio_0_hold_ctrl1().bits();
            w.hp_gpio_0_hold_ctrl1()
                .bits(if enable { bits | mask } else { bits & !mask })
        });
    }
}

/// Returns whether something holds the pad of `gpio`.
pub(crate) fn is_digital_pad_held(gpio: u8) -> bool {
    let Some(bit) = gpio.checked_sub(LP_PAD_COUNT) else {
        return false;
    };

    if bit < 32 {
        LP_SYS::regs()
            .hp_gpio_o_hold_ctrl0()
            .read()
            .hp_gpio_0_hold_ctrl0()
            .bits()
            & (1 << bit)
            != 0
    } else {
        LP_SYS::regs()
            .hp_gpio_o_hold_ctrl1()
            .read()
            .hp_gpio_0_hold_ctrl1()
            .bits()
            & (1 << (bit - 32))
            != 0
    }
}

#[cfg(lp_io_has_gpio_matrix)]
pub(crate) fn set_open_drain_output(lp: u8, enable: bool) {
    LP_GPIO::regs()
        .pin(lp as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}
