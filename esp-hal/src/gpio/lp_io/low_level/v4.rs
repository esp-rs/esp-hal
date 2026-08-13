use crate::{
    gpio::{LpPin, lp_io::LpFunction},
    peripherals::LP_AON,
};

cfg_select! {
    esp32c6 => {
        use crate::peripherals::{LP_IO as LP_GPIO, LP_IO as LP_IO_MUX};
    }
    any(esp32c5, esp32c61) => {
        // Only a low-power peripheral or a low-power core reads and drives the pads.
        #[cfg(any(
            ulp_riscv_driver_supported,
            lp_io_has_gpio_matrix,
            lp_i2c_master_driver_supported,
        ))]
        use crate::peripherals::LP_GPIO;
        use crate::peripherals::LP_IO_MUX;
    }
}

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

// The low-power number of a pad is its digital number on these chips, and one register holds every
// pad, so both paths write the same bit.
pub(crate) fn pad_hold(lp: u8, enable: bool) {
    digital_pad_hold(lp, enable);
}

/// Returns whether something holds the pad.
pub(crate) fn is_pad_held(lp: u8) -> bool {
    is_digital_pad_held(lp)
}

/// Takes or releases the hold of the pad of `gpio`.
pub(crate) fn digital_pad_hold(gpio: u8, enable: bool) {
    let mask = 1 << gpio;
    LP_AON::regs().gpio_hold0().modify(|r, w| unsafe {
        let bits = r.gpio_hold0().bits();
        w.gpio_hold0()
            .bits(if enable { bits | mask } else { bits & !mask })
    });
}

/// Returns whether something holds the pad of `gpio`.
pub(crate) fn is_digital_pad_held(gpio: u8) -> bool {
    LP_AON::regs().gpio_hold0().read().gpio_hold0().bits() & (1 << gpio) != 0
}

pub(crate) fn set_config(lp: u8, input_enable: bool, mux: bool, func: LpFunction) {
    let mask = 1 << lp;
    LP_AON::regs().gpio_mux().modify(|r, w| unsafe {
        let bits = r.sel().bits();
        w.sel().bits(if mux { bits | mask } else { bits & !mask })
    });

    LP_IO_MUX::regs().gpio(lp as usize).modify(|_, w| unsafe {
        w.slp_sel().bit(false);
        w.fun_ie().bit(input_enable);
        w.mcu_sel().bits(func as u8)
    });
}

#[cfg(any(ulp_riscv_driver_supported, lp_io_has_gpio_matrix))]
pub(crate) fn init_pin(lp: u8, input_enable: bool) -> u8 {
    set_config(lp, input_enable, true, LpFunction::LP_GPIO);
    lp
}

#[cfg(any(
    ulp_riscv_driver_supported,
    lp_io_has_gpio_matrix,
    lp_i2c_master_driver_supported,
))]
pub(crate) fn output_enable(lp: u8, enable: bool) {
    if enable {
        LP_GPIO::regs()
            .out_enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << lp) });
    } else {
        LP_GPIO::regs()
            .out_enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << lp) });
    }
}

#[cfg(any(ulp_riscv_driver_supported, lp_io_has_gpio_matrix))]
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

// On these chips, the pad driver bit is part of the low-power GPIO peripheral, so this function
// takes the low-power number, like the functions beside it.
#[cfg(any(
    ulp_riscv_driver_supported,
    lp_io_has_gpio_matrix,
    lp_i2c_master_driver_supported,
))]
pub(crate) fn set_open_drain_output(lp: u8, enable: bool) {
    LP_GPIO::regs()
        .pin(lp as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}

#[cfg(lp_i2c_master_driver_supported)]
pub(crate) fn reset_pin(lp: u8) {
    output_enable(lp, false);
    set_open_drain_output(lp, false);

    // Resistors, input enable and the pad's LP function all live in this register.
    LP_IO_MUX::regs().gpio(lp as usize).reset();

    // Hand the pad back to the digital IO MUX.
    LP_AON::regs()
        .gpio_mux()
        .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() & !(1 << lp)) });
}
