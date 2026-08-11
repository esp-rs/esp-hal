use crate::{
    gpio::{
        Level,
        LpPin,
        lp_io::{LpFunction, hold_bit},
    },
    peripherals::{GPIO, LPWR, RTC_IO, SENS},
};

#[rustfmt::skip]
macro_rules! pin_reg {
    (GPIO0)  => { RTC_IO::regs().touch_pad(0) };
    (GPIO1)  => { RTC_IO::regs().touch_pad(1) };
    (GPIO2)  => { RTC_IO::regs().touch_pad(2) };
    (GPIO3)  => { RTC_IO::regs().touch_pad(3) };
    (GPIO4)  => { RTC_IO::regs().touch_pad(4) };
    (GPIO5)  => { RTC_IO::regs().touch_pad(5) };
    (GPIO6)  => { RTC_IO::regs().touch_pad(6) };
    (GPIO7)  => { RTC_IO::regs().touch_pad(7) };
    (GPIO8)  => { RTC_IO::regs().touch_pad(8) };
    (GPIO9)  => { RTC_IO::regs().touch_pad(9) };
    (GPIO10) => { RTC_IO::regs().touch_pad(10) };
    (GPIO11) => { RTC_IO::regs().touch_pad(11) };
    (GPIO12) => { RTC_IO::regs().touch_pad(12) };
    (GPIO13) => { RTC_IO::regs().touch_pad(13) };
    (GPIO14) => { RTC_IO::regs().touch_pad(14) };
    (GPIO15) => { RTC_IO::regs().xtal_32p_pad() };
    (GPIO16) => { RTC_IO::regs().xtal_32n_pad() };
    (GPIO17) => { RTC_IO::regs().pad_dac1() };
    (GPIO18) => { RTC_IO::regs().pad_dac2() };
    (GPIO19) => { RTC_IO::regs().rtc_pad19() };
    (GPIO20) => { RTC_IO::regs().rtc_pad20() };
    (GPIO21) => { RTC_IO::regs().rtc_pad21() };
}

#[rustfmt::skip]
macro_rules! hold_field {
    ($reg:ident, GPIO0)  => { $reg.touch_pad0() };
    ($reg:ident, GPIO1)  => { $reg.touch_pad1() };
    ($reg:ident, GPIO2)  => { $reg.touch_pad2() };
    ($reg:ident, GPIO3)  => { $reg.touch_pad3() };
    ($reg:ident, GPIO4)  => { $reg.touch_pad4() };
    ($reg:ident, GPIO5)  => { $reg.touch_pad5() };
    ($reg:ident, GPIO6)  => { $reg.touch_pad6() };
    ($reg:ident, GPIO7)  => { $reg.touch_pad7() };
    ($reg:ident, GPIO8)  => { $reg.touch_pad8() };
    ($reg:ident, GPIO9)  => { $reg.touch_pad9() };
    ($reg:ident, GPIO10) => { $reg.touch_pad10() };
    ($reg:ident, GPIO11) => { $reg.touch_pad11() };
    ($reg:ident, GPIO12) => { $reg.touch_pad12() };
    ($reg:ident, GPIO13) => { $reg.touch_pad13() };
    ($reg:ident, GPIO14) => { $reg.touch_pad14() };
    ($reg:ident, GPIO15) => { $reg.x32p() };
    ($reg:ident, GPIO16) => { $reg.x32n() };
    ($reg:ident, GPIO17) => { $reg.pdac1() };
    ($reg:ident, GPIO18) => { $reg.pdac2() };
    ($reg:ident, GPIO19) => { $reg.pad19() };
    ($reg:ident, GPIO20) => { $reg.pad20() };
    ($reg:ident, GPIO21) => { $reg.pad21() };
}

// Generates one big match statement because the pin registers have different types.
for_each_lp_function!(
    (LP_GPIOn $(
        (($_lp:ident, LP_GPIOn, $n:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt)
    ),*) => {
        macro_rules! with_pin_reg {
            ($pin:expr, |$reg:ident| $code:expr) => {{
                match $pin {
                    $(
                        $n => {
                            let $reg = pin_reg!($gpio);
                            $code
                        }
                    )*
                    _ => unreachable!(),
                }
            }};
        }
    };
);

for_each_lp_function! {
    (($_lp:ident, LP_GPIOn, $n:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl LpPin for crate::peripherals::$gpio<'_> {
            fn lp_number(&self) -> u8 {
                $n
            }
        }
    };

    // The hold register has one named field for each pad, and no index, so this code selects the
    // field by the low-power number.
    (LP_GPIOn $( (($_lp:ident, LP_GPIOn, $n:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) ),*) => {
        pub(crate) fn pad_hold(lp: u8, enable: bool) {
            LPWR::regs().pad_hold().modify(|_, w| {
                match lp {
                    $( $n => hold_field!(w, $gpio).bit(enable), )*
                    _ => unreachable!(),
                }
            });
        }

        /// One bit for each low-power pad.
        #[cfg(sleep_driver_supported)]
        const ALL_PADS: u32 = 0 $( | 1 << $n )*;
    };
}

/// Reads the hold bit of the pad of `gpio`, and writes it first if `enable` is [`Some`].
///
/// The register starts at the first pad of the digital supply, and the low-power pads take the
/// numbers below it.
pub(crate) fn digital_pad_hold(gpio: u8, enable: Option<bool>) -> bool {
    let Some(bit) = gpio.checked_sub(21) else {
        return false;
    };

    hold_bit!(LPWR::regs().dig_pad_hold(), dig_pad_hold, bit, enable)
}

pub(crate) fn set_config(lp: u8, input_enable: bool, mux: bool, func: LpFunction) {
    enable_iomux_clk_gate();
    with_pin_reg!(lp, |reg| reg.modify(|_, w| unsafe {
        w.fun_ie().bit(input_enable);
        w.mux_sel().bit(mux);
        w.fun_sel().bits(func as u8)
    }));
}

pub(crate) fn apply_wakeup(lp: u8, wakeup: bool, level: Level) {
    RTC_IO::regs().pin(lp as usize).modify(|_, w| unsafe {
        w.wakeup_enable().bit(wakeup);
        w.int_type().bits(crate::gpio::lp_io::wake_trigger(level))
    });
}

/// Returns the pads whose per-pin wakeup path triggered, as a mask of low-power numbers.
#[cfg(sleep_driver_supported)]
pub(crate) fn wakeup_status() -> u32 {
    let status = RTC_IO::regs().rtc_gpio_status().read();
    cfg_select! {
        esp32s2 => status.gpio_status_int().bits(),
        esp32s3 => status.int().bits(),
    }
}

/// Returns the pads that can wake the chip through the per-pin path, as a mask of low-power
/// numbers.
#[cfg(sleep_driver_supported)]
pub(crate) fn wakeup_enabled_mask() -> u32 {
    let mut mask = 0;
    let mut pads = ALL_PADS;
    while pads != 0 {
        let lp = pads.trailing_zeros();
        pads &= !(1 << lp);

        if RTC_IO::regs()
            .pin(lp as usize)
            .read()
            .wakeup_enable()
            .bit_is_set()
        {
            mask |= 1 << lp;
        }
    }
    mask
}

/// Clears [`wakeup_status`], so that it reports the next sleep and no earlier sleep.
#[cfg(sleep_driver_supported)]
pub(crate) fn clear_wakeup_status() {
    RTC_IO::regs().rtc_gpio_status_w1tc().write(|w| unsafe {
        cfg_select! {
            esp32s2 => w.gpio_status_int_w1tc().bits(ALL_PADS),
            esp32s3 => w.rtc_gpio_status_int_w1tc().bits(ALL_PADS),
        }
    });
}

for_each_analog_function! {
    (($_ch:ident, ADCn_CHm, $_n:literal, $_m:literal), $gpio:ident) => {
        impl crate::peripherals::$gpio<'_> {
            #[cfg(feature = "unstable")]
            pub(crate) fn set_analog_impl(&self) {
                use crate::gpio::{LpPin, Pin};

                enable_iomux_clk_gate();

                output_enable(self.lp_number(), false);
                set_open_drain_output(self.number(), false);

                pin_reg!($gpio).modify(|_, w| {
                    w.fun_ie().clear_bit();
                    w.mux_sel().set_bit();
                    unsafe { w.fun_sel().bits(LpFunction::LP_GPIO as u8) };
                    w.rue().bit(false);
                    w.rde().bit(false)
                });
            }
        }
    };
}

pub(crate) fn init_pin(lp: u8, input_enable: bool) -> u8 {
    set_config(lp, input_enable, true, LpFunction::LP_GPIO);
    lp
}

pub(crate) fn output_enable(lp: u8, enable: bool) {
    if enable {
        RTC_IO::regs()
            .rtc_gpio_enable_w1ts()
            .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << lp) });
    } else {
        RTC_IO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << lp) });
    }
}

pub(crate) fn input_enable(lp: u8, enable: bool) {
    with_pin_reg!(lp, |reg| reg.modify(|_, w| w.fun_ie().bit(enable)));
}

pub(crate) fn pullup_enable(lp: u8, enable: bool) {
    with_pin_reg!(lp, |reg| reg.modify(|_, w| w.rue().bit(enable)));
}

pub(crate) fn pulldown_enable(lp: u8, enable: bool) {
    with_pin_reg!(lp, |reg| reg.modify(|_, w| w.rde().bit(enable)));
}

// The pad driver bit is part of the digital GPIO peripheral, so this function takes the digital
// number.
pub(crate) fn set_open_drain_output(gpio: u8, enable: bool) {
    GPIO::regs()
        .pin(gpio as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}

#[cfg(lp_i2c_master_driver_supported)]
pub(crate) fn reset_pin(lp: u8) {
    output_enable(lp, false);
    // On these chips, each low-power pad has the same digital pin number.
    set_open_drain_output(lp, false);

    // Resistors, input enable, the pad's LP function and whether it is muxed to the LP IO at all
    // are all held in this register.
    enable_iomux_clk_gate();
    with_pin_reg!(lp, |reg| reg.reset());
}

fn enable_iomux_clk_gate() {
    cfg_select! {
        esp32s2 => {
            SENS::regs()
                .sar_io_mux_conf()
                .modify(|_, w| w.iomux_clk_gate_en().set_bit());
        }
        esp32s3 => {
            SENS::regs()
                .sar_peri_clk_gate_conf()
                .modify(|_, w| w.iomux_clk_en().set_bit());
        }
    }
}
