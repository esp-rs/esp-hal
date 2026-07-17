use crate::{
    gpio::{RtcFunction, RtcPin, RtcPinWithResistors},
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
    (RTC_GPIOn $(
        (($_rtc:ident, RTC_GPIOn, $n:literal), $gpio:ident)
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
    (($_rtc:ident, RTC_GPIOn, $n:literal), $gpio:ident) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl RtcPin for crate::peripherals::$gpio<'_> {
            fn rtc_number(&self) -> u8 {
                $n
            }

            fn rtc_set_config(&self, input_enable: bool, mux: bool, func: RtcFunction) {
                enable_iomux_clk_gate();
                pin_reg!($gpio).modify(|_, w| unsafe {
                    w.fun_ie().bit(input_enable);
                    w.mux_sel().bit(mux);
                    w.fun_sel().bits(func as u8)
                });
            }

            fn rtcio_pad_hold(&self, enable: bool) {
                LPWR::regs()
                    .pad_hold()
                    .modify(|_, w| hold_field!(w, $gpio).bit(enable));
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl RtcPinWithResistors for crate::peripherals::$gpio<'_> {
            fn rtcio_pullup(&self, enable: bool) {
                pullup_enable($n, enable);
            }

            fn rtcio_pulldown(&self, enable: bool) {
                pulldown_enable($n, enable);
            }
        }
    };
}

for_each_analog_function! {
    (($_ch:ident, ADCn_CHm, $_n:literal, $_m:literal), $gpio:ident) => {
        impl crate::peripherals::$gpio<'_> {
            #[cfg(feature = "unstable")]
            pub(crate) fn set_analog_impl(&self) {
                use crate::gpio::RtcPin;

                enable_iomux_clk_gate();

                output_enable(self.rtc_number(), false);
                set_open_drain_output(self.rtc_number(), false);

                pin_reg!($gpio).modify(|_, w| {
                    w.fun_ie().clear_bit();
                    w.mux_sel().set_bit();
                    unsafe { w.fun_sel().bits(0) };
                    w.rue().bit(false);
                    w.rde().bit(false)
                });
            }
        }
    };
}

pub(super) fn init_pin(pin: &impl RtcPin, input_enable: bool) -> u8 {
    pin.rtc_set_config(input_enable, true, RtcFunction::Rtc);
    pin.rtc_number()
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    if enable {
        RTC_IO::regs()
            .rtc_gpio_enable_w1ts()
            .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << pin) });
    } else {
        RTC_IO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.enable_w1tc().bits(1 << pin) });
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    with_pin_reg!(pin, |reg| reg.modify(|_, w| w.fun_ie().bit(enable)));
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    with_pin_reg!(pin, |reg| reg.modify(|_, w| w.rue().bit(enable)));
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    with_pin_reg!(pin, |reg| reg.modify(|_, w| w.rde().bit(enable)));
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
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
