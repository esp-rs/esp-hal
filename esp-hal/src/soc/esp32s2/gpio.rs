//! # GPIO configuration module (ESP32-S2)
//!
//! ## Overview
//!
//! The `GPIO` module provides functions and configurations for controlling the
//! `General Purpose Input/Output` pins on the `ESP32-S2` chip. It allows you to
//! configure pins as inputs or outputs, set their state and read their state.
//!
//! Let's get through the functionality and configurations provided by this GPIO
//! module:
//!   - `impl_get_rtc_pad`:
//!       * This macro_rule generates a function to get a specific RTC pad. It takes a single
//!         argument `$pad_name`, which is an identifier representing the name of the pad. Returns a
//!         reference to the corresponding RTC pad.
//!   - `impl_get_rtc_pad_indexed`:
//!       * This macro_rule generates a function similar to the previous one but for indexed RTC
//!         pads. It takes two arguments: `$pad_name`, which represents the name of the pad, and
//!         `$idx`, which is the index of the specific pad. Returns a reference to the indexed RTC
//!         pad.
//!   - `gpio` block:
//!       * Defines the pin configurations for various GPIO pins. Each line represents a pin and its
//!         associated options such as input/output mode, analog capability, and corresponding
//!         functions.
//!   - `analog` block:
//!       * Block defines the analog capabilities of various GPIO pins. Each line represents a pin
//!         and its associated options such as mux selection, function selection, and input enable.
//!   - `enum InputSignal`:
//!       * This enumeration defines input signals for the GPIO mux. Each input signal is assigned a
//!         specific value.
//!   - `enum OutputSignal`:
//!       * This enumeration defines output signals for the GPIO mux. Each output signal is assigned
//!         a specific value.
//!
//! This trait provides functions to read the interrupt status and NMI status
//! registers for both the `PRO CPU` and `APP CPU`. The implementation uses the
//! `gpio` peripheral to access the appropriate registers.

#[rustfmt::skip]
macro_rules! pin_reg {
    ($rtcio:ident, GPIO0 ) => { $rtcio.touch_pad(0) };
    ($rtcio:ident, GPIO1 ) => { $rtcio.touch_pad(1) };
    ($rtcio:ident, GPIO2 ) => { $rtcio.touch_pad(2) };
    ($rtcio:ident, GPIO3 ) => { $rtcio.touch_pad(3) };
    ($rtcio:ident, GPIO4 ) => { $rtcio.touch_pad(4) };
    ($rtcio:ident, GPIO5 ) => { $rtcio.touch_pad(5) };
    ($rtcio:ident, GPIO6 ) => { $rtcio.touch_pad(6) };
    ($rtcio:ident, GPIO7 ) => { $rtcio.touch_pad(7) };
    ($rtcio:ident, GPIO8 ) => { $rtcio.touch_pad(8) };
    ($rtcio:ident, GPIO9 ) => { $rtcio.touch_pad(9) };
    ($rtcio:ident, GPIO10) => { $rtcio.touch_pad(10) };
    ($rtcio:ident, GPIO11) => { $rtcio.touch_pad(11) };
    ($rtcio:ident, GPIO12) => { $rtcio.touch_pad(12) };
    ($rtcio:ident, GPIO13) => { $rtcio.touch_pad(13) };
    ($rtcio:ident, GPIO14) => { $rtcio.touch_pad(14) };
    ($rtcio:ident, GPIO15) => { $rtcio.xtal_32p_pad() };
    ($rtcio:ident, GPIO16) => { $rtcio.xtal_32n_pad() };
    ($rtcio:ident, GPIO17) => { $rtcio.pad_dac1() };
    ($rtcio:ident, GPIO18) => { $rtcio.pad_dac2() };
    ($rtcio:ident, GPIO19) => { $rtcio.rtc_pad19() };
    ($rtcio:ident, GPIO20) => { $rtcio.rtc_pad20() };
    ($rtcio:ident, GPIO21) => { $rtcio.rtc_pad21() };
}

#[rustfmt::skip]
macro_rules! hold_field {
    ($reg:ident, GPIO0 ) => { $reg.touch_pad0() };
    ($reg:ident, GPIO1 ) => { $reg.touch_pad1() };
    ($reg:ident, GPIO2 ) => { $reg.touch_pad2() };
    ($reg:ident, GPIO3 ) => { $reg.touch_pad3() };
    ($reg:ident, GPIO4 ) => { $reg.touch_pad4() };
    ($reg:ident, GPIO5 ) => { $reg.touch_pad5() };
    ($reg:ident, GPIO6 ) => { $reg.touch_pad6() };
    ($reg:ident, GPIO7 ) => { $reg.touch_pad7() };
    ($reg:ident, GPIO8 ) => { $reg.touch_pad8() };
    ($reg:ident, GPIO9 ) => { $reg.touch_pad9() };
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

for_each_lp_function! {
    (($_rtc:ident, RTC_GPIOn, $n:literal), $gpio:ident) => {
        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl crate::gpio::RtcPin for crate::peripherals::$gpio<'_> {
            fn rtc_number(&self) -> u8 {
                $n
            }

            /// Set the RTC properties of the pin. If `mux` is true then then pin is
            /// routed to RTC, when false it is routed to IO_MUX.
            fn rtc_set_config(&self, input_enable: bool, mux: bool, func: crate::gpio::RtcFunction) {
                enable_iomux_clk_gate();

                let rtcio = crate::peripherals::RTC_IO::regs();
                pin_reg!(rtcio, $gpio)
                    .modify(|_, w| unsafe {
                        w.fun_ie().bit(input_enable);
                        w.mux_sel().bit(mux);
                        w.fun_sel().bits(func as u8)
                    });
            }

            fn rtcio_pad_hold(&self, enable: bool) {
                crate::peripherals::LPWR::regs()
                    .pad_hold()
                    .modify(|_, w| hold_field!(w, $gpio).bit(enable));
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
        impl crate::gpio::RtcPinWithResistors for crate::peripherals::$gpio<'_> {
            fn rtcio_pullup(&self, enable: bool) {
                let rtcio = crate::peripherals::RTC_IO::regs();
                pin_reg!(rtcio, $gpio).modify(|_, w| w.rue().bit(enable));
            }

            fn rtcio_pulldown(&self, enable: bool) {
                let rtcio = crate::peripherals::RTC_IO::regs();
                pin_reg!(rtcio, $gpio).modify(|_, w| w.rde().bit(enable));
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

                let rtcio = crate::peripherals::RTC_IO::regs();

                // disable output
                rtcio.enable_w1tc().write(|w| unsafe { w.enable_w1tc().bits(1 << self.rtc_number()) });

                // disable open drain
                rtcio.pin(self.rtc_number() as usize).modify(|_,w| w.pad_driver().bit(false));

                pin_reg!(rtcio, $gpio).modify(|_,w| {
                    w.fun_ie().clear_bit();

                    // Connect pin to analog / RTC module instead of standard GPIO
                    w.mux_sel().set_bit();

                    // Select function "RTC function 1" (GPIO) for analog use
                    unsafe { w.fun_sel().bits(0b00) };

                    // Disable pull-up and pull-down resistors on the pin
                    w.rue().bit(false);
                    w.rde().bit(false);

                    w
                });
            }
        }
    };
}

fn enable_iomux_clk_gate() {
    crate::peripherals::SENS::regs()
        .sar_io_mux_conf()
        .modify(|_, w| w.iomux_clk_gate_en().set_bit());
}
