use crate::{
    gpio::{
        Level,
        LpPin,
        lp_io::{LpFunction, low_level},
    },
    peripherals::LP_AON,
    rtc_cntl::{
        Rtc,
        RtcSleepConfig,
        WakeSource,
        WakeTriggers,
        WakeupSource,
        sleep::Ext1WakeupSource,
    },
};

impl Ext1WakeupSource<'_, '_> {
    fn wakeup_pins() -> u8 {
        LP_AON::regs()
            .ext_wakeup_cntl()
            .read()
            .ext_wakeup_sel()
            .bits()
    }

    /// Releases the pad hold that a previous sleep took, so the pads follow their drivers again.
    ///
    /// This releases the hold and nothing else. Reassigning the pad function would steal a pad that
    /// the application has since handed to an LP core, which owns its pads for as long as it runs.
    pub(in crate::rtc_cntl::sleep) fn wake_io_reset() {
        fn uninit_pin(pin: impl LpPin, wakeup_pins: u8) {
            let pin_number = pin.lp_number();

            if wakeup_pins & (1 << pin_number) != 0 {
                low_level::pad_hold(pin_number, false);
            }
        }

        let wakeup_pins = Self::wakeup_pins();
        for_each_lp_function! {
            (($_lp:ident, LP_GPIOn, $_pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
                uninit_pin(unsafe { $crate::peripherals::$gpio::steal() }, wakeup_pins);
            };
        }
    }
}

impl WakeSource for Ext1WakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.insert(WakeupSource::Ext1);

        let mut pins = self.pins.borrow_mut();
        let mut pin_mask = 0u8;
        let mut level_mask = 0u8;
        for (pin, level) in pins.iter_mut() {
            let pin_number = pin.lp_number();

            pin_mask |= 1 << pin_number;
            level_mask |= match level {
                Level::High => 1 << pin_number,
                Level::Low => 0,
            };

            low_level::set_config(pin_number, true, !cfg!(esp32h2), LpFunction::LP_GPIO);
            low_level::pad_hold(pin_number, true);
        }

        LP_AON::regs()
            .ext_wakeup_cntl()
            .modify(|_, w| w.ext_wakeup_status_clr().set_bit());

        LP_AON::regs().ext_wakeup_cntl().modify(|r, w| unsafe {
            w.ext_wakeup_sel()
                .bits(r.ext_wakeup_sel().bits() | pin_mask);
            w.ext_wakeup_lv()
                .bits(r.ext_wakeup_lv().bits() & !pin_mask | level_mask)
        });
    }
}

impl Drop for Ext1WakeupSource<'_, '_> {
    fn drop(&mut self) {
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            let pin_number = pin.lp_number();

            if cfg!(esp32h2) {
                low_level::pad_hold(pin_number, false);
            }
            low_level::set_config(pin_number, true, false, LpFunction::LP_GPIO);
        }
    }
}
