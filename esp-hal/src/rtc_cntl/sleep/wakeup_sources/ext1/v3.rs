use crate::{
    gpio::{
        Level,
        LpPin,
        lp_io::{LpFunction, low_level},
    },
    peripherals::PMU,
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
    fn wakeup_pins() -> u32 {
        PMU::regs().ext_wakeup_sel().read().ext_wakeup_sel().bits()
    }

    /// Releases the pad hold that a previous sleep took, so the pads follow their drivers again.
    ///
    /// This releases the hold and nothing else. Reassigning the pad function would steal a pad that
    /// the application has since handed to an LP core, which owns its pads for as long as it runs.
    pub(in crate::rtc_cntl::sleep) fn wake_io_reset() {
        fn uninit_pin(pin: impl LpPin, wakeup_pins: u32) {
            if wakeup_pins & (1 << pin.number()) != 0 {
                low_level::pad_hold(pin.lp_number(), false);
            }
        }

        let wakeup_pins = Self::wakeup_pins();
        for_each_lp_function! {
            (($_lp:ident, LP_GPIOn, $_pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
                uninit_pin(unsafe { $crate::peripherals::$gpio::steal() }, wakeup_pins);
            };
        }
        PMU::regs().ext_wakeup_sel().reset();
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
        let mut pin_mask = 0u32;
        let mut level_mask = 0u32;
        for (pin, level) in pins.iter_mut() {
            pin_mask |= 1 << pin.number();
            level_mask |= match level {
                Level::High => 1 << pin.number(),
                Level::Low => 0,
            };

            low_level::set_config(pin.lp_number(), true, true, LpFunction::LP_GPIO);
            low_level::pad_hold(pin.lp_number(), true);
        }

        PMU::regs()
            .ext_wakeup_cntl()
            .modify(|_, w| w.ext_wakeup_status_clr().set_bit());
        PMU::regs()
            .ext_wakeup_cntl()
            .modify(|_, w| w.ext_wakeup_status_clr().clear_bit());

        PMU::regs()
            .ext_wakeup_sel()
            .write(|w| unsafe { w.ext_wakeup_sel().bits(pin_mask) });
        PMU::regs()
            .ext_wakeup_lv()
            .write(|w| unsafe { w.ext_wakeup_lv().bits(level_mask) });
    }
}

impl Drop for Ext1WakeupSource<'_, '_> {
    fn drop(&mut self) {
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            low_level::set_config(pin.lp_number(), true, false, LpFunction::LP_GPIO);
        }
    }
}
