use crate::{
    gpio::{
        Level,
        lp_io::{LpFunction, low_level},
    },
    peripherals::LPWR,
    rtc_cntl::{
        Rtc,
        RtcSleepConfig,
        WakeSource,
        WakeTriggers,
        WakeupSource,
        sleep::Ext1WakeupSource,
    },
};

impl WakeSource for Ext1WakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.insert(WakeupSource::Ext1);

        let mut pins = self.pins.borrow_mut();
        let mut bits = 0u32;
        for pin in pins.iter_mut() {
            let lp_pin = pin.lp_number();

            low_level::set_config(lp_pin, true, true, LpFunction::LP_GPIO);
            low_level::pad_hold(lp_pin, true);
            bits |= 1 << lp_pin;
        }

        LPWR::regs()
            .ext_wakeup1()
            .modify(|_, w| w.status_clr().set_bit());
        LPWR::regs()
            .ext_wakeup1()
            .modify(|_, w| unsafe { w.sel().bits(bits) });

        LPWR::regs()
            .ext_wakeup_conf()
            .modify(|_, w| w.ext_wakeup1_lv().bit(self.level == Level::High));
    }
}

impl Drop for Ext1WakeupSource<'_, '_> {
    fn drop(&mut self) {
        let mut pins = self.pins.borrow_mut();
        for pin in pins.iter_mut() {
            low_level::set_config(pin.lp_number(), true, false, LpFunction::LP_GPIO);
        }
    }
}
