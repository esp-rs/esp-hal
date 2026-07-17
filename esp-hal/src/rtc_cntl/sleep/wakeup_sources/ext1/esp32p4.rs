use crate::{
    gpio::{RtcFunction, RtcPin},
    peripherals::PMU,
    rtc_cntl::{
        Rtc,
        RtcSleepConfig,
        WakeSource,
        WakeTriggers,
        WakeupSource,
        sleep::{Ext1WakeupSource, WakeupLevel},
    },
};

impl Ext1WakeupSource<'_, '_> {
    fn wakeup_pins() -> u32 {
        PMU::regs().ext_wakeup_sel().read().ext_wakeup_sel().bits()
    }

    pub(in crate::rtc_cntl::sleep) fn wake_io_reset() {
        fn uninit_pin(pin: impl RtcPin, wakeup_pins: u32) {
            if wakeup_pins & (1 << pin.number()) != 0 {
                pin.rtcio_pad_hold(false);
                pin.rtc_set_config(false, false, RtcFunction::Rtc);
            }
        }

        let wakeup_pins = Self::wakeup_pins();
        for_each_lp_function! {
            (($_lp:ident, LP_GPIOn, $_pin:literal), $gpio:ident) => {
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
                WakeupLevel::High => 1 << pin.number(),
                WakeupLevel::Low => 0,
            };

            pin.rtc_set_config(true, true, RtcFunction::Rtc);
            pin.rtcio_pad_hold(true);
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
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}
