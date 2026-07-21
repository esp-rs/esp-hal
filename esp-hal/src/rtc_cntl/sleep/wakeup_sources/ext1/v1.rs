use crate::{
    gpio::{Level, RtcFunction},
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

        cfg_select! {
            esp32s2 => {
                // TODO: disable clock when not in use
                crate::peripherals::SENS::regs()
                    .sar_io_mux_conf()
                    .modify(|_, w| w.iomux_clk_gate_en().set_bit());
            }
            _ => {}
        }

        let mut pins = self.pins.borrow_mut();
        let mut bits = 0u32;
        for pin in pins.iter_mut() {
            pin.rtc_set_config(true, true, RtcFunction::Rtc);
            pin.rtcio_pad_hold(true);
            bits |= 1 << pin.rtc_number();
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
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}
