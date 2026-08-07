use super::RtcioWakeupSource;
use crate::{
    gpio::{
        Level,
        LpPin,
        WakeEvent,
        lp_io::{LpFunction, low_level},
    },
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource},
};

impl RtcioWakeupSource<'_, '_> {
    fn apply_pin(&self, pin: &mut dyn LpPin, level: Level) {
        let lp_pin = pin.lp_number();

        low_level::set_config(lp_pin, true, true, LpFunction::LP_GPIO);

        low_level::apply_wakeup(
            lp_pin,
            true,
            match level {
                Level::Low => WakeEvent::LowLevel,
                Level::High => WakeEvent::HighLevel,
            },
        );
    }
}

impl WakeSource for RtcioWakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        let mut pins = self.pins.borrow_mut();

        if pins.is_empty() {
            return;
        }

        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.insert(WakeupSource::Gpio);

        // Since we only use RTCIO pins, we can keep deep sleep enabled.
        let sens = crate::peripherals::SENS::regs();

        // TODO: disable clock when not in use
        cfg_select! {
            esp32s2 => {
                sens.sar_io_mux_conf()
                    .modify(|_, w| w.iomux_clk_gate_en().set_bit());
            }
            esp32s3 => {
                sens.sar_peri_clk_gate_conf()
                    .modify(|_, w| w.iomux_clk_en().set_bit());
            }
        }

        for (pin, level) in pins.iter_mut() {
            self.apply_pin(*pin, *level);
        }
    }
}

impl Drop for RtcioWakeupSource<'_, '_> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            low_level::set_config(pin.lp_number(), true, false, LpFunction::LP_GPIO);
        }
    }
}
