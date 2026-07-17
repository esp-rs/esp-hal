use super::{RtcioWakeupSource, WakeupLevel};
use crate::{
    gpio::{RtcFunction, RtcPin},
    peripherals::RTC_IO,
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource},
};

impl RtcioWakeupSource<'_, '_> {
    fn apply_pin(&self, pin: &mut dyn RtcPin, level: WakeupLevel) {
        let rtcio = RTC_IO::regs();

        pin.rtc_set_config(true, true, RtcFunction::Rtc);

        rtcio.pin(pin.number() as usize).modify(|_, w| unsafe {
            w.gpio_pin_wakeup_enable().set_bit();
            w.gpio_pin_int_type().bits(match level {
                WakeupLevel::Low => 4,
                WakeupLevel::High => 5,
            })
        });
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
        sens.sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());

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
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}
