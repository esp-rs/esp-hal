use crate::{
    gpio::{RtcFunction, RtcPin},
    peripherals::LP_AON,
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
    fn wakeup_pins() -> u8 {
        LP_AON::regs()
            .ext_wakeup_cntl()
            .read()
            .ext_wakeup_sel()
            .bits()
    }

    pub(in crate::rtc_cntl::sleep) fn wake_io_reset() {
        fn uninit_pin(pin: impl RtcPin, wakeup_pins: u8) {
            let pin_number = cfg_select! {
                esp32h2 => pin.rtc_number(),
                _ => pin.number(),
            };

            if wakeup_pins & (1 << pin_number) != 0 {
                pin.rtcio_pad_hold(false);
                cfg_select! {
                    esp32h2 => pin.degrade().init_gpio(),
                    _ => pin.rtc_set_config(false, false, RtcFunction::Rtc),
                }
            }
        }

        let wakeup_pins = Self::wakeup_pins();
        for_each_lp_function! {
            (($_lp:ident, LP_GPIOn, $_pin:literal), $gpio:ident) => {
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
            let pin_number = cfg_select! {
                esp32h2 => pin.rtc_number(),
                _ => pin.number(),
            };

            pin_mask |= 1 << pin_number;
            level_mask |= match level {
                WakeupLevel::High => 1 << pin_number,
                WakeupLevel::Low => 0,
            };

            pin.rtc_set_config(true, !cfg!(esp32h2), RtcFunction::Rtc);
            pin.rtcio_pad_hold(true);
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
            if cfg!(esp32h2) {
                pin.rtcio_pad_hold(false);
            }
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}
