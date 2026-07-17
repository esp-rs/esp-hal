use core::cell::RefCell;

use crate::{
    gpio::RtcFunction,
    peripherals::{LPWR, RTC_IO},
    rtc_cntl::{
        Rtc,
        WakeupSource,
        sleep::{RtcIoWakeupPinType, RtcSleepConfig, WakeSource, WakeTriggers, WakeupLevel},
    },
};

#[procmacros::doc_replace]
/// External wake-up source (Ext0).
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext0WakeupSource, LowPower, TimerWakeupSource, WakeupLevel}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull};
/// # use esp_hal::time::Duration;
///
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_input = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin_4_input);
/// let ext0 = Ext0WakeupSource::new(pin_4, WakeupLevel::High);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext0]);
///
/// # }
/// ```
pub struct Ext0WakeupSource<P: RtcIoWakeupPinType> {
    /// The pin used as the wake-up source.
    pin: RefCell<P>,
    /// The level at which the wake-up event is triggered.
    level: WakeupLevel,
}

impl<P: RtcIoWakeupPinType> Ext0WakeupSource<P> {
    /// Creates a new external wake-up source (Ext0``) with the specified pin
    /// and wake-up level.
    pub fn new(pin: P, level: WakeupLevel) -> Self {
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

impl<P: RtcIoWakeupPinType> WakeSource for Ext0WakeupSource<P> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.insert(WakeupSource::Ext0);

        // TODO: disable clock when not in use
        #[cfg(esp32s2)]
        crate::peripherals::SENS::regs()
            .sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());

        // set pin to RTC function
        self.pin
            .borrow_mut()
            .rtc_set_config(true, true, RtcFunction::Rtc);

        unsafe {
            // set pin register field
            RTC_IO::regs()
                .ext_wakeup0()
                .modify(|_, w| w.sel().bits(self.pin.borrow().rtc_number()));
            // set level register field
            LPWR::regs()
                .ext_wakeup_conf()
                .modify(|_r, w| w.ext_wakeup0_lv().bit(self.level == WakeupLevel::High));
        }
    }
}

impl<P: RtcIoWakeupPinType> Drop for Ext0WakeupSource<P> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        self.pin
            .borrow_mut()
            .rtc_set_config(true, false, RtcFunction::Rtc);
    }
}
