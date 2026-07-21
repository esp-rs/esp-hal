use crate::rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource};

/// GPIO wakeup source
///
/// Wake up from GPIO high or low level. Any pin can be used with this wake up
/// source. Configure the pin for wake up via
/// [crate::gpio::Input::wakeup_enable].
///
/// This wakeup source can be used to wake up from light sleep only.
pub struct GpioWakeupSource {}

impl GpioWakeupSource {
    /// Create a new instance of [GpioWakeupSource]
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for GpioWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

impl WakeSource for GpioWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.insert(WakeupSource::Gpio);
    }
}
