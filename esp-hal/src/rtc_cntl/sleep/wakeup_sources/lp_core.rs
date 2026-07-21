use crate::rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource};

/// LP Core wakeup source
///
/// Wake up from LP core. This wakeup source
/// can be used to wake up from both light and deep sleep.
pub struct WakeFromLpCoreWakeupSource {}

impl WakeFromLpCoreWakeupSource {
    /// Create a new instance of `WakeFromLpCoreWakeupSource`
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for WakeFromLpCoreWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

impl WakeSource for WakeFromLpCoreWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.insert(WakeupSource::LpCore);
    }
}
