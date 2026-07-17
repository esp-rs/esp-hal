use crate::time::Duration;

#[cfg_attr(any(esp32, esp32c2, esp32c3, esp32s2, esp32s3), path = "timer/v1.rs")]
#[cfg_attr(
    any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4),
    path = "timer/v2.rs"
)]
mod timer;

#[procmacros::doc_replace]
/// Represents a timer wake-up source, triggering an event after a specified
/// duration.
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{LowPower, TimerWakeupSource}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::time::Duration;
///
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(5));
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer]);
///
/// # {after_snippet}
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct TimerWakeupSource {
    /// The duration after which the wake-up event is triggered.
    duration: Duration,
}

impl TimerWakeupSource {
    /// Creates a new timer wake-up source with the specified duration.
    pub fn new(duration: Duration) -> Self {
        Self { duration }
    }
}
