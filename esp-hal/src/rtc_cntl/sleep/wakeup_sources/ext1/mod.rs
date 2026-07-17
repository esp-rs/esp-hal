use core::cell::RefCell;

use crate::rtc_cntl::sleep::{RtcIoWakeupPinType, WakeupLevel};

#[cfg_attr(any(esp32, esp32s2, esp32s3), path = "v1.rs")]
#[cfg_attr(any(esp32c5, esp32c6, esp32c61, esp32h2), path = "v2.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
mod implementation;

#[procmacros::doc_replace(
    "pin_low" => {
        cfg(esp32h2) => "GPIO9",
        _ => "GPIO2",
    },
    "pin_high" => {
        cfg(esp32h2) => "GPIO10",
        _ => "GPIO3",
    },
)]
/// External wake-up source (Ext1).
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource, WakeupLevel}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull, RtcPinWithResistors};
/// # use esp_hal::time::Duration;
/// #
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_low_input = Input::new(peripherals.__pin_low__.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin_low_input);
///
/// let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] =
/// &mut [
///     (&mut peripherals.__pin_low__, WakeupLevel::Low),
///     (&mut peripherals.__pin_high__, WakeupLevel::High),
/// ];
///
/// let ext1 = Ext1WakeupSource::new(wakeup_pins);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext1]);
///
/// # {after_snippet}
/// ```
#[cfg(not(any(esp32c2, esp32c3)))]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}
