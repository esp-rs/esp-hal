use core::cell::RefCell;

use crate::{gpio::Level, rtc_cntl::sleep::RtcIoWakeupPinType};

#[procmacros::doc_replace(
    "pin0" => {
        cfg(any(esp32c3, esp32c2)) => "GPIO2",
        cfg(any(esp32s2, esp32s3)) => "GPIO17"
    },
    "pin1" => {
        cfg(any(esp32c3, esp32c2)) => "GPIO3",
        cfg(any(esp32s2, esp32s3)) => "GPIO18"
    },
    "rtc_pin_trait" => {
        cfg(any(esp32c3, esp32c2)) => "gpio::RtcPinWithResistors",
        cfg(any(esp32s2, esp32s3)) => "gpio::RtcPin"
    },
)]
/// RTC_IO wakeup source
///
/// RTC_IO wakeup allows configuring any combination of RTC_IO pins with
/// arbitrary wakeup levels to wake up the chip from sleep. This wakeup source
/// can be used to wake up from both light and deep sleep.
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::gpio::{self, Input, InputConfig, Level, Pull};
/// # use esp_hal::rtc_cntl::{reset_reason,
/// #   sleep::{LowPower, RtcioWakeupSource, TimerWakeupSource},
/// #   wakeup_cause, SocResetReason
/// # };
/// # use esp_hal::system::Cpu;
/// # use esp_hal::time::Duration;
///
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let delay = Delay::new();
/// let timer = TimerWakeupSource::new(Duration::from_secs(10));
/// let wakeup_pins: &mut [(&mut dyn __rtc_pin_trait__, Level)] = &mut [
///     (&mut peripherals.__pin0__, Level::Low),
///     (&mut peripherals.__pin1__, Level::High),
/// ];
///
/// let rtcio = RtcioWakeupSource::new(wakeup_pins);
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &rtcio]);
///
/// # {after_snippet}
/// ```
pub struct RtcioWakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, Level)]>,
}

impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    /// Creates a new external GPIO wake-up source.
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, Level)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

#[cfg_attr(any(esp32s2, esp32s3), path = "s2s3.rs")]
#[cfg_attr(any(esp32c2, esp32c3), path = "c2c3.rs")]
mod implementation;
