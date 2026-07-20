use core::cell::RefCell;

use crate::{gpio::Level, rtc_cntl::sleep::RtcIoWakeupPinType};

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
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Level, Pull, RtcPinWithResistors};
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
/// let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, Level)] =
/// &mut [
///     (&mut peripherals.__pin_low__, Level::Low),
///     (&mut peripherals.__pin_high__, Level::High),
/// ];
///
/// let ext1 = Ext1WakeupSource::new(wakeup_pins);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext1]);
///
/// # {after_snippet}
/// ```
#[cfg(not(any(esp32, esp32s2, esp32s3)))]
#[instability::unstable]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, Level)]>,
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    #[instability::unstable]
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, Level)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

#[procmacros::doc_replace]
/// External wake-up source (Ext1).
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource}, wakeup_cause, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Level, Pull, RtcPin};
/// # use esp_hal::time::Duration;
///
/// let delay = Delay::new();
/// let mut lpwr = LowPower::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_2 = peripherals.GPIO2;
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_driver = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {:?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// // Drop the driver to access `pin_4`
/// core::mem::drop(pin_4_driver);
///
/// let mut wakeup_pins: [&mut dyn RtcPin; 2] = [&mut pin_4, &mut pin_2];
///
/// let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, Level::High);
///
/// delay.delay_millis(100);
/// lpwr.sleep_deep(&[&timer, &ext1]);
///
/// # }
/// ```
#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct Ext1WakeupSource<'a, 'b> {
    /// A collection of pins used as wake-up sources.
    pins: RefCell<&'a mut [&'b mut dyn RtcIoWakeupPinType]>,
    /// The level at which the wake-up event is triggered across all pins.
    level: Level,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [&'b mut dyn RtcIoWakeupPinType], level: Level) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
        }
    }
}
