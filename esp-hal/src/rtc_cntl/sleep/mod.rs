//! # RTC Control Sleep Module
//!
//! ## Overview
//! The `sleep` module allows configuring various wakeup sources and setting up
//! the sleep behavior based on those sources. The supported wakeup sources
//! include:
//!    * `GPIO` pins - light sleep only
//!    * timers
//!    * `SDIO (Secure Digital Input/Output) - light sleep only`
//!    * `MAC (Media Access Control)` wake - light sleep only
//!    * `UART0` - light sleep only
//!    * `UART1` - light sleep only
//!    * `touch`
//!    * `ULP (Ultra-Low Power)` wake
//!    * `BT (Bluetooth) wake` - light sleep only

use core::cell::RefCell;
#[cfg(any(esp32, esp32c3, esp32s3, esp32c6, esp32c2))]
use core::time::Duration;

#[cfg(any(esp32, esp32s3))]
use crate::gpio::RtcPin as RtcIoWakeupPinType;
#[cfg(any(esp32c3, esp32c6, esp32c2))]
use crate::gpio::RtcPinWithResistors as RtcIoWakeupPinType;
use crate::rtc_cntl::Rtc;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
mod sleep_impl;

pub use sleep_impl::*;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
/// Level at which a wake-up event is triggered
pub enum WakeupLevel {
    /// The wake-up event is triggered when the pin is low.
    Low,
    #[default]
    ///  The wake-up event is triggered when the pin is high.
    High,
}

/// Represents a timer wake-up source, triggering an event after a specified
/// duration.
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use core::time::Duration;
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::TimerWakeupSource, wakeup_cause, Rtc, SocResetReason};
/// # use esp_hal::system::Cpu;
///
/// let delay = Delay::new();
/// let mut rtc = Rtc::new(peripherals.LPWR);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(5));
/// delay.delay_millis(100);
/// rtc.sleep_deep(&[&timer]);
///
/// # }
/// ```
#[derive(Debug, Default, Clone, Copy)]
#[cfg(any(esp32, esp32c3, esp32s3, esp32c6, esp32c2))]
pub struct TimerWakeupSource {
    /// The duration after which the wake-up event is triggered.
    duration: Duration,
}

#[cfg(any(esp32, esp32c3, esp32s3, esp32c6, esp32c2))]
impl TimerWakeupSource {
    /// Creates a new timer wake-up source with the specified duration.
    pub fn new(duration: Duration) -> Self {
        Self { duration }
    }
}

/// Errors that can occur when configuring RTC wake-up sources.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The selected pin is not a valid RTC pin.
    NotRtcPin,
    /// The maximum number of wake-up sources has been exceeded.
    TooManyWakeupSources,
}

/// External wake-up source (Ext0).
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use core::time::Duration;
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel}, wakeup_cause, Rtc, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull};
///
/// let delay = Delay::new();
/// let mut rtc = Rtc::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_input = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin_4_input);
/// let ext0 = Ext0WakeupSource::new(pin_4, WakeupLevel::High);
///
/// delay.delay_millis(100);
/// rtc.sleep_deep(&[&timer, &ext0]);
///
/// # }
/// ```
#[cfg(any(esp32, esp32s3))]
pub struct Ext0WakeupSource<P: RtcIoWakeupPinType> {
    /// The pin used as the wake-up source.
    pin: RefCell<P>,
    /// The level at which the wake-up event is triggered.
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s3))]
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

/// External wake-up source (Ext1).
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use core::time::Duration;
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel}, wakeup_cause, Rtc, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull, RtcPin};
/// # use esp_hal::peripheral::Peripheral;
///
/// let delay = Delay::new();
/// let mut rtc = Rtc::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin_2 = peripherals.GPIO2;
/// let mut pin_4 = peripherals.GPIO4;
/// let pin_4_driver = Input::new(pin_4.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// // Drop the driver to access `pin_4`
/// core::mem::drop(pin_4_driver);
///
/// let mut wakeup_pins: [&mut dyn RtcPin; 2] = [&mut pin_4, &mut pin_2];
///
/// let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);
///
/// delay.delay_millis(100);
/// rtc.sleep_deep(&[&timer, &ext1]);
///
/// # }
/// ```
#[cfg(any(esp32, esp32s3))]
pub struct Ext1WakeupSource<'a, 'b> {
    /// A collection of pins used as wake-up sources.
    pins: RefCell<&'a mut [&'b mut dyn RtcIoWakeupPinType]>,
    /// The level at which the wake-up event is triggered across all pins.
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s3))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [&'b mut dyn RtcIoWakeupPinType], level: WakeupLevel) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
        }
    }
}

/// External wake-up source (Ext1).
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use core::time::Duration;
/// # use esp_hal::delay::Delay;
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel}, wakeup_cause, Rtc, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::gpio::{Input, InputConfig, Pull, RtcPinWithResistors};
/// # use esp_hal::peripheral::Peripheral;
///
/// let delay = Delay::new();
/// let mut rtc = Rtc::new(peripherals.LPWR);
///
/// let config = InputConfig::default().with_pull(Pull::None);
/// let mut pin2 = peripherals.GPIO2;
/// let mut pin3 = peripherals.GPIO3;
/// let mut pin2_input = Input::new(pin2.reborrow(), config);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {?}", reason, wake_reason);
///
/// let timer = TimerWakeupSource::new(Duration::from_secs(30));
///
/// core::mem::drop(pin2_input);
///
/// let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] =
/// &mut [
///     (&mut pin2, WakeupLevel::Low),
///     (&mut pin3, WakeupLevel::High),
/// ];
///
/// let ext1 = Ext1WakeupSource::new(wakeup_pins);
///
/// delay.delay_millis(100);
/// rtc.sleep_deep(&[&timer, &ext1]);
///
/// # }
/// ```
#[cfg(esp32c6)]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(esp32c6)]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1) with the specified pins and
    /// wake-up level.
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

/// RTC_IO wakeup source
///
/// RTC_IO wakeup allows configuring any combination of RTC_IO pins with
/// arbitrary wakeup levels to wake up the chip from sleep. This wakeup source
/// can be used to wake up from both light and deep sleep.
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use core::time::Duration;
/// # use esp_hal::delay::Delay;
/// # use esp_hal::gpio::{self, Input, InputConfig, Pull};
/// # use esp_hal::rtc_cntl::{reset_reason, sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel}, wakeup_cause, Rtc, SocResetReason};
/// # use esp_hal::system::Cpu;
/// # use esp_hal::peripheral::Peripheral;
///
/// let mut rtc = Rtc::new(peripherals.LPWR);
///
/// let reason = reset_reason(Cpu::ProCpu);
/// let wake_reason = wakeup_cause();
///
/// println!("{:?} {?}", reason, wake_reason);
///
/// let delay = Delay::new();
/// let timer = TimerWakeupSource::new(Duration::from_secs(10));
#[cfg_attr(any(esp32c3, esp32c2), doc = "let mut pin_0 = peripherals.GPIO2;")]
#[cfg_attr(any(esp32c3, esp32c2), doc = "let mut pin_1 = peripherals.GPIO3;")]
#[cfg_attr(esp32s3, doc = "let mut pin_0 = peripherals.GPIO17;")]
#[cfg_attr(esp32s3, doc = "let mut pin_1 = peripherals.GPIO18;")]
#[cfg_attr(
    any(esp32c3, esp32c2),
    doc = "let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] = &mut ["
)]
#[cfg_attr(
    esp32s3,
    doc = "let wakeup_pins: &mut [(&mut dyn gpio::RtcPin, WakeupLevel)] = &mut ["
)]
///     (&mut pin_0, WakeupLevel::Low),
///     (&mut pin_1, WakeupLevel::High),
/// ];
///
/// let rtcio = RtcioWakeupSource::new(wakeup_pins);
/// delay.delay_millis(100);
/// rtc.sleep_deep(&[&timer, &rtcio]);
///
/// # }
/// ```
#[cfg(any(esp32c3, esp32s3, esp32c2))]
pub struct RtcioWakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(any(esp32c3, esp32s3, esp32c2))]
impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    /// Creates a new external wake-up source (Ext1).
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

/// LP Core wakeup source
///
/// Wake up from LP core. This wakeup source
/// can be used to wake up from both light and deep sleep.
#[cfg(esp32c6)]
pub struct WakeFromLpCoreWakeupSource {}

#[cfg(esp32c6)]
impl WakeFromLpCoreWakeupSource {
    /// Create a new instance of `WakeFromLpCoreWakeupSource`
    pub fn new() -> Self {
        Self {}
    }
}

#[cfg(esp32c6)]
impl Default for WakeFromLpCoreWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

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
        triggers.set_gpio(true);
    }
}

macro_rules! uart_wakeup_impl {
    ($num:literal) => {
        paste::paste! {
            #[doc = concat!("UART", $num, " wakeup source")]
            ///
            /// The chip can be woken up by reverting RXD for multiple cycles until the
            /// number of rising edges is equal to or greater than the given value.
            ///
            /// Note that the character which triggers wakeup (and any characters before
            /// it) will not be received by the UART after wakeup. This means that the
            /// external device typically needs to send an extra character to trigger
            /// wakeup before sending the data.
            ///
            /// After waking-up from UART, you should send some extra data through the UART
            /// port in Active mode, so that the internal wakeup indication signal can be
            /// cleared. Otherwise, the next UART wake-up would trigger with two less
            /// rising edges than the configured threshold value.
            ///
            /// Wakeup from light sleep takes some time, so not every character sent to the
            /// UART can be received by the application.
            ///
            /// This wakeup source can be used to wake up from light sleep only.
            pub struct [< Uart $num WakeupSource >] {
                threshold: u16,
            }

            impl [< Uart $num WakeupSource >] {
                #[doc = concat!("Create a new instance of UART", $num, " wakeup source>") ]
                ///
                /// # Panics
                ///
                /// Panics if `threshold` is out of bounds.
                pub fn new(threshold: u16) -> Self {
                    if threshold > 1023 {
                        panic!("Invalid threshold");
                    }
                    Self { threshold }
                }
            }

            impl WakeSource for [< Uart $num WakeupSource >] {
                fn apply(&self, _rtc: &Rtc<'_>, triggers: &mut WakeTriggers, _sleep_config: &mut RtcSleepConfig) {
                    triggers.[< set_uart $num >](true);
                    let uart = crate::peripherals::[< UART $num >]::regs();

                    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
                    uart.sleep_conf()
                        .modify(|_, w| unsafe { w.active_threshold().bits(self.threshold) });

                    #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3)))]
                    uart.sleep_conf2().modify(|_, w| unsafe {
                        w.wk_mode_sel()
                            .bits(0)
                            .active_threshold()
                            .bits(self.threshold)
                    });
                }
            }
        }
    };
}

uart_wakeup_impl!(0);
uart_wakeup_impl!(1);

#[cfg(not(pmu))]
bitfield::bitfield! {
    /// Represents the wakeup triggers.
    #[derive(Default, Clone, Copy)]
    pub struct WakeTriggers(u16);
    impl Debug;
    /// EXT0 GPIO wakeup
    pub ext0, set_ext0: 0;
    /// EXT1 GPIO wakeup
    pub ext1, set_ext1: 1;
    /// GPIO wakeup (light sleep only)
    pub gpio, set_gpio: 2;
    /// Timer wakeup
    pub timer, set_timer: 3;
    /// SDIO wakeup (light sleep only)
    pub sdio, set_sdio: 4;
    /// MAC wakeup (light sleep only)
    pub mac, set_mac: 5;
    /// UART0 wakeup (light sleep only)
    pub uart0, set_uart0: 6;
    /// UART1 wakeup (light sleep only)
    pub uart1, set_uart1: 7;
    /// Touch wakeup
    pub touch, set_touch: 8;
    /// ULP wakeup
    pub ulp, set_ulp: 9;
    /// BT wakeup (light sleep only)
    pub bt, set_bt: 10;
}

#[cfg(pmu)]
bitfield::bitfield! {
    /// Represents the wakeup triggers.
    #[derive(Default, Clone, Copy)]
    pub struct WakeTriggers(u16);
    impl Debug;

    /// EXT0 GPIO wakeup
    pub ext0, set_ext0: 0;
    /// EXT1 GPIO wakeup
    pub ext1, set_ext1: 1;
    /// GPIO wakeup
    pub gpio, set_gpio: 2;
    /// WiFi beacon wakeup
    pub wifi_beacon, set_wifi_beacon: 3;
    /// Timer wakeup
    pub timer, set_timer: 4;
    /// WiFi SoC wakeup
    pub wifi_soc, set_wifi_soc: 5;
    /// UART0 wakeup
    pub uart0, set_uart0: 6;
    /// UART1 wakeup
    pub uart1, set_uart1: 7;
    /// SDIO wakeup
    pub sdio, set_sdio: 8;
    /// BT wakeup
    pub bt, set_bt: 10;
    /// LP core wakeup
    pub lp_core, set_lp_core: 11;
    /// USB wakeup
    pub usb, set_usb: 14;
}

/// Trait representing a wakeup source.
pub trait WakeSource {
    /// Configures the RTC and applies the wakeup triggers.
    fn apply(&self, rtc: &Rtc<'_>, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig);
}
