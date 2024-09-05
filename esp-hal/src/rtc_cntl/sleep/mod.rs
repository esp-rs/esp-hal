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
#[cfg(any(esp32, esp32s3))]
use crate::{
    into_ref,
    peripheral::{Peripheral, PeripheralRef},
};

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
#[cfg(any(esp32, esp32s3))]
pub struct Ext0WakeupSource<'a, P: RtcIoWakeupPinType> {
    /// The pin used as the wake-up source.
    pin: RefCell<PeripheralRef<'a, P>>,
    /// The level at which the wake-up event is triggered.
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s3))]
impl<'a, P: RtcIoWakeupPinType> Ext0WakeupSource<'a, P> {
    /// Creates a new external wake-up source (Ext0``) with the specified pin
    /// and wake-up level.
    pub fn new(pin: impl Peripheral<P = P> + 'a, level: WakeupLevel) -> Self {
        into_ref!(pin);
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

/// External wake-up source (Ext1).
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
                    let uart = unsafe { crate::peripherals::[< UART $num >]::steal() };

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
