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

use enumset::EnumSet;

use crate::{
    peripherals::LPWR,
    rtc_cntl::{Rtc, WakeupSource},
};

cfg_select! {
    any(esp32, esp32s2, esp32s3) => {
        use crate::gpio::RtcPin as RtcIoWakeupPinType;
    }
    any(esp32c2, esp32c3, esp32c5, esp32c6, esp32c61, esp32h2, esp32p4) => {
        use crate::gpio::RtcPinWithResistors as RtcIoWakeupPinType;
    }
}

#[cfg(soc_has_pmu)]
mod pmu_common;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c5, path = "esp32c5.rs")]
#[cfg_attr(esp32c61, path = "esp32c61.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
#[cfg_attr(esp32h2, path = "esp32h2.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
mod sleep_impl;
pub use sleep_impl::*;

mod wakeup_sources;
#[instability::unstable]
pub use wakeup_sources::*;

/// The set of wakeup sources configured to end a sleep.
///
/// This is a thin wrapper around a set of [`WakeupSource`]s. Wakeup source implementations enable
/// the sources they need via [`WakeTriggers::insert`] from within [`WakeSource::apply`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WakeTriggers(EnumSet<WakeupSource>);

impl WakeTriggers {
    /// Enables the given source as a wakeup trigger.
    pub fn insert(&mut self, source: WakeupSource) {
        self.0.insert(source);
    }

    /// Returns `true` if the given source is enabled as a wakeup trigger.
    pub fn contains(&self, source: WakeupSource) -> bool {
        self.0.contains(source)
    }

    /// Returns the raw wakeup-enable register bitmask.
    pub(crate) fn as_u32(&self) -> u32 {
        self.0.as_u32()
    }
}

/// Trait representing a wakeup source.
pub trait WakeSource {
    /// Configures the RTC and applies the wakeup triggers.
    fn apply(&self, rtc: &Rtc<'_>, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig);
}

/// Low-power management.
#[instability::unstable]
pub struct LowPower<'d> {
    _inner: LPWR<'d>,
}

impl<'d> LowPower<'d> {
    /// Creates a new `LowPower` driver.
    pub fn new(lpwr: LPWR<'d>) -> Self {
        Self { _inner: lpwr }
    }

    /// Enter deep sleep and wake with the provided `wake_sources`.
    ///
    /// In Deep-sleep mode, the CPUs, most of the RAM, and all digital
    /// peripherals that are clocked from APB_CLK are powered off.
    ///
    /// You can use the [`#[esp_hal::ram(persistent)]`][procmacros::ram]
    /// attribute to persist a variable though deep sleep.
    #[cfg(sleep_deep_sleep)]
    pub fn sleep_deep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let config = RtcSleepConfig::deep();
        self.sleep(&config, wake_sources);
        unreachable!();
    }

    /// Enter light sleep and wake with the provided `wake_sources`.
    #[cfg(sleep_light_sleep)]
    pub fn sleep_light(&mut self, wake_sources: &[&dyn WakeSource]) {
        let config = RtcSleepConfig::default();
        self.sleep(&config, wake_sources);
    }

    /// Enter sleep with the provided `config` and wake with the provided
    /// `wake_sources`.
    #[cfg(sleep_driver_supported)]
    #[crate::ram]
    pub fn sleep(&mut self, config: &RtcSleepConfig, wake_sources: &[&dyn WakeSource]) {
        let rtc = Rtc::new(unsafe { crate::peripherals::RTC_TIMER::steal() });

        let mut config = *config;
        let mut wakeup_triggers = WakeTriggers::default();
        for wake_source in wake_sources {
            wake_source.apply(&rtc, &mut wakeup_triggers, &mut config)
        }

        config.apply();

        sleep_uart_prepare();

        // Latch the systimer value *before* sleeping. The systimer keeps running during
        // the sleep enter/exit sequences, so we must not advance from the post-wake
        // value (that would count the enter/exit time twice). Instead we set an absolute
        // target of `before + slept`, measured by the always-running LP timer.
        let before_ticks = crate::time::implem::raw_counter();
        let before = rtc.time_since_boot_raw();

        let _uart0_sclk_guard = crate::system::ensure_uart0_sclk_enabled();
        config.start_sleep(wakeup_triggers);

        if config.is_deep_sleep() {
            // Because RTC is in a slower clock domain than the CPU, it
            // can take several CPU cycles for the sleep mode to start.
            loop {
                core::hint::spin_loop();
            }
        }

        config.finish_sleep();

        let after = rtc.time_since_boot_raw();

        let slept_us = crate::clock::rtc_ticks_to_us(after.wrapping_sub(before));
        let slept_ticks = crate::time::implem::us_to_ticks(slept_us);

        unsafe { crate::time::implem::update_counter(before_ticks + slept_ticks) };
        sleep_uart_resume();

        // Unlike deep sleep, light sleep does not reset the chip, so `wakeup_cause` cannot rely on
        // the reset reason to tell whether a wakeup occurred.
        // https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/esp_hw_support/sleep_modes.c#L2158
        if !config.deep_slp() {
            super::LIGHT_SLEEP_WAKEUP.store(true, portable_atomic::Ordering::Relaxed);
        }
    }
}

#[cfg(sleep_driver_supported)]
fn sleep_uart_prepare() {
    use crate::uart::Instance;
    for_each_uart! {
        ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident, wakeup_source = $_:literal) => {
            unsafe {
                crate::peripherals::$inst::steal().info().suspend_for_sleep();
            }
        };
    }
}

#[cfg(sleep_driver_supported)]
fn sleep_uart_resume() {
    use crate::uart::Instance;
    for_each_uart! {
        ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident, wakeup_source = $_:literal) => {
            unsafe {
                crate::peripherals::$inst::steal().info().resume_from_sleep();
            }
        };
    }
}
