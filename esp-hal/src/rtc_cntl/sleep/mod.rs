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

use crate::{
    peripherals::LPWR,
    rtc_cntl::{Rtc, WakeupSource},
};

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

#[cfg(any(sleep_ext1_version = "2", sleep_ext1_version = "3"))]
mod ext1;

#[cfg(sleep_has_wakeup_source_timer)]
mod timer;

mod wakeup;
pub(crate) use wakeup::*;

/// Prepares the sleep hardware, and clears the wakeup sources of the previous run.
///
/// The wakeup-enable mask survives a deep-sleep wake, so at this point it still holds the intent of
/// the run that went to sleep. Two things need it in that order: the pads that run armed have to be
/// released, and then the mask has to start this run empty, because a program begins with no
/// configured wakeup sources and drivers build the mask up from there.
pub(crate) fn init(rtc: &Rtc<'_>) {
    // ESP-IDF releases the pads after a deep-sleep wake only, and only when the previous run armed
    // an IO wake source.
    #[cfg(any(sleep_ext1_version = "2", sleep_ext1_version = "3"))]
    if super::reset_reason(crate::system::Cpu::ProCpu) == Some(super::SocResetReason::CoreDeepSleep)
        && io_wake_enabled()
    {
        ext1::wake_io_reset();
    }

    RtcSleepConfig::base_settings(rtc);

    set_mask(0);
}

/// Low-power management.
///
/// The wakeup sources that end a sleep are not passed to the sleep calls. Each driver enables the
/// source it owns, and the hardware wakeup-enable mask keeps that intent until the driver clears
/// it — across a light sleep, and across a deep-sleep wake. A sleep call reads the mask back and
/// derives the rest from it.
#[instability::unstable]
pub struct LowPower<'d> {
    _inner: LPWR<'d>,
}

impl<'d> LowPower<'d> {
    /// Creates a new `LowPower` driver.
    pub fn new(lpwr: LPWR<'d>) -> Self {
        Self { _inner: lpwr }
    }

    /// Arms the sleep alarm for `deadline`, and enables the timer wakeup source.
    ///
    /// The deadline is absolute, so the time spent between arming it and sleeping does not shorten
    /// the sleep. It is a standing request: the wake it causes does not disarm it, and a later call
    /// replaces it. [`Self::clear_wakeup_deadline`] ends it.
    ///
    /// A deadline in the past ends a light sleep immediately, and panics in [`Self::sleep_deep`].
    #[cfg(sleep_has_wakeup_source_timer)]
    pub fn set_wakeup_deadline(&mut self, deadline: crate::time::Instant) {
        timer::set_deadline(deadline);
    }

    /// Disarms the sleep alarm, and disables the timer wakeup source.
    #[cfg(sleep_has_wakeup_source_timer)]
    pub fn clear_wakeup_deadline(&mut self) {
        timer::clear_deadline();
    }

    /// Enters deep sleep, and does not return.
    ///
    /// In deep sleep the CPUs, most of the RAM, and all digital peripherals that are clocked from
    /// APB_CLK are powered off. Waking resets the chip, so use the
    /// [`#[esp_hal::ram(persistent)]`][procmacros::ram] attribute to carry a variable across.
    ///
    /// This call cannot be rejected, because it cannot return to report it — see
    /// [`Self::sleep_deep_with_rejection`].
    ///
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled, because nothing could end the sleep, and if the
    /// armed wakeup deadline is too near for the sleep transition to catch. Both would otherwise
    /// leave a chip that never wakes and says nothing about why.
    #[cfg(sleep_deep_sleep)]
    pub fn sleep_deep(&mut self, config: RtcSleepConfig) -> ! {
        #[cfg(sleep_has_wakeup_source_timer)]
        if enabled_sources().contains(WakeupSource::Timer) {
            assert!(
                !timer::deadline_missed(),
                "the wakeup deadline is too near to be caught by the sleep transition"
            );
        }

        self.sleep(config, SleepKind::Deep, false);

        unreachable!("deep sleep without rejection cannot return")
    }

    /// Enters deep sleep, and returns only if the hardware rejects the request.
    ///
    /// A sleep is rejected when one of its wakeup sources is already asserted, which would
    /// otherwise mean sleeping through the event the caller wants to wake on. The return is the
    /// whole message, so there is nothing to report beyond it.
    ///
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled.
    #[cfg(sleep_deep_sleep)]
    pub fn sleep_deep_with_rejection(&mut self, config: RtcSleepConfig) {
        self.sleep(config, SleepKind::Deep, true);
    }

    /// Enters light sleep, and returns when a wakeup source ends it.
    ///
    /// Light sleep keeps the digital domain's state, so execution continues where it left off.
    ///
    /// The call also returns at once, without sleeping, when no wakeup source is enabled, or when
    /// the hardware rejects the request because a wakeup source is already asserted. Neither is
    /// reported: a sleep that was refused, rejected, or merely very short are the same thing to the
    /// caller.
    #[cfg(sleep_light_sleep)]
    pub fn sleep_light(&mut self, config: RtcSleepConfig) {
        self.sleep(config, SleepKind::Light, true);
    }

    /// Derives the sleep from the wakeup-enable mask, and enters it.
    #[cfg(sleep_driver_supported)]
    #[crate::ram]
    fn sleep(&mut self, config: RtcSleepConfig, kind: SleepKind, allow_reject: bool) {
        let rtc = Rtc::new(unsafe { crate::peripherals::RTC_TIMER::steal() });

        let mut config = config;
        config.set_sleep_kind(kind);

        // Hooks run before `apply`, so that a vote to keep a power domain alive reaches hardware,
        // and before the mask is read for the last time, because a hook may enable a source of its
        // own: GPIO allocates its pins across the paths here.
        run_entry_hooks(kind, &mut config);

        config.apply();

        // A sleep with no wakeup source would never end, and no counter wrap would save it.
        let wakeup_mask = mask();
        if wakeup_mask == 0 {
            match kind {
                // Refusing is the same outcome as a rejected sleep, which light sleep does not
                // report either.
                SleepKind::Light => return,
                SleepKind::Deep => {
                    panic!("no wakeup source is enabled, so nothing could end the sleep")
                }
            }
        }

        let reject_mask = if allow_reject { reject_mask() } else { 0 };

        sleep_uart_prepare();

        // Last, because it takes the pads away from whatever was driving them: the wakeup sources
        // have taken their holds by now, and nothing after this point needs a pad.
        #[cfg(sleep_deep_sleep_needs_gpio_isolation)]
        if kind == SleepKind::Deep {
            crate::gpio::wakeup::isolate_pads_for_deep_sleep();
        }

        // Latch the systimer value *before* sleeping. The systimer keeps running during
        // the sleep enter/exit sequences, so we must not advance from the post-wake
        // value (that would count the enter/exit time twice). Instead we set an absolute
        // target of `before + slept`, measured by the always-running LP timer.
        let before_ticks = crate::time::implem::raw_counter();
        let before = rtc.time_since_boot_raw();

        let _uart0_sclk_guard = crate::system::ensure_uart0_sclk_enabled();
        config.start_sleep(wakeup_mask, reject_mask);

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

        run_exit_hooks();

        // Unlike deep sleep, light sleep does not reset the chip, so `wakeup_cause` cannot rely on
        // the reset reason to tell whether a wakeup occurred.
        // https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/esp_hw_support/sleep_modes.c#L2158
        if !config.is_deep_sleep() {
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
