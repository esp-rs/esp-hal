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
    gpio,
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

#[cfg(sleep_has_wakeup_source_timer)]
mod timer;

mod wakeup;
pub(crate) use wakeup::*;

/// Prepares the sleep hardware, and clears the wakeup sources of the previous run.
///
/// The wakeup-enable mask survives a deep-sleep wake, so here it still holds the request of the run
/// that went to sleep. Two steps need the mask, in this order. First, the code releases the pads
/// that the previous run armed. Then it clears the mask, because a program starts with no wakeup
/// sources, and the drivers of the new run build the mask again.
pub(crate) fn init(rtc: &Rtc<'_>) {
    // First, because the pads that ended a deep sleep are readable only until the code below
    // changes a path.
    gpio::wakeup::record_wakeup();

    // Release the pads after a deep-sleep wake only, and only if the previous
    // run armed an IO wake source.
    if super::reset_reason(crate::system::Cpu::ProCpu) == Some(super::SocResetReason::CoreDeepSleep)
        && gpio::wakeup::wake_enabled()
    {
        gpio::wakeup::wake_io_reset();
    }

    RtcSleepConfig::base_settings(rtc);

    set_mask(0);
}

/// Low-power management.
///
/// The sleep calls do not take the wakeup sources that end the sleep. Each driver enables the
/// source that it owns, and the hardware wakeup-enable mask keeps that request until the driver
/// clears it. The mask keeps it through a light sleep, and through a deep-sleep wake. A sleep call
/// reads the mask back, and calculates the rest of the configuration from it.
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
    /// The deadline is absolute, so the time between this call and the sleep does not make the
    /// sleep shorter. The deadline is a standing request. The wake that it causes does not
    /// disarm it, a later call replaces it, and [`Self::clear_wakeup_deadline`] removes it.
    ///
    /// A deadline in the past ends a light sleep immediately, and makes [`Self::sleep_deep`] panic.
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
    /// APB_CLK are powered off. The wake resets the chip, so use the
    /// [`#[esp_hal::ram(persistent)]`][procmacros::ram] attribute to keep a variable through the
    /// sleep.
    ///
    /// The hardware cannot reject this sleep, because the function cannot return to report the
    /// rejection. Use [`Self::sleep_deep_with_rejection`] for that.
    ///
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled, because then nothing can end the sleep. Panics also
    /// if the armed wakeup deadline is too near for the sleep transition to catch it. In both
    /// cases the chip never wakes again, and it gives no report of the cause.
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
    /// The hardware rejects a sleep if one of its wakeup sources is already asserted. Without the
    /// rejection, the chip sleeps through the event that the caller wants to wake on. The return of
    /// this function is the complete report, so it gives no other result.
    ///
    /// A rejected request returns the wake pads to their drivers, but it cannot return every pad.
    /// Sleep entry disconnects the pads that no hold keeps, on the chips that need that step to
    /// reach the deep-sleep current, and it cannot know their earlier configuration. Configure
    /// those pads again if this function returns. ESP-IDF has the same limit in
    /// `esp_deep_sleep_try_to_start`.
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
    /// Light sleep keeps the state of the digital domain, so the program continues at the same
    /// place.
    ///
    /// The function also returns immediately, without a sleep, if no wakeup source is enabled, or
    /// if the hardware rejects the request because a wakeup source is already asserted. It
    /// reports neither case. For the caller, a refused sleep, a rejected sleep and a very short
    /// sleep have the same result.
    #[cfg(sleep_light_sleep)]
    pub fn sleep_light(&mut self, config: RtcSleepConfig) {
        self.sleep(config, SleepKind::Light, true);
    }

    /// Calculates the sleep configuration from the wakeup-enable mask, and enters the sleep.
    #[cfg(sleep_driver_supported)]
    #[crate::ram]
    fn sleep(&mut self, config: RtcSleepConfig, kind: SleepKind, allow_reject: bool) {
        let rtc = Rtc::new(unsafe { crate::peripherals::RTC_TIMER::steal() });

        let mut config = config;
        config.set_sleep_kind(kind);

        // The hooks run before `apply`, so that a request to keep a power domain powered reaches
        // the hardware. They also run before the last read of the mask, because a hook can
        // enable another source. The GPIO hook does this while it allocates its pins to the
        // paths.
        run_entry_hooks(kind, &mut config);

        config.apply();

        // A sleep with no wakeup source never ends. No counter overflow ends it either.
        let wakeup_mask = mask();
        if wakeup_mask == 0 {
            match kind {
                // A refused sleep gives the same result as a rejected sleep, and light sleep does
                // not report that case either.
                SleepKind::Light => return,
                SleepKind::Deep => {
                    panic!("no wakeup source is enabled, so nothing could end the sleep")
                }
            }
        }

        let reject_mask = if allow_reject { reject_mask() } else { 0 };

        sleep_uart_prepare();

        // Last, because this step takes the pads away from the peripherals that drove them. The
        // wakeup sources have their holds now, and no later step needs a pad.
        #[cfg(sleep_deep_sleep_needs_gpio_isolation)]
        if kind == SleepKind::Deep {
            gpio::wakeup::isolate_pads_for_deep_sleep();
        }

        // Latch the systimer value *before* sleeping. The systimer keeps running during
        // the sleep enter/exit sequences, so we must not advance from the post-wake
        // value (that would count the enter/exit time twice). Instead we set an absolute
        // target of `before + slept`, measured by the always-running LP timer.
        let before_ticks = crate::time::implem::raw_counter();
        let before = rtc.time_since_boot_raw();

        let _uart0_sclk_guard = crate::system::ensure_uart0_sclk_enabled();
        let rejected = {
            // A chip can keep a guard for the length of the sleep, to restore what sleep entry
            // changed for the sleep only. The guard must therefore outlive the wait below.
            let _sleep_guard = config.start_sleep(wakeup_mask, reject_mask);
            let rejected = wait_for_sleep_result();

            if config.is_deep_sleep() && !rejected {
                // The chip is entering deep sleep, and the wake resets it. Because RTC is in a
                // slower clock domain than the CPU, the power-down can take several CPU cycles.
                loop {
                    core::hint::spin_loop();
                }
            }

            rejected
        };

        config.finish_sleep();

        let after = rtc.time_since_boot_raw();

        let slept_us = crate::clock::rtc_ticks_to_us(after.wrapping_sub(before));
        let slept_ticks = crate::time::implem::us_to_ticks(slept_us);

        unsafe { crate::time::implem::update_counter(before_ticks + slept_ticks) };
        sleep_uart_resume();

        run_exit_hooks();

        // Unlike deep sleep, light sleep does not reset the chip, so `wakeup_cause` cannot rely on
        // the reset reason to tell whether a wakeup occurred. A rejected request is not a wakeup,
        // and it must not name a wakeup source.
        // https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/esp_hw_support/sleep_modes.c#L2158
        if !config.is_deep_sleep() && !rejected {
            super::LIGHT_SLEEP_WAKEUP.store(true, portable_atomic::Ordering::Relaxed);
        }

        // Last, because this call reads the wakeup cause, and after a light sleep the cause is
        // available only after the line above.
        gpio::wakeup::record_wakeup();
    }
}

/// Waits for the hardware to report the result of the sleep request, and returns whether the
/// hardware rejected the request.
///
/// A deep sleep powers the CPU down inside this loop, and a light sleep stops the CPU here until a
/// wakeup source ends the sleep. A rejected request does neither, so the reject interrupt is the
/// only report of that case. ESP-IDF waits in the same place, in `rtc_sleep_start` and in
/// `pmu_sleep_start`.
#[cfg(sleep_driver_supported)]
fn wait_for_sleep_result() -> bool {
    loop {
        cfg_select! {
            soc_has_pmu => {
                let int_raw = crate::peripherals::PMU::regs().int_raw().read();
                if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                    return int_raw.soc_sleep_reject().bit_is_set();
                }
            }
            _ => {
                let int_raw = LPWR::regs().int_raw().read();
                if int_raw.slp_wakeup().bit_is_set() || int_raw.slp_reject().bit_is_set() {
                    return int_raw.slp_reject().bit_is_set();
                }
            }
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
