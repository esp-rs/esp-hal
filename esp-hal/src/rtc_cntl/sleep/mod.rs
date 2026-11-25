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

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

#[cfg(sleep_deep_sleep)]
use crate::rtc_cntl::WakeupSource;
use crate::{gpio, peripherals::LPWR, rtc_cntl::Rtc};

/// The CPU clock that [`SleepCpuClock::switch_to_xtal`] replaced for the length of the sleep.
///
/// The PLL is powered down across the sleep.
#[cfg(not(soc_has_pmu))]
#[must_use]
struct SleepCpuClock {
    cpu_clk: Option<crate::soc::clocks::CpuClkConfig>,
}

#[cfg(not(soc_has_pmu))]
impl SleepCpuClock {
    /// Runs the CPU from the crystal until [`Self::restore`].
    fn switch_to_xtal() -> Self {
        use crate::soc::clocks::{self, ClockTree, CpuClkConfig};

        ClockTree::with(|clocks| {
            let cpu_clk = clocks.cpu_clk();
            clocks::configure_cpu_clk(clocks, CpuClkConfig::Xtal);
            Self { cpu_clk }
        })
    }

    /// Configures the PLL again and restores the CPU clock.
    fn restore(self) {
        use crate::soc::clocks::{self, ClockTree};

        ClockTree::with(|clocks| {
            // If another consumer kept `PLL_CLK` requested while the CPU ran from `XTAL`,
            // restoring `CPU_CLK` below will not re-enable the PLL. `PLL_CLK` on esp32c2 and
            // esp32c3 has no other consumer, so restoring `CPU_CLK` configures it again.
            #[cfg(not(any(esp32c2, esp32c3)))]
            clocks::reconfigure_pll(clocks);
            if let Some(cpu_clk) = self.cpu_clk {
                clocks::configure_cpu_clk(clocks, cpu_clk);
            }
        });
    }
}

/// The wakeup waits in `TIMER1` for the PLL, the XTL buffer, and CK8M.
///
/// `TIMER1` counts these waits in RTC slow clock cycles, so the XTL wait of a light sleep is set in
/// time. With the 32 kHz crystal as the slow clock, the reset value of 80 cycles makes a light
/// sleep wakeup about 2.4 ms long.
/// See <https://github.com/espressif/esp-idf/blob/e4df0c12f70/components/esp_hw_support/port/esp32s3/rtc_sleep.c#L271-L277>.
#[cfg(all(not(soc_has_pmu), feature = "rt"))]
struct WakeupWaits {
    pll: u8,
    xtl: u16,
    ck8m: u8,
}

#[cfg(all(not(soc_has_pmu), feature = "rt"))]
impl WakeupWaits {
    fn for_sleep(kind: SleepKind) -> Self {
        let (pll, xtl, ck8m) = match kind {
            SleepKind::Deep => (
                sleep_impl::RTC_CNTL_PLL_BUF_WAIT_DEFAULT,
                u64::from(sleep_impl::RTC_CNTL_XTL_BUF_WAIT_DEFAULT),
                sleep_impl::RTC_CNTL_CK8M_WAIT_DEFAULT,
            ),
            SleepKind::Light => (
                sleep_impl::RTC_CNTL_PLL_BUF_WAIT_SLP_CYCLES,
                crate::clock::us_to_rtc_ticks(u64::from(sleep_impl::RTC_CNTL_XTL_BUF_WAIT_SLP_US)),
                sleep_impl::RTC_CNTL_CK8M_WAIT_SLP_CYCLES,
            ),
        };
        Self {
            pll,
            // The field has 10 bits, and `bits` drops the high bits of a longer wait.
            xtl: xtl.min(0x3FF) as u16,
            ck8m,
        }
    }

    #[inline(always)]
    fn write(self) {
        LPWR::regs().timer1().modify(|_, w| unsafe {
            w.pll_buf_wait().bits(self.pll);
            w.xtl_buf_wait().bits(self.xtl);
            w.ck8m_wait().bits(self.ck8m)
        });
    }
}

#[cfg(soc_has_pmu)]
pub(crate) mod pmu_common;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c5, path = "esp32c5.rs")]
#[cfg_attr(esp32c61, path = "esp32c61.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
#[cfg_attr(esp32h2, path = "esp32h2.rs")]
#[cfg_attr(esp32h4, path = "esp32h4.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
#[cfg_attr(esp32s31, path = "esp32s31.rs")]
mod sleep_impl;
pub use sleep_impl::*;

#[cfg(sleep_has_wakeup_source_timer)]
mod timer;

mod wakeup;
pub(crate) use wakeup::*;
#[instability::unstable]
pub use wakeup::{SleepEntryHook, SleepExitHook, WrappedSleepConfig};

/// The result of a light sleep.
#[cfg(sleep_light_sleep)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum LightSleep {
    /// The call returned from the sleep.
    ///
    /// The sleep may also never have started, because no wakeup source was enabled, because the
    /// hardware rejected the request, or because a sleep limit was too short for the transition to
    /// catch. This result does not tell those apart from a sleep that a wakeup source ended.
    Ended,

    /// A wakeup source or the check of [`LowPower::sleep_light_if`] refused the sleep, so the chip
    /// stayed awake.
    ///
    /// The refusal holds until the state that caused it changes, so a second call refuses too.
    /// Wait for an interrupt instead.
    Refused,
}

#[cfg(supports_cpu_power_down)]
use crate::rtc_cntl::cpu_retention;
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
    ///
    /// A wakeup source can limit one sleep to an earlier wake. The limit does not replace this
    /// deadline. The deadline stays armed for the next sleep.
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
    /// A wakeup source cannot refuse this sleep. The wake resets the chip, so the call cannot
    /// return a refusal. [`Self::sleep_light`] reports a refusal.
    #[cfg_attr(
        cpu_retention = "software",
        doc = "\n\nDeep sleep shares the entry code of light sleep. See [Optimization \
               level](crate#optimization-level) for the build requirement.\n"
    )]
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled, because then nothing can end the sleep. Panics also
    /// if the armed wakeup deadline is too near for the sleep transition to catch it, or if a
    /// wakeup source limits the sleep to less time than the transition can catch. In these cases
    /// the chip never wakes again, and it gives no report of the cause.
    #[cfg(sleep_deep_sleep)]
    #[cfg(feature = "rt")]
    pub fn sleep_deep(&mut self, config: RtcSleepConfig) -> ! {
        #[cfg(sleep_has_wakeup_source_timer)]
        if enabled_sources().contains(WakeupSource::Timer) {
            assert!(
                !timer::deadline_missed(),
                "the wakeup deadline is too near to be caught by the sleep transition"
            );
        }

        self.sleep(config, SleepKind::Deep, false, None);

        unreachable!("deep sleep without rejection cannot return")
    }

    /// Enters deep sleep, and returns only if the hardware rejects the request.
    ///
    /// The hardware rejects a sleep if one of its wakeup sources is already asserted. Without the
    /// rejection, the chip sleeps through the event that the caller wants to wake on. The return of
    /// this function is the complete report, so it gives no other result.
    ///
    /// A wakeup source cannot refuse this sleep. A wakeup source can limit it. The function also
    /// returns when that limit is too short for the sleep transition to catch. The chip stays
    /// awake. The return does not say which case it was.
    ///
    /// A rejected request returns the wake pads to their drivers, but it cannot return every pad.
    /// Sleep entry disconnects the pads that no hold keeps, on the chips that need that step to
    /// reach the deep-sleep current, and it cannot know their earlier configuration. Configure
    /// those pads again if this function returns. ESP-IDF has the same limit in
    /// `esp_deep_sleep_try_to_start`.
    ///
    /// Sleep entry also powers down the RF of a radio that still runs, and a rejected request does
    /// not power it up again. Shut the radio down before this call if it has to work after a
    /// rejection. ESP-IDF has the same limit.
    #[cfg_attr(
        cpu_retention = "software",
        doc = "\n\nDeep sleep shares the entry code of light sleep. See [Optimization \
               level](crate#optimization-level) for the build requirement.\n"
    )]
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled.
    #[cfg(sleep_deep_sleep)]
    #[cfg(feature = "rt")]
    pub fn sleep_deep_with_rejection(&mut self, config: RtcSleepConfig) {
        self.sleep(config, SleepKind::Deep, true, None);
    }

    /// Enters light sleep, and returns when a wakeup source ends it.
    ///
    /// Light sleep keeps the state of the digital domain, so the program continues at the same
    /// place.
    ///
    /// A wakeup source can refuse the sleep, which keeps the chip awake. A direct call and
    /// automatic light sleep both honor a refusal. The return reports it, because the caller then
    /// has to wait for the event itself.
    ///
    /// A wakeup source can also limit the length of the sleep. The chip wakes at the sooner of that
    /// limit and the armed wakeup deadline. When the timer source is not enabled, the limit enables
    /// it for this sleep and disables it again afterwards. The sleep does not start when that wake
    /// is already due, or when the transition cannot catch it. The result is [`LightSleep::Ended`],
    /// not [`LightSleep::Refused`].
    ///
    /// The function also returns without a sleep if no wakeup source is enabled, or if the
    /// hardware rejects the request because a wakeup source is already asserted. It reports
    /// neither case. For the caller, a rejected sleep and a very short sleep have the same
    /// result.
    #[cfg_attr(
        cpu_retention = "software",
        doc = "\n\nLight sleep suspends the caches. See [Optimization \
               level](crate#optimization-level) for the build requirement."
    )]
    #[cfg(sleep_light_sleep)]
    #[cfg(feature = "rt")]
    pub fn sleep_light(&mut self, config: RtcSleepConfig) -> LightSleep {
        self.sleep_light_checked(config, None)
    }

    /// Enters light sleep if `may_sleep` agrees, and returns when a wakeup source ends it.
    ///
    /// The documentation of [`Self::sleep_light`] applies. Before the sleep starts, this function
    /// parks the other cores, and then calls `may_sleep`. A `false` answer refuses the sleep, and
    /// the result is [`LightSleep::Refused`]. No other core runs until `may_sleep` returns, so the
    /// answer holds for the whole sleep. A light sleep that the APP core starts can run on the PRO
    /// core, and `may_sleep` then runs on the PRO core too.
    ///
    /// `may_sleep` runs with interrupts disabled. It must not take a lock that a core holds while
    /// it parks another core.
    #[cfg_attr(
        cpu_retention = "software",
        doc = "\n\nLight sleep suspends the caches. See [Optimization \
               level](crate#optimization-level) for the build requirement."
    )]
    #[cfg(sleep_light_sleep)]
    #[cfg(feature = "rt")]
    pub fn sleep_light_if(
        &mut self,
        config: RtcSleepConfig,
        may_sleep: fn() -> bool,
    ) -> LightSleep {
        self.sleep_light_checked(config, Some(may_sleep))
    }

    #[cfg(sleep_light_sleep)]
    #[cfg(feature = "rt")]
    fn sleep_light_checked(
        &mut self,
        config: RtcSleepConfig,
        may_sleep: Option<fn() -> bool>,
    ) -> LightSleep {
        if self.sleep(config, SleepKind::Light, true, may_sleep) {
            LightSleep::Refused
        } else {
            LightSleep::Ended
        }
    }

    /// Calculates the sleep configuration from the wakeup-enable mask, and enters the sleep.
    ///
    /// Returns whether a wakeup source or `may_sleep` refused a light sleep. A deep sleep ignores
    /// a refusal, so this function does not return `true` for a deep sleep.
    ///
    /// A sleep limit can end the call before the sleep starts. That result is `false`. It is not a
    /// refusal. A deep sleep whose limit the transition cannot catch panics when the caller cannot
    /// accept a rejection.
    ///
    /// A light sleep parks the other cores, and then calls `may_sleep`, as
    /// [`Self::sleep_light_if`] describes.
    #[cfg(sleep_driver_supported)]
    #[cfg(feature = "rt")]
    pub(crate) fn sleep(
        &mut self,
        config: RtcSleepConfig,
        kind: SleepKind,
        allow_reject: bool,
        may_sleep: Option<fn() -> bool>,
    ) -> bool {
        // ESP-IDF runs `esp_light_sleep_start` inside `portENTER_CRITICAL(&light_sleep_lock)`.
        // A sleep ends when `wait_for_sleep_result` sees a wakeup or reject bit, so nothing in
        // this path needs interrupts.
        let irq_token = unsafe { SingleCoreInterruptLock.enter() };
        let refused = cfg_select! {
            all(cpu_retention = "software", multi_core) => {
                // The PRO CPU requests every light sleep, because the wake releases the APP CPU
                // from the PRO CPU.
                match (kind, crate::system::Cpu::current()) {
                    (SleepKind::Light, crate::system::Cpu::AppCpu) => {
                        cpu_retention::rendezvous::sleep_from_app_cpu(
                            config,
                            allow_reject,
                            may_sleep,
                        )
                    }
                    (SleepKind::Light, crate::system::Cpu::ProCpu) => {
                        let refused = self.sleep_with_interrupts_disabled(
                            config,
                            kind,
                            allow_reject,
                            may_sleep,
                        );
                        if !refused {
                            cpu_retention::rendezvous::answer_app_cpu();
                        }
                        refused
                    }
                    _ => self.sleep_with_interrupts_disabled(config, kind, allow_reject, may_sleep),
                }
            }
            _ => self.sleep_with_interrupts_disabled(config, kind, allow_reject, may_sleep),
        };
        unsafe { SingleCoreInterruptLock.exit(irq_token) };
        refused
    }

    #[cfg(sleep_driver_supported)]
    #[cfg(feature = "rt")]
    fn sleep_with_interrupts_disabled(
        &mut self,
        config: RtcSleepConfig,
        kind: SleepKind,
        allow_reject: bool,
        may_sleep: Option<fn() -> bool>,
    ) -> bool {
        // First, so that no other core changes what the check and the hooks read. Dropped last, so
        // that the other cores run again with the time base and the wakeup cause of this sleep.
        #[cfg(multi_core)]
        let _parked = ParkedCores::new(kind);

        if kind == SleepKind::Light && may_sleep.is_some_and(|may_sleep| !may_sleep()) {
            return true;
        }

        let rtc = Rtc::new(unsafe { crate::peripherals::RTC_TIMER::steal() });

        let mut config = config;
        config.set_sleep_kind(kind);
        #[cfg(modem_retention = "regdma")]
        crate::rtc_cntl::regdma::prepare_sleep(kind == SleepKind::Light);

        // The slow clock drifts after the calibration at boot, and a deep sleep can be long. The
        // calibration comes before the entry hooks, because a wakeup source that limits the sleep
        // converts the limit to slow clock ticks.
        if kind == SleepKind::Deep {
            #[cfg(sleep_has_wakeup_source_timer)]
            let old_period = crate::clock::rtc_slow_cal_period();
            crate::clock::calibrate_rtc_slow_clock();
            #[cfg(sleep_has_wakeup_source_timer)]
            timer::rescale_deadline(old_period);
        }

        // Retention serves light sleep only. Deep sleep resets the chip, so it has no CPU state to
        // bring back, and an armed descriptor would outlive the sleep in the RTC domain. esp-idf
        // arms retention from its light sleep path alone.
        //
        // The buffer is read once, because the save of the APP CPU and the power-down must agree on
        // it. A buffer that another core installs during this sleep serves the next one.
        #[cfg(supports_cpu_power_down)]
        let retention_buffer = match kind {
            SleepKind::Light => crate::rtc_cntl::installed_buffer_ptr(),
            SleepKind::Deep => None,
        };

        // The hooks run before `apply`, so that a request to keep a power domain powered reaches
        // the hardware. They also run before the last read of the mask, because a hook can
        // enable another source. The GPIO hook does this while it allocates its pins to the
        // paths.
        let entry = run_entry_hooks(&mut config);

        // Stop before `apply` when a hook refused a light sleep. `apply` writes the sleep
        // configuration, and the chip must stay awake with that configuration unchanged.
        // Entry hooks can already have moved a pad. The exit hooks put the pad back.
        // A deep sleep ignores the refusal. The wake resets the chip, so there is no caller.
        if kind == SleepKind::Light && entry.refused {
            #[cfg(sleep_has_wakeup_source_timer)]
            if let Some(clamp) = entry.clamp {
                timer::restore_after_limit(clamp);
            }
            run_exit_hooks();
            return true;
        }

        // A limit inside the transition window cannot wake the chip. esp32 cannot reject on the
        // timer, so the hardware does not report the miss. Stay awake. The exit hooks undo pad
        // changes from the entry hooks. The comparator was not written.
        #[cfg(sleep_has_wakeup_source_timer)]
        if matches!(&entry.clamp, Some(timer::LimitClamp::TooShort)) {
            run_exit_hooks();
            if kind == SleepKind::Deep && !allow_reject {
                panic!("the wakeup deadline is too near to be caught by the sleep transition");
            }
            return false;
        }

        // After the software checks that can stop the sleep, so that the radio keeps working when
        // one of them does. The RF power-down needs the PLL clock, which `sleep_transition` stops.
        // See <https://github.com/espressif/esp-idf/blob/ad20ded62a2d67a26e501fdc1905f43ba7ad6c81/components/esp_sleep/sleep_modes.c#L997-L1002>.
        #[cfg(sleep_deep_sleep)]
        if kind == SleepKind::Deep {
            unsafe { __esp_phy_deep_sleep_prepare() };
        }

        // A deep sleep keeps what `RtcSleepConfig::deep` asked for, because the wake resets the
        // chip and keeps no CPU state to lose.
        #[cfg(supports_cpu_power_down)]
        if kind == SleepKind::Light {
            cpu_retention::configure_cpu_retention(&mut config, retention_buffer);
        }

        // The PMU chips write the configuration when the sleep starts.
        #[cfg(not(soc_has_pmu))]
        config.apply();

        // A sleep with no wakeup source never ends. No counter overflow ends it either.
        let wakeup_mask = mask();
        if wakeup_mask == 0 {
            // No sleep starts, so a clamp must not keep the comparator.
            #[cfg(sleep_has_wakeup_source_timer)]
            if let Some(clamp) = entry.clamp {
                timer::restore_after_limit(clamp);
            }

            match kind {
                // Not a software refusal. Light sleep does not report a missing wakeup source.
                SleepKind::Light => return false,
                SleepKind::Deep => {
                    panic!("no wakeup source is enabled, so nothing could end the sleep")
                }
            }
        }

        let reject_mask = if allow_reject { reject_mask() } else { 0 };

        // Each core saves itself, and a parked core saves nothing on its own.
        #[cfg(all(cpu_retention = "software", multi_core))]
        let app_cpu_saved = retention_buffer.is_some_and(cpu_retention::rendezvous::save_app_cpu);

        let suspended_uarts = sleep_uart_prepare();

        // Latch the systimer value *before* sleeping. The systimer keeps running during
        // the sleep enter/exit sequences, so we must not advance from the post-wake
        // value (that would count the enter/exit time twice). Instead we set an absolute
        // target of `before + slept`, measured by the always-running LP timer.
        let before_ticks = crate::time::implem::raw_counter();
        let before = rtc.time_since_boot_raw();

        let _uart0_sclk_guard = crate::system::ensure_uart0_sclk_enabled();

        let rejected = sleep_transition(
            &config,
            kind,
            wakeup_mask,
            reject_mask,
            #[cfg(not(soc_has_pmu))]
            WakeupWaits::for_sleep(kind),
            #[cfg(supports_cpu_power_down)]
            retention_buffer,
            #[cfg(all(cpu_retention = "software", multi_core))]
            app_cpu_saved,
        );

        let after = rtc.time_since_boot_raw();

        let slept_us = crate::clock::rtc_ticks_to_us(after.wrapping_sub(before));
        let slept_ticks = crate::time::implem::us_to_ticks(slept_us);

        unsafe { crate::time::implem::update_counter(before_ticks + slept_ticks) };
        sleep_uart_resume(suspended_uarts);

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

        // After the cause is recorded. A limit must not keep the comparator, or the next sleep
        // inherits a deadline that no hook requested.
        #[cfg(sleep_has_wakeup_source_timer)]
        if let Some(clamp) = entry.clamp {
            timer::restore_after_limit(clamp);
        }

        false
    }
}

#[cfg(all(sleep_deep_sleep, feature = "rt"))]
unsafe extern "C" {
    /// Powers down the RF of a running radio before a deep sleep. `hal-defaults.x` maps it to
    /// `__esp_phy_default_deep_sleep_prepare` unless esp-phy defines it.
    fn __esp_phy_deep_sleep_prepare();
}

#[cfg(all(sleep_deep_sleep, feature = "rt"))]
#[unsafe(export_name = "__esp_phy_default_deep_sleep_prepare")]
extern "C" fn esp_phy_default_deep_sleep_prepare() {}

/// Requests the sleep, and returns whether the hardware rejected the request.
///
/// This part switches the CPU clock, and it can suspend the caches or power down the CPU. The code
/// before and after it can run from flash.
#[cfg(all(sleep_driver_supported, feature = "rt"))]
#[crate::ram]
fn sleep_transition(
    config: &RtcSleepConfig,
    kind: SleepKind,
    wakeup_mask: u32,
    reject_mask: u32,
    #[cfg(not(soc_has_pmu))] wakeup_waits: WakeupWaits,
    #[cfg(supports_cpu_power_down)] retention_buffer: Option<core::ptr::NonNull<u8>>,
    #[cfg(all(cpu_retention = "software", multi_core))] app_cpu_saved: bool,
) -> bool {
    // After the light-sleep park, so that the other core cannot force the SAR on again.
    let sar_power = SarPower::for_sleep(kind).replace();

    let rejected = {
        // ESP-IDF arms RTC_CNTL retention in `misc_modules_sleep_prepare`, before it arms the
        // wakeup sources.

        #[cfg(cpu_retention = "rtc_cntl")]
        cpu_retention::prepare_cpu_retention(retention_buffer);

        let saved = config.start_sleep(wakeup_mask, reject_mask);
        // After the CPU clock switch, because the RC_FAST disable path writes `CK8M_WAIT`.
        #[cfg(not(soc_has_pmu))]
        wakeup_waits.write();

        // Last, because this step takes the pads away from the peripherals that drove them. The
        // wakeup sources have their holds now, and no later step needs a pad.
        #[cfg(any(
            sleep_deep_sleep_needs_gpio_isolation,
            gpio_need_soft_isolate_during_pd
        ))]
        if kind == SleepKind::Deep {
            gpio::wakeup::isolate_pads_for_deep_sleep();
        }

        // A CPU power-down loses the dirty cache lines, and with them the writes to PSRAM.
        // See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/sleep_cache.c#L55-L72>.
        #[cfg(soc_cpu_powers_down_cache)]
        if retention_buffer.is_some()
            && config.pd_flags.pd_cpu()
            && !crate::psram::psram_range().is_empty()
        {
            crate::soc::cache_writeback_all();
        }

        let rejected = cfg_select! {
            cpu_retention = "software" => {
                // The software chips save the CPU inside the request, so the request belongs to
                // them. ESP-IDF wraps `pmu_sleep_start` the same way (`sleep_modes.c:963-964`).
                cpu_retention::enter_sleep_with_retention(config, &saved, retention_buffer)
            }
            _ => config.enter_sleep(),
        };

        if config.is_deep_sleep() && !rejected {
            // The chip is entering deep sleep, and the wake resets it. Because RTC is in a
            // slower clock domain than the CPU, the power-down can take several CPU cycles.
            loop {
                core::hint::spin_loop();
            }
        }

        config.finish_sleep();
        saved.restore();

        rejected
    };

    #[cfg(supports_cpu_power_down)]
    if kind == SleepKind::Light && retention_buffer.is_some() {
        // First, because the other core comes back through the wake stub that the next call
        // disarms.
        #[cfg(all(cpu_retention = "software", multi_core))]
        if app_cpu_saved {
            cpu_retention::rendezvous::resume_app_cpu();
        }

        cpu_retention::finish_cpu_retention(rejected);
    }

    sar_power.replace();

    rejected
}

/// The parks of the other running cores for the length of a light sleep, as a bit for each
/// [`Cpu`](crate::system::Cpu). The drop releases them.
///
/// A parked core holds no lock. Where the retention DMA saves the CPU domain, the hardware brings
/// the parked core back with this one. Where each core saves itself, the parked core saves itself
/// later in the sleep path.
#[cfg(all(multi_core, sleep_driver_supported, feature = "rt"))]
struct ParkedCores(u8);

#[cfg(all(multi_core, sleep_driver_supported, feature = "rt"))]
impl ParkedCores {
    /// Parks the other running cores for a light sleep. A deep sleep resets the chip, so it parks
    /// no core.
    #[inline(always)]
    fn new(kind: SleepKind) -> Self {
        let mut parked = 0;
        if kind == SleepKind::Light {
            // The handler of the workaround waits for the parked core.
            #[cfg(cache_livelock_workaround)]
            crate::soc::cache_livelock::pause();

            for cpu in crate::system::Cpu::other() {
                if crate::system::multi_core::park(cpu) {
                    parked |= 1 << cpu as u8;
                }
            }

            #[cfg(cache_livelock_workaround)]
            if parked == 0 {
                crate::soc::cache_livelock::resume();
            }
        }
        Self(parked)
    }
}

#[cfg(all(multi_core, sleep_driver_supported, feature = "rt"))]
impl Drop for ParkedCores {
    #[inline(always)]
    fn drop(&mut self) {
        if self.0 == 0 {
            return;
        }
        for cpu in crate::system::Cpu::other() {
            if self.0 & (1 << cpu as u8) != 0 {
                crate::system::multi_core::unpark(cpu);
            }
        }
        #[cfg(cache_livelock_workaround)]
        crate::soc::cache_livelock::resume();
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
#[inline(always)]
pub(crate) fn wait_for_sleep_result() -> bool {
    loop {
        cfg_select! {
            soc_has_pmu => {
                let int_raw = crate::peripherals::PMU::regs().int_raw().read();
                let rejected = int_raw.soc_sleep_reject().bit_is_set();
                let wakeup = int_raw.soc_wakeup().bit_is_set();
            }
            _ => {
                let int_raw = LPWR::regs().int_raw().read();
                let rejected = int_raw.slp_reject().bit_is_set();
                let wakeup = int_raw.slp_wakeup().bit_is_set();
            }
        }

        if wakeup || rejected {
            return rejected;
        }
    }
}

/// Suspends the transmitter of every UART with a running function clock. Returns a mask with one
/// bit per suspended instance.
///
/// See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_sleep/sleep_uart.c#L95-L131>.
#[cfg(sleep_driver_supported)]
fn sleep_uart_prepare() -> u32 {
    use crate::{soc::clocks::ClockTree, uart::Instance};
    ClockTree::with(|clocks| {
        let mut suspended = 0;
        for_each_uart! {
            ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident, wakeup_source = $_:literal) => {
                if unsafe { crate::peripherals::$inst::steal() }.info().suspend_for_sleep(clocks) {
                    suspended |= 1 << $id;
                }
            };
        }
        suspended
    })
}

/// The SAR ADC power override, as one 2-bit `force_xpd_sar` mode per SAR unit.
///
/// The ADC drivers and the TRNG force the SAR on. The override survives a light sleep, and also a
/// deep sleep where it lives in the RTC or LP domain.
#[cfg(sleep_driver_supported)]
#[derive(Clone, Copy)]
struct SarPower {
    sar1: u8,
    #[cfg(any(esp32p4, esp32s31))]
    sar2: u8,
    /// The drive of the radio power detector, which shares the SAR.
    #[cfg(adc_sar_power_override)]
    pwdet_drv: bool,
}

#[cfg(sleep_driver_supported)]
impl SarPower {
    const FSM: u8 = 0;
    const OFF: u8 = 0b10;

    /// Returns the override for a sleep.
    ///
    /// The FSM setting powers the SAR down when nothing samples, and leaves it to a ULP program.
    /// On the ESP32-C2, ESP32-C3 and ESP32-P4 it hands the SAR to the digital controller, which the
    /// ADC driver or the TRNG forces on, so a light sleep forces the SAR off. A deep sleep keeps
    /// the FSM setting, because the override survives the wake and not every driver that uses the
    /// SAR writes it.
    fn for_sleep(kind: SleepKind) -> Self {
        let mode = match kind {
            SleepKind::Light if cfg!(any(esp32c2, esp32c3, esp32p4)) => Self::OFF,
            _ => Self::FSM,
        };

        Self {
            sar1: mode,
            #[cfg(any(esp32p4, esp32s31))]
            sar2: mode,
            #[cfg(adc_sar_power_override)]
            pwdet_drv: false,
        }
    }

    /// Writes the override, and returns the previous one.
    fn replace(self) -> Self {
        cfg_select! {
            adc_sar_power_override => {
                let mut previous = self;
                // A dropped ADC driver leaves the override set and the bus clock gated, which drops
                // this access. A peripheral guard would reset the peripheral.
                let bus_clock = crate::peripherals::SYSTEM::regs().saradc_conf();
                let clocked = bus_clock.read().saradc_reg_clk_en().bit();
                bus_clock.modify(|_, w| w.saradc_reg_clk_en().set_bit());

                crate::peripherals::APB_SARADC::regs()
                    .ctrl()
                    .modify(|r, w| {
                        previous.sar1 = r.xpd_sar_force().bits();
                        previous.pwdet_drv = r.saradc2_pwdet_drv().bit();
                        unsafe { w.xpd_sar_force().bits(self.sar1) };
                        w.saradc2_pwdet_drv().bit(self.pwdet_drv)
                    });

                bus_clock.modify(|_, w| w.saradc_reg_clk_en().bit(clocked));
                previous
            }
            esp32s31 => {
                let mut previous = self;
                let regs = crate::peripherals::APB_SARADC::regs();
                regs.ctrl0().modify(|r, w| {
                    previous.sar1 = r.xpd_sar1_force().bits();
                    unsafe { w.xpd_sar1_force().bits(self.sar1) }
                });
                regs.ctrl1().modify(|r, w| {
                    previous.sar2 = r.xpd_sar2_force().bits();
                    unsafe { w.xpd_sar2_force().bits(self.sar2) }
                });
                previous
            }
            esp32p4 => {
                let mut previous = self;
                crate::peripherals::LP_ADC::regs()
                    .force_wpd_sar()
                    .modify(|r, w| {
                        previous.sar1 = r.force_xpd_sar1().bits();
                        previous.sar2 = r.force_xpd_sar2().bits();
                        unsafe {
                            w.force_xpd_sar1().bits(self.sar1);
                            w.force_xpd_sar2().bits(self.sar2)
                        }
                    });
                previous
            }
            any(esp32, esp32s2, esp32s3, esp32c2, esp32c3) => {
                let mut previous = self;
                let reg = cfg_select! {
                    esp32 => crate::peripherals::SENS::regs().sar_meas_wait2(),
                    any(esp32s2, esp32s3) => crate::peripherals::SENS::regs().sar_power_xpd_sar(),
                    esp32c2 => LPWR::regs().cntl_sensor_ctrl(),
                    esp32c3 => LPWR::regs().sensor_ctrl(),
                };
                reg.modify(|r, w| {
                    previous.sar1 = r.force_xpd_sar().bits();
                    unsafe { w.force_xpd_sar().bits(self.sar1) }
                });
                previous
            }
            _ => {
                // No driver forces the SAR on, so there is no override to write.
                let _ = self.sar1;
                self
            }
        }
    }
}

/// Resumes the UARTs that [`sleep_uart_prepare`] suspended.
#[cfg(sleep_driver_supported)]
fn sleep_uart_resume(suspended: u32) {
    use crate::{soc::clocks::ClockTree, uart::Instance};
    ClockTree::with(|clocks| {
        for_each_uart! {
            ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident, wakeup_source = $_:literal) => {
                if suspended & (1 << $id) != 0 {
                    unsafe { crate::peripherals::$inst::steal() }.info().resume_from_sleep(clocks);
                }
            };
        }
    });
}
