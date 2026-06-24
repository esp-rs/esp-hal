use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{
        Rtc,
        WakeLock,
        sleep::{GpioWakeupSource, TimerWakeupSource},
    },
    system::Cpu,
};

use crate::{SCHEDULER, task::IdleFn};

const LIGHT_SLEEP_MIN_US: u64 =
    esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US") as u64;

/// Builds an idle hook that automatically enters light sleep when the system is
/// idle.
///
/// Pass the returned hook to [`start_with_idle_hook`].
///
/// Each time the scheduler runs out of ready tasks, the hook (with interrupts
/// disabled) checks that:
/// - no [`WakeLock`] is held,
/// - this is the primary core and no second core scheduler is running,
/// - there is a finite next scheduled wakeup, and
/// - that wakeup is at least `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` microseconds away.
///
/// If all hold, it calls [`Rtc::light_sleep`] for the next wakeup; otherwise
/// it falls back to `WFI`. The minimum-residency threshold is configurable via the
/// `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` build-time option (default `1000`).
///
/// # Limitations
///
/// - **Single-core only.** On a system with a started second core, the hook degrades to `WFI` and
///   never puts the chip to sleep.
///
/// See [`WakeLock`] for the wake-lock contract that governs when sleeping is safe.
///
/// [`start_with_idle_hook`]: crate::start_with_idle_hook
/// [`Rtc::light_sleep`]: esp_hal::rtc_cntl::Rtc::light_sleep
pub fn auto_light_sleep() -> IdleFn {
    auto_light_sleep_hook
}

extern "C" fn auto_light_sleep_hook() -> ! {
    loop {
        SCHEDULER.with(|scheduler| {
            if WakeLock::is_active() {
                return;
            }

            // MVP: only commit to sleep on the primary core, and never while a second
            // core scheduler is running (it would keep running while the chip sleeps).
            if Cpu::current() != Cpu::ProCpu {
                return;
            }
            #[cfg(multi_core)]
            if scheduler.per_cpu[1].initialized {
                return;
            }

            let Some(time_driver) = scheduler.time_driver.as_mut() else {
                return;
            };
            let next_wakeup = time_driver.next_wakeup();
            if next_wakeup == u64::MAX {
                return;
            }

            let now = crate::now();
            let sleep_duration = next_wakeup.saturating_sub(now);
            if sleep_duration < LIGHT_SLEEP_MIN_US {
                return;
            }

            unsafe {
                let mut rtc = Rtc::new(LPWR::steal());
                let timer =
                    TimerWakeupSource::new(core::time::Duration::from_micros(sleep_duration));
                let gpio = GpioWakeupSource::new();
                rtc.sleep_light(&[&timer, &gpio]);
            }

            // The alarm timer was gated during light sleep, so its pre-armed alarm is
            // stale. Force a re-arm against the restored time base so the tick handler
            // fires promptly and drains the timer queue.
            time_driver.rearm(crate::now());
        });

        esp_hal::interrupt::wait_for_interrupt();
    }
}
