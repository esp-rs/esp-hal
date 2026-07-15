//! Power management utilities.

#[cfg(multi_core)]
use esp_hal::{peripherals::CPU_CTRL, system::Cpu, system::CpuControl};
use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{
        WakeLock,
        sleep::{GpioWakeupSource, LowPower, TimerWakeupSource, WakeSource},
    },
    time::Duration,
};

use crate::{SCHEDULER, task::IdleFn};
#[cfg(multi_core)]
use crate::{run_queue::RunSchedulerOn, task};

const LIGHT_SLEEP_MIN_US: u64 =
    esp_config::esp_config_int!(u32, "ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US") as u64;

/// Sleep handles.
pub struct Sleep {
    /// The handle that allows you to enter deep sleep.
    #[cfg(sleep_deep_sleep)]
    pub deep_sleep: DeepSleep,

    /// The idle hook to use for light sleep.
    pub light_sleep_hook: IdleFn,
}

/// A handle you can use to enter deep sleep.
#[cfg(sleep_deep_sleep)]
pub struct DeepSleep {
    lpwr: LPWR<'static>,
}

#[cfg(sleep_deep_sleep)]
impl DeepSleep {
    /// Puts the system into deep sleep, waking up from the specified wake sources.
    pub fn deep_sleep(&mut self, wake_sources: &[&dyn WakeSource]) -> ! {
        let mut lpwr = LowPower::new(self.lpwr.reborrow());
        lpwr.sleep_deep(wake_sources)
    }
}

/// Creates resources for managing light/deep sleep with `esp-rtos`.
///
/// The returned [`Sleep`] struct contains the idle hook and a deep sleep handle,
/// if deep sleep is supported.
///
/// Pass the idle hook to [`start_with_idle_hook`] to enable automatic light sleep.
///
/// Each time the scheduler runs out of ready tasks, the hook (with interrupts
/// disabled) checks that:
/// - no [`WakeLock`] is held,
/// - all cores are idle,
/// - the next wakeup is at least `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` microseconds away.
///
/// If all hold, it calls [`LowPower::sleep_light`] for the next wakeup; otherwise
/// it falls back to `WFI`. The minimum-residency threshold is configurable via the
/// `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` build-time option (default `1000`).
///
/// On multi-core chips, the core that commits to sleep hardware-stalls the other core(s)
/// for the duration of the sleep so their CPU state is frozen and restored coherently,
/// then thaws them on wakeup.
///
/// See [`WakeLock`] for the wake-lock contract that governs when sleeping is safe.
///
/// [`start_with_idle_hook`]: crate::start_with_idle_hook
pub fn configure(lpwr: LPWR<'static>) -> Sleep {
    Sleep {
        #[cfg(sleep_deep_sleep)]
        deep_sleep: DeepSleep { lpwr },
        light_sleep_hook: auto_light_sleep_hook,
    }
}

extern "C" fn auto_light_sleep_hook() -> ! {
    loop {
        // ESP32-P4 HP wakeup handling is coordinated by the primary core. If
        // AppCpu enters sleep after parking ProCpu, an HP peripheral wakeup
        // such as GPIO cannot resume the parked primary core.
        #[cfg(all(multi_core, esp32p4))]
        if Cpu::current() == Cpu::AppCpu {
            // Kick the other core so that it can put the system to sleep.
            task::trigger_scheduler(RunSchedulerOn::RunOnCore(Cpu::ProCpu));
            esp_hal::interrupt::wait_for_interrupt();
            continue;
        }

        SCHEDULER.with(|scheduler| {
            if WakeLock::is_active() {
                return;
            }

            #[cfg(multi_core)]
            {
                if scheduler.run_queue.has_ready_tasks() {
                    return;
                }
                for cpu in Cpu::all() {
                    if !scheduler.cpu_idle(cpu) {
                        return;
                    }
                }

                // All cores are ready to sleep. Since we are here in a critical section,
                // the other core must be waiting for the scheduler lock. We will go to sleep,
                // and after wakeup the other core will reattempt this check.

                // FIXME: We hardware-stall the other core(s) for the duration of the sleep so
                // their CPU state is frozen and restored coherently. The other core is frozen
                // wherever it happens to be - including in the middle of an interrupt handler
                // that holds a cross-core lock (e.g. the clock tree, peripheral refcount, or
                // UART locks taken by the light-sleep enter/exit path in `Rtc::sleep`). If that
                // happens, this core will spin forever trying to take that lock during sleep
                // prep, because the frozen core can never release it. We accept this (unlikely)
                // deadlock risk for now rather than ordering all lock-taking work before the
                // stall.
            }

            let Some(time_driver) = scheduler.time_driver.as_mut() else {
                return;
            };
            let next_wakeup = time_driver.next_wakeup();

            let sleep_duration = if next_wakeup != u64::MAX {
                let now = crate::now();
                let sleep_duration = next_wakeup.saturating_sub(now);
                if sleep_duration < LIGHT_SLEEP_MIN_US {
                    return;
                }
                Some(Duration::from_micros(sleep_duration))
            } else {
                None
            };

            // We have committed to sleeping. Park (hardware-stall) the other core(s) so their
            // CPU state is frozen and restored coherently across the sleep, then enter light
            // sleep, then thaw them.
            cfg_select! {
                multi_core => {
                    let mut cpu_control = CpuControl::new(unsafe { CPU_CTRL::steal() });
                    for cpu in Cpu::other() {
                        if scheduler.per_cpu[cpu as usize].initialized {
                            unsafe { cpu_control.park_core(cpu) };
                            // FIXME: this is insufficient when we power down the CPU - we will
                            // need to force the other core to be parked in a known place, saving
                            // its state so we can restore it after wakeup.
                        }
                    }
                }
                _ => {}
            }

            unsafe {
                let mut lpwr = LowPower::new(LPWR::steal());

                // Set up wake sources. We could use heapless here, but this
                // code should be replaced by something more flexible anyway.
                let gpio = GpioWakeupSource::new();
                let timer;

                let mut wakeup_sources: [&dyn WakeSource; 2] = [&gpio, &gpio];

                if let Some(duration) = sleep_duration {
                    timer = TimerWakeupSource::new(duration);
                    wakeup_sources[0] = &timer;
                }

                lpwr.sleep_light(&wakeup_sources);
            }

            // The alarm timer was gated during light sleep, so its pre-armed alarm is
            // stale. Force a re-arm against the restored time base so the tick handler
            // fires promptly and drains the timer queue.
            time_driver.rearm(crate::now());

            // Trigger the scheduler on the other core to prevent it from putting
            // the system back to sleep immediately.
            #[cfg(multi_core)]
            for cpu in Cpu::other() {
                if scheduler.per_cpu[cpu as usize].initialized {
                    cpu_control.unpark_core(cpu);
                    task::trigger_scheduler(RunSchedulerOn::RunOnCore(cpu));
                }
            }
        });

        esp_hal::interrupt::wait_for_interrupt();
    }
}
