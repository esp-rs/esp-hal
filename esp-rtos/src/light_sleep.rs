#[cfg(multi_core)]
use esp_hal::{peripherals::CPU_CTRL, system::Cpu, system::CpuControl};
use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{
        Rtc,
        WakeLock,
        sleep::{GpioWakeupSource, TimerWakeupSource},
    },
};

use crate::{SCHEDULER, task::IdleFn};
#[cfg(multi_core)]
use crate::{run_queue::RunSchedulerOn, task};

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
/// - all cores are idle,
/// - there is a finite next scheduled wakeup, and
/// - that wakeup is at least `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` microseconds away.
///
/// If all hold, it calls [`Rtc::light_sleep`] for the next wakeup; otherwise
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
            if next_wakeup == u64::MAX {
                return;
            }

            let now = crate::now();
            let sleep_duration = next_wakeup.saturating_sub(now);
            if sleep_duration < LIGHT_SLEEP_MIN_US {
                return;
            }

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
                let mut rtc = Rtc::new(LPWR::steal());
                let timer =
                    TimerWakeupSource::new(esp_hal::time::Duration::from_micros(sleep_duration));
                let gpio = GpioWakeupSource::new();
                rtc.sleep_light(&[&timer, &gpio]);
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
