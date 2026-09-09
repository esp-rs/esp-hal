//! Power management utilities.

#[cfg(supports_tagmem_power_down)]
use esp_hal::rtc_cntl::{CacheTagRetentionMemory, CacheTagRetentionMemoryError};
#[cfg(supports_cpu_power_down)]
use esp_hal::rtc_cntl::{CpuRetentionMemory, CpuRetentionMemoryError};
#[cfg(multi_core)]
use esp_hal::{peripherals::CPU_CTRL, system::Cpu, system::CpuControl};
use esp_hal::{
    peripherals::LPWR,
    rtc_cntl::{
        WakeLock,
        sleep::{LowPower, RtcSleepConfig},
    },
    time::{Duration, Instant},
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

#[cfg(supports_cpu_power_down)]
impl Sleep {
    /// Lets automatic light sleep power the CPU domain down.
    ///
    /// A light sleep keeps the CPU state in `memory` while the CPU domain has no power, which
    /// draws less current than a light sleep that keeps the domain powered. The saving costs the
    /// memory, and it adds the save and the restore to the sleep and the wake.
    ///
    /// Every light sleep after this call powers the CPU domain down. Nothing turns it off again,
    /// because the memory stays with the driver for the life of the program.
    ///
    /// # Errors
    ///
    /// Returns [`CpuRetentionMemoryError`] if a buffer is installed already, or if `memory` is
    /// outside the range that the retention hardware reaches. See
    /// [`LowPower::install_cpu_retention_memory`] for how to place the static.
    pub fn enable_cpu_powerdown(
        &mut self,
        memory: &'static mut CpuRetentionMemory,
    ) -> Result<(), CpuRetentionMemoryError> {
        // `configure` took `LPWR`, so this is the only driver, and the idle hook takes it the same
        // way for every sleep.
        LowPower::new(unsafe { LPWR::steal() }).install_cpu_retention_memory(memory)
    }

    /// Keeps the cache tag memory across a light sleep that powers the CPU domain down.
    ///
    /// This shortens the wake, because the caches keep what they held before the sleep. Without
    /// it, the wake path invalidates both caches, and the code that runs next takes a miss for
    /// every line it needs. The memory it costs buys latency only, never correctness.
    ///
    /// This does nothing on its own: the tag memory is lost only when the CPU domain powers down,
    /// which needs [`enable_cpu_powerdown`][Self::enable_cpu_powerdown].
    ///
    /// # Errors
    ///
    /// Returns [`CacheTagRetentionMemoryError`] if a buffer is installed already, or if `memory`
    /// is outside the range that the retention hardware reaches.
    #[cfg(supports_tagmem_power_down)]
    pub fn keep_cache_tags(
        &mut self,
        memory: &'static mut CacheTagRetentionMemory,
    ) -> Result<(), CacheTagRetentionMemoryError> {
        LowPower::new(unsafe { LPWR::steal() }).install_cache_tag_retention_memory(memory)
    }
}

/// A handle you can use to enter deep sleep.
#[cfg(sleep_deep_sleep)]
pub struct DeepSleep {
    lpwr: LPWR<'static>,
}

#[cfg(sleep_deep_sleep)]
impl DeepSleep {
    /// Puts the system into deep sleep.
    ///
    /// The sleep ends when one of the wakeup sources that the drivers enabled becomes active. The
    /// wake from deep sleep resets the chip, so this function does not return.
    ///
    /// # Panics
    ///
    /// Panics if no wakeup source is enabled, because nothing could end the sleep.
    pub fn deep_sleep(&mut self) -> ! {
        let mut lpwr = LowPower::new(self.lpwr.reborrow());
        lpwr.sleep_deep(RtcSleepConfig::deep())
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
/// Light sleep keeps the CPU domain powered, unless the program calls
/// [`Sleep::enable_cpu_powerdown`].
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

            let mut lpwr = LowPower::new(unsafe { LPWR::steal() });

            // The deadline stays until other code clears it, so each pass writes it, also a pass
            // that makes no sleep. A deadline from an earlier pass expires, and the
            // next sleep then returns immediately.
            if next_wakeup == u64::MAX {
                lpwr.clear_wakeup_deadline();
            } else {
                lpwr.set_wakeup_deadline(Instant::EPOCH + Duration::from_micros(next_wakeup));

                if next_wakeup.saturating_sub(crate::now()) < LIGHT_SLEEP_MIN_US {
                    return;
                }
            }

            // We have committed to sleeping. Park (hardware-stall) the other core(s) so their
            // CPU state is frozen and restored coherently across the sleep, then enter light
            // sleep, then thaw them.
            cfg_select! {
                multi_core => {
                    let mut cpu_control = CpuControl::new(unsafe { CPU_CTRL::steal() });
                    for cpu in Cpu::other() {
                        if scheduler.active_cores.contains(cpu) {
                            unsafe { cpu_control.park_core(cpu) };
                            // A stall is enough where the retention DMA saves the CPU domain,
                            // because the domain holds both cores and the hardware brings the
                            // parked core back with this one. esp-idf stalls the other core the
                            // same way and adds nothing more (`sleep_modes.c`, where the SMP
                            // retention work is gated on `SOC_PM_CPU_RETENTION_BY_SW`).
                            // FIXME: the chips that retain the CPU in software need more: each
                            // core saves itself, so the other core must park in a known place
                            // and save its state there.
                        }
                    }
                }
                _ => {}
            }

            // The driver of each other wakeup source enables it. A listening pin wakes the chip
            // because it listens, and this hook cannot know which pins listen. If no source is
            // enabled, the call refuses the sleep and returns immediately. The code then reaches
            // the same `WFI` that this hook would select.
            lpwr.sleep_light(RtcSleepConfig::default());

            // The alarm timer was gated during light sleep, so its pre-armed alarm is
            // stale. Force a re-arm against the restored time base so the tick handler
            // fires promptly and drains the timer queue.
            time_driver.rearm(crate::now());

            // Trigger the scheduler on the other core to prevent it from putting
            // the system back to sleep immediately.
            #[cfg(multi_core)]
            for cpu in Cpu::other() {
                if scheduler.active_cores.contains(cpu) {
                    cpu_control.unpark_core(cpu);
                    task::trigger_scheduler(RunSchedulerOn::RunOnCore(cpu));
                }
            }
        });

        esp_hal::interrupt::wait_for_interrupt();
    }
}
