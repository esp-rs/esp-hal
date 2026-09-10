//! Power management utilities.

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
/// See [`WakeLock`] for the wake-lock contract that governs when sleeping is safe.
///
/// [`start_with_idle_hook`]: crate::start_with_idle_hook
/// When set, the automatic light-sleep idle hook keeps the main XTAL powered
/// (`RtcSleepConfig::xtal_fpu`). Needed only when a BLE connection sleeps on the
/// MAIN_XTAL sleep clock (no external 32.768 kHz crystal), where the controller
/// needs the XTAL running to keep connection-event timing. Default off.
static KEEP_MAIN_XTAL_PU: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Opt into keeping the main XTAL powered during automatic light sleep (see
/// [`KEEP_MAIN_XTAL_PU`]). Call once at boot, before the idle hook can run.
pub fn set_main_xtal_powered_in_light_sleep(on: bool) {
    KEEP_MAIN_XTAL_PU.store(on, core::sync::atomic::Ordering::Relaxed);
}

/// Index into [`sleep_diag`]: passes refused because a wake lock was held.
pub const DIAG_GATE_WAKELOCK: usize = 0;
/// Index into [`sleep_diag`]: passes refused because a sleep deadline had passed.
pub const DIAG_GATE_DEADLINE: usize = 1;
/// Index into [`sleep_diag`]: passes refused because a core or the run queue was busy.
pub const DIAG_GATE_NOT_IDLE: usize = 2;
/// Index into [`sleep_diag`]: passes refused because the next wakeup was too near.
pub const DIAG_GATE_TOO_SOON: usize = 3;
/// Index into [`sleep_diag`]: light sleeps entered.
pub const DIAG_SLEPT: usize = 4;
/// Index into [`sleep_diag`]: microseconds spent in light sleep.
pub const DIAG_SLEPT_US: usize = 5;

/// Index into [`sleep_diag`]: first of four microsecond totals, one per gate in the same
/// order as the gate counters, attributing the time between idle passes to the gate that
/// refused the previous pass.
pub const DIAG_GATE_US: usize = 6;

static DIAG: [portable_atomic::AtomicU64; 10] =
    [const { portable_atomic::AtomicU64::new(0) }; 10];
static DIAG_LAST_PASS_US: portable_atomic::AtomicU64 = portable_atomic::AtomicU64::new(0);
static DIAG_LAST_GATE: portable_atomic::AtomicUsize = portable_atomic::AtomicUsize::new(usize::MAX);

fn diag_count(idx: usize) {
    DIAG[idx].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if idx < DIAG_SLEPT {
        let now = crate::now();
        let last = DIAG_LAST_PASS_US.swap(now, core::sync::atomic::Ordering::Relaxed);
        let prev = DIAG_LAST_GATE.swap(idx, core::sync::atomic::Ordering::Relaxed);
        if prev < DIAG_SLEPT && last != 0 {
            DIAG[DIAG_GATE_US + prev]
                .fetch_add(now.saturating_sub(last), core::sync::atomic::Ordering::Relaxed);
        }
    } else {
        DIAG_LAST_GATE.store(usize::MAX, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Snapshot of the light-sleep idle hook's gate/sleep counters, indexed by the `DIAG_*`
/// constants. Diagnostic aid for judging why the chip does or does not sleep.
pub fn sleep_diag() -> [u64; 10] {
    core::array::from_fn(|i| DIAG[i].load(core::sync::atomic::Ordering::Relaxed))
}

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
                diag_count(DIAG_GATE_WAKELOCK);
                return;
            }

            // A driver that released its wake lock for a bounded gap sets a deadline; past it
            // the chip must stay awake until the driver takes its lock back.
            let deadline = WakeLock::sleep_deadline().map(|d| d.duration_since_epoch().as_micros());
            if let Some(deadline) = deadline
                && crate::now() >= deadline
            {
                diag_count(DIAG_GATE_DEADLINE);
                return;
            }

            #[cfg(multi_core)]
            {
                if scheduler.run_queue.has_ready_tasks() {
                    diag_count(DIAG_GATE_NOT_IDLE);
                    return;
                }
                for cpu in Cpu::all() {
                    if !scheduler.cpu_idle(cpu) {
                        diag_count(DIAG_GATE_NOT_IDLE);
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
            let next_wakeup = match deadline {
                Some(d) => next_wakeup.min(d),
                None => next_wakeup,
            };

            let mut lpwr = LowPower::new(unsafe { LPWR::steal() });

            // The deadline stays until other code clears it, so each pass writes it, also a pass
            // that makes no sleep. A deadline from an earlier pass expires, and the
            // next sleep then returns immediately.
            if next_wakeup == u64::MAX {
                lpwr.clear_wakeup_deadline();
            } else {
                lpwr.set_wakeup_deadline(Instant::EPOCH + Duration::from_micros(next_wakeup));

                if next_wakeup.saturating_sub(crate::now()) < LIGHT_SLEEP_MIN_US {
                    diag_count(DIAG_GATE_TOO_SOON);
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
                            // FIXME: this is insufficient when we power down the CPU - we will
                            // need to force the other core to be parked in a known place, saving
                            // its state so we can restore it after wakeup.
                        }
                    }
                }
                _ => {}
            }

            // The driver of each other wakeup source enables it. A listening pin wakes the chip
            // because it listens, and this hook cannot know which pins listen. If no source is
            // enabled, the call refuses the sleep and returns immediately. The code then reaches
            // the same `WFI` that this hook would select.
            // Keep the main XTAL powered only when a board opts in (the no-32k-crystal
            // connected-BLE path). Default off powers it down for a lower sleep floor.
            let mut cfg = RtcSleepConfig::default();
            if KEEP_MAIN_XTAL_PU.load(core::sync::atomic::Ordering::Relaxed) {
                cfg.set_xtal_fpu(true);
            }
            let before = crate::now();
            lpwr.sleep_light(cfg);
            diag_count(DIAG_SLEPT);
            DIAG[DIAG_SLEPT_US].fetch_add(
                crate::now().saturating_sub(before),
                core::sync::atomic::Ordering::Relaxed,
            );

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
