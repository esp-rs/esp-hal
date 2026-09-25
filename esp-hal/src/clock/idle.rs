#[cfg(idle_frequency_scaling)]
use portable_atomic::{AtomicBool, AtomicUsize, Ordering};

#[cfg(idle_frequency_scaling)]
static LOCK_COUNT: AtomicUsize = AtomicUsize::new(0);

/// A guard that keeps the CPU clock unchanged while the CPU waits for an interrupt.
///
/// When the `ESP_HAL_CONFIG_IDLE_FREQUENCY_SCALING` option is enabled,
/// [`wait_for_interrupt`](crate::interrupt::wait_for_interrupt) switches the CPU clock to
/// XTAL_CLK until an interrupt occurs. The CPU clock returns to its configured source before the
/// interrupt handler runs, which increases the interrupt latency. While at least one
/// `CpuFrequencyLock` is held, the CPU clock does not change. The lock is released when the guard
/// is dropped.
///
/// When the option is disabled, or the chip does not support it, `CpuFrequencyLock` does nothing.
#[instability::unstable]
#[non_exhaustive]
pub struct CpuFrequencyLock;

impl CpuFrequencyLock {
    /// Acquires a CPU frequency lock, which is released when it is dropped.
    #[instability::unstable]
    pub fn new() -> Self {
        Self::acquire();
        Self
    }

    /// Acquires a CPU frequency lock.
    #[instability::unstable]
    pub fn acquire() {
        #[cfg(idle_frequency_scaling)]
        LOCK_COUNT.fetch_add(1, Ordering::AcqRel);
    }

    /// Releases a CPU frequency lock.
    ///
    /// Must only be called to release a lock acquired via [`Self::acquire`].
    #[instability::unstable]
    pub fn release() {
        #[cfg(idle_frequency_scaling)]
        {
            let previous = LOCK_COUNT.fetch_sub(1, Ordering::AcqRel);
            debug_assert_ne!(previous, 0, "CPU frequency lock counter underflow");
        }
    }

    /// Returns whether at least one CPU frequency lock is currently held.
    ///
    /// Also returns `true` when the CPU clock never changes, because the option is disabled or
    /// the chip does not support it.
    #[instability::unstable]
    pub fn is_active() -> bool {
        cfg_select! {
            idle_frequency_scaling => LOCK_COUNT.load(Ordering::Acquire) != 0,
            _ => true,
        }
    }
}

#[instability::unstable]
impl Clone for CpuFrequencyLock {
    fn clone(&self) -> Self {
        Self::new()
    }
}

#[instability::unstable]
impl Default for CpuFrequencyLock {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for CpuFrequencyLock {
    fn drop(&mut self) {
        Self::release();
    }
}

// Accessed only while the clock tree is locked.
#[cfg(idle_frequency_scaling)]
static CLOCK_LOWERED: AtomicBool = AtomicBool::new(false);
#[cfg(all(idle_frequency_scaling, multi_core))]
static IDLE_CORES: AtomicUsize = AtomicUsize::new(0);
#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
static PSRAM_LOWERED: AtomicBool = AtomicBool::new(false);

/// Runs `f` while the other running cores are stalled.
///
/// The PSRAM speed change freezes the cache. An access to flash or PSRAM from another core would
/// then block the cache for all cores.
#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
fn with_other_cores_stalled<R>(f: impl FnOnce() -> R) -> R {
    cfg_select! {
        multi_core => {
            use crate::{soc::cpu_control, system::Cpu};

            let stalled = Cpu::other().find(|&cpu| cpu_control::is_running(cpu));
            if let Some(cpu) = stalled {
                unsafe { cpu_control::internal_park_core(cpu, true) };
            }
            let result = f();
            if let Some(cpu) = stalled {
                unsafe { cpu_control::internal_park_core(cpu, false) };
            }
            result
        }
        _ => f(),
    }
}

/// Executes `wfi`, with the CPU clock from XTAL_CLK if possible.
///
/// Interrupts stay disabled until the CPU clock is restored, so that no interrupt handler runs at
/// the lower frequency. A pending interrupt ends `wfi` also while interrupts are disabled.
///
/// The CPU clock is shared by all cores, so it only changes when all running cores wait.
#[cfg(idle_frequency_scaling)]
pub(crate) fn wait_for_interrupt() {
    use crate::soc::clocks::{self, ClockTree};

    // The critical section implementation also raises `mintthresh` on some chips, which would
    // prevent `wfi` from ending. Only clear `mstatus.MIE`.
    let mstatus: usize;
    unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", out(reg) mstatus) };

    ClockTree::with(|tree| {
        #[cfg(multi_core)]
        let all_cores_idle = {
            use crate::system::Cpu;

            let idle_cores = IDLE_CORES.fetch_add(1, Ordering::Relaxed) + 1;
            let running_cores = Cpu::all()
                .filter(|&cpu| crate::soc::cpu_control::is_running(cpu))
                .count();
            idle_cores >= running_cores
        };
        #[cfg(not(multi_core))]
        let all_cores_idle = true;

        if all_cores_idle
            && !CpuFrequencyLock::is_active()
            && !CLOCK_LOWERED.load(Ordering::Relaxed)
            && clocks::cpu_clock_from_pll(tree)
        {
            // PSRAM slows down before the CPU clock, and speeds up after it.
            #[cfg(psram_idle_low_speed_switch)]
            PSRAM_LOWERED.store(
                with_other_cores_stalled(crate::psram::implem::low_speed::enter),
                Ordering::Relaxed,
            );

            clocks::switch_cpu_clock_to_xtal(tree);
            CLOCK_LOWERED.store(true, Ordering::Relaxed);
        }
    });

    unsafe { core::arch::asm!("wfi") };

    ClockTree::with(|tree| {
        #[cfg(multi_core)]
        IDLE_CORES.fetch_sub(1, Ordering::Relaxed);

        if CLOCK_LOWERED.swap(false, Ordering::Relaxed) {
            clocks::restore_cpu_clock(tree);

            #[cfg(psram_idle_low_speed_switch)]
            if PSRAM_LOWERED.swap(false, Ordering::Relaxed) {
                with_other_cores_stalled(crate::psram::implem::low_speed::exit);
            }
        }
    });

    if mstatus & 0b1000 != 0 {
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }
}
