#[cfg(idle_frequency_scaling)]
mod implem {
    use portable_atomic::{AtomicBool, AtomicUsize, Ordering};
    static LOCK_COUNT: AtomicUsize = AtomicUsize::new(0);

    // Accessed only while the clock tree is locked.
    static CLOCK_LOWERED: AtomicBool = AtomicBool::new(false);
    #[cfg(multi_core)]
    static IDLE_CORES: AtomicUsize = AtomicUsize::new(0);

    pub(super) fn acquire() {
        LOCK_COUNT.fetch_add(1, Ordering::AcqRel);
    }

    pub(super) fn release() {
        let previous = LOCK_COUNT.fetch_sub(1, Ordering::AcqRel);
        debug_assert_ne!(previous, 0, "CPU frequency lock counter underflow");
    }

    /// Executes `wfi`, with the CPU clock from XTAL_CLK if possible.
    ///
    /// Interrupts stay disabled until the CPU clock is restored, so that no interrupt handler runs
    /// at the lower frequency. A pending interrupt ends `wfi` also while interrupts are disabled.
    ///
    /// The CPU clock is shared by all cores, so it only changes when all running cores wait.
    pub(crate) fn wait_for_interrupt() {
        use crate::soc::clocks::{self, ClockTree};

        // The critical section implementation also raises `mintthresh` on some chips, which would
        // prevent `wfi` from ending. Only clear `mstatus.MIE`.
        let mstatus: usize;
        unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", out(reg) mstatus) };

        ClockTree::with(|tree| {
            let all_cores_idle = cfg_select! {
                multi_core => {{
                    use crate::system::Cpu;

                    let idle_cores = IDLE_CORES.fetch_add(1, Ordering::Relaxed) + 1;
                    let running_cores = Cpu::all()
                        .filter(|&cpu| crate::soc::cpu_control::is_running(cpu))
                        .count();
                    idle_cores >= running_cores
                }}
                _ => true,
            };

            if all_cores_idle && LOCK_COUNT.load(Ordering::Acquire) == 0 {
                CLOCK_LOWERED.store(clocks::switch_cpu_clock_to_xtal(tree), Ordering::Relaxed);
            }
        });

        unsafe { core::arch::asm!("wfi") };

        ClockTree::with(|tree| {
            #[cfg(multi_core)]
            IDLE_CORES.fetch_sub(1, Ordering::Relaxed);

            if CLOCK_LOWERED.swap(false, Ordering::Relaxed) {
                clocks::restore_cpu_clock(tree);
            }
        });

        if mstatus & 0b1000 != 0 {
            unsafe { core::arch::asm!("csrsi mstatus, 8") };
        }
    }
}

#[cfg(idle_frequency_scaling)]
pub(crate) use implem::wait_for_interrupt;

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
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct CpuFrequencyLock;

impl CpuFrequencyLock {
    /// Acquires a CPU frequency lock, which is released when it is dropped.
    #[instability::unstable]
    #[inline]
    pub fn new() -> Self {
        Self::acquire();
        Self
    }

    /// Acquires a CPU frequency lock.
    #[instability::unstable]
    #[inline]
    pub fn acquire() {
        #[cfg(idle_frequency_scaling)]
        implem::acquire();
    }

    /// Releases a CPU frequency lock.
    ///
    /// Must only be called to release a lock acquired via [`Self::acquire`].
    #[instability::unstable]
    #[inline]
    pub fn release() {
        #[cfg(idle_frequency_scaling)]
        implem::release();
    }
}

#[instability::unstable]
impl Clone for CpuFrequencyLock {
    #[inline]
    fn clone(&self) -> Self {
        Self::new()
    }
}

#[instability::unstable]
impl Default for CpuFrequencyLock {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for CpuFrequencyLock {
    #[inline]
    fn drop(&mut self) {
        Self::release();
    }
}
