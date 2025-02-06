//! Multicore-aware thread-mode embassy executor.

use core::marker::PhantomData;

use embassy_executor::Spawner;
#[cfg(all(low_power_wait, multi_core))]
use esp_hal::interrupt::software::SoftwareInterrupt;
use esp_hal::{interrupt::Priority, system::Cpu};
#[cfg(low_power_wait)]
use portable_atomic::{AtomicBool, Ordering};

use super::InnerExecutor;

pub(crate) const THREAD_MODE_CONTEXT: usize = 16;

/// global atomic used to keep track of whether there is work to do since sev()
/// is not available on either Xtensa or RISC-V
#[cfg(low_power_wait)]
static SIGNAL_WORK_THREAD_MODE: [AtomicBool; Cpu::COUNT] =
    [const { AtomicBool::new(false) }; Cpu::COUNT];

pub(crate) fn pend_thread_mode(_core: usize) {
    #[cfg(low_power_wait)]
    {
        // Signal that there is work to be done.
        SIGNAL_WORK_THREAD_MODE[_core].store(true, Ordering::Relaxed);

        // If we are pending a task on the current core, we're done. Otherwise, we
        // need to make sure the other core wakes up.
        #[cfg(multi_core)]
        if _core != Cpu::current() as usize {
            // We need to clear the interrupt from software. We don't actually
            // need it to trigger and run the interrupt handler, we just need to
            // kick waiti to return.
            unsafe { SoftwareInterrupt::<3>::steal().raise() };
        }
    }
}

/// Thread mode executor.
///
/// This is the simplest and most common kind of executor. It runs on thread
/// mode (at the lowest priority level).
#[cfg_attr(multi_core, doc = "")]
#[cfg_attr(
    multi_core,
    doc = "This executor is safe to use on multiple cores. You need to
create one instance per core. The executors don't steal tasks from each other."
)]
pub struct Executor {
    inner: InnerExecutor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create a new Executor.
    #[cfg_attr(
        multi_core,
        doc = r#"

This will use software-interrupt 3 which isn't available for anything else to wake the other core(s)."#
    )]
    pub fn new() -> Self {
        Self {
            inner: InnerExecutor::new(
                Priority::Priority1,
                (THREAD_MODE_CONTEXT + Cpu::current() as usize) as *mut (),
            ),
            not_send: PhantomData,
        }
    }

    /// Run the executor.
    ///
    /// The `init` closure is called with a [`Spawner`] that spawns tasks on
    /// this executor. Use it to spawn the initial task(s). After `init`
    /// returns, the executor starts running the tasks.
    ///
    /// To spawn more tasks later, you may keep copies of the [`Spawner`] (it is
    /// `Copy`), for example by passing it as an argument to the initial
    /// tasks.
    ///
    /// This function requires `&'static mut self`. This means you have to store
    /// the Executor instance in a place where it'll live forever and grants
    /// you mutable access. There's a few ways to do this:
    ///
    /// - a [StaticCell](https://docs.rs/static_cell/latest/static_cell/) (safe)
    /// - a `static mut` (unsafe, not recommended)
    /// - a local variable in a function you know never returns (like `fn main()
    ///   -> !`), upgrading its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        unwrap!(esp_hal::interrupt::enable(
            esp_hal::peripherals::Interrupt::FROM_CPU_INTR3,
            Priority::min(),
        ));

        self.inner.init();

        init(self.inner.inner.spawner());

        #[cfg(low_power_wait)]
        let cpu = Cpu::current() as usize;

        loop {
            unsafe { self.inner.inner.poll() };

            #[cfg(low_power_wait)]
            Self::wait_impl(cpu);
        }
    }

    #[cfg(all(xtensa, low_power_wait))]
    fn wait_impl(cpu: usize) {
        // Manual critical section implementation that only masks interrupts handlers.
        // We must not acquire the cross-core on dual-core systems because that would
        // prevent the other core from doing useful work while this core is sleeping.
        let token: critical_section::RawRestoreState;
        unsafe { core::arch::asm!("rsil {0}, 5", out(reg) token) };

        // we do not care about race conditions between the load and store operations,
        // interrupts will only set this value to true.
        // Acquire makes no sense but at this time it's slightly faster than Relaxed.
        if SIGNAL_WORK_THREAD_MODE[cpu].load(Ordering::Acquire) {
            // if there is work to do, exit critical section and loop back to polling
            unsafe {
                core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync",
                    in(reg) token
                );
            }
        } else {
            // `waiti` sets the `PS.INTLEVEL` when slipping into sleep because critical
            // sections in Xtensa are implemented via increasing `PS.INTLEVEL`.
            // The critical section ends here. Take care not add code after
            // `waiti` if it needs to be inside the CS.
            // Do not lower INTLEVEL below the current value.
            match token & 0x0F {
                0 => unsafe { core::arch::asm!("waiti 0") },
                1 => unsafe { core::arch::asm!("waiti 1") },
                2 => unsafe { core::arch::asm!("waiti 2") },
                3 => unsafe { core::arch::asm!("waiti 3") },
                4 => unsafe { core::arch::asm!("waiti 4") },
                _ => unsafe { core::arch::asm!("waiti 5") },
            }
        }
        // If this races and some waker sets the signal, we'll reset it, but still poll.
        SIGNAL_WORK_THREAD_MODE[cpu].store(false, Ordering::Relaxed);
    }

    #[cfg(all(riscv, low_power_wait))]
    fn wait_impl(cpu: usize) {
        // we do not care about race conditions between the load and store operations,
        // interrupts will only set this value to true.
        critical_section::with(|_| {
            // if there is work to do, loop back to polling
            if !SIGNAL_WORK_THREAD_MODE[cpu].load(Ordering::Relaxed) {
                // if not, wait for interrupt
                unsafe { core::arch::asm!("wfi") };
            }
        });
        // if an interrupt occurred while waiting, it will be serviced here
        // If this races and some waker sets the signal, we'll reset it, but still poll.
        SIGNAL_WORK_THREAD_MODE[cpu].store(false, Ordering::Relaxed);
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
