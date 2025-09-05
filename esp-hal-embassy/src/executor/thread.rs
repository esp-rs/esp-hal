//! Multicore-aware thread-mode embassy executor.

use core::marker::PhantomData;

use embassy_executor::Spawner;
#[cfg(all(low_power_wait, multi_core))]
use esp_hal::interrupt::software::SoftwareInterrupt;
use esp_hal::{interrupt::Priority, system::Cpu};
use portable_atomic::{AtomicBool, Ordering};

use super::InnerExecutor;

pub(crate) const THREAD_MODE_CONTEXT: usize = 16;

/// global atomic used to keep track of whether there is work to do since sev()
/// is not available on either Xtensa or RISC-V
static SIGNAL_WORK_THREAD_MODE: [AtomicBool; Cpu::COUNT] =
    [const { AtomicBool::new(false) }; Cpu::COUNT];

pub(crate) fn pend_thread_mode(core: usize) {
    // Signal that there is work to be done.
    SIGNAL_WORK_THREAD_MODE[core].store(true, Ordering::Relaxed);

    // If we are pending a task on the current core, we're done. Otherwise, we
    // need to make sure the other core wakes up.
    #[cfg(all(low_power_wait, multi_core))]
    if core != Cpu::current() as usize {
        // We need to clear the interrupt from software. We don't actually
        // need it to trigger and run the interrupt handler, we just need to
        // kick waiti to return.
        unsafe { SoftwareInterrupt::<3>::steal().raise() };
    }
}

/// Callbacks to run code before/after polling the task queue.
pub trait Callbacks {
    /// Called just before polling the executor.
    fn before_poll(&mut self);

    /// Called after the executor is polled, if there is no work scheduled.
    ///
    /// Note that tasks can become ready at any point during the execution
    /// of this function.
    fn on_idle(&mut self);
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
    cpu: Cpu,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create a new Executor.
    #[cfg_attr(
        all(multi_core, low_power_wait),
        doc = r#"

This will use software-interrupt 3 which isn't available for anything else to wake the other core(s)."#
    )]
    pub fn new() -> Self {
        let cpu = Cpu::current();
        Self {
            inner: InnerExecutor::new(
                // Priority 1 means the timer queue can be accessed at interrupt priority 1 - for
                // the thread mode executor it needs to be one higher than the base run level, to
                // allow alarm interrupts to be handled.
                Priority::Priority1,
                (THREAD_MODE_CONTEXT + cpu as usize) as *mut (),
            ),
            cpu,
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
    /// - a local variable in a function you know never returns (like `fn main() -> !`), upgrading
    ///   its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        #[cfg_attr(not(low_power_wait), expect(dead_code, reason = "cpu index is unused"))]
        struct NoHooks(usize);

        impl Callbacks for NoHooks {
            fn before_poll(&mut self) {
                #[cfg(low_power_wait)]
                SIGNAL_WORK_THREAD_MODE[self.0].store(false, Ordering::Relaxed);
            }

            fn on_idle(&mut self) {}
        }

        self.run_inner(init, NoHooks(self.cpu as usize))
    }

    /// Run the executor with callbacks.
    ///
    /// See [Callbacks] on when the callbacks are called.
    ///
    /// See [Self::run] for more information about running the executor.
    ///
    /// This function never returns.
    pub fn run_with_callbacks(
        &'static mut self,
        init: impl FnOnce(Spawner),
        callbacks: impl Callbacks,
    ) -> ! {
        struct Hooks<'a, CB: Callbacks>(CB, &'a AtomicBool);

        impl<CB: Callbacks> Callbacks for Hooks<'_, CB> {
            fn before_poll(&mut self) {
                // Clear the flag unconditionally since we'll use it to decide
                // if on_idle should be called.
                self.1.store(false, Ordering::Relaxed);

                self.0.before_poll()
            }

            fn on_idle(&mut self) {
                // Make sure we only call on_idle if the executor would otherwise go to sleep.
                if !self.1.load(Ordering::Acquire) {
                    self.0.on_idle();
                }
            }
        }

        self.run_inner(
            init,
            Hooks(callbacks, &SIGNAL_WORK_THREAD_MODE[self.cpu as usize]),
        )
    }

    fn run_inner(&'static mut self, init: impl FnOnce(Spawner), mut hooks: impl Callbacks) -> ! {
        #[cfg(all(multi_core, low_power_wait))]
        unwrap!(esp_hal::interrupt::enable(
            esp_hal::peripherals::Interrupt::FROM_CPU_INTR3,
            Priority::min(),
        ));

        self.inner.init();

        init(self.inner.inner.spawner());

        loop {
            hooks.before_poll();

            unsafe { self.inner.inner.poll() };

            hooks.on_idle();

            #[cfg(low_power_wait)]
            Self::wait_impl(self.cpu as usize);
        }
    }

    #[cfg(all(xtensa, low_power_wait))]
    // This function must be in RAM. Loading parts of it from flash can cause a race
    // that results in the core not waking up. Placing `wait_impl` in RAM ensures that
    // it is shorter than the interrupt handler that would clear the interrupt source.
    #[macros::ram]
    fn wait_impl(cpu: usize) {
        // Manual critical section implementation that only masks interrupts handlers.
        // We must not acquire the cross-core on dual-core systems because that would
        // prevent the other core from doing useful work while this core is sleeping.
        let token: u32;
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
    }

    #[cfg(all(riscv, low_power_wait))]
    fn wait_impl(cpu: usize) {
        // we do not care about race conditions between the load and store operations,
        // interrupts will only set this value to true.
        riscv::interrupt::free(|| {
            // if there is work to do, loop back to polling
            if !SIGNAL_WORK_THREAD_MODE[cpu].load(Ordering::Relaxed) {
                // if not, wait for interrupt
                unsafe { core::arch::asm!("wfi") };
            }
        });
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
