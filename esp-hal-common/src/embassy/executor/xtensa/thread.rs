//! Multicore-aware thread-mode embassy executor.
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_executor::{raw, Spawner};
#[cfg(all(dport, multi_core))]
use peripherals::DPORT as SystemPeripheral;
#[cfg(all(system, multi_core))]
use peripherals::SYSTEM as SystemPeripheral;

use crate::{get_core, prelude::interrupt};
#[cfg(multi_core)]
use crate::{interrupt, peripherals};

/// global atomic used to keep track of whether there is work to do since sev()
/// is not available on Xtensa
#[cfg(not(multi_core))]
static SIGNAL_WORK_THREAD_MODE: [AtomicBool; 1] = [AtomicBool::new(false)];
#[cfg(multi_core)]
static SIGNAL_WORK_THREAD_MODE: [AtomicBool; 2] = [AtomicBool::new(false), AtomicBool::new(false)];

#[interrupt]
fn FROM_CPU_INTR0() {
    #[cfg(multi_core)]
    {
        // This interrupt is fired when the thread-mode executor's core needs to be
        // woken. It doesn't matter which core handles this interrupt first, the
        // point is just to wake up the core that is currently executing
        // `waiti`.
        let system = unsafe { &*SystemPeripheral::PTR };
        system
            .cpu_intr_from_cpu_0
            .write(|w| w.cpu_intr_from_cpu_0().bit(false));
    }
}

pub(super) fn pend_thread_mode(core: usize) {
    // Signal that there is work to be done.
    SIGNAL_WORK_THREAD_MODE[core].store(true, Ordering::SeqCst);

    // If we are pending a task on the current core, we're done. Otherwise, we
    // need to make sure the other core wakes up.
    #[cfg(multi_core)]
    if core != crate::get_core() as usize {
        // We need to clear the interrupt from software. We don't actually
        // need it to trigger and run the interrupt handler, we just need to
        // kick waiti to return.

        let system = unsafe { &*SystemPeripheral::PTR };
        system
            .cpu_intr_from_cpu_0
            .write(|w| w.cpu_intr_from_cpu_0().bit(true));
    }
}

/// Multi-core Xtensa Executor
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create a new Executor.
    pub fn new() -> Self {
        #[cfg(multi_core)]
        interrupt::enable(
            peripherals::Interrupt::FROM_CPU_INTR0,
            interrupt::Priority::Priority1,
        )
        .unwrap();

        Self {
            inner: raw::Executor::new(usize::from_le_bytes([0, get_core() as u8, 0, 0]) as *mut ()),
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
    /// - a `static mut` (unsafe)
    /// - a local variable in a function you know never returns (like `fn main()
    ///   -> !`), upgrading its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        let cpu = get_core() as usize;

        loop {
            unsafe {
                self.inner.poll();

                // Manual critical section implementation that only masks interrupts handlers.
                // We must not acquire the cross-core on dual-core systems because that would
                // prevent the other core from doing useful work while this core is sleeping.
                let token: critical_section::RawRestoreState;
                core::arch::asm!("rsil {0}, 5", out(reg) token);

                // we do not care about race conditions between the load and store operations,
                // interrupts will only set this value to true.
                if SIGNAL_WORK_THREAD_MODE[cpu].load(Ordering::SeqCst) {
                    SIGNAL_WORK_THREAD_MODE[cpu].store(false, Ordering::SeqCst);

                    // if there is work to do, exit critical section and loop back to polling
                    core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync", in(reg) token)
                } else {
                    // waiti sets the PS.INTLEVEL when slipping into sleep
                    // because critical sections in Xtensa are implemented via increasing
                    // PS.INTLEVEL the critical section ends here
                    // take care not add code after `waiti` if it needs to be inside the CS
                    core::arch::asm!("waiti 0"); // critical section ends here
                }
            }
        }
    }
}
