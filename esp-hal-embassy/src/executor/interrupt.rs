//! Interrupt-mode executor.

use core::{cell::UnsafeCell, mem::MaybeUninit};

use embassy_executor::SendSpawner;
use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt, InterruptHandler},
    Cpu,
};
use portable_atomic::{AtomicUsize, Ordering};

use super::InnerExecutor;

const COUNT: usize = 3 + cfg!(not(multi_core)) as usize;
static mut EXECUTORS: [CallbackContext; COUNT] = [const { CallbackContext::new() }; COUNT];

/// Interrupt mode executor.
///
/// This executor runs tasks in interrupt mode. The interrupt handler is set up
/// to poll tasks, and when a task is woken the interrupt is pended from
/// software.
pub struct InterruptExecutor<const SWI: u8> {
    core: AtomicUsize,
    executor: UnsafeCell<MaybeUninit<InnerExecutor>>,
    interrupt: SoftwareInterrupt<SWI>,
}

unsafe impl<const SWI: u8> Send for InterruptExecutor<SWI> {}
unsafe impl<const SWI: u8> Sync for InterruptExecutor<SWI> {}

struct CallbackContext {
    raw_executor: UnsafeCell<*mut InnerExecutor>,
}

impl CallbackContext {
    const fn new() -> Self {
        Self {
            raw_executor: UnsafeCell::new(core::ptr::null_mut()),
        }
    }

    fn get(&self) -> *mut InnerExecutor {
        unsafe { *self.raw_executor.get() }
    }

    fn set(&self, executor: *mut InnerExecutor) {
        unsafe { self.raw_executor.get().write(executor) };
    }
}

extern "C" fn handle_interrupt<const NUM: u8>() {
    let swi = unsafe { SoftwareInterrupt::<NUM>::steal() };
    swi.reset();

    unsafe {
        let executor = unwrap!(EXECUTORS[NUM as usize].get().as_mut());
        executor.inner.poll();
    }
}

impl<const SWI: u8> InterruptExecutor<SWI> {
    /// Create a new `InterruptExecutor`.
    /// This takes the software interrupt to be used internally.
    #[inline]
    pub const fn new(interrupt: SoftwareInterrupt<SWI>) -> Self {
        Self {
            core: AtomicUsize::new(usize::MAX),
            executor: UnsafeCell::new(MaybeUninit::uninit()),
            interrupt,
        }
    }

    /// Start the executor at the given priority level.
    ///
    /// This initializes the executor, enables the interrupt, and returns.
    /// The executor keeps running in the background through the interrupt.
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on it. A
    /// [`SendSpawner`] is returned instead of a
    /// [`Spawner`](embassy_executor::Spawner) because the
    /// executor effectively runs in a different "thread" (the interrupt),
    /// so spawning tasks on it is effectively sending them.
    ///
    /// To obtain a [`Spawner`](embassy_executor::Spawner) for this executor,
    /// use [`Spawner::for_current_executor`](embassy_executor::Spawner::for_current_executor)
    /// from a task running in it.
    pub fn start(&'static mut self, priority: interrupt::Priority) -> SendSpawner {
        if self
            .core
            .compare_exchange(
                usize::MAX,
                Cpu::current() as usize,
                Ordering::Acquire,
                Ordering::Relaxed,
            )
            .is_err()
        {
            panic!("InterruptExecutor::start() called multiple times on the same executor.");
        }

        unsafe {
            (*self.executor.get())
                .as_mut_ptr()
                .write(InnerExecutor::new((SWI as usize) as *mut ()));

            EXECUTORS[SWI as usize].set((*self.executor.get()).as_mut_ptr());
        }

        let swi_handler = match SWI {
            0 => handle_interrupt::<0>,
            1 => handle_interrupt::<1>,
            2 => handle_interrupt::<2>,
            #[cfg(not(multi_core))]
            3 => handle_interrupt::<3>,
            _ => unreachable!(),
        };

        self.interrupt
            .set_interrupt_handler(InterruptHandler::new(swi_handler, priority));

        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.init();
        executor.inner.spawner().make_send()
    }

    /// Get a SendSpawner for this executor
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on this
    /// executor.
    ///
    /// This MUST only be called on an executor that has already been started.
    /// The function will panic otherwise.
    pub fn spawner(&'static self) -> SendSpawner {
        if self.core.load(Ordering::Acquire) == usize::MAX {
            panic!("InterruptExecutor::spawner() called on uninitialized executor.");
        }
        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.inner.spawner().make_send()
    }
}
