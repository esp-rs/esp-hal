//! OS-aware embassy executors.

use core::{cell::UnsafeCell, mem::MaybeUninit, sync::atomic::Ordering};

use embassy_executor::{SendSpawner, Spawner, raw};
use esp_hal::interrupt::{InterruptHandler, Priority, software::SoftwareInterrupt};
use portable_atomic::AtomicPtr;

use crate::task::flags::ThreadFlags;

#[unsafe(export_name = "__pender")]
fn __pender(context: *mut ()) {
    match context as usize {
        0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        _ => {
            // This forces us to keep the embassy timer queue separate, otherwise we'd need to
            // reentrantly lock SCHEDULER.
            let flags = unwrap!(unsafe { context.cast::<ThreadFlags>().as_ref() });
            flags.set(1);
        }
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

/// Thread-mode executor.
///
/// This executor runs in an OS thread.
pub struct Executor {
    executor: UnsafeCell<MaybeUninit<raw::Executor>>,
}

impl Executor {
    /// Create a new thread-mode executor.
    pub const fn new() -> Self {
        Self {
            executor: UnsafeCell::new(MaybeUninit::uninit()),
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
        let flags = ThreadFlags::new();
        struct NoHooks;

        impl Callbacks for NoHooks {
            fn before_poll(&mut self) {}

            fn on_idle(&mut self) {}
        }

        self.run_inner(init, &flags, NoHooks)
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
        let flags = ThreadFlags::new();
        struct Hooks<'a, CB: Callbacks>(CB, &'a ThreadFlags);

        impl<CB: Callbacks> Callbacks for Hooks<'_, CB> {
            fn before_poll(&mut self) {
                self.0.before_poll()
            }

            fn on_idle(&mut self) {
                // Make sure we only call on_idle if the executor would otherwise go to sleep.
                if self.1.get() == 0 {
                    self.0.on_idle();
                }
            }
        }

        self.run_inner(init, &flags, Hooks(callbacks, &flags))
    }

    fn run_inner(
        &'static self,
        init: impl FnOnce(Spawner),
        flags: &ThreadFlags,
        mut hooks: impl Callbacks,
    ) -> ! {
        let executor = unsafe {
            (&mut *self.executor.get()).write(raw::Executor::new(
                (flags as *const ThreadFlags).cast::<()>().cast_mut(),
            ))
        };

        init(executor.spawner());

        loop {
            hooks.before_poll();

            unsafe { executor.poll() };

            hooks.on_idle();

            // Wait for work to become available.
            flags.wait(1, None);
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

/// Interrupt mode executor.
///
/// This executor runs tasks in interrupt mode. The interrupt handler is set up
/// to poll tasks, and when a task is woken the interrupt is pended from
/// software.
///
/// Interrupt executors have potentially lower latency than thread-mode executors, but only a
/// limited number can be created.
pub struct InterruptExecutor<const SWI: u8> {
    executor: UnsafeCell<MaybeUninit<raw::Executor>>,
    interrupt: SoftwareInterrupt<'static, SWI>,
}

const COUNT: usize = 4;
static INTERRUPT_EXECUTORS: [InterruptExecutorStorage; COUNT] =
    [const { InterruptExecutorStorage::new() }; COUNT];

unsafe impl<const SWI: u8> Send for InterruptExecutor<SWI> {}
unsafe impl<const SWI: u8> Sync for InterruptExecutor<SWI> {}

struct InterruptExecutorStorage {
    raw_executor: AtomicPtr<raw::Executor>,
}

impl InterruptExecutorStorage {
    const fn new() -> Self {
        Self {
            raw_executor: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    /// # Safety:
    ///
    /// The caller must ensure `set` has been called before.
    unsafe fn get(&self) -> &raw::Executor {
        unsafe { &*self.raw_executor.load(Ordering::Relaxed) }
    }

    fn set(&self, executor: *mut raw::Executor) {
        self.raw_executor.store(executor, Ordering::Relaxed);
    }
}

extern "C" fn handle_interrupt<const NUM: u8>() {
    let swi = unsafe { SoftwareInterrupt::<NUM>::steal() };
    swi.reset();

    unsafe {
        // SAFETY: The executor is always initialized before the interrupt is enabled.
        let executor = INTERRUPT_EXECUTORS[NUM as usize].get();
        executor.poll();
    }
}

impl<const SWI: u8> InterruptExecutor<SWI> {
    /// Create a new `InterruptExecutor`.
    /// This takes the software interrupt to be used internally.
    #[inline]
    pub const fn new(interrupt: SoftwareInterrupt<'static, SWI>) -> Self {
        Self {
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
    /// [`SendSpawner`] is returned instead of a [`Spawner`] because the
    /// executor effectively runs in a different "thread" (the interrupt),
    /// so spawning tasks on it is effectively sending them.
    ///
    /// To obtain a [`Spawner`] for this executor, use [`Spawner::for_current_executor`]
    /// from a task running in it.
    pub fn start(&'static mut self, priority: Priority) -> SendSpawner {
        unsafe {
            (*self.executor.get()).write(raw::Executor::new((SWI as usize) as *mut ()));

            INTERRUPT_EXECUTORS[SWI as usize].set((*self.executor.get()).as_mut_ptr());
        }

        let swi_handler = match SWI {
            0 => handle_interrupt::<0>,
            1 => handle_interrupt::<1>,
            2 => handle_interrupt::<2>,
            3 => handle_interrupt::<3>,
            _ => unreachable!(),
        };

        self.interrupt
            .set_interrupt_handler(InterruptHandler::new(swi_handler, priority));

        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.spawner().make_send()
    }

    /// Get a SendSpawner for this executor
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on this
    /// executor.
    ///
    /// This MUST only be called on an executor that has already been started.
    /// The function will panic otherwise.
    pub fn spawner(&'static self) -> SendSpawner {
        if INTERRUPT_EXECUTORS[SWI as usize]
            .raw_executor
            .load(Ordering::Acquire)
            .is_null()
        {
            panic!("InterruptExecutor::spawner() called on uninitialized executor.");
        }
        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.spawner().make_send()
    }
}
