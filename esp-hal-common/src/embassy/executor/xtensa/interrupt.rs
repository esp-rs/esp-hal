//! Multicore-aware interrupt-mode executor.
use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    mem::MaybeUninit,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

use embassy_executor::{
    raw::{self, Pender},
    SendSpawner,
};
#[cfg(dport)]
use peripherals::DPORT as SystemPeripheral;
#[cfg(system)]
use peripherals::SYSTEM as SystemPeripheral;

use crate::{get_core, interrupt, peripherals};

static FROM_CPU_IRQ_USED: AtomicUsize = AtomicUsize::new(0);

pub trait SwPendableInterrupt {
    fn enable(priority: interrupt::Priority);
    fn pend();
    fn clear();
}

macro_rules! from_cpu {
    ($cpu:literal) => {
        paste::paste! {
            pub struct [<FromCpu $cpu>];

            /// We don't allow using the same interrupt for multiple executors.

            impl [<FromCpu $cpu>] {
                fn set_bit(value: bool) {
                    let system = unsafe { &*SystemPeripheral::PTR };
                    system
                        .[<cpu_intr_from_cpu_ $cpu>]
                        .write(|w| w.[<cpu_intr_from_cpu_ $cpu>]().bit(value));
                }
            }

            impl SwPendableInterrupt for [<FromCpu $cpu>] {
                fn enable(priority: interrupt::Priority) {
                    let mask = 1 << $cpu;
                    if FROM_CPU_IRQ_USED.fetch_or(mask, Ordering::SeqCst) & mask != 0 {
                        panic!(concat!("FROM_CPU_", $cpu, " is already used by a different executor."));
                    }

                    interrupt::enable(peripherals::Interrupt::[<FROM_CPU_INTR $cpu>], priority).unwrap();
                }

                fn pend() {
                    Self::set_bit(true);
                }

                fn clear() {
                    Self::set_bit(false);
                }
            }
        }
    };
}

from_cpu!(1);
from_cpu!(2);
from_cpu!(3);

/// Interrupt mode executor.
///
/// This executor runs tasks in interrupt mode. The interrupt handler is set up
/// to poll tasks, and when a task is woken the interrupt is pended from
/// software.
///
/// # Interrupt requirements
///
/// You must write the interrupt handler yourself, and make it call
/// [`Self::on_interrupt()`]
///
/// ```rust,ignore
/// #[interrupt]
/// fn FROM_CPU_INTR1() {
///     unsafe { INT_EXECUTOR.on_interrupt() }
/// }
/// ```
pub struct InterruptExecutor<SWI>
where
    SWI: SwPendableInterrupt,
{
    core: AtomicUsize,
    executor: UnsafeCell<MaybeUninit<raw::Executor>>,
    _interrupt: PhantomData<SWI>,
}

unsafe impl<SWI: SwPendableInterrupt> Send for InterruptExecutor<SWI> {}
unsafe impl<SWI: SwPendableInterrupt> Sync for InterruptExecutor<SWI> {}

impl<SWI> InterruptExecutor<SWI>
where
    SWI: SwPendableInterrupt,
{
    /// Create a new `InterruptExecutor`.
    #[inline]
    pub const fn new() -> Self {
        Self {
            core: AtomicUsize::new(usize::MAX),
            executor: UnsafeCell::new(MaybeUninit::uninit()),
            _interrupt: PhantomData,
        }
    }

    /// Executor interrupt callback.
    ///
    /// # Safety
    ///
    /// You MUST call this from the interrupt handler, and from nowhere else.
    // TODO: it would be pretty sweet if we could register our own interrupt handler
    // when vectoring is enabled. The user shouldn't need to provide the handler for
    // us.
    pub unsafe fn on_interrupt(&'static self) {
        SWI::clear();
        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.poll();
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
    /// To obtain a [`Spawner`] for this executor, use
    /// [`Spawner::for_current_executor()`] from a task running in it.
    ///
    /// # Interrupt requirements
    ///
    /// You must write the interrupt handler yourself, and make it call
    /// [`Self::on_interrupt()`]
    ///
    /// This method already enables (unmasks) the interrupt, you must NOT do it
    /// yourself.
    ///
    /// [`Spawner`]: embassy_executor::Spawner
    /// [`Spawner::for_current_executor()`]: embassy_executor::Spawner::for_current_executor()
    pub fn start(&'static self, priority: interrupt::Priority) -> SendSpawner {
        if self
            .core
            .compare_exchange(
                usize::MAX,
                get_core() as usize,
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
                .write(raw::Executor::new(Pender::new_from_callback(
                    |_| SWI::pend(),
                    ptr::null_mut(),
                )))
        }

        SWI::enable(priority);

        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.spawner().make_send()
    }
}
