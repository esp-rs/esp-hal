use embassy_executor::raw;
use esp_hal::interrupt::Priority;

pub use self::{interrupt::*, thread::*};
#[cfg(not(single_queue))]
use crate::timer_queue::TimerQueue;

mod interrupt;
mod thread;

#[unsafe(export_name = "__pender")]
fn __pender(context: *mut ()) {
    use esp_hal::interrupt::software::SoftwareInterrupt;

    match context as usize {
        // For interrupt executors, the context value is the
        // software interrupt number
        0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        #[cfg(not(multi_core))]
        3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        // THREAD_MODE_CONTEXT + core ID
        16 => thread::pend_thread_mode(0),
        #[cfg(multi_core)]
        17 => thread::pend_thread_mode(1),
        _ => unreachable!(),
    }
}

#[repr(C)]
pub(crate) struct InnerExecutor {
    inner: raw::Executor,
    #[cfg(not(single_queue))]
    pub(crate) timer_queue: TimerQueue,
}

impl InnerExecutor {
    /// Create a new executor.
    ///
    /// When the executor has work to do, it will call the pender function and
    /// pass `context` to it.
    ///
    /// See [`Executor`] docs for details on the pender.
    pub(crate) fn new(_prio: Priority, context: *mut ()) -> Self {
        Self {
            inner: raw::Executor::new(context),
            #[cfg(not(single_queue))]
            timer_queue: TimerQueue::new(_prio),
        }
    }

    pub(crate) fn init(&self) {
        let inner_executor_ptr = self as *const _ as *mut InnerExecutor;
        // Expose provenance so that casting back to InnerExecutor in the time driver is
        // not UB.
        _ = inner_executor_ptr.expose_provenance();
        #[cfg(not(single_queue))]
        self.timer_queue.set_context(inner_executor_ptr.cast());
    }
}
