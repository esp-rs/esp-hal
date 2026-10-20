use core::ptr::NonNull;

use esp_hal::time::Instant;

use crate::{
    SCHEDULER,
    scheduler::SchedulerState,
    task::{TaskList, TaskPtr, TaskReadyQueueElement},
};

/// A list of tasks that wait for an event.
///
/// The methods take the locked scheduler, because the caller usually needs to inspect the state
/// the queue protects, in the same critical section.
pub(crate) struct WaitQueue {
    // A task is either blocked, or ready. Since it can't be both, we can reuse the ready queue
    // element. Note however, that a task can simultaneously be in the timer queue and a wait
    // queue!
    pub(crate) waiting_tasks: TaskList<TaskReadyQueueElement>,
}

impl WaitQueue {
    pub(crate) const fn new() -> Self {
        Self {
            waiting_tasks: TaskList::new(),
        }
    }

    /// Wakes up every waiting task.
    pub(crate) fn notify(&mut self, scheduler: &mut SchedulerState) {
        // Expergiscere eos. Novit enim Ordinator qui sunt eius.
        while let Some(waken_task) = self.waiting_tasks.pop() {
            scheduler.resume_task(waken_task);
        }
    }

    /// Puts the current task to sleep until the queue is notified, or the deadline passes.
    pub(crate) fn wait_with_deadline(&mut self, scheduler: &mut SchedulerState, deadline: Instant) {
        let mut task = SCHEDULER.current_task();
        if scheduler.sleep_task_until(task, deadline) {
            self.waiting_tasks.push(task);
            unsafe {
                task.as_mut().current_wait_queue = Some(NonNull::from(self));
            }
            crate::task::yield_task();
        }
    }

    pub(crate) fn remove(&mut self, task: TaskPtr) {
        self.waiting_tasks.remove(task);
    }
}

impl Drop for WaitQueue {
    fn drop(&mut self) {
        debug_assert!(
            self.waiting_tasks.is_empty(),
            "WaitQueue dropped while tasks are still waiting"
        );
    }
}
