use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};

use crate::{
    SCHEDULER,
    task::{TaskPtr, TaskQueue, TaskReadyQueueElement},
};

pub(crate) struct WaitQueue {
    // A task is either blocked, or ready. Since it can't be both, we can reuse the ready queue
    // element. Note however, that a task can simultaneously be in the timer queue and a wait
    // queue!
    pub(crate) waiting_tasks: TaskQueue<TaskReadyQueueElement>,
}

impl WaitQueue {
    pub(crate) const fn new() -> Self {
        Self {
            waiting_tasks: TaskQueue::new(),
        }
    }

    pub(crate) fn notify(&mut self) {
        SCHEDULER.with(|scheduler| {
            // Expergiscere eos. Novit enim Ordinator qui sunt eius.
            while let Some(waken_task) = self.waiting_tasks.pop() {
                scheduler.resume_task(waken_task);
            }
        });
    }

    pub(crate) fn wait_with_deadline(&mut self, deadline: Option<Instant>) {
        SCHEDULER.with(|scheduler| {
            let mut task = unwrap!(scheduler.current_task);

            let wake_at = if let Some(deadline) = deadline {
                deadline
            } else {
                Instant::EPOCH + Duration::MAX
            };

            if scheduler.sleep_until(wake_at) {
                self.waiting_tasks.push(task);
                unsafe {
                    task.as_mut().current_queue = Some(NonNull::from(self));
                }
            }
        });
    }

    pub(crate) fn remove(&mut self, task: TaskPtr) {
        self.waiting_tasks.remove(task);
    }
}
