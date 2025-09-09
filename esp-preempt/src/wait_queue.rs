use esp_hal::time::{Duration, Instant};

use crate::{
    SCHEDULER,
    task::{TaskQueue, TaskReadyQueueElement},
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
            // Let's wake up all waiting tasks. This is far from optimal, but it's simple and will
            // ensure that the highest priority task is notified first.
            while let Some(waken_task) = self.waiting_tasks.pop() {
                scheduler.resume_task(waken_task);
            }
        });
    }

    pub(crate) fn wait_with_deadline(&mut self, deadline: Option<Instant>) {
        SCHEDULER.with(|scheduler| {
            self.waiting_tasks.push(unwrap!(scheduler.current_task));

            let wake_at = if let Some(deadline) = deadline {
                deadline
            } else {
                Instant::EPOCH + Duration::MAX
            };
            scheduler.sleep_until(wake_at);
        });
    }
}
