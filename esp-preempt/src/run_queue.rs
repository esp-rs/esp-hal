use crate::task::{TaskExt, TaskPtr, TaskQueue, TaskReadyQueueElement, TaskState};

#[derive(Clone, Copy)]
pub(crate) struct MaxPriority {
    max: usize,
    mask: usize,
}

impl MaxPriority {
    pub const MAX_PRIORITY: usize = 31;

    const fn new() -> Self {
        Self { max: 0, mask: 0 }
    }

    fn mark_ready(&mut self, level: usize) {
        self.max = self.max.max(level);
        self.mask |= 1 << level;
    }

    fn unmark(&mut self, level: usize) {
        self.mask &= !(1 << level);
        self.max = Self::MAX_PRIORITY.saturating_sub(self.mask.leading_zeros() as usize);
    }

    fn ready(&self) -> usize {
        // Priority 0 must always be ready
        self.max
    }
}

pub(crate) struct RunQueue {
    pub(crate) ready_priority: MaxPriority,

    pub(crate) ready_tasks: [TaskQueue<TaskReadyQueueElement>; MaxPriority::MAX_PRIORITY + 1],
}

impl RunQueue {
    pub(crate) const fn new() -> Self {
        Self {
            ready_priority: MaxPriority::new(),
            ready_tasks: [const { TaskQueue::new() }; MaxPriority::MAX_PRIORITY + 1],
        }
    }

    pub(crate) fn mark_task_ready(&mut self, mut ready_task: TaskPtr) -> bool {
        let priority = ready_task.priority(self);
        let current_prio = self.ready_priority.ready();

        ready_task.set_state(TaskState::Ready);
        if let Some(mut containing_queue) = unsafe { ready_task.as_mut().current_queue.take() } {
            unsafe {
                containing_queue.as_mut().remove(ready_task);
            }
        }
        self.ready_tasks[priority].remove(ready_task);
        self.ready_tasks[priority].push(ready_task);

        self.ready_priority.mark_ready(priority);
        if priority > current_prio || current_prio == 0 {
            debug!(
                "mark_task_ready - New prio level: {}",
                self.ready_priority.ready()
            );
            true
        } else {
            false
        }
    }

    pub(crate) fn pop(&mut self) -> Option<TaskPtr> {
        let current_prio = self.ready_priority.ready();
        debug!("pop - from level: {}", current_prio);

        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                use esp_hal::system::Cpu;

                let other = match Cpu::current() {
                    Cpu::AppCpu => Cpu::ProCpu,
                    Cpu::ProCpu => Cpu::AppCpu,
                };

                // Look iteratively through priority levels - this will ensure we can find a task even if all high priority tasks are pinned to the other CPU.
                let mut priority_level = self.ready_priority;

                let mut current_prio = current_prio;
                let mut popped;
                loop {
                    popped = self.ready_tasks[current_prio].pop_if(|t| t.pinned_to != Some(other));

                    if popped.is_none() {
                        priority_level.unmark(current_prio);
                        if current_prio == 0 {
                            break;
                        }
                        current_prio = priority_level.ready();
                        continue;
                    }

                    break;
                }

            } else {
                let popped = self.ready_tasks[current_prio].pop();
            }
        }

        if self.ready_tasks[current_prio].is_empty() {
            self.ready_priority.unmark(current_prio);
            debug!("pop - New prio level: {}", self.ready_priority.ready());
        }

        popped
    }

    pub(crate) fn is_level_empty(&self, level: usize) -> bool {
        self.ready_tasks[level].is_empty()
    }

    pub(crate) fn remove(&mut self, to_delete: TaskPtr) {
        let priority = to_delete.priority(self);
        self.ready_tasks[priority].remove(to_delete);

        if self.ready_tasks[priority].is_empty() {
            self.ready_priority.unmark(priority);
            debug!(
                "remove - last task removed - New prio level: {}",
                self.ready_priority.ready()
            );
        }
    }
}
