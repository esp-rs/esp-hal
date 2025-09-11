use crate::task::{TaskPtr, TaskQueue, TaskReadyQueueElement, TaskState};

pub(crate) struct RunQueue {
    // TODO: one queue per priority level
    pub(crate) ready_tasks: TaskQueue<TaskReadyQueueElement>,
}

impl RunQueue {
    pub(crate) const PRIORITY_LEVELS: usize = 32;

    pub(crate) const fn new() -> Self {
        Self {
            ready_tasks: TaskQueue::new(),
        }
    }

    pub(crate) fn mark_same_priority_task_ready(&mut self, ready_task: TaskPtr) {
        self.mark_task_ready(ready_task);
    }

    pub(crate) fn mark_task_ready(&mut self, mut ready_task: TaskPtr) {
        // TODO: this will need to track max ready priority.
        unsafe { ready_task.as_mut().state = TaskState::Ready };
        if let Some(mut containing_queue) = unsafe { ready_task.as_mut().current_queue.take() } {
            unsafe {
                containing_queue.as_mut().remove(ready_task);
            }
        }
        self.ready_tasks.push(ready_task);
    }

    pub(crate) fn pop(&mut self) -> Option<TaskPtr> {
        // TODO: on current prio level
        self.ready_tasks.pop()
    }

    pub(crate) fn is_empty(&self) -> bool {
        // TODO: on current prio level
        self.ready_tasks.is_empty()
    }

    pub(crate) fn remove(&mut self, to_delete: TaskPtr) {
        self.ready_tasks.remove(to_delete);
    }
}
