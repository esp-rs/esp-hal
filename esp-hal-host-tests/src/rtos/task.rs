//! Stand-in for `esp-rtos/src/task/mod.rs`.

use core::ptr::NonNull;

use esp_hal::system::Cpu;

use crate::{
    run_queue::{Priority, RunQueue},
    wait_queue::WaitQueue,
};

#[macro_use]
#[path = "../../../esp-rtos/src/task/list.rs"]
#[allow(dead_code)]
mod list;
pub(crate) use list::*;

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum TaskState {
    Ready,
}

pub(crate) type TaskPtr = NonNull<Task>;

pub(crate) struct Task {
    pub(crate) state: TaskState,
    pub(crate) priority: Priority,
    pub(crate) pinned_to: Option<Cpu>,
    pub(crate) current_wait_queue: Option<NonNull<WaitQueue>>,
    pub(crate) in_run_or_wait_queue: bool,
    pub(crate) ready_queue_item: Option<TaskPtr>,
}

impl Task {
    pub(crate) fn new(priority: usize, pinned_to: Option<Cpu>) -> Self {
        Self {
            state: TaskState::Ready,
            priority: Priority::new(priority),
            pinned_to,
            current_wait_queue: None,
            in_run_or_wait_queue: false,
            ready_queue_item: None,
        }
    }
}

task_list_item!(
    TaskReadyQueueElement,
    ready_queue_item,
    in_run_or_wait_queue
);

pub(crate) trait TaskExt {
    fn priority(self, _: &mut RunQueue) -> Priority;
    fn set_state(self, state: TaskState);
}

impl TaskExt for TaskPtr {
    fn priority(self, _: &mut RunQueue) -> Priority {
        unsafe { self.as_ref().priority }
    }

    fn set_state(mut self, state: TaskState) {
        unsafe { self.as_mut().state = state };
    }
}
