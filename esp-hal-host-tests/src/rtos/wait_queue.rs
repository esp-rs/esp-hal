//! Stand-in for `esp-rtos/src/wait_queue.rs`.

use crate::task::TaskPtr;

pub(crate) struct WaitQueue;

impl WaitQueue {
    pub(crate) fn remove(&mut self, _task: TaskPtr) {
        unreachable!("tasks under host test do not wait in a wait queue")
    }
}
