//! Stand-in for `esp-rtos/src/scheduler.rs`.

use esp_hal::system::Cpu;

use crate::{run_queue::Priority, task::Task};

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum ActiveCores {
    #[allow(dead_code, reason = "`RunQueue::mark_task_ready` matches this variant")]
    Single(Cpu),
    All,
}

pub(crate) struct CpuState {
    pub(crate) current_task: *mut Task,
}

pub(crate) struct SchedulerState;

impl SchedulerState {
    pub(crate) fn priority_of_core(per_cpu: &[CpuState], core: usize) -> Priority {
        unsafe { per_cpu[core].current_task.as_ref() }
            .map(|task| task.priority)
            .unwrap_or(Priority::ZERO)
    }
}
