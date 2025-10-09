use esp_hal::system::Cpu;

use crate::{
    scheduler::CpuSchedulerState,
    task::{TaskExt, TaskPtr, TaskQueue, TaskReadyQueueElement, TaskState},
};

#[derive(Clone, Copy)]
pub(crate) struct MaxPriority {
    // Using Priority here tells the compiler about the maximum value, removing bounds checks.
    max: Priority,
    mask: usize,
}

impl MaxPriority {
    pub const MAX_PRIORITY: usize = const {
        ::core::assert!((P::MAX as usize) < 32);
        P::MAX as usize
    };

    const fn new() -> Self {
        Self {
            max: Priority::ZERO,
            mask: 0,
        }
    }

    fn mark_ready(&mut self, level: Priority) {
        self.max = if level > self.max { level } else { self.max };
        self.mask |= 1 << level.get();
    }

    fn unmark(&mut self, level: usize) {
        self.mask &= !(1 << level);
        self.max =
            Priority::new(Self::MAX_PRIORITY.saturating_sub(self.mask.leading_zeros() as usize));
    }

    fn ready(&self) -> usize {
        // Priority 0 must always be ready
        self.max.get()
    }
}

// Annoying but safe way to ensure indexing by priority has no bounds check panics.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(usize)]
enum P {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
    P16,
    P17,
    P18,
    P19,
    P20,
    P21,
    P22,
    P23,
    P24,
    P25,
    P26,
    P27,
    P28,
    P29,
    P30,
    P31,
}

impl P {
    const MAX: Self = Self::P31;

    const fn from_usize(p: usize) -> Self {
        match p {
            0 => Self::P0,
            1 => Self::P1,
            2 => Self::P2,
            3 => Self::P3,
            4 => Self::P4,
            5 => Self::P5,
            6 => Self::P6,
            7 => Self::P7,
            8 => Self::P8,
            9 => Self::P9,
            10 => Self::P10,
            11 => Self::P11,
            12 => Self::P12,
            13 => Self::P13,
            14 => Self::P14,
            15 => Self::P15,
            16 => Self::P16,
            17 => Self::P17,
            18 => Self::P18,
            19 => Self::P19,
            20 => Self::P20,
            21 => Self::P21,
            22 => Self::P22,
            23 => Self::P23,
            24 => Self::P24,
            25 => Self::P25,
            26 => Self::P26,
            27 => Self::P27,
            28 => Self::P28,
            29 => Self::P29,
            30 => Self::P30,
            _ => Self::P31,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct Priority(P);

impl Priority {
    pub const ZERO: Self = Self(P::P0);

    pub const fn new(p: usize) -> Self {
        Self(P::from_usize(p))
    }

    pub fn get(self) -> usize {
        self.0 as usize
    }
}

pub(crate) struct RunQueue {
    pub(crate) ready_priority: MaxPriority,

    pub(crate) ready_tasks: [TaskQueue<TaskReadyQueueElement>; MaxPriority::MAX_PRIORITY + 1],
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum RunSchedulerOn {
    DontRun,
    CurrentCore,
    #[cfg(multi_core)]
    OtherCore,
}

impl RunQueue {
    pub(crate) const fn new() -> Self {
        Self {
            ready_priority: MaxPriority::new(),
            ready_tasks: [const { TaskQueue::new() }; MaxPriority::MAX_PRIORITY + 1],
        }
    }

    pub(crate) fn mark_task_ready(
        &mut self,
        _state: &[CpuSchedulerState; Cpu::COUNT],
        mut ready_task: TaskPtr,
    ) -> RunSchedulerOn {
        let priority = ready_task.priority(self);
        let priority_n = priority.get();

        ready_task.set_state(TaskState::Ready);
        if let Some(mut containing_queue) = unsafe { ready_task.as_mut().current_queue.take() } {
            unsafe {
                containing_queue.as_mut().remove(ready_task);
            }
        }
        self.ready_tasks[priority_n].remove(ready_task);
        self.ready_tasks[priority_n].push(ready_task);

        cfg_if::cfg_if! {
            if #[cfg(multi_core)] {
                let run_on = if _state[1].initialized {
                    self.select_scheduler_trigger_multi_core(_state, ready_task)
                } else {
                    self.select_scheduler_trigger_single_core(priority_n)
                };
            } else {
                let run_on = self.select_scheduler_trigger_single_core(priority_n);
            }
        }

        self.ready_priority.mark_ready(priority);

        if run_on != RunSchedulerOn::DontRun {
            debug!(
                "mark_task_ready - New prio level: {}",
                self.ready_priority.ready()
            );
        }

        run_on
    }

    #[cfg(multi_core)]
    fn select_scheduler_trigger_multi_core(
        &mut self,
        state: &[CpuSchedulerState; Cpu::COUNT],
        task: TaskPtr,
    ) -> RunSchedulerOn {
        // We're running both schedulers, try to figure out where to schedule the context switch.
        let task_ref = unsafe { task.as_ref() };
        let ready_task_prio = task_ref.priority;

        let (target_cpu, target_cpu_prio) = if let Some(pinned_to) = task_ref.pinned_to {
            // Task is pinned, we have no choice in our target
            let cpu = pinned_to as usize;
            (cpu, state[cpu].current_priority())
        } else {
            // Task is not pinned, pick the core that runs the lower priority task.

            // Written like this because `state.iter()` leaves an unnecessary panic in the code.
            let mut target = (0, state[0].current_priority());
            #[allow(clippy::needless_range_loop)]
            for i in 1..Cpu::COUNT {
                if state[i].current_priority() < target.1 {
                    target = (i, state[i].current_priority());
                }
            }
            target
        };

        if ready_task_prio >= target_cpu_prio {
            if target_cpu == Cpu::current() as usize {
                RunSchedulerOn::CurrentCore
            } else {
                RunSchedulerOn::OtherCore
            }
        } else {
            RunSchedulerOn::DontRun
        }
    }

    fn select_scheduler_trigger_single_core(&self, priority: usize) -> RunSchedulerOn {
        // Run the scheduler if the new priority is >= current maximum priority. This will trigger a
        // run even if the new task's priority is equal, to make sure time slicing is set up.
        if priority >= self.ready_priority.ready() {
            RunSchedulerOn::CurrentCore
        } else {
            RunSchedulerOn::DontRun
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

    pub(crate) fn is_level_empty(&self, level: Priority) -> bool {
        self.ready_tasks[level.get()].is_empty()
    }

    pub(crate) fn remove(&mut self, to_delete: TaskPtr) {
        let priority = to_delete.priority(self).get();
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
