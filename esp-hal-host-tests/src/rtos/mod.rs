//! Checks of the esp-rtos run queue.

use core::ptr::{self, NonNull};

use esp_hal::system::Cpu;

use crate::{
    run_queue::{MaxPriority, Priority, RunQueue},
    scheduler::{ActiveCores, CpuState},
    task::Task,
};

#[cfg(kani)]
mod proofs;
mod tests;

fn can_run(task: &Task, cpu: Cpu, other_cpu_task: *const Task) -> bool {
    task.pinned_to.is_none_or(|pinned_to| pinned_to == cpu) && !ptr::eq(task, other_cpu_task)
}

/// Returns the index of the popped task, and whether time slicing is on.
///
/// Panics unless:
/// - `cpu` gets a queued task of the highest priority that it can run.
/// - Time slicing is on exactly when `cpu` can run another queued task of that priority.
fn check_pop(
    tasks: &mut [Task],
    queued: &[bool],
    cpu: Cpu,
    other_cpu_task: Option<usize>,
) -> Option<(usize, bool)> {
    let len = tasks.len();
    // `RunQueue` stores these pointers, so this function keeps no reference to `tasks`.
    let base = tasks.as_mut_ptr();
    let task = |i: usize| unsafe { NonNull::new_unchecked(base.add(i)) };
    let other_cpu_task = other_cpu_task.map_or(ptr::null_mut(), |i| task(i).as_ptr());

    let mut per_cpu = [const {
        CpuState {
            current_task: ptr::null_mut(),
        }
    }; Cpu::COUNT];
    let other_cpu = match cpu {
        Cpu::ProCpu => Cpu::AppCpu,
        Cpu::AppCpu => Cpu::ProCpu,
    };
    per_cpu[other_cpu as usize].current_task = other_cpu_task;

    let mut run_queue = RunQueue::new();
    for i in (0..len).filter(|&i| queued[i]) {
        run_queue.mark_task_ready(&per_cpu, ActiveCores::All, task(i));
    }

    let next = run_queue.pop(cpu, other_cpu_task);

    let priority = |i: usize| unsafe { task(i).as_ref().priority };
    let runnable =
        |i: usize| queued[i] && can_run(unsafe { task(i).as_ref() }, cpu, other_cpu_task);
    let highest = (0..len).filter(|&i| runnable(i)).map(priority).max();

    let Some(next) = next else {
        assert!(highest.is_none(), "the CPU can run a queued task");
        return None;
    };

    let popped = (0..len).find(|&i| task(i) == next.task);
    let popped = popped.expect("the popped task is one of the tasks");
    assert!(runnable(popped), "the CPU can run the popped task");
    assert!(
        Some(priority(popped)) == highest,
        "the popped task has the highest priority"
    );

    let same_priority_runnable =
        (0..len).any(|i| i != popped && runnable(i) && priority(i) == priority(popped));
    assert_eq!(
        next.time_slice, same_priority_runnable,
        "time slicing is on exactly when the CPU can run another task of the same priority"
    );

    Some((popped, next.time_slice))
}

const LEVELS: usize = MaxPriority::MAX_PRIORITY + 1;

#[derive(Clone, Copy)]
enum PriorityOp {
    MarkReady(usize),
    Unmark(usize),
}

/// Returns the highest level that `mask` has a bit for, or 0 when `mask` is empty.
fn highest_level(mask: u32) -> usize {
    let mut highest = 0;
    for level in 0..LEVELS {
        if mask & (1 << level) != 0 {
            highest = level;
        }
    }
    highest
}

fn max_priority_from_mask(mask: u32) -> MaxPriority {
    let mut max_priority = MaxPriority::new();
    for level in 0..LEVELS {
        if mask & (1 << level) != 0 {
            max_priority.mark_ready(Priority::new(level));
        }
    }
    max_priority
}

/// Applies `op` to the `MaxPriority` that has the levels of `mask` marked, and returns the ready
/// level after it.
///
/// Panics unless:
/// - `ready` gives the highest marked level, or 0 when no level is marked, before and after `op`.
/// - After `op`, the state equals the one built from the new mask.
fn check_max_priority(mask: u32, op: PriorityOp) -> usize {
    let mut max_priority = max_priority_from_mask(mask);
    assert_eq!(
        max_priority.ready(),
        highest_level(mask),
        "the ready level is the highest marked level"
    );

    let mask = match op {
        PriorityOp::MarkReady(level) => {
            max_priority.mark_ready(Priority::new(level));
            mask | (1 << level)
        }
        PriorityOp::Unmark(level) => {
            max_priority.unmark(level);
            mask & !(1 << level)
        }
    };

    assert_eq!(
        max_priority.ready(),
        highest_level(mask),
        "the ready level after the operation is the highest marked level"
    );
    assert!(
        max_priority == max_priority_from_mask(mask),
        "the operation gives the state built from the new mask"
    );

    max_priority.ready()
}
