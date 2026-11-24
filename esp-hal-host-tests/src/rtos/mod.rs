//! Checks of the esp-rtos run queue.

use core::ptr::{self, NonNull};

use esp_hal::system::Cpu;

use crate::{
    run_queue::RunQueue,
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
