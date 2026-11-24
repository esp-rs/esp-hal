use core::array;

use esp_hal::system::Cpu;
use kani::{any, any_where, assume};

use super::check_pop;
use crate::task::Task;

const TASKS: usize = 4;

fn any_task() -> Task {
    let priority = any_where(|priority: &usize| *priority < 3);
    let pinned_to = match any_where(|pin: &u8| *pin < 3) {
        0 => None,
        1 => Some(Cpu::ProCpu),
        _ => Some(Cpu::AppCpu),
    };
    Task::new(priority, pinned_to)
}

/// Proves the property of `check_pop` for every run queue of up to 4 tasks.
///
/// Kani gives each input a symbolic value, so one run covers every value that the constraints
/// allow. When an assertion in `check_pop` can fail, the proof fails and Kani names the
/// assertion.
///
/// Each task gets a priority from 0 to 2, no pin or a pin to either CPU, and is queued or not.
/// `check_pop` queues the tasks in index order. The tasks get their attributes independently, so
/// this covers every order of the attributes in the queue. Priorities 0 to 2 give each pair of
/// tasks an equal, lower or higher priority. The popping CPU is either CPU, and the other CPU
/// runs no task or any task that is not pinned to the popping CPU.
///
/// `unwind(6)` bounds the loops above their longest run, a walk over a queue of all 4 tasks. The
/// proof takes about 7 minutes, so CI runs the same check over every set of 3 tasks instead.
#[kani::proof]
#[kani::unwind(6)]
fn pop_selects_the_task_and_the_time_slice() {
    let mut tasks: [Task; TASKS] = array::from_fn(|_| any_task());
    let queued: [bool; TASKS] = any();
    let cpu = if any() { Cpu::ProCpu } else { Cpu::AppCpu };
    let other_cpu_task = if any() {
        Some(any_where(|i: &usize| *i < TASKS))
    } else {
        None
    };
    if let Some(i) = other_cpu_task {
        // The other CPU does not run a task that is pinned to this CPU.
        assume(tasks[i].pinned_to != Some(cpu));
    }

    check_pop(&mut tasks, &queued, cpu, other_cpu_task);
}
