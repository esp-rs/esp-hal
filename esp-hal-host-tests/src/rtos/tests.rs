use core::array;

use esp_hal::system::Cpu;

use super::{LEVELS, PriorityOp, check_max_priority, check_pop};
use crate::task::Task;

#[test]
fn time_slice_with_two_tasks_of_equal_priority() {
    let mut tasks = [Task::new(1, None), Task::new(1, None)];

    let next = check_pop(&mut tasks, &[true, true], Cpu::ProCpu, None);

    assert_eq!(next, Some((0, true)));
}

#[test]
fn no_time_slice_when_the_other_task_is_pinned_to_the_other_cpu() {
    let mut tasks = [Task::new(1, None), Task::new(1, Some(Cpu::AppCpu))];

    let next = check_pop(&mut tasks, &[true, true], Cpu::ProCpu, None);

    assert_eq!(next, Some((0, false)));
}

#[test]
fn no_time_slice_when_the_other_cpu_runs_the_other_task() {
    let mut tasks = [Task::new(1, None), Task::new(1, None)];

    let next = check_pop(&mut tasks, &[true, true], Cpu::ProCpu, Some(1));

    assert_eq!(next, Some((0, false)));
}

/// Checks every set of 3 tasks: each task has a priority from 0 to 2, is pinned to no CPU or to
/// one, and is queued or not. Each set runs with both CPUs, and with each task the other CPU can
/// run.
#[test]
#[cfg_attr(miri, ignore = "too slow under Miri")]
fn pop_follows_the_property_for_every_set_of_three_tasks() {
    const TASKS: usize = 3;
    const PRIORITIES: usize = 3;
    const PINS: [Option<Cpu>; 3] = [None, Some(Cpu::ProCpu), Some(Cpu::AppCpu)];
    const SETUPS: usize = PRIORITIES * PINS.len();
    // Choices from `SETUPS` up are the same setups, queued.
    const CHOICES: usize = 2 * SETUPS;

    for set in 0..CHOICES.pow(TASKS as u32) {
        let choice = |task: usize| set / CHOICES.pow(task as u32) % CHOICES;
        let queued: [bool; TASKS] = array::from_fn(|i| choice(i) >= SETUPS);

        for cpu in [Cpu::ProCpu, Cpu::AppCpu] {
            for other_cpu_task in [None, Some(0), Some(1), Some(2)] {
                let mut tasks: [Task; TASKS] = array::from_fn(|i| {
                    let setup = choice(i) % SETUPS;
                    Task::new(setup % PRIORITIES, PINS[setup / PRIORITIES])
                });
                if other_cpu_task.is_some_and(|i| tasks[i].pinned_to == Some(cpu)) {
                    continue;
                }

                check_pop(&mut tasks, &queued, cpu, other_cpu_task);
            }
        }
    }
}

#[test]
fn lower_priority_task_when_the_higher_one_is_pinned_to_the_other_cpu() {
    let mut tasks = [Task::new(2, Some(Cpu::AppCpu)), Task::new(1, None)];

    let next = check_pop(&mut tasks, &[true, true], Cpu::ProCpu, None);

    assert_eq!(next, Some((1, false)));
}

#[test]
fn unmark_of_the_highest_level_gives_the_next_marked_level() {
    let ready = check_max_priority(1 << 31 | 1 << 5, PriorityOp::Unmark(31));

    assert_eq!(ready, 5);
}

#[test]
fn unmark_of_the_last_level_gives_zero() {
    let ready = check_max_priority(1 << 3, PriorityOp::Unmark(3));

    assert_eq!(ready, 0);
}

#[test]
fn mark_ready_below_the_highest_level_keeps_it() {
    let ready = check_max_priority(1 << 4, PriorityOp::MarkReady(2));

    assert_eq!(ready, 4);
}

/// Checks every mask of the 4 lowest and the 4 highest levels, with each operation on each level.
#[test]
#[cfg_attr(miri, ignore = "too slow under Miri")]
fn max_priority_follows_the_property_for_the_lowest_and_highest_levels() {
    for bits in 0..1u32 << 8 {
        let mask = (bits & 0xF) | (bits >> 4) << 28;
        for level in 0..LEVELS {
            check_max_priority(mask, PriorityOp::MarkReady(level));
            check_max_priority(mask, PriorityOp::Unmark(level));
        }
    }
}
