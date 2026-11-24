use core::array;

use esp_hal::system::Cpu;

use super::check_pop;
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
