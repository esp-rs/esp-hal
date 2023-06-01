use atomic_polyfill::*;

pub static mut FIRST_SWITCH: AtomicBool = AtomicBool::new(true);

static mut TASK_TOP: usize = 0;

static mut CTX_NOW: usize = 0;

macro_rules! sum {
    ($h:expr) => ($h);
    ($h:expr, $($t:expr),*) =>
        ($h + sum!($($t),*));
}

macro_rules! task_stack {
    ($($task_stack_size:literal),+) => {
        const TASKS_COUNT: usize = [$($task_stack_size),+].len();
        const TASK_STACK_SIZE: [usize; TASKS_COUNT] = [$($task_stack_size),+];
        const TOTAL_STACK_SIZE: usize = sum!($($task_stack_size),+);
        static mut TASK_STACK: [u8; TOTAL_STACK_SIZE] = [0u8; TOTAL_STACK_SIZE];
        const MAX_TASK: usize = [$($task_stack_size),+].len() + 1;
    };
}

#[cfg(feature = "coex")]
task_stack!(8192, 8192, 8192);

#[cfg(not(feature = "coex"))]
task_stack!(8192, 8192);

#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
pub mod preempt;
