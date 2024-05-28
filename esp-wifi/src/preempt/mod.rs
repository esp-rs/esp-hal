macro_rules! sum {
    ($h:expr) => ($h);
    ($h:expr, $($t:expr),*) =>
        ($h + sum!($($t),*));
}

macro_rules! task_stack {
    ($($task_stack_size:literal),+) => {
        const TASK_COUNT: usize = [$($task_stack_size),+].len();
        const TASK_STACK_SIZE: [usize; TASK_COUNT] = [$($task_stack_size),+];
        const TOTAL_STACK_SIZE: usize = sum!($($task_stack_size),+);
        const MAX_TASK: usize = TASK_COUNT + 1; // +1 for the user program

        static mut TASK_STACK: [u8; TOTAL_STACK_SIZE] = [0u8; TOTAL_STACK_SIZE];
    };
}

static mut TASK_TOP: usize = 1;
static mut CTX_NOW: usize = 0;

fn allocate_task() -> usize {
    unsafe {
        let i = TASK_TOP - 1;
        CTX_NOW = TASK_TOP;
        TASK_TOP += 1;
        i
    }
}

fn next_task() {
    unsafe {
        CTX_NOW = (CTX_NOW + 1) % TASK_TOP;
    }
}

pub fn current_task() -> usize {
    unsafe { CTX_NOW }
}

#[cfg(coex)]
task_stack!(8192, 8192, 8192);

#[cfg(not(coex))]
task_stack!(8192, 8192);

#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
#[allow(clippy::module_inception)]
pub mod preempt;
