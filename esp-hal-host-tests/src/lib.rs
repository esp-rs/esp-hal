//! Host tests of esp-hal code that does not need hardware.

#![cfg(test)]

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        let _ = ($( & $x ),*);
    };
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        let _ = ($( & $x ),*);
    };
}

mod ipc;
mod sdio_slave;

// The included files name their dependencies as `crate::` items.
#[path = "../../esp-rtos/src/run_queue.rs"]
#[allow(dead_code)]
mod run_queue;
#[path = "rtos/scheduler.rs"]
mod scheduler;
#[path = "rtos/task.rs"]
mod task;
#[path = "rtos/wait_queue.rs"]
mod wait_queue;

mod rtos;
