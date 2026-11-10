//! Host tests of the IPC request queue of esp-hal.

#![cfg(test)]

#[path = "../../esp-hal/src/interrupt/ipc/queue.rs"]
#[allow(dead_code)]
mod queue;

mod tests;
