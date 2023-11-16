//! Multicore-aware embassy executors.

#[cfg(feature = "embassy-executor-thread")]
pub mod thread;

#[cfg(feature = "embassy-executor-thread")]
pub use thread::*;
