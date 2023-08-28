//! Multicore-aware embassy executors.

#[cfg(feature = "embassy-executor-interrupt")]
pub mod interrupt;

#[cfg(feature = "embassy-executor-interrupt")]
pub use interrupt::*;

#[cfg(feature = "embassy-executor-thread")]
pub mod thread;

#[cfg(feature = "embassy-executor-thread")]
pub use thread::*;
