//! The prelude
//!
//! Re-exports all traits required for interacting with the various peripheral
//! drivers implemented in this crate.

pub use embedded_hal::{digital::v2::OutputPin, prelude::*};
pub use riscv_rt_macros::entry;

pub use crate::delay;
