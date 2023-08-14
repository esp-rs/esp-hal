//! The prelude
//!
//! Re-exports all traits required for interacting with the various peripheral
//! drivers implemented in this crate.

pub use embedded_hal::{
    digital::v2::{
        InputPin as _embedded_hal_digital_vs_InputPin,
        OutputPin as _embedded_hal_digital_vs_OutputPin,
        StatefulOutputPin as _embedded_hal_digital_vs_StatefulOutputPin,
        ToggleableOutputPin as _embedded_hal_digital_vs_ToggleableOutputPin,
    },
    prelude::*,
};
pub use riscv_rt_macros::entry;
