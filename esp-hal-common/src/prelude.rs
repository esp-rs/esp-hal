//! The prelude
//!
//! Re-exports all traits required for interacting with the various peripheral
//! drivers implemented in this crate.

pub use embedded_hal::{
    blocking::delay::{DelayMs as _, DelayUs as _},
    digital::v2::{
        InputPin as _,
        OutputPin as _,
        StatefulOutputPin as _,
        ToggleableOutputPin as _,
    },
    prelude::*,
};
pub use fugit::{ExtU32 as _, ExtU64 as _, RateExtU32 as _, RateExtU64 as _};
