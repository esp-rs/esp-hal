//! Sleep frame layouts.
//!
//! The chips fall into three groups, and the groups do not share a layout. See
//! `components/esp_hw_support/lowpower/port/<chip>/rvsleep-frames.h`.

#[cfg(interrupt_controller = "plic")]
pub(crate) mod c6_h2;
#[cfg(interrupt_controller = "clic")]
pub(crate) mod clic;

#[cfg(interrupt_controller = "plic")]
pub(crate) use c6_h2 as chip;
#[cfg(interrupt_controller = "clic")]
pub(crate) use clic as chip;
