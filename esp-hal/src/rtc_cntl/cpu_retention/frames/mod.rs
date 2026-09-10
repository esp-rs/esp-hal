//! Sleep frame layouts.
//!
//! The chips fall into three groups, and the groups do not share a layout. See
//! `components/esp_hw_support/lowpower/port/<chip>/rvsleep-frames.h`.

#[cfg_attr(cpu_retention_frame = "c6_h2", path = "c6_h2.rs")]
#[cfg_attr(cpu_retention_frame = "clic", path = "clic.rs")]
#[cfg_attr(cpu_retention_frame = "p4", path = "p4.rs")]
#[cfg_attr(cpu_retention_frame = "s31", path = "s31.rs")]
pub(crate) mod chip;
