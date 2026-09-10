//! Sleep frame layouts.
//!
//! The chips fall into three groups, and the groups do not share a layout. See
//! `components/esp_hw_support/lowpower/port/<chip>/rvsleep-frames.h`.

#[cfg(cpu_retention_frame = "c6_h2")]
pub(crate) mod c6_h2;
#[cfg(cpu_retention_frame = "clic")]
pub(crate) mod clic;
#[cfg(cpu_retention_frame = "p4")]
pub(crate) mod p4;
#[cfg(cpu_retention_frame = "s31")]
pub(crate) mod s31;

#[cfg(cpu_retention_frame = "c6_h2")]
pub(crate) use c6_h2 as chip;
#[cfg(cpu_retention_frame = "clic")]
pub(crate) use clic as chip;
#[cfg(cpu_retention_frame = "p4")]
pub(crate) use p4 as chip;
#[cfg(cpu_retention_frame = "s31")]
pub(crate) use s31 as chip;
