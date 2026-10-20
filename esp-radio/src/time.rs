//! Time conversions
//!
//! We're using 1ms per tick, to offer a decent-ish timeout range on u32.

#![allow(unused)]

#[inline(always)]
pub(crate) const fn blob_ticks_to_micros(ticks: u32) -> u32 {
    ticks.saturating_mul(1_000)
}

#[inline(always)]
pub(crate) const fn micros_to_blob_ticks(micros: u32) -> u32 {
    micros / 1_000
}

#[inline(always)]
pub(crate) const fn blob_ticks_to_millis(ticks: u32) -> u32 {
    ticks
}

#[inline(always)]
pub(crate) const fn millis_to_blob_ticks(millis: u32) -> u32 {
    millis
}
