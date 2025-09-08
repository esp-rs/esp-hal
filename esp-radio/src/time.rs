//! Time conversions
#![allow(unused)]
pub const TICKS_PER_SECOND: u64 = 1_000_000;

pub(crate) fn micros_to_ticks(us: u64) -> u64 {
    us * (TICKS_PER_SECOND / 1_000_000)
}

pub(crate) fn millis_to_ticks(ms: u64) -> u64 {
    ms * (TICKS_PER_SECOND / 1_000)
}

pub(crate) fn ticks_to_micros(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000_000)
}

pub(crate) fn ticks_to_millis(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000)
}
