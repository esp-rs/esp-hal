// Time keeping
pub const TICKS_PER_SECOND: u64 = 1_000_000;

/// Current systimer count value
/// A tick is 1 / 1_000_000 seconds
/// This function must not be called in a critical section. Doing so may return
/// an incorrect value.
pub(crate) fn systimer_count() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros()
}

// TODO: use an Instance type instead...
#[cfg(target_arch = "riscv32")]
pub(crate) fn time_diff(start: u64, end: u64) -> u64 {
    // 52-bit wrapping sub
    end.wrapping_sub(start) & 0x000f_ffff_ffff_ffff
}

// TODO: use an Instance type instead...
#[cfg(target_arch = "xtensa")]
pub(crate) fn time_diff(start: u64, end: u64) -> u64 {
    end.wrapping_sub(start)
}

#[allow(unused)]
pub(crate) fn micros_to_ticks(us: u64) -> u64 {
    us * (TICKS_PER_SECOND / 1_000_000)
}

#[allow(unused)]
pub(crate) fn millis_to_ticks(ms: u64) -> u64 {
    ms * (TICKS_PER_SECOND / 1_000)
}

#[allow(unused)]
pub(crate) fn ticks_to_micros(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000_000)
}

#[allow(unused)]
pub(crate) fn ticks_to_millis(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000)
}

/// Do not call this in a critical section!
pub(crate) fn elapsed_time_since(start: u64) -> u64 {
    let now = systimer_count();
    time_diff(start, now)
}
