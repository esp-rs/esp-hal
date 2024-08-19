#![no_std]

// By default, we don't want probe-rs to interfere with test timings by halting
// cores and polling RTT. The tests don't produce output most of the time
// anyway. The only cases where output can be interesting are: during
// development, and when a test fails. In these cases, you can enable
// the `defmt` feature to get the output.

#[cfg(not(feature = "defmt"))]
#[defmt::global_logger]
struct Logger;

#[cfg(not(feature = "defmt"))]
unsafe impl defmt::Logger for Logger {
    fn acquire() {}
    unsafe fn flush() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
}

#[cfg(feature = "defmt")]
use defmt_rtt as _;
// Make sure esp_backtrace is not removed.
use esp_backtrace as _;
