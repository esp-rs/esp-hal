use core::ptr::NonNull;

use crate::rtc_cntl::{
    cpu_retention,
    sleep::{self, RtcSleepConfig},
};

/// Couples CPU power-down to the installed retention buffer, for a light sleep.
///
/// The bit is written and not only set, so that a configuration from [`RtcSleepConfig::deep`]
/// cannot carry a power-down into a light sleep that has no retention memory.
///
/// A second running core must save itself, so the power-down also needs the rendezvous.
#[cfg(feature = "rt")]
pub(crate) fn configure_cpu_retention(config: &mut RtcSleepConfig, buffer: Option<NonNull<u8>>) {
    let allow_pd = buffer.is_some();

    #[cfg(multi_core)]
    let allow_pd = allow_pd && cpu_retention::rendezvous::retention_allowed();

    config.pd_flags.set_pd_cpu(allow_pd);
}

/// Requests the sleep, and retains the CPU across it if the sleep powers the CPU domain down.
///
/// The retained path returns twice, so it owns the request and the wait. A sleep that keeps the
/// domain powered needs no frames, and it must not write them back: the registers still hold what
/// a save would have read, and a restore repeats side effects such as an interrupt claim.
#[crate::ram]
pub(crate) fn enter_sleep_with_retention(
    config: &RtcSleepConfig,
    buffer: Option<NonNull<u8>>,
) -> bool {
    match buffer.filter(|_| config.pd_flags.pd_cpu()) {
        Some(buffer) => cpu_retention::sleep_retained(
            buffer.as_ptr(),
            sleep::pmu_common::request_sleep,
            sleep::wait_for_sleep_result,
        ),
        None => {
            config.enter_sleep();
            sleep::wait_for_sleep_result()
        }
    }
}

/// Finishes CPU retention after the sleep request returns.
///
/// The disarm is unconditional, because the tail of the sleep runs on a wake and on a rejected
/// request. A stale stub address would otherwise outlive the sleep that armed it.
pub(crate) fn finish_cpu_retention(buffer: Option<NonNull<u8>>, _rejected: bool) {
    if buffer.is_some() {
        cpu_retention::disarm_wake_stub();
    }
}
