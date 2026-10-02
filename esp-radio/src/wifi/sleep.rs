//! Light sleep while the station is in power save.
//!
//! The blob wakes the chip for each beacon through the TSF hardware and the `Wifi` wakeup source.
//! The entry hook refuses every sleep that the blob is not ready for.

use esp_hal::rtc_cntl::{WakeupSource, sleep::WrappedSleepConfig};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

unsafe extern "C" {
    fn esp_wifi_internal_is_tsf_active() -> bool;
    fn esp_wifi_internal_update_light_sleep_wake_ahead_time(time_us: u32);
}

/// Time from the wake of the chip to the moment the blob can receive, in microseconds.
///
/// The blob starts this much earlier before a beacon. esp-hal does not measure its wake time, so
/// this is an estimate with margin. ESP-IDF uses about 370 µs on the C6.
const WAKE_AHEAD_US: u32 = 2000;

static POWER_SAVE: AtomicBool = AtomicBool::new(false);
static STATION_ONLY: AtomicBool = AtomicBool::new(false);

/// The number of holders of the blob's sleep lock.
static SLEEP_LOCK: AtomicU32 = AtomicU32::new(0);

/// Records whether the station uses a power save mode.
pub(crate) fn set_power_save(enabled: bool) {
    POWER_SAVE.store(enabled, Ordering::Relaxed);
}

/// Records whether the configured mode is Station mode, without an access point.
pub(crate) fn set_station_only(station_only: bool) {
    STATION_ONLY.store(station_only, Ordering::Relaxed);
}

/// The blob takes this lock while it must stay awake, for example during a scan or while it waits
/// for a beacon.
pub(crate) fn acquire_sleep_lock() {
    SLEEP_LOCK.fetch_add(1, Ordering::Relaxed);
}

/// Releases the lock taken by [`acquire_sleep_lock`].
pub(crate) fn release_sleep_lock() {
    let _ = SLEEP_LOCK.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_sub(1));
}

/// Claims the `Wifi` wakeup source, so that the chip can sleep while the station is in power
/// save.
///
/// Call this after `esp_wifi_init`.
pub(crate) fn claim_wake_source() {
    let mhz = esp_hal::clock::cpu_clock().as_mhz() as i32;
    unsafe {
        esp_wifi_internal_update_light_sleep_wake_ahead_time(WAKE_AHEAD_US);
        // Without this call, the blob does not arm the TSF wakeup for the next beacon.
        crate::sys::include::esp_wifi_internal_update_light_sleep_default_params(mhz, mhz);
    }

    WakeupSource::Wifi.enable_with_hooks(Some(sleep_entry), None);
}

/// Releases the claim taken by [`claim_wake_source`].
///
/// Call this before `esp_wifi_deinit`.
pub(crate) fn release_wake_source() {
    WakeupSource::Wifi.disable();
}

fn sleep_entry(config: &mut WrappedSleepConfig<'_>) {
    if config.is_deep_sleep() {
        return;
    }

    let station_idle = POWER_SAVE.load(Ordering::Relaxed) && STATION_ONLY.load(Ordering::Relaxed);

    if !station_idle
        || SLEEP_LOCK.load(Ordering::Relaxed) > 0
        || unsafe { esp_wifi_internal_is_tsf_active() }
    {
        config.reject_sleep();
    }
}
