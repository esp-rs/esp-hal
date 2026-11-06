//! Powers the Wi-Fi MAC and baseband down while the chip is in light sleep.
//!
//! The blob saves the MAC in `pm_mac_sleep`, and restores it in `pm_mac_wakeup`. With the backup
//! DMA, the blob saves the MAC to the retention memory, then reports through
//! `wifi_rtc_enable_iso` that the MAC and baseband can lose power. After the sleep,
//! `wifi_rtc_disable_iso` powers them up.

use core::ptr;

use esp_hal::rtc_cntl::{RetentionMemoryError, RetentionSlot};
use portable_atomic::{AtomicPtr, Ordering};

use super::{WifiError, esp_wifi_result};
use crate::{
    modem_power::{self, Radio},
    sys::{c_types::c_void, include::esp_err_t},
};

unsafe extern "C" {
    fn esp_wifi_internal_set_mac_sleep(enable: bool) -> esp_err_t;
    fn pm_mac_sleep();
    fn pm_mac_wakeup();
}

// `esp_wifi_internal_set_mac_sleep(true)` allocates one buffer of this size for the MAC.
const BUFFER_SIZE: usize = 3280;

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type RetentionStorage = esp_hal::rtc_cntl::RetentionStorage<BUFFER_SIZE>;

/// Memory that the Wi-Fi driver uses to retain the MAC while it has no power.
///
/// Install the memory with [`install_retention_memory`].
#[instability::unstable]
pub type RetentionMemory = esp_hal::rtc_cntl::RetentionMemory<BUFFER_SIZE>;

static MEMORY: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();
/// The retention memory while `esp_wifi_internal_set_mac_sleep(true)` runs and the blob did not
/// take it yet, null otherwise.
static OFFERED: AtomicPtr<u8> = AtomicPtr::new(ptr::null_mut());

#[procmacros::doc_replace]
/// Gives the Wi-Fi driver the memory that it uses to retain the MAC.
/// The memory stays with the driver for the life of the program.
///
/// While the station is in power save, light sleep then powers the MAC and baseband down. The
/// modem domain loses power only when no other radio uses it. The power-down also takes 768 bytes
/// of internal heap for the PHY state. Install the memory before the Wi-Fi controller starts.
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::wifi::{RetentionStorage, install_retention_memory};
///
/// #[esp_hal::ram(reclaimed, unstable(zeroed))]
/// static RETENTION: RetentionStorage = RetentionStorage::new();
///
/// install_retention_memory(RETENTION.take())?;
/// # {after_snippet}
/// ```
///
/// # Errors
///
/// Returns [`RetentionMemoryError`] if the driver has retention memory already, or if the memory
/// is outside the range that the backup DMA can reach.
#[instability::unstable]
pub fn install_retention_memory(
    memory: &'static mut RetentionMemory,
) -> Result<(), RetentionMemoryError> {
    MEMORY.install(memory)
}

/// Returns the retention memory for the MAC buffer that `esp_wifi_internal_set_mac_sleep(true)`
/// allocates.
///
/// Returns null for every other allocation.
pub(super) fn malloc_retention(size: usize) -> *mut c_void {
    if size != BUFFER_SIZE {
        return ptr::null_mut();
    }

    let memory = OFFERED.load(Ordering::Relaxed);
    OFFERED.store(ptr::null_mut(), Ordering::Relaxed);
    memory.cast()
}

/// Returns whether `ptr` is the retention memory, which needs no free.
pub(super) fn try_free(ptr: *mut c_void) -> bool {
    MEMORY
        .get()
        .is_some_and(|memory| memory.as_ptr() == ptr.cast())
}

/// Records that Wi-Fi is up, and lets light sleep power the MAC and baseband down, if retention
/// memory is installed.
///
/// Call this after `esp_wifi_init_internal`.
pub(super) fn init() -> Result<(), WifiError> {
    modem_power::start(Radio::Wifi);
    if !retain_mac()? {
        return Ok(());
    }

    if !modem_power::enable_power_down(Radio::Wifi) {
        return stop_power_down();
    }
    Ok(())
}

/// Lets the blob retain the MAC in the retention memory. Returns whether it does.
fn retain_mac() -> Result<bool, WifiError> {
    let Some(memory) = MEMORY.get() else {
        return Ok(false);
    };

    OFFERED.store(memory.as_ptr(), Ordering::Relaxed);
    let result = esp_wifi_result!(unsafe { esp_wifi_internal_set_mac_sleep(true) });
    let in_retention_memory = OFFERED.load(Ordering::Relaxed).is_null();
    OFFERED.store(ptr::null_mut(), Ordering::Relaxed);
    result?;

    if !in_retention_memory {
        warn!(
            "The Wi-Fi MAC buffer does not fit the retention memory, the MAC stays powered in sleep"
        );
        stop_power_down()?;
    }
    Ok(in_retention_memory)
}

/// Stops the power-down, and lets the blob free its MAC buffer.
///
/// Call this before `esp_wifi_deinit_internal`.
pub(super) fn stop_power_down() -> Result<(), WifiError> {
    esp_wifi_result!(unsafe { esp_wifi_internal_set_mac_sleep(false) })
}

/// Records that Wi-Fi is down.
///
/// Call this after `esp_wifi_deinit_internal`, because the deinit still uses the MAC.
pub(super) fn deinit() {
    modem_power::stop(Radio::Wifi);
}

/// Saves the MAC and baseband, then lets them lose power.
///
/// Call this only when the chip enters light sleep. Without the power-down, this does nothing.
pub(super) fn power_down() {
    unsafe { pm_mac_sleep() };
}

/// Powers the MAC and baseband up after light sleep, then restores them.
///
/// Without a power-down before, this does nothing.
pub(super) fn power_up() {
    unsafe { pm_mac_wakeup() };
}
