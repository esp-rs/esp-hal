//! Powers the Wi-Fi MAC and baseband down while the chip is in light sleep.
//!
//! The blob saves the MAC in `pm_mac_sleep`, and restores it in `pm_mac_wakeup`. With the backup
//! DMA, the blob saves the MAC to the retention memory, then reports through
//! `wifi_rtc_enable_iso` that the MAC and baseband can lose power. After the sleep,
//! `wifi_rtc_disable_iso` powers them up.

use super::{WifiError, esp_wifi_result, retention};
use crate::{
    modem_power::{self, Radio},
    sys::include::esp_err_t,
};

unsafe extern "C" {
    fn esp_wifi_internal_set_mac_sleep(enable: bool) -> esp_err_t;
}

/// Records that Wi-Fi is up, and lets light sleep power the MAC and baseband down, if retention
/// memory is installed.
///
/// Call this after `esp_wifi_init_internal`.
pub(super) fn init() -> Result<(), WifiError> {
    modem_power::start(Radio::Wifi);
    if !prepare_retention()? {
        return Ok(());
    }

    if !modem_power::enable_power_down(Radio::Wifi) {
        return stop_power_down();
    }
    Ok(())
}

/// Lets the blob retain the MAC. Returns whether it can.
#[cfg(modem_retention = "backup_dma")]
fn prepare_retention() -> Result<bool, WifiError> {
    if !retention::is_installed() {
        return Ok(false);
    }

    // The blob allocates its MAC buffer here.
    let in_retention_memory =
        retention::offer(|| esp_wifi_result!(unsafe { esp_wifi_internal_set_mac_sleep(true) }))?;

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

esp_hal::if_unstable_hal! {
    /// Saves the MAC and baseband, then lets them lose power.
    ///
    /// Call this only when the chip enters light sleep. Without the power-down, this does nothing.
    pub(super) fn power_down() {
        unsafe extern "C" {
            fn pm_mac_sleep();
        }

        unsafe { pm_mac_sleep() };
    }

    /// Powers the MAC and baseband up after light sleep, then restores them.
    ///
    /// Without a power-down before, this does nothing.
    pub(super) fn power_up() {
        unsafe extern "C" {
            fn pm_mac_wakeup();
        }

        unsafe { pm_mac_wakeup() };
    }
}
