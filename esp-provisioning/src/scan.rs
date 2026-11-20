//! Wi-Fi scan → provisioning cache bridge.

use esp_radio::wifi::{AuthenticationMethod, scan::AccessPointInfo};

use crate::{CachedAp, MAX_CACHED_APS, set_cached_aps};

/// Fill the global provisioning scan cache from a radio scan.
///
/// Maps [`AccessPointInfo`] into the protocol [`crate::CachedAp`] layout used by
/// `prov-scan` (kept separate so protocol-only builds need no `esp-radio`).
#[instability::unstable]
pub fn cache_scan_results(results: &[AccessPointInfo]) {
    let mut buf = [CachedAp::EMPTY; MAX_CACHED_APS];
    let count = results.len().min(MAX_CACHED_APS);
    for (i, ap) in results.iter().take(count).enumerate() {
        let ssid_bytes = ap.ssid.as_bytes();
        let ssid_len = ssid_bytes.len().min(32);
        let mut ssid = [0u8; 33];
        ssid[..ssid_len].copy_from_slice(&ssid_bytes[..ssid_len]);
        let auth = ap
            .auth_method
            .map(AuthenticationMethod::as_wifi_auth_mode)
            .unwrap_or(0);
        buf[i] = CachedAp {
            ssid,
            ssid_len: ssid_len as u8,
            channel: ap.channel,
            rssi: ap.signal_strength,
            bssid: ap.bssid,
            auth,
        };
    }
    set_cached_aps(&buf[..count]);
}
