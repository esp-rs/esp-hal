//! Shared provisioning types and constants.

use core::net::Ipv4Addr;

/// Maximum number of Wi-Fi APs retained for the `prov-scan` endpoint.
#[instability::unstable]
pub const MAX_CACHED_APS: usize = 16;

/// Default SoftAP SSID and BLE advertised name (`PROV_esp`).
#[instability::unstable]
pub const DEFAULT_PROV_NAME: &str = "PROV_esp";

/// Default proof-of-possession string for sec1.
#[instability::unstable]
pub const DEFAULT_POP: &str = "abcd1234";

/// Default provisioning SoftAP gateway (`192.168.4.1/24`).
#[instability::unstable]
pub const DEFAULT_AP_GATEWAY: Ipv4Addr = Ipv4Addr::new(192, 168, 4, 1);

/// Default provisioning session timeout (seconds).
#[instability::unstable]
pub const DEFAULT_TIMEOUT_SECS: u64 = 300;

/// JSON body for the `proto-ver` endpoint (sec1 + wifi_scan capability).
#[instability::unstable]
pub const PROTO_VER_JSON: &[u8] =
    b"{\"prov\":{\"ver\":\"v1.1\",\"sec_ver\":1,\"sec_patch_ver\":0,\"cap\":[\"wifi_scan\"]}}";

/// 128-bit BLE provisioning service UUID (little-endian) used by the Espressif
/// provisioning apps.
///
/// Decoded: `021a9004-0382-4aea-bff4-6b3f1c5adfb4`
#[instability::unstable]
pub const PROV_SERVICE_UUID_128: [u8; 16] = [
    0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
];

/// SessionData protobuf response for sec0 (no encryption).
#[instability::unstable]
pub const SESSION_RESPONSE_SEC0: [u8; 4] = [0x10, 0x00, 0x52, 0x00];

/// Minimum `response_buf` length for [`crate::sec1::handle_session`] Resp0 / Resp1.
///
/// Smaller buffers yield [`crate::sec1::SessionResponse::BufferTooSmall`] instead of a
/// misleading sec0 fallback.
#[instability::unstable]
pub const SESSION_RESPONSE_MIN_LEN: usize = 128;

/// Wraps an RNG that implements `rand_core` 0.9 (e.g. `esp_hal::rng::Rng`) to
/// implement `RngCore + CryptoRng` (rand_core 0.6) for use with sec1 /
/// `x25519-dalek`.
#[instability::unstable]
pub struct CryptoRngWrap<R>(pub R);

impl<R> rand_core::RngCore for CryptoRngWrap<R>
where
    R: rand_core09::RngCore,
{
    fn next_u32(&mut self) -> u32 {
        rand_core09::RngCore::next_u32(&mut self.0)
    }
    fn next_u64(&mut self) -> u64 {
        rand_core09::RngCore::next_u64(&mut self.0)
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        rand_core09::RngCore::fill_bytes(&mut self.0, dest)
    }
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

impl<R: rand_core09::RngCore> rand_core::CryptoRng for CryptoRngWrap<R> {}

/// One Wi-Fi access point entry for the `prov-scan` protobuf cache.
///
/// This is **not** a duplicate of [`esp_radio::wifi::scan::AccessPointInfo`].
/// Protocol-only builds (no SoftAP/BLE / no `esp-radio`) still need a small,
/// `Copy` layout that matches wifi_scan.proto (`auth` as `u8`, SSID length
/// separate from a fixed buffer). SoftAP fills it via
/// [`crate::cache_scan_results`] from `AccessPointInfo`.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct CachedAp {
    /// SSID bytes (UTF-8 when valid); unused bytes after [`Self::ssid_len`] are ignored.
    pub ssid: [u8; 33],
    /// Length of the SSID in [`Self::ssid`] (0..=32).
    pub ssid_len: u8,
    /// Channel number.
    pub channel: u8,
    /// Signal strength (RSSI).
    pub rssi: i8,
    /// BSSID (MAC).
    pub bssid: [u8; 6],
    /// Auth mode as used by Espressif wifi_scan.proto (`0` = Open, …).
    pub auth: u8,
}

impl CachedAp {
    /// Empty / unused entry.
    pub const EMPTY: Self = Self {
        ssid: [0u8; 33],
        ssid_len: 0,
        channel: 0,
        rssi: 0,
        bssid: [0u8; 6],
        auth: 0,
    };
}

static mut CACHED_APS: [CachedAp; MAX_CACHED_APS] = [CachedAp::EMPTY; MAX_CACHED_APS];
static mut CACHED_AP_COUNT: usize = 0;

/// Replace the global scan-result cache used by [`crate::protocomm`] scan handlers
/// that call [`cached_aps`].
///
/// Copies at most [`MAX_CACHED_APS`] entries. Intended for single-threaded use
/// during a provisioning session.
#[instability::unstable]
pub fn set_cached_aps(results: &[CachedAp]) {
    let count = results.len().min(MAX_CACHED_APS);
    // SAFETY: provisioning is single-threaded; no concurrent readers/writers.
    unsafe {
        let aps = &raw mut CACHED_APS;
        for (i, ap) in results.iter().take(count).enumerate() {
            (*aps)[i] = *ap;
        }
        for i in count..MAX_CACHED_APS {
            (*aps)[i] = CachedAp::EMPTY;
        }
        (&raw mut CACHED_AP_COUNT).write(count);
    }
    info!("[esp-prov] cached {} WiFi scan results", count);
}

/// Returns the valid cached scan results (length equals the count).
///
/// # Safety considerations
///
/// The returned slice aliases a static buffer updated by [`set_cached_aps`].
/// Do not hold it across a call to [`set_cached_aps`].
#[instability::unstable]
pub fn cached_aps() -> (&'static [CachedAp], usize) {
    // SAFETY: single-threaded provisioning; count ≤ MAX_CACHED_APS.
    unsafe {
        let count = (&raw const CACHED_AP_COUNT).read();
        (
            core::slice::from_raw_parts((&raw const CACHED_APS).cast::<CachedAp>(), count),
            count,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_cached_aps_clears_stale_tail() {
        let mut first = [CachedAp::EMPTY; 2];
        first[0].ssid_len = 1;
        first[0].ssid[0] = b'a';
        first[1].ssid_len = 1;
        first[1].ssid[0] = b'b';
        set_cached_aps(&first);
        set_cached_aps(&first[..1]);
        // SAFETY: host test, single-threaded.
        unsafe {
            assert_eq!((&raw const CACHED_AP_COUNT).read(), 1);
            assert_eq!((&raw const CACHED_APS).read()[1].ssid_len, 0);
            assert_eq!((&raw const CACHED_APS).read()[1].ssid[0], 0);
        }
    }
}
