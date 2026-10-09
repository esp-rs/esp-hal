//! Wi-Fi scanning.

use core::{marker::PhantomData, mem::MaybeUninit};

use esp_hal::time::Duration;
use procmacros::BuilderLite;

#[cfg(feature = "unstable")]
use crate::wifi::CountryInfo;
use crate::{
    drop_guard::DropGuard,
    sys::include,
    wifi::{
        AuthenticationMethod,
        SecondaryChannel,
        Ssid,
        WifiController,
        WifiError,
        esp_wifi_result,
    },
};

/// Configuration for active or passive scan.
///
/// # Comparison of active and passive scan
///
/// |                                      | **Active** | **Passive** |
/// |--------------------------------------|------------|-------------|
/// | **Power consumption**                |    High    |     Low     |
/// | **Time required (typical behavior)** |     Low    |     High    |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ScanTypeConfig {
    /// Active scan with min and max scan time per channel. This is the default
    /// and recommended if you are unsure.
    ///
    /// # Procedure
    /// 1. Send probe request on each channel.
    /// 2. Wait for probe response. Wait at least `min` time, but if no response is received, wait
    ///    up to `max` time.
    /// 3. Switch channel.
    /// 4. Repeat from 1.
    Active {
        /// Minimum scan time per channel. Defaults to 10ms.
        min: Duration,
        /// Maximum scan time per channel. Defaults to 20ms.
        max: Duration,
    },
    /// Passive scan
    ///
    /// # Procedure
    /// 1. Wait for beacon for given duration.
    /// 2. Switch channel.
    /// 3. Repeat from 1.
    ///
    /// # Note
    /// It is recommended to avoid duration longer than 1500ms, as it may cause
    /// a station to disconnect from the Access Point.
    Passive(Duration),
}

impl Default for ScanTypeConfig {
    fn default() -> Self {
        Self::Active {
            min: Duration::from_millis(10),
            max: Duration::from_millis(20),
        }
    }
}

impl ScanTypeConfig {
    pub(crate) fn validate(&self) {
        if matches!(self, Self::Passive(dur) if *dur > Duration::from_millis(1500)) {
            warn!(
                "Passive scan duration longer than 1500ms may cause a station to disconnect from the access point"
            );
        }
    }
}

/// Scan configuration.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ScanConfig {
    /// SSID to filter for.
    /// If [`None`] is passed, all SSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given SSID will be
    /// returned.
    pub(crate) ssid: Option<Ssid>,
    /// BSSID to filter for.
    /// If [`None`] is passed, all BSSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given BSSID will be
    /// returned.
    pub(crate) bssid: Option<[u8; 6]>,
    /// Channel to filter for.
    /// If [`None`] is passed, all channels will be returned.
    /// If [`Some`] is passed, only the APs on the given channel will be
    /// returned.
    pub(crate) channel: Option<u8>,
    /// Whether to show hidden networks.
    pub(crate) show_hidden: bool,
    /// Scan type, active or passive.
    pub(crate) scan_type: ScanTypeConfig,
    /// The maximum number of networks to return when scanning.
    /// If [`None`] is passed, all networks will be returned.
    /// If [`Some`] is passed, the specified number of networks will be returned.
    pub(crate) max: Option<usize>,
}

/// Information about a detected Wi-Fi access point.
#[derive(Debug, Default, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct AccessPointInfo {
    /// The SSID of the access point.
    pub ssid: Ssid,
    /// The BSSID (MAC address) of the access point.
    pub bssid: [u8; 6],
    /// The channel the access point is operating on.
    pub channel: u8,
    /// The secondary channel configuration of the access point.
    pub secondary_channel: SecondaryChannel,
    /// The signal strength of the access point (RSSI).
    pub signal_strength: i8,
    /// The authentication method used by the access point.
    pub auth_method: Option<AuthenticationMethod>,
    #[cfg(feature = "unstable")]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    /// The country information of the access point (if available from beacon frames).
    pub country: Option<CountryInfo>,
}

#[allow(non_upper_case_globals)]
pub(crate) fn convert_ap_info(record: &include::wifi_ap_record_t) -> AccessPointInfo {
    // `record.ssid` is 33 bytes to always fit the NUL terminator of a
    // maximum-length SSID - clamp to 32 in case the driver ever hands us one
    // without it.
    let str_len = record.ssid.iter().position(|&c| c == 0).unwrap_or(32);
    let ssid = Ssid::try_from(&record.ssid[..str_len]).expect("SSID length is valid");

    AccessPointInfo {
        ssid,
        bssid: record.bssid,
        channel: record.primary,
        secondary_channel: SecondaryChannel::from_raw(record.second),
        signal_strength: record.rssi,
        auth_method: Some(AuthenticationMethod::from_raw(record.authmode)),
        #[cfg(feature = "unstable")]
        country: CountryInfo::try_from_c(&record.country),
    }
}

/// Wi-Fi scan results.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ScanResults<'d> {
    /// Number of APs to return
    remaining: usize,
    /// Ensures the result list is free'd when this struct is dropped.
    _drop_guard: FreeApListOnDrop,
    /// Hold a lifetime to ensure the scan list is freed before a new scan is started.
    _marker: PhantomData<&'d mut ()>,
}

impl<'d> ScanResults<'d> {
    /// Create new Wi-Fi scan results.
    pub fn new(_controller: &'d mut WifiController<'_>) -> Result<Self, WifiError> {
        // Construct Self first. This ensures we'll free the result list even if `get_ap_num`
        // returns an error.
        let mut this = Self {
            remaining: 0,
            _drop_guard: free_ap_list_on_drop(),
            _marker: PhantomData,
        };

        let mut bss_total = 0;
        unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_num(&mut bss_total))? };

        this.remaining = bss_total as usize;

        Ok(this)
    }
}

impl Iterator for ScanResults<'_> {
    type Item = AccessPointInfo;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }

        self.remaining -= 1;

        let mut record: MaybeUninit<include::wifi_ap_record_t> = MaybeUninit::uninit();

        // We could detect ESP_FAIL to see if we've exhausted the list, but we know the number of
        // results. Reading the number of results also ensures we're in the correct state, so
        // unwrapping here should never fail.
        unwrap!(unsafe {
            esp_wifi_result!(include::esp_wifi_scan_get_ap_record(record.as_mut_ptr()))
        });

        Some(convert_ap_info(unsafe { record.assume_init_ref() }))
    }
}

/// AP list on-drop guard.
pub(super) type FreeApListOnDrop = DropGuard<(), fn(())>;

pub(super) fn free_ap_list_on_drop() -> FreeApListOnDrop {
    DropGuard::new((), |_| unsafe {
        include::esp_wifi_clear_ap_list();
    })
}
