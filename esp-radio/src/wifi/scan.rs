//! Wi-Fi scanning.

use core::{marker::PhantomData, mem::MaybeUninit};

use esp_hal::time::Duration;
use procmacros::BuilderLite;

use crate::{
    esp_wifi_result,
    sys::include,
    wifi::{
        Ssid,
        WifiController,
        WifiError,
        ap::{AccessPointInfo, convert_ap_info},
    },
};

/// Wi-Fi scan method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[instability::unstable]
pub enum ScanMethod {
    /// Fast scan.
    Fast,
    /// Scan all channels.
    AllChannels,
}

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
#[derive(Clone, Copy, Default, PartialEq, Eq, BuilderLite)]
pub struct ScanConfig {
    /// SSID to filter for.
    /// If [`None`] is passed, all SSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given SSID will be
    /// returned.
    #[builder_lite(skip_setter)]
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

impl ScanConfig {
    /// Set the SSID of the access point.
    pub fn with_ssid(mut self, ssid: impl Into<Ssid>) -> Self {
        self.ssid = Some(ssid.into());
        self
    }

    /// Clears the SSID.
    pub fn with_ssid_none(mut self) -> Self {
        self.ssid = None;
        self
    }
}

/// Wi-Fi scan results.
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
            _drop_guard: FreeApListOnDrop,
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
pub(super) struct FreeApListOnDrop;

impl FreeApListOnDrop {
    /// Do not automatically free the AP list when the guard is dropped.
    pub fn defuse(self) {
        core::mem::forget(self);
    }
}

impl Drop for FreeApListOnDrop {
    fn drop(&mut self) {
        unsafe {
            include::esp_wifi_clear_ap_list();
        }
    }
}
