//! Wi-Fi station.

use procmacros::BuilderLite;

use super::{AuthenticationMethod, DisconnectReason, Protocols, Ssid};
use crate::{WifiError, wifi::AuthenticationMethodConfig};

unstable_module!(
    #[cfg(feature = "wifi-eap")]
    #[cfg_attr(docsrs, doc(cfg(feature = "wifi-eap")))]
    pub mod eap;
);

/// Wi-Fi scan method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[repr(u8)]
#[instability::unstable]
pub enum ScanMethod {
    /// Fast scan.
    Fast,
    /// Scan all channels.
    AllChannels,
}

/// Station configuration for a Wi-Fi connection.
#[derive(BuilderLite, Clone, Eq, PartialEq, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StationConfig {
    /// The SSID of the Wi-Fi network.
    pub(crate) ssid: Ssid,
    /// The BSSID (MAC address) of the station.
    pub(crate) bssid: Option<[u8; 6]>,
    /// The authentication method for the Wi-Fi connection.
    pub(crate) authentication: AuthenticationMethodConfig,
    /// The Wi-Fi channel to connect to.
    pub(crate) channel: Option<u8>,
    /// The set of protocols supported by the access point.
    pub(crate) protocols: Protocols,
    /// Interval for station to listen to beacon from access point.
    ///
    /// The unit of listen interval is one beacon interval.
    /// For example, if beacon interval is 100 ms and listen interval is 3,
    /// the interval for station to listen to beacon is 300 ms
    #[builder_lite(unstable)]
    pub(crate) listen_interval: u16,
    /// Time to disconnect from access point if no data is received.
    ///
    /// Must be between 6 and 31.
    #[builder_lite(unstable)]
    pub(crate) beacon_timeout: u16,
    /// Number of connection retries station will do before moving to next access point.
    ///
    /// `scan_method` should be set as [`ScanMethod::AllChannels`] to use this config.
    ///
    /// Note: Enabling this may cause connection time to increase in case the best access point
    /// doesn't behave properly.
    #[builder_lite(unstable)]
    pub(crate) failure_retry_cnt: u8,
    /// Scan method.
    #[builder_lite(unstable)]
    pub(crate) scan_method: ScanMethod,
    /// Enable 802.11k (Radio Resource Measurement) support.
    ///
    /// When enabled, the station can request Neighbor Reports
    /// from the associated AP, enabling faster roaming decisions
    /// without a full channel scan.
    #[builder_lite(unstable)]
    pub(crate) rm_enabled: bool,
    /// Enable 802.11v (BSS Transition Management) support.
    ///
    /// When enabled, the AP can request or recommend the station
    /// to roam to a better BSS, enabling AP-initiated roaming.
    #[builder_lite(unstable)]
    pub(crate) btm_enabled: bool,
}

impl StationConfig {
    pub(crate) fn validate(&self) -> Result<(), WifiError> {
        if !(6..=31).contains(&self.beacon_timeout) {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl Default for StationConfig {
    fn default() -> Self {
        StationConfig {
            ssid: Ssid::default(),
            bssid: None,
            authentication: AuthenticationMethodConfig::Wpa2Personal(
                "".try_into().expect("Password length is valid"),
            ),
            channel: None,
            protocols: Protocols::default(),
            listen_interval: 3,
            beacon_timeout: 6,
            failure_retry_cnt: 1,
            scan_method: ScanMethod::Fast,
            rm_enabled: false,
            btm_enabled: false,
        }
    }
}

/// Information about the access point the station connected to.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ConnectedInfo {
    /// The SSID of the access point.
    pub ssid: Ssid,
    /// The BSSID of the access point.
    pub bssid: [u8; 6],
    /// The channel of the access point.
    pub channel: u8,
    /// The authentication method used.
    pub authmode: AuthenticationMethod,
    /// The Association ID (AID) assigned by the access point.
    pub aid: u16,
}

/// Information about the access point the station disconnected from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct DisconnectedInfo {
    /// The SSID of the access point.
    pub ssid: Ssid,
    /// The BSSID of the access point.
    pub bssid: [u8; 6],
    /// The disconnect reason.
    pub reason: DisconnectReason,
    /// The RSSI.
    pub rssi: i8,
}
