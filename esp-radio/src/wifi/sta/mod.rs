//! Wi-Fi station.

use alloc::string::String;
use core::fmt;

use enumset::EnumSet;
use procmacros::BuilderLite;

use super::{AuthMethod, Protocol, scan::ScanMethod};
use crate::WifiError;

#[cfg(feature = "wifi-eap")]
pub mod eap;

/// Station configuration for a Wi-Fi connection.
#[derive(BuilderLite, Clone, Eq, PartialEq, Hash)]
pub struct StationConfig {
    /// The SSID of the Wi-Fi network.
    #[builder_lite(reference)]
    pub(crate) ssid: String,
    /// The BSSID (MAC address) of the station.
    pub(crate) bssid: Option<[u8; 6]>,
    /// The authentication method for the Wi-Fi connection.
    pub(crate) auth_method: AuthMethod,
    /// The password for the Wi-Fi connection.
    #[builder_lite(reference)]
    pub(crate) password: String,
    /// The Wi-Fi channel to connect to.
    pub(crate) channel: Option<u8>,
    /// The set of protocols supported by the access point.
    pub(crate) protocols: EnumSet<Protocol>,
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
}

impl StationConfig {
    pub(crate) fn validate(&self) -> Result<(), WifiError> {
        if self.ssid.len() > 32 {
            return Err(WifiError::InvalidArguments);
        }

        if self.password.len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        if !(6..=31).contains(&self.beacon_timeout) {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl Default for StationConfig {
    fn default() -> Self {
        StationConfig {
            ssid: String::new(),
            bssid: None,
            auth_method: AuthMethod::Wpa2Personal,
            password: String::new(),
            channel: None,
            protocols: (Protocol::P802D11B | Protocol::P802D11BG | Protocol::P802D11BGN),
            listen_interval: 3,
            beacon_timeout: 6,
            failure_retry_cnt: 1,
            scan_method: ScanMethod::Fast,
        }
    }
}

impl fmt::Debug for StationConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("StationConfig")
            .field("ssid", &self.ssid)
            .field("bssid", &self.bssid)
            .field("auth_method", &self.auth_method)
            .field("password", &"**REDACTED**")
            .field("channel", &self.channel)
            .field("protocols", &self.protocols)
            .field("listen_interval", &self.listen_interval)
            .field("beacon_timeout", &self.beacon_timeout)
            .field("failure_retry_cnt", &self.failure_retry_cnt)
            .field("scan_method", &self.scan_method)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for StationConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "StationConfig {{\
            ssid: {}, \
            bssid: {:?}, \
            auth_method: {:?}, \
            password: **REDACTED**, \
            channel: {:?}, \
            protocols: {}, \
            listen_interval: {}, \
            beacon_timeout: {}, \
            failure_retry_cnt: {}, \
            scan_method: {} \
            }}",
            self.ssid.as_str(),
            self.bssid,
            self.auth_method,
            self.channel,
            self.protocols,
            self.listen_interval,
            self.beacon_timeout,
            self.failure_retry_cnt,
            self.scan_method
        )
    }
}
