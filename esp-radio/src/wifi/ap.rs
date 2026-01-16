//! Wi-Fi access point.

use alloc::string::String;
use core::fmt;

use enumset::EnumSet;
use procmacros::BuilderLite;

use super::{AuthenticationMethod, CountryInfo, Protocol, SecondaryChannel};
use crate::{WifiError, sys::include::wifi_ap_record_t};

/// Information about a detected Wi-Fi access point.
#[derive(Debug, Default, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct AccessPointInfo {
    /// The SSID of the access point.
    // TODO: we can use the `alloc` feature once we have `defmt` 1.0.2
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub ssid: String,
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
    /// The country information of the access point (if available from beacon frames).
    pub country: Option<CountryInfo>,
}

/// Configuration for a Wi-Fi access point.
#[derive(Clone, PartialEq, Eq, BuilderLite, Hash)]
pub struct AccessPointConfig {
    /// The SSID of the access point.
    #[builder_lite(reference)]
    pub(crate) ssid: String,
    /// Whether the SSID is hidden or visible.
    pub(crate) ssid_hidden: bool,
    /// The channel the access point will operate on.
    pub(crate) channel: u8,
    /// The secondary channel configuration.
    pub(crate) secondary_channel: Option<u8>,
    /// The set of protocols supported by the access point.
    pub(crate) protocols: EnumSet<Protocol>,
    /// The authentication method to be used by the access point.
    pub(crate) auth_method: AuthenticationMethod,
    /// The password for securing the access point (if applicable).
    #[builder_lite(reference)]
    pub(crate) password: String,
    /// The maximum number of connections allowed on the access point.
    pub(crate) max_connections: u16,
    /// Dtim period of the access point (Range: 1 ~ 10).
    pub(crate) dtim_period: u8,
    /// Time to force deauth the station if the Soft-AccessPoint doesn't receive any data.
    #[builder_lite(unstable)]
    pub(crate) beacon_timeout: u16,
}

impl AccessPointConfig {
    pub(crate) fn validate(&self) -> Result<(), WifiError> {
        if self.ssid.len() > 32 {
            return Err(WifiError::InvalidArguments);
        }

        if self.password.len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        if !(1..=10).contains(&self.dtim_period) {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl Default for AccessPointConfig {
    fn default() -> Self {
        Self {
            ssid: String::from("iot-device"),
            ssid_hidden: false,
            channel: 1,
            secondary_channel: None,
            protocols: (Protocol::P802D11B | Protocol::P802D11BG | Protocol::P802D11BGN),
            auth_method: AuthenticationMethod::None,
            password: String::new(),
            max_connections: 255,
            dtim_period: 2,
            beacon_timeout: 300,
        }
    }
}

impl fmt::Debug for AccessPointConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AccessPointConfig")
            .field("ssid", &self.ssid)
            .field("ssid_hidden", &self.ssid_hidden)
            .field("channel", &self.channel)
            .field("secondary_channel", &self.secondary_channel)
            .field("protocols", &self.protocols)
            .field("auth_method", &self.auth_method)
            .field("password", &"**REDACTED**")
            .field("max_connections", &self.max_connections)
            .field("dtim_period", &self.dtim_period)
            .field("beacon_timeout", &self.beacon_timeout)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AccessPointConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "AccessPointConfig {{\
            ssid: {}, \
            ssid_hidden: {}, \
            channel: {}, \
            secondary_channel: {}, \
            protocols: {}, \
            auth_method: {}, \
            password: **REDACTED**, \
            max_connections: {}, \
            dtim_period: {}, \
            beacon_timeout: {} \
            }}",
            self.ssid.as_str(),
            self.ssid_hidden,
            self.channel,
            self.secondary_channel,
            self.protocols,
            self.auth_method,
            self.max_connections,
            self.dtim_period,
            self.beacon_timeout
        );
    }
}

#[allow(non_upper_case_globals)]
pub(crate) fn convert_ap_info(record: &wifi_ap_record_t) -> AccessPointInfo {
    let str_len = record
        .ssid
        .iter()
        .position(|&c| c == 0)
        .unwrap_or(record.ssid.len());
    let ssid = alloc::string::String::from_utf8_lossy(&record.ssid[..str_len]).into_owned();

    AccessPointInfo {
        ssid,
        bssid: record.bssid,
        channel: record.primary,
        secondary_channel: SecondaryChannel::from_raw(record.second),
        signal_strength: record.rssi,
        auth_method: Some(AuthenticationMethod::from_raw(record.authmode)),
        country: CountryInfo::try_from_c(&record.country),
    }
}
