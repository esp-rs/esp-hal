//! Wi-Fi access point.

use procmacros::BuilderLite;

use super::{DisconnectReason, Protocols, SecondaryChannel, Ssid};
use crate::{WifiError, wifi::AuthenticationMethodConfig};

/// Configuration for a Wi-Fi access point.
#[derive(Clone, PartialEq, Eq, BuilderLite, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccessPointConfig {
    /// The SSID of the access point.
    pub(crate) ssid: Ssid,
    /// Whether the SSID is hidden or visible.
    pub(crate) ssid_hidden: bool,
    /// The channel the access point will operate on.
    ///
    /// The channel must be allowed in the configured country. The default country only allows
    /// channels that are permitted in all regions.
    pub(crate) channel: u8,
    /// The secondary channel configuration.
    pub(crate) secondary_channel: Option<SecondaryChannel>,
    /// The set of protocols supported by the access point.
    pub(crate) protocols: Protocols,
    /// The authentication method to be used by the access point.
    pub(crate) authentication: AuthenticationMethodConfig,
    /// The maximum number of connections allowed on the access point.
    /// When set, this number can be clipped to a true upper limit because
    /// ESPNow and access point connections share a common pool of hardware
    /// encryption keys.
    #[builder_lite(unstable)]
    pub(crate) max_connections: u16,
    /// Dtim period of the access point (Range: 1 ~ 10).
    #[builder_lite(unstable)]
    pub(crate) dtim_period: u8,
    /// Time to force deauth the station if the Soft-AccessPoint doesn't receive any data.
    #[builder_lite(unstable)]
    pub(crate) beacon_timeout: u16,
}

impl AccessPointConfig {
    pub(crate) fn validate(&self) -> Result<(), WifiError> {
        // Soft-AP doesn't support WEP (nor WAPI/OWE, which
        // `AuthenticationMethodConfig` doesn't include).
        if matches!(self.authentication, AuthenticationMethodConfig::Wep(_)) {
            warn!("WEP is not supported in access point mode.");
            return Err(WifiError::Unsupported);
        }

        if let Some(password) = self.authentication.password()
            && password.is_empty()
        {
            warn!("Access point password is empty.");
            return Err(WifiError::InvalidPassword);
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
            ssid: "iot-device".try_into().expect("SSID length is valid"),
            ssid_hidden: false,
            channel: 1,
            secondary_channel: None,
            protocols: Protocols::default(),
            authentication: AuthenticationMethodConfig::Open,
            max_connections: 255,
            dtim_period: 2,
            beacon_timeout: 300,
        }
    }
}

/// Information about a station connected to the access point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ConnectedInfo {
    /// The MAC address.
    pub mac: [u8; 6],
    /// The Association ID (AID) of the connected station.
    pub aid: u16,
    /// If this is a mesh child.
    pub is_mesh_child: bool,
}

/// Information about a station disconnected from the access point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct DisconnectedInfo {
    /// The MAC address.
    pub mac: [u8; 6],
    /// The Association ID (AID) of the connected station.
    pub aid: u16,
    /// If this is a mesh child.
    pub is_mesh_child: bool,
    /// The disconnect reason.
    pub reason: DisconnectReason,
}

/// Either the [ConnectedInfo] or [DisconnectedInfo].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EventInfo {
    /// Information about a station connected to the access point.
    Connected(ConnectedInfo),
    /// Information about a station disconnected from the access point.
    Disconnected(DisconnectedInfo),
}
