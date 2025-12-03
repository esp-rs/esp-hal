//! Wi-Fi extensible authentication protocol.

use alloc::string::String;
use core::fmt;

use enumset::EnumSet;
use procmacros::BuilderLite;

use crate::{
    WifiError,
    wifi::{AuthMethod, Protocol, scan::ScanMethod},
};

/// Configuration for EAP-FAST authentication protocol.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct EapFastConfig {
    /// Specifies the provisioning mode for EAP-FAST.
    pub fast_provisioning: u8,
    /// The maximum length of the PAC (Protected Access Credentials) list.
    pub fast_max_pac_list_len: u8,
    /// Indicates whether the PAC file is in binary format.
    pub fast_pac_format_binary: bool,
}

/// Phase 2 authentication methods
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TtlsPhase2Method {
    /// EAP (Extensible Authentication Protocol).
    Eap,
    /// MSCHAPv2 (Microsoft Challenge Handshake Authentication Protocol 2).
    Mschapv2,
    /// MSCHAP (Microsoft Challenge Handshake Authentication Protocol).
    Mschap,
    /// PAP (Password Authentication Protocol).
    Pap,
    /// CHAP (Challenge Handshake Authentication Protocol).
    Chap,
}

impl TtlsPhase2Method {
    /// Maps the phase 2 method to a raw `u32` representation.
    pub(crate) fn to_raw(&self) -> u32 {
        match self {
            TtlsPhase2Method::Eap => {
                crate::sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_EAP
            }
            TtlsPhase2Method::Mschapv2 => {
                crate::sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_MSCHAPV2
            }
            TtlsPhase2Method::Mschap => {
                crate::sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_MSCHAP
            }
            TtlsPhase2Method::Pap => {
                crate::sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_PAP
            }
            TtlsPhase2Method::Chap => {
                crate::sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_CHAP
            }
        }
    }
}

type CertificateAndKey = (&'static [u8], &'static [u8], Option<&'static [u8]>);

/// Configuration for an EAP (Extensible Authentication Protocol) station.
#[derive(BuilderLite, Clone, PartialEq, Eq)]
#[instability::unstable]
pub struct EapStationConfig {
    /// The SSID of the network the station is connecting to.
    #[builder_lite(reference)]
    pub(crate) ssid: String,
    /// The BSSID (MAC Address) of the specific access point.
    pub(crate) bssid: Option<[u8; 6]>,
    /// The authentication method used for EAP.
    pub(crate) auth_method: AuthMethod,
    /// The identity used during authentication.
    #[builder_lite(reference)]
    pub(crate) identity: Option<String>,
    /// The username used for inner authentication.
    /// Some EAP methods require a username for authentication.
    #[builder_lite(reference)]
    pub(crate) username: Option<String>,
    /// The password used for inner authentication.
    #[builder_lite(reference)]
    pub(crate) password: Option<String>,
    /// A new password to be set during the authentication process.
    /// Some methods support password changes during authentication.
    #[builder_lite(reference)]
    pub(crate) new_password: Option<String>,
    /// Configuration for EAP-FAST.
    #[builder_lite(reference)]
    pub(crate) eap_fast_config: Option<EapFastConfig>,
    /// A PAC (Protected Access Credential) file for EAP-FAST.
    pub(crate) pac_file: Option<&'static [u8]>,
    /// A boolean flag indicating whether time checking is enforced during
    /// authentication.
    pub(crate) time_check: bool,
    /// A CA (Certificate Authority) certificate for validating the
    /// authentication server's certificate.
    pub(crate) ca_cert: Option<&'static [u8]>,
    /// A tuple containing the station's certificate, private key, and an
    /// intermediate certificate.
    pub(crate) certificate_and_key: Option<CertificateAndKey>,
    /// The Phase 2 authentication method used for EAP-TTLS.
    #[builder_lite(reference)]
    pub(crate) ttls_phase2_method: Option<TtlsPhase2Method>,
    /// The specific Wi-Fi channel to use for the connection.
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

impl EapStationConfig {
    pub(crate) fn validate(&self) -> Result<(), WifiError> {
        if self.ssid.len() > 32 {
            return Err(WifiError::InvalidArguments);
        }

        if self.identity.as_ref().unwrap_or(&String::new()).len() > 128 {
            return Err(WifiError::InvalidArguments);
        }

        if self.username.as_ref().unwrap_or(&String::new()).len() > 128 {
            return Err(WifiError::InvalidArguments);
        }

        if self.password.as_ref().unwrap_or(&String::new()).len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        if self.new_password.as_ref().unwrap_or(&String::new()).len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        if !(6..=31).contains(&self.beacon_timeout) {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl Default for EapStationConfig {
    fn default() -> Self {
        EapStationConfig {
            ssid: String::new(),
            bssid: None,
            auth_method: AuthMethod::Wpa2Enterprise,
            identity: None,
            username: None,
            password: None,
            channel: None,
            eap_fast_config: None,
            time_check: false,
            new_password: None,
            pac_file: None,
            ca_cert: None,
            certificate_and_key: None,
            ttls_phase2_method: None,
            protocols: (Protocol::P802D11B | Protocol::P802D11BG | Protocol::P802D11BGN),
            listen_interval: 3,
            beacon_timeout: 6,
            failure_retry_cnt: 1,
            scan_method: ScanMethod::Fast,
        }
    }
}

impl fmt::Debug for EapStationConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EapStationConfig")
            .field("ssid", &self.ssid)
            .field("bssid", &self.bssid)
            .field("auth_method", &self.auth_method)
            .field("channel", &self.channel)
            .field("identity", &self.identity)
            .field("username", &self.username)
            .field("password", &"**REDACTED**")
            .field("new_password", &"**REDACTED**")
            .field("eap_fast_config", &self.eap_fast_config)
            .field("time_check", &self.time_check)
            .field("pac_file set", &self.pac_file.is_some())
            .field("ca_cert set", &self.ca_cert.is_some())
            .field("certificate_and_key set", &"**REDACTED**")
            .field("ttls_phase2_method", &self.ttls_phase2_method)
            .field("protocols", &self.protocols)
            .field("listen_interval", &self.listen_interval)
            .field("beacon_timeout", &self.beacon_timeout)
            .field("failure_retry_cnt", &self.failure_retry_cnt)
            .field("scan_method", &self.scan_method)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for EapStationConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "EapStationConfig {{\
            ssid: {}, \
            bssid: {:?}, \
            auth_method: {:?}, \
            channel: {:?}, \
            identity: {:?}, \
            username: {:?}, \
            password: **REDACTED**, \
            new_password: **REDACTED**, \
            eap_fast_config: {:?}, \
            time_check: {}, \
            pac_file: {}, \
            ca_cert: {}, \
            certificate_and_key: **REDACTED**, \
            ttls_phase2_method: {:?}, \
            protocols: {}, \
            listen_interval: {}, \
            beacon_timeout: {}, \
            failure_retry_cnt: {}, \
            scan_method: {},
            }}",
            self.ssid.as_str(),
            self.bssid,
            self.auth_method,
            self.channel,
            &self.identity.as_ref().map_or("", |v| v.as_str()),
            &self.username.as_ref().map_or("", |v| v.as_str()),
            self.eap_fast_config,
            self.time_check,
            self.pac_file,
            self.ca_cert,
            self.ttls_phase2_method,
            self.protocols,
            self.listen_interval,
            self.beacon_timeout,
            self.failure_retry_cnt,
            self.scan_method
        )
    }
}
