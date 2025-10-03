//! Wi-Fi

#![deny(missing_docs)]

pub mod event;
mod internal;
pub(crate) mod os_adapter;
pub(crate) mod state;
use alloc::{collections::vec_deque::VecDeque, string::String};
use core::{
    fmt::Debug,
    marker::PhantomData,
    mem::MaybeUninit,
    ptr::addr_of,
    task::Poll,
    time::Duration,
};

use enumset::{EnumSet, EnumSetType};
use esp_config::esp_config_int;
use esp_hal::{asynch::AtomicWaker, system::Cpu};
use esp_sync::NonReentrantMutex;
#[cfg(all(any(feature = "sniffer", feature = "esp-now"), feature = "unstable"))]
use esp_wifi_sys::include::wifi_pkt_rx_ctrl_t;
#[cfg(feature = "wifi-eap")]
use esp_wifi_sys::include::{
    esp_eap_client_clear_ca_cert,
    esp_eap_client_clear_certificate_and_key,
    esp_eap_client_clear_identity,
    esp_eap_client_clear_new_password,
    esp_eap_client_clear_password,
    esp_eap_client_clear_username,
    esp_eap_client_set_ca_cert,
    esp_eap_client_set_certificate_and_key,
    esp_eap_client_set_disable_time_check,
    esp_eap_client_set_fast_params,
    esp_eap_client_set_identity,
    esp_eap_client_set_new_password,
    esp_eap_client_set_pac_file,
    esp_eap_client_set_password,
    esp_eap_client_set_ttls_phase2_method,
    esp_eap_client_set_username,
    esp_eap_fast_config,
    esp_wifi_sta_enterprise_enable,
};
#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use esp_wifi_sys::include::{
    esp_wifi_80211_tx,
    esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_rx_cb,
    wifi_promiscuous_pkt_t,
    wifi_promiscuous_pkt_type_t,
};
use esp_wifi_sys::{
    c_types::c_uint,
    include::{
        WIFI_INIT_CONFIG_MAGIC,
        WIFI_PROTOCOL_11AX,
        WIFI_PROTOCOL_11B,
        WIFI_PROTOCOL_11G,
        WIFI_PROTOCOL_11N,
        WIFI_PROTOCOL_LR,
        esp_wifi_connect_internal,
        esp_wifi_disconnect_internal,
        wifi_init_config_t,
        wifi_scan_channel_bitmap_t,
    },
};
use num_derive::FromPrimitive;
#[doc(hidden)]
pub(crate) use os_adapter::*;
use portable_atomic::{AtomicUsize, Ordering};
use procmacros::BuilderLite;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(all(feature = "smoltcp", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
pub use state::*;

use crate::{
    Controller,
    common_adapter::*,
    esp_wifi_result,
    hal::ram,
    wifi::private::PacketBuffer,
};

const MTU: usize = esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MTU");

#[cfg(all(feature = "csi", esp32c6))]
use crate::binary::include::wifi_csi_acquire_config_t;
#[cfg(feature = "csi")]
#[instability::unstable]
pub use crate::binary::include::wifi_csi_info_t;
#[cfg(feature = "csi")]
#[instability::unstable]
use crate::binary::include::{
    esp_wifi_set_csi,
    esp_wifi_set_csi_config,
    esp_wifi_set_csi_rx_cb,
    wifi_csi_config_t,
};
use crate::binary::{
    c_types,
    include::{
        self,
        __BindgenBitfieldUnit,
        esp_err_t,
        esp_interface_t_ESP_IF_WIFI_AP,
        esp_interface_t_ESP_IF_WIFI_STA,
        esp_supplicant_deinit,
        esp_supplicant_init,
        esp_wifi_deinit_internal,
        esp_wifi_get_mode,
        esp_wifi_init_internal,
        esp_wifi_internal_free_rx_buffer,
        esp_wifi_internal_reg_rxcb,
        esp_wifi_internal_tx,
        esp_wifi_scan_start,
        esp_wifi_set_config,
        esp_wifi_set_country,
        esp_wifi_set_mode,
        esp_wifi_set_protocol,
        esp_wifi_set_tx_done_cb,
        esp_wifi_sta_get_rssi,
        esp_wifi_start,
        esp_wifi_stop,
        g_wifi_default_wpa_crypto_funcs,
        wifi_active_scan_time_t,
        wifi_ap_config_t,
        wifi_auth_mode_t,
        wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP,
        wifi_config_t,
        wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        wifi_country_t,
        wifi_interface_t,
        wifi_interface_t_WIFI_IF_AP,
        wifi_interface_t_WIFI_IF_STA,
        wifi_mode_t,
        wifi_mode_t_WIFI_MODE_AP,
        wifi_mode_t_WIFI_MODE_APSTA,
        wifi_mode_t_WIFI_MODE_NULL,
        wifi_mode_t_WIFI_MODE_STA,
        wifi_pmf_config_t,
        wifi_scan_config_t,
        wifi_scan_threshold_t,
        wifi_scan_time_t,
        wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
        wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
        wifi_sta_config_t,
    },
};

/// Supported Wi-Fi authentication methods.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[non_exhaustive]
pub enum AuthMethod {
    /// No authentication (open network).
    None,

    /// Wired Equivalent Privacy (WEP) authentication.
    Wep,

    /// Wi-Fi Protected Access (WPA) authentication.
    Wpa,

    /// Wi-Fi Protected Access 2 (WPA2) Personal authentication (default).
    #[default]
    Wpa2Personal,

    /// WPA/WPA2 Personal authentication (supports both).
    WpaWpa2Personal,

    /// WPA2 Enterprise authentication.
    Wpa2Enterprise,

    /// WPA3 Personal authentication.
    Wpa3Personal,

    /// WPA2/WPA3 Personal authentication (supports both).
    Wpa2Wpa3Personal,

    /// WLAN Authentication and Privacy Infrastructure (WAPI).
    WapiPersonal,
}

/// Supported Wi-Fi protocols.
#[derive(Debug, Default, PartialOrd, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[non_exhaustive]
pub enum Protocol {
    /// 802.11b protocol.
    P802D11B,

    /// 802.11b/g protocol.
    P802D11BG,

    /// 802.11b/g/n protocol (default).
    #[default]
    P802D11BGN,

    /// 802.11b/g/n long-range (LR) protocol.
    P802D11BGNLR,

    /// 802.11 long-range (LR) protocol.
    P802D11LR,

    /// 802.11b/g/n/ax protocol.
    P802D11BGNAX,
}

impl Protocol {
    fn to_mask(self) -> u32 {
        match self {
            Protocol::P802D11B => WIFI_PROTOCOL_11B,
            Protocol::P802D11BG => WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G,
            Protocol::P802D11BGN => WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
            Protocol::P802D11BGNLR => {
                WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR
            }
            Protocol::P802D11LR => WIFI_PROTOCOL_LR,
            Protocol::P802D11BGNAX => {
                WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX
            }
        }
    }
}

/// Secondary Wi-Fi channels.
#[derive(Clone, Debug, Default, Eq, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum SecondaryChannel {
    // TODO: Need to extend that for 5GHz
    /// No secondary channel (default).
    #[default]
    None,

    /// Secondary channel is above the primary channel.
    Above,

    /// Secondary channel is below the primary channel.
    Below,
}

/// Access point country information.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Country([u8; 2]);

impl Country {
    fn try_from_c(info: &wifi_country_t) -> Option<Self> {
        // Find the null terminator or end of array
        let cc_len = info
            .cc
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(info.cc.len());

        if cc_len < 2 {
            return None;
        }

        // Validate that we have at least 2 valid ASCII characters
        let cc_slice = &info.cc[..cc_len.min(2)];
        if cc_slice.iter().all(|&b| b.is_ascii_uppercase()) {
            Some(Self([cc_slice[0], cc_slice[1]]))
        } else {
            None
        }
    }

    /// Returns the country code as a string slice.
    pub fn country_code(&self) -> &str {
        unsafe {
            // SAFETY: we have verified in the constructor that the bytes are upper-case ASCII.
            core::str::from_utf8_unchecked(&self.0)
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Country {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        self.country_code().format(fmt)
    }
}

/// Information about a detected Wi-Fi access point.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub auth_method: Option<AuthMethod>,

    /// The country information of the access point (if available from beacon frames).
    pub country: Option<Country>,
}

/// Configuration for a Wi-Fi access point.
#[derive(BuilderLite, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccessPointConfig {
    /// The SSID of the access point.
    #[builder_lite(reference)]
    ssid: String,

    /// Whether the SSID is hidden or visible.
    ssid_hidden: bool,

    /// The channel the access point will operate on.
    channel: u8,

    /// The secondary channel configuration.
    secondary_channel: Option<u8>,

    /// The set of protocols supported by the access point.
    protocols: EnumSet<Protocol>,

    /// The authentication method to be used by the access point.
    auth_method: AuthMethod,

    /// The password for securing the access point (if applicable).
    #[builder_lite(reference)]
    password: String,

    /// The maximum number of connections allowed on the access point.
    max_connections: u16,
    /// Dtim period of the access point (Range: 1 ~ 10).
    dtim_period: u8,

    /// Time to force deauth the STA if the SoftAP doesn't receive any data.
    #[builder_lite(unstable)]
    beacon_timeout: u16,
}

impl AccessPointConfig {
    fn validate(&self) -> Result<(), WifiError> {
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
            auth_method: AuthMethod::None,
            password: String::new(),
            max_connections: 255,
            dtim_period: 2,
            beacon_timeout: 300,
        }
    }
}

impl core::fmt::Debug for AccessPointConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
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
            "AccessPointConfiguration {{\
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

/// Wi-Fi scan method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[instability::unstable]
pub enum ScanMethod {
    /// Fast scan.
    Fast,

    /// Scan all channels.
    AllChannels,
}

/// Client configuration for a Wi-Fi connection.
#[derive(BuilderLite, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ClientConfig {
    /// The SSID of the Wi-Fi network.
    #[builder_lite(reference)]
    ssid: String,

    /// The BSSID (MAC address) of the client.
    bssid: Option<[u8; 6]>,

    /// The authentication method for the Wi-Fi connection.
    auth_method: AuthMethod,

    /// The password for the Wi-Fi connection.
    #[builder_lite(reference)]
    password: String,

    /// The Wi-Fi channel to connect to.
    channel: Option<u8>,

    /// The set of protocols supported by the access point.
    protocols: EnumSet<Protocol>,

    /// Interval for station to listen to beacon from AP.
    ///
    /// The unit of listen interval is one beacon interval.
    /// For example, if beacon interval is 100 ms and listen interval is 3,
    /// the interval for station to listen to beacon is 300 ms
    #[builder_lite(unstable)]
    listen_interval: u16,

    /// Time to disconnect from AP if no data is received.
    ///
    /// Must be between 6 and 31.
    #[builder_lite(unstable)]
    beacon_timeout: u16,

    /// Number of connection retries station will do before moving to next AP.
    ///
    /// `scan_method` should be set as [`ScanMethod::AllChannels`] to use this config.
    ///
    /// Note: Enabling this may cause connection time to increase in case the best AP
    /// doesn't behave properly.
    #[builder_lite(unstable)]
    failure_retry_cnt: u8,

    /// Scan method.
    #[builder_lite(unstable)]
    scan_method: ScanMethod,
}

impl ClientConfig {
    fn validate(&self) -> Result<(), WifiError> {
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

impl Default for ClientConfig {
    fn default() -> Self {
        ClientConfig {
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

impl core::fmt::Debug for ClientConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ClientConfig")
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
impl defmt::Format for ClientConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "ClientConfiguration {{\
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

/// Configuration for EAP-FAST authentication protocol.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg(feature = "wifi-eap")]
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
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg(feature = "wifi-eap")]
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

#[cfg(feature = "wifi-eap")]
impl TtlsPhase2Method {
    /// Maps the phase 2 method to a raw `u32` representation.
    fn to_raw(&self) -> u32 {
        match self {
            TtlsPhase2Method::Eap => {
                esp_wifi_sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_EAP
            }
            TtlsPhase2Method::Mschapv2 => {
                esp_wifi_sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_MSCHAPV2
            }
            TtlsPhase2Method::Mschap => {
                esp_wifi_sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_MSCHAP
            }
            TtlsPhase2Method::Pap => {
                esp_wifi_sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_PAP
            }
            TtlsPhase2Method::Chap => {
                esp_wifi_sys::include::esp_eap_ttls_phase2_types_ESP_EAP_TTLS_PHASE2_CHAP
            }
        }
    }
}

#[cfg(feature = "wifi-eap")]
type CertificateAndKey = (&'static [u8], &'static [u8], Option<&'static [u8]>);

/// Configuration for an EAP (Extensible Authentication Protocol) client.
#[derive(BuilderLite, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[cfg(feature = "wifi-eap")]
#[instability::unstable]
pub struct EapClientConfig {
    /// The SSID of the network the client is connecting to.
    #[builder_lite(reference)]
    ssid: String,

    /// The BSSID (MAC Address) of the specific access point.
    bssid: Option<[u8; 6]>,

    /// The authentication method used for EAP.
    auth_method: AuthMethod,

    /// The identity used during authentication.
    #[builder_lite(reference)]
    identity: Option<String>,

    /// The username used for inner authentication.
    /// Some EAP methods require a username for authentication.
    #[builder_lite(reference)]
    username: Option<String>,

    /// The password used for inner authentication.
    #[builder_lite(reference)]
    password: Option<String>,

    /// A new password to be set during the authentication process.
    /// Some methods support password changes during authentication.
    #[builder_lite(reference)]
    new_password: Option<String>,

    /// Configuration for EAP-FAST.
    #[builder_lite(reference)]
    eap_fast_config: Option<EapFastConfig>,

    /// A PAC (Protected Access Credential) file for EAP-FAST.
    pac_file: Option<&'static [u8]>,

    /// A boolean flag indicating whether time checking is enforced during
    /// authentication.
    time_check: bool,

    /// A CA (Certificate Authority) certificate for validating the
    /// authentication server's certificate.
    ca_cert: Option<&'static [u8]>,

    /// A tuple containing the client's certificate, private key, and an
    /// intermediate certificate.
    certificate_and_key: Option<CertificateAndKey>,
    /// The Phase 2 authentication method used for EAP-TTLS.
    #[builder_lite(reference)]
    ttls_phase2_method: Option<TtlsPhase2Method>,

    /// The specific Wi-Fi channel to use for the connection.
    channel: Option<u8>,

    /// The set of protocols supported by the access point.
    protocols: EnumSet<Protocol>,

    /// Interval for station to listen to beacon from AP.
    ///
    /// The unit of listen interval is one beacon interval.
    /// For example, if beacon interval is 100 ms and listen interval is 3,
    /// the interval for station to listen to beacon is 300 ms
    #[builder_lite(unstable)]
    listen_interval: u16,

    /// Time to disconnect from AP if no data is received.
    ///
    /// Must be between 6 and 31.
    #[builder_lite(unstable)]
    beacon_timeout: u16,

    /// Number of connection retries station will do before moving to next AP.
    ///
    /// `scan_method` should be set as [`ScanMethod::AllChannels`] to use this config.
    ///
    /// Note: Enabling this may cause connection time to increase in case the best AP
    /// doesn't behave properly.
    #[builder_lite(unstable)]
    failure_retry_cnt: u8,

    /// Scan method.
    #[builder_lite(unstable)]
    scan_method: ScanMethod,
}

#[cfg(feature = "wifi-eap")]
impl EapClientConfig {
    fn validate(&self) -> Result<(), WifiError> {
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

#[cfg(feature = "wifi-eap")]
impl Debug for EapClientConfig {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EapClientConfig")
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
#[cfg(feature = "wifi-eap")]
impl defmt::Format for EapClientConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "EapClientConfiguration {{\
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

#[cfg(feature = "wifi-eap")]
impl Default for EapClientConfig {
    fn default() -> Self {
        EapClientConfig {
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

/// Introduces Wi-Fi configuration options.
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[non_exhaustive]
pub enum Capability {
    /// The device operates as a client, connecting to an existing network.
    Client,

    /// The device operates as an access point, allowing other devices to
    /// connect to it.
    AccessPoint,

    /// The device can operate in both client and access point modes
    /// simultaneously.
    ApSta,
}

/// Configuration of Wi-Fi operation mode.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[non_exhaustive]
pub enum Config {
    /// No configuration (default).
    #[default]
    None,

    /// Client-only configuration.
    Client(ClientConfig),

    /// Access point-only configuration.
    AccessPoint(AccessPointConfig),

    /// Simultaneous client and access point configuration.
    ApSta(ClientConfig, AccessPointConfig),

    /// EAP client configuration for enterprise Wi-Fi.
    #[cfg(feature = "wifi-eap")]
    #[cfg_attr(feature = "serde", serde(skip))]
    EapClient(EapClientConfig),
}

impl Config {
    fn validate(&self) -> Result<(), WifiError> {
        match self {
            Config::None => Ok(()),
            Config::Client(client_configuration) => client_configuration.validate(),
            Config::AccessPoint(access_point_configuration) => {
                access_point_configuration.validate()
            }
            Config::ApSta(client_configuration, access_point_configuration) => {
                client_configuration.validate()?;
                access_point_configuration.validate()
            }
            #[cfg(feature = "wifi-eap")]
            Config::EapClient(eap_client_configuration) => eap_client_configuration.validate(),
        }
    }
}

trait AuthMethodExt {
    fn to_raw(&self) -> wifi_auth_mode_t;
    fn from_raw(raw: wifi_auth_mode_t) -> Self;
}

impl AuthMethodExt for AuthMethod {
    fn to_raw(&self) -> wifi_auth_mode_t {
        match self {
            AuthMethod::None => include::wifi_auth_mode_t_WIFI_AUTH_OPEN,
            AuthMethod::Wep => include::wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::Wpa => include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::Wpa2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WpaWpa2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
            AuthMethod::Wpa2Enterprise => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE,
            AuthMethod::Wpa3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthMethod::Wpa2Wpa3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK,
            AuthMethod::WapiPersonal => include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
        }
    }

    fn from_raw(raw: wifi_auth_mode_t) -> Self {
        match raw {
            include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
            include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::Wep,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthMethod::Wpa,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => AuthMethod::Wpa2Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => AuthMethod::WpaWpa2Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => AuthMethod::Wpa2Enterprise,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthMethod::Wpa3Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => AuthMethod::Wpa2Wpa3Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => AuthMethod::WapiPersonal,
            _ => unreachable!(),
        }
    }
}

/// Wi-Fi Mode (Sta and/or Ap)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[non_exhaustive]
pub enum WifiMode {
    /// Station mode.
    Sta,
    /// Access Point mode.
    Ap,
    /// Both Station and Access Point modes.
    ApSta,
}

impl WifiMode {
    pub(crate) fn current() -> Result<Self, WifiError> {
        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut mode) })?;

        Self::try_from(mode)
    }

    /// Returns true if this mode works as a client
    pub fn is_sta(&self) -> bool {
        match self {
            Self::Sta | Self::ApSta => true,
            Self::Ap => false,
        }
    }

    /// Returns true if this mode works as an access point
    pub fn is_ap(&self) -> bool {
        match self {
            Self::Sta => false,
            Self::Ap | Self::ApSta => true,
        }
    }
}

impl TryFrom<&Config> for WifiMode {
    type Error = WifiError;

    /// Based on the current `Config`, derives a `WifiMode` based on it.
    fn try_from(config: &Config) -> Result<Self, Self::Error> {
        let mode = match config {
            Config::None => return Err(WifiError::UnknownWifiMode),
            Config::AccessPoint(_) => Self::Ap,
            Config::Client(_) => Self::Sta,
            Config::ApSta(_, _) => Self::ApSta,
            #[cfg(feature = "wifi-eap")]
            Config::EapClient(_) => Self::Sta,
        };

        Ok(mode)
    }
}

#[doc(hidden)]
impl TryFrom<wifi_mode_t> for WifiMode {
    type Error = WifiError;

    /// Converts a `wifi_mode_t` C-type into a `WifiMode`.
    fn try_from(value: wifi_mode_t) -> Result<Self, Self::Error> {
        #[allow(non_upper_case_globals)]
        match value {
            include::wifi_mode_t_WIFI_MODE_STA => Ok(Self::Sta),
            include::wifi_mode_t_WIFI_MODE_AP => Ok(Self::Ap),
            include::wifi_mode_t_WIFI_MODE_APSTA => Ok(Self::ApSta),
            _ => Err(WifiError::UnknownWifiMode),
        }
    }
}

#[doc(hidden)]
impl From<WifiMode> for wifi_mode_t {
    fn from(val: WifiMode) -> Self {
        #[allow(non_upper_case_globals)]
        match val {
            WifiMode::Sta => wifi_mode_t_WIFI_MODE_STA,
            WifiMode::Ap => wifi_mode_t_WIFI_MODE_AP,
            WifiMode::ApSta => wifi_mode_t_WIFI_MODE_APSTA,
        }
    }
}

#[cfg(feature = "csi")]
pub(crate) trait CsiCallback: FnMut(crate::binary::include::wifi_csi_info_t) {}

#[cfg(feature = "csi")]
impl<T> CsiCallback for T where T: FnMut(crate::binary::include::wifi_csi_info_t) {}

#[cfg(feature = "csi")]
unsafe extern "C" fn csi_rx_cb<C: CsiCallback>(
    ctx: *mut crate::wifi::c_types::c_void,
    data: *mut crate::binary::include::wifi_csi_info_t,
) {
    unsafe {
        let csi_callback = &mut *(ctx as *mut C);
        csi_callback(*data);
    }
}

/// Channel state information (CSI) configuration.
#[derive(Clone, PartialEq, Eq)]
#[cfg(all(not(esp32c6), feature = "csi"))]
pub struct CsiConfig {
    /// Enable to receive legacy long training field(lltf) data.
    pub lltf_en: bool,
    /// Enable to receive HT long training field(htltf) data.
    pub htltf_en: bool,
    /// Enable to receive space time block code HT long training
    /// field(stbc-htltf2) data.
    pub stbc_htltf2_en: bool,
    /// Enable to generate htlft data by averaging lltf and ht_ltf data when
    /// receiving HT packet. Otherwise, use ht_ltf data directly.
    pub ltf_merge_en: bool,
    /// Enable to turn on channel filter to smooth adjacent sub-carrier. Disable
    /// it to keep independence of adjacent sub-carrier.
    pub channel_filter_en: bool,
    /// Manually scale the CSI data by left shifting or automatically scale the
    /// CSI data. If set true, please set the shift bits. false: automatically.
    /// true: manually.
    pub manu_scale: bool,
    /// Manually left shift bits of the scale of the CSI data. The range of the
    /// left shift bits is 0~15.
    pub shift: u8,
    /// Enable to dump 802.11 ACK frame.
    pub dump_ack_en: bool,
}

/// Channel state information (CSI) configuration.
#[derive(Clone, PartialEq, Eq)]
#[cfg(all(esp32c6, feature = "csi"))]
pub struct CsiConfig {
    /// Enable to acquire CSI.
    pub enable: u32,
    /// Enable to acquire L-LTF when receiving a 11g PPDU.
    pub acquire_csi_legacy: u32,
    /// Enable to acquire HT-LTF when receiving an HT20 PPDU.
    pub acquire_csi_ht20: u32,
    /// Enable to acquire HT-LTF when receiving an HT40 PPDU.
    pub acquire_csi_ht40: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 SU PPDU.
    pub acquire_csi_su: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 MU PPDU.
    pub acquire_csi_mu: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 DCM applied PPDU.
    pub acquire_csi_dcm: u32,
    /// Enable to acquire HE-LTF when receiving an HE20 Beamformed applied PPDU.
    pub acquire_csi_beamformed: u32,
    /// When receiving an STBC applied HE PPDU, 0- acquire the complete
    /// HE-LTF1,  1- acquire the complete HE-LTF2, 2- sample evenly among the
    /// HE-LTF1 and HE-LTF2.
    pub acquire_csi_he_stbc: u32,
    /// Vvalue 0-3.
    pub val_scale_cfg: u32,
    /// Enable to dump 802.11 ACK frame, default disabled.
    pub dump_ack_en: u32,
    /// Reserved.
    pub reserved: u32,
}

#[cfg(feature = "csi")]
impl Default for CsiConfig {
    #[cfg(not(esp32c6))]
    fn default() -> Self {
        Self {
            lltf_en: true,
            htltf_en: true,
            stbc_htltf2_en: true,
            ltf_merge_en: true,
            channel_filter_en: true,
            manu_scale: false,
            shift: 0,
            dump_ack_en: false,
        }
    }

    #[cfg(esp32c6)]
    fn default() -> Self {
        // https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/headers/esp_wifi_he_types.h#L67-L82
        Self {
            enable: 1,
            acquire_csi_legacy: 1,
            acquire_csi_ht20: 1,
            acquire_csi_ht40: 1,
            acquire_csi_su: 1,
            acquire_csi_mu: 1,
            acquire_csi_dcm: 1,
            acquire_csi_beamformed: 1,
            acquire_csi_he_stbc: 2,
            val_scale_cfg: 2,
            dump_ack_en: 1,
            reserved: 19,
        }
    }
}

#[doc(hidden)]
#[cfg(feature = "csi")]
impl From<CsiConfig> for wifi_csi_config_t {
    fn from(config: CsiConfig) -> Self {
        #[cfg(not(esp32c6))]
        {
            wifi_csi_config_t {
                lltf_en: config.lltf_en,
                htltf_en: config.htltf_en,
                stbc_htltf2_en: config.stbc_htltf2_en,
                ltf_merge_en: config.ltf_merge_en,
                channel_filter_en: config.channel_filter_en,
                manu_scale: config.manu_scale,
                shift: config.shift,
                dump_ack_en: config.dump_ack_en,
            }
        }
        #[cfg(esp32c6)]
        {
            wifi_csi_acquire_config_t {
                _bitfield_align_1: [0; 0],
                _bitfield_1: wifi_csi_acquire_config_t::new_bitfield_1(
                    config.enable,
                    config.acquire_csi_legacy,
                    config.acquire_csi_ht20,
                    config.acquire_csi_ht40,
                    config.acquire_csi_su,
                    config.acquire_csi_mu,
                    config.acquire_csi_dcm,
                    config.acquire_csi_beamformed,
                    config.acquire_csi_he_stbc,
                    config.val_scale_cfg,
                    config.dump_ack_en,
                    config.reserved,
                ),
            }
        }
    }
}

#[cfg(feature = "csi")]
impl CsiConfig {
    /// Set CSI data configuration
    pub(crate) fn apply_config(&self) -> Result<(), WifiError> {
        let conf: wifi_csi_config_t = self.clone().into();

        unsafe {
            esp_wifi_result!(esp_wifi_set_csi_config(&conf))?;
        }
        Ok(())
    }

    /// Register the RX callback function of CSI data. Each time a CSI data is
    /// received, the callback function will be called.
    pub(crate) fn set_receive_cb<C: CsiCallback>(&mut self, cb: C) -> Result<(), WifiError> {
        let cb = alloc::boxed::Box::new(cb);
        let cb_ptr = alloc::boxed::Box::into_raw(cb) as *mut crate::wifi::c_types::c_void;

        unsafe {
            esp_wifi_result!(esp_wifi_set_csi_rx_cb(Some(csi_rx_cb::<C>), cb_ptr))?;
        }
        Ok(())
    }

    /// Enable or disable CSI
    pub(crate) fn set_csi(&self, enable: bool) -> Result<(), WifiError> {
        // https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/headers/esp_wifi.h#L1241
        unsafe {
            esp_wifi_result!(esp_wifi_set_csi(enable))?;
        }
        Ok(())
    }
}

static RX_QUEUE_SIZE: AtomicUsize = AtomicUsize::new(0);
static TX_QUEUE_SIZE: AtomicUsize = AtomicUsize::new(0);

pub(crate) static DATA_QUEUE_RX_AP: NonReentrantMutex<VecDeque<PacketBuffer>> =
    NonReentrantMutex::new(VecDeque::new());

pub(crate) static DATA_QUEUE_RX_STA: NonReentrantMutex<VecDeque<PacketBuffer>> =
    NonReentrantMutex::new(VecDeque::new());

/// Common errors.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WifiError {
    /// Wi-Fi module is not initialized or not initialized for `Wi-Fi`
    /// operations.
    NotInitialized,

    /// Internal Wi-Fi error.
    InternalError(InternalWifiError),

    /// The device disconnected from the network or failed to connect to it.
    Disconnected,

    /// Unknown Wi-Fi mode (not Sta/Ap/ApSta).
    UnknownWifiMode,

    /// Unsupported operation or mode.
    Unsupported,

    /// Passed arguments are invalid.
    InvalidArguments,
}

/// Events generated by the Wi-Fi driver.
impl core::fmt::Display for WifiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WifiError::NotInitialized => write!(f, "Wi-Fi not initialized."),
            WifiError::InternalError(err) => write!(f, "Internal Wi-Fi error: {err:?}."),
            WifiError::Disconnected => write!(f, "Wi-Fi disconnected."),
            WifiError::UnknownWifiMode => write!(f, "Unknown Wi-Fi mode."),
            WifiError::Unsupported => write!(f, "Unsupported operation or mode."),
            WifiError::InvalidArguments => write!(f, "Invalid arguments."),
        }
    }
}

impl core::error::Error for WifiError {}

/// Events generated by the Wi-Fi driver.
#[derive(Debug, FromPrimitive, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[repr(i32)]
pub enum WifiEvent {
    /// Wi-Fi is ready for operation.
    WifiReady = 0,
    /// Scan operation has completed.
    ScanDone,
    /// Station mode started.
    StaStart,
    /// Station mode stopped.
    StaStop,
    /// Station connected to a network.
    StaConnected,
    /// Station disconnected from a network.
    StaDisconnected,
    /// Station authentication mode changed.
    StaAuthmodeChange,

    /// Station WPS succeeds in enrollee mode.
    StaWpsErSuccess,
    /// Station WPS fails in enrollee mode.
    StaWpsErFailed,
    /// Station WPS timeout in enrollee mode.
    StaWpsErTimeout,
    /// Station WPS pin code in enrollee mode.
    StaWpsErPin,
    /// Station WPS overlap in enrollee mode.
    StaWpsErPbcOverlap,

    /// Soft-AP start.
    ApStart,
    /// Soft-AP stop.
    ApStop,
    /// A station connected to Soft-AP.
    ApStaConnected,
    /// A station disconnected from Soft-AP.
    ApStaDisconnected,
    /// Received probe request packet in Soft-AP interface.
    ApProbeReqReceived,

    /// Received report of FTM procedure.
    FtmReport,

    /// AP's RSSI crossed configured threshold.
    StaBssRssiLow,
    /// Status indication of Action Tx operation.
    ActionTxStatus,
    /// Remain-on-Channel operation complete.
    RocDone,

    /// Station beacon timeout.
    StaBeaconTimeout,

    /// Connectionless module wake interval has started.
    ConnectionlessModuleWakeIntervalStart,

    /// Soft-AP WPS succeeded in registrar mode.
    ApWpsRgSuccess,
    /// Soft-AP WPS failed in registrar mode.
    ApWpsRgFailed,
    /// Soft-AP WPS timed out in registrar mode.
    ApWpsRgTimeout,
    /// Soft-AP WPS pin code in registrar mode.
    ApWpsRgPin,
    /// Soft-AP WPS overlap in registrar mode.
    ApWpsRgPbcOverlap,

    /// iTWT setup.
    ItwtSetup,
    /// iTWT teardown.
    ItwtTeardown,
    /// iTWT probe.
    ItwtProbe,
    /// iTWT suspended.
    ItwtSuspend,
    /// TWT wakeup event.
    TwtWakeup,
    /// bTWT setup.
    BtwtSetup,
    /// bTWT teardown.
    BtwtTeardown,

    /// NAN (Neighbor Awareness Networking) discovery has started.
    NanStarted,
    /// NAN discovery has stopped.
    NanStopped,
    /// NAN service discovery match found.
    NanSvcMatch,
    /// Replied to a NAN peer with service discovery match.
    NanReplied,
    /// Received a follow-up message in NAN.
    NanReceive,
    /// Received NDP (Neighbor Discovery Protocol) request from a NAN peer.
    NdpIndication,
    /// NDP confirm indication.
    NdpConfirm,
    /// NAN datapath terminated indication.
    NdpTerminated,
    /// Wi-Fi home channel change, doesn't occur when scanning.
    HomeChannelChange,

    /// Received Neighbor Report response.
    StaNeighborRep,
}

/// Error originating from the underlying drivers
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[repr(i32)]
pub enum InternalWifiError {
    /// Out of memory
    NoMem            = 0x101,

    /// Invalid argument
    InvalidArg       = 0x102,

    /// Wi-Fi driver was not installed by [esp_wifi_init](crate::binary::include::esp_wifi_init)
    NotInit          = 0x3001,

    /// Wi-Fi driver was not started by [esp_wifi_start]
    NotStarted       = 0x3002,

    /// Wi-Fi driver was not stopped by [esp_wifi_stop]
    NotStopped       = 0x3003,

    /// Wi-Fi interface error
    Interface        = 0x3004,

    /// Wi-Fi mode error
    Mode             = 0x3005,

    /// Wi-Fi internal state error
    State            = 0x3006,

    /// Wi-Fi internal control block of station or soft-AP error
    Conn             = 0x3007,

    /// Wi-Fi internal NVS module error
    Nvs              = 0x3008,

    /// MAC address is invalid
    InvalidMac       = 0x3009,

    /// SSID is invalid
    InvalidSsid      = 0x300A,

    /// Password is invalid
    InvalidPassword  = 0x300B,

    /// Timeout error
    Timeout          = 0x300C,

    /// Wi-Fi is in sleep state (RF closed) and wakeup failed
    WakeFail         = 0x300D,

    /// The caller would block
    WouldBlock       = 0x300E,

    /// Station still in disconnect status
    NotConnected     = 0x300F,

    /// Failed to post the event to Wi-Fi task
    PostFail         = 0x3012,

    /// Invalid Wi-Fi state when init/deinit is called
    InvalidInitState = 0x3013,

    /// Returned when Wi-Fi is stopping
    StopState        = 0x3014,

    /// The Wi-Fi connection is not associated
    NotAssociated    = 0x3015,

    /// The Wi-Fi TX is disallowed
    TxDisallowed     = 0x3016,
}

/// Get the AP MAC address of the device.
pub fn ap_mac() -> [u8; 6] {
    let mut mac = [0u8; 6];
    unsafe {
        read_mac(mac.as_mut_ptr(), 1);
    }
    mac
}

/// Get the STA MAC address of the device.
pub fn sta_mac() -> [u8; 6] {
    let mut mac = [0u8; 6];
    unsafe {
        read_mac(mac.as_mut_ptr(), 0);
    }
    mac
}

#[cfg(esp32)]
fn set_mac_time_update_cb(wifi: crate::hal::peripherals::WIFI<'_>) {
    use esp_phy::MacTimeExt;
    use esp_wifi_sys::include::esp_wifi_internal_update_mac_time;
    unsafe {
        wifi.set_mac_time_update_cb(|duration| {
            esp_wifi_internal_update_mac_time(duration.as_micros() as u32);
        });
    }
}

pub(crate) fn wifi_init(_wifi: crate::hal::peripherals::WIFI<'_>) -> Result<(), WifiError> {
    #[cfg(esp32)]
    set_mac_time_update_cb(_wifi);
    unsafe {
        #[cfg(coex)]
        esp_wifi_result!(coex_init())?;

        esp_wifi_result!(esp_wifi_init_internal(addr_of!(internal::G_CONFIG)))?;
        esp_wifi_result!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;

        esp_wifi_result!(esp_supplicant_init())?;

        esp_wifi_result!(esp_wifi_set_tx_done_cb(Some(esp_wifi_tx_done_cb)))?;

        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_STA,
            Some(recv_cb_sta)
        ))?;

        // until we support APSTA we just register the same callback for AP and STA
        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_AP,
            Some(recv_cb_ap)
        ))?;

        Ok(())
    }
}

#[cfg(coex)]
pub(crate) fn coex_initialize() -> i32 {
    debug!("call coex-initialize");
    unsafe {
        let res = crate::binary::include::esp_coex_adapter_register(
            core::ptr::addr_of_mut!(internal::G_COEX_ADAPTER_FUNCS).cast(),
        );
        if res != 0 {
            error!("Error: esp_coex_adapter_register {}", res);
            return res;
        }
        let res = crate::binary::include::coex_pre_init();
        if res != 0 {
            error!("Error: coex_pre_init {}", res);
            return res;
        }
        0
    }
}

pub(crate) unsafe extern "C" fn coex_init() -> i32 {
    #[cfg(coex)]
    {
        debug!("coex-init");
        #[allow(clippy::needless_return)]
        return unsafe { crate::binary::include::coex_init() };
    }

    #[cfg(not(coex))]
    0
}

fn wifi_deinit() -> Result<(), crate::InitializationError> {
    esp_wifi_result!(unsafe { esp_wifi_stop() })?;
    esp_wifi_result!(unsafe { esp_wifi_deinit_internal() })?;
    esp_wifi_result!(unsafe { esp_supplicant_deinit() })?;
    Ok(())
}

unsafe extern "C" fn recv_cb_sta(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let packet = PacketBuffer { buffer, len, eb };
    // We must handle the result outside of the lock because
    // PacketBuffer::drop must not be called in a critical section.
    // Dropping an PacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    match DATA_QUEUE_RX_STA.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE.load(Ordering::Relaxed) {
            queue.push_back(packet);
            Ok(())
        } else {
            Err(packet)
        }
    }) {
        Ok(()) => {
            embassy::STA_RECEIVE_WAKER.wake();
            include::ESP_OK as esp_err_t
        }
        _ => {
            debug!("RX QUEUE FULL");
            include::ESP_ERR_NO_MEM as esp_err_t
        }
    }
}

unsafe extern "C" fn recv_cb_ap(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let packet = PacketBuffer { buffer, len, eb };
    // We must handle the result outside of the critical section because
    // PacketBuffer::drop must not be called in a critical section.
    // Dropping an PacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    match DATA_QUEUE_RX_AP.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE.load(Ordering::Relaxed) {
            queue.push_back(packet);
            Ok(())
        } else {
            Err(packet)
        }
    }) {
        Ok(()) => {
            embassy::AP_RECEIVE_WAKER.wake();
            include::ESP_OK as esp_err_t
        }
        _ => {
            debug!("RX QUEUE FULL");
            include::ESP_ERR_NO_MEM as esp_err_t
        }
    }
}

pub(crate) static WIFI_TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);

fn decrement_inflight_counter() {
    unwrap!(
        WIFI_TX_INFLIGHT.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
            Some(x.saturating_sub(1))
        })
    );
}

#[ram]
unsafe extern "C" fn esp_wifi_tx_done_cb(
    _ifidx: u8,
    _data: *mut u8,
    _data_len: *mut u16,
    _tx_status: bool,
) {
    trace!("esp_wifi_tx_done_cb");

    decrement_inflight_counter();

    embassy::TRANSMIT_WAKER.wake();
}

/// Configuration for active or passive scan.
///
/// # Comparison of active and passive scan
///
/// |                                      | **Active** | **Passive** |
/// |--------------------------------------|------------|-------------|
/// | **Power consumption**                |    High    |     Low     |
/// | **Time required (typical behavior)** |     Low    |     High    |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    /// It is recommended to avoid duration longer thean 1500ms, as it may cause
    /// a station to disconnect from the AP.
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
    fn validate(&self) {
        if matches!(self, Self::Passive(dur) if *dur > Duration::from_millis(1500)) {
            warn!(
                "Passive scan duration longer than 1500ms may cause a station to disconnect from the AP"
            );
        }
    }
}

/// Scan configuration
#[derive(Clone, Copy, Default, PartialEq, Eq, BuilderLite)]
pub struct ScanConfig<'a> {
    /// SSID to filter for.
    /// If [`None`] is passed, all SSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given SSID will be
    /// returned.
    ssid: Option<&'a str>,
    /// BSSID to filter for.
    /// If [`None`] is passed, all BSSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given BSSID will be
    /// returned.
    bssid: Option<[u8; 6]>,
    /// Channel to filter for.
    /// If [`None`] is passed, all channels will be returned.
    /// If [`Some`] is passed, only the APs on the given channel will be
    /// returned.
    channel: Option<u8>,
    /// Whether to show hidden networks.
    show_hidden: bool,
    /// Scan type, active or passive.
    scan_type: ScanTypeConfig,
    /// The maximum number of networks to return when scanning.
    /// If [`None`] is passed, all networks will be returned.
    /// If [`Some`] is passed, the specified number of networks will be returned.
    max: Option<usize>,
}

pub(crate) fn wifi_start_scan(
    block: bool,
    ScanConfig {
        ssid,
        mut bssid,
        channel,
        show_hidden,
        scan_type,
        ..
    }: ScanConfig<'_>,
) -> i32 {
    scan_type.validate();
    let (scan_time, scan_type) = match scan_type {
        ScanTypeConfig::Active { min, max } => (
            wifi_scan_time_t {
                active: wifi_active_scan_time_t {
                    min: min.as_millis() as u32,
                    max: max.as_millis() as u32,
                },
                passive: 0,
            },
            wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        ),
        ScanTypeConfig::Passive(dur) => (
            wifi_scan_time_t {
                active: wifi_active_scan_time_t { min: 0, max: 0 },
                passive: dur.as_millis() as u32,
            },
            wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
        ),
    };

    let mut ssid_buf = ssid.map(|m| {
        let mut buf = alloc::vec::Vec::from_iter(m.bytes());
        buf.push(b'\0');
        buf
    });

    let ssid = ssid_buf
        .as_mut()
        .map(|e| e.as_mut_ptr())
        .unwrap_or_else(core::ptr::null_mut);
    let bssid = bssid
        .as_mut()
        .map(|e| e.as_mut_ptr())
        .unwrap_or_else(core::ptr::null_mut);

    let scan_config = wifi_scan_config_t {
        ssid,
        bssid,
        channel: channel.unwrap_or(0),
        show_hidden,
        scan_type,
        scan_time,
        home_chan_dwell_time: 0,
        channel_bitmap: wifi_scan_channel_bitmap_t {
            ghz_2_channels: 0,
            ghz_5_channels: 0,
        },
        coex_background_scan: false,
    };

    unsafe { esp_wifi_scan_start(&scan_config, block) }
}

mod private {
    use super::*;

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    /// Take care not to drop this while in a critical section.
    ///
    /// Dropping an PacketBuffer will call
    /// `esp_wifi_internal_free_rx_buffer` which will try to lock an
    /// internal mutex. If the mutex is already taken, the function will try
    /// to trigger a context switch, which will fail if we are in a critical
    /// section.
    pub struct PacketBuffer {
        pub(crate) buffer: *mut c_types::c_void,
        pub(crate) len: u16,
        pub(crate) eb: *mut c_types::c_void,
    }

    unsafe impl Send for PacketBuffer {}

    impl Drop for PacketBuffer {
        fn drop(&mut self) {
            trace!("Dropping PacketBuffer, freeing memory");
            unsafe { esp_wifi_internal_free_rx_buffer(self.eb) };
        }
    }

    impl PacketBuffer {
        pub fn as_slice_mut(&mut self) -> &mut [u8] {
            unsafe { core::slice::from_raw_parts_mut(self.buffer as *mut u8, self.len as usize) }
        }
    }
}

/// Wi-Fi device operational modes.
#[derive(Debug, Clone, Copy)]
enum WifiDeviceMode {
    /// Station mode.
    Sta,
    /// Access Point mode.
    Ap,
}

impl WifiDeviceMode {
    fn mac_address(&self) -> [u8; 6] {
        match self {
            WifiDeviceMode::Sta => sta_mac(),
            WifiDeviceMode::Ap => ap_mac(),
        }
    }

    fn data_queue_rx(&self) -> &'static NonReentrantMutex<VecDeque<PacketBuffer>> {
        match self {
            WifiDeviceMode::Sta => &DATA_QUEUE_RX_STA,
            WifiDeviceMode::Ap => &DATA_QUEUE_RX_AP,
        }
    }

    fn can_send(&self) -> bool {
        WIFI_TX_INFLIGHT.load(Ordering::SeqCst) < TX_QUEUE_SIZE.load(Ordering::Relaxed)
    }

    fn increase_in_flight_counter(&self) {
        WIFI_TX_INFLIGHT.fetch_add(1, Ordering::SeqCst);
    }

    fn tx_token(&self) -> Option<WifiTxToken> {
        if !self.can_send() {
            // TODO: perhaps we can use a counting semaphore with a short blocking timeout
            crate::preempt::yield_task();
        }

        if self.can_send() {
            Some(WifiTxToken { mode: *self })
        } else {
            None
        }
    }

    fn rx_token(&self) -> Option<(WifiRxToken, WifiTxToken)> {
        let is_empty = self.data_queue_rx().with(|q| q.is_empty());
        if is_empty || !self.can_send() {
            // TODO: use an OS queue with a short timeout
            crate::preempt::yield_task();
        }

        let is_empty = is_empty && self.data_queue_rx().with(|q| q.is_empty());

        if !is_empty {
            self.tx_token().map(|tx| (WifiRxToken { mode: *self }, tx))
        } else {
            None
        }
    }

    fn interface(&self) -> wifi_interface_t {
        match self {
            WifiDeviceMode::Sta => wifi_interface_t_WIFI_IF_STA,
            WifiDeviceMode::Ap => wifi_interface_t_WIFI_IF_AP,
        }
    }

    fn register_transmit_waker(&self, cx: &mut core::task::Context<'_>) {
        embassy::TRANSMIT_WAKER.register(cx.waker())
    }

    fn register_receive_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Sta => embassy::STA_RECEIVE_WAKER.register(cx.waker()),
            WifiDeviceMode::Ap => embassy::AP_RECEIVE_WAKER.register(cx.waker()),
        }
    }

    fn register_link_state_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Sta => embassy::STA_LINK_STATE_WAKER.register(cx.waker()),
            WifiDeviceMode::Ap => embassy::AP_LINK_STATE_WAKER.register(cx.waker()),
        }
    }

    fn link_state(&self) -> embassy_net_driver::LinkState {
        match self {
            WifiDeviceMode::Sta => {
                if matches!(sta_state(), WifiStaState::Connected) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
            WifiDeviceMode::Ap => {
                if matches!(ap_state(), WifiApState::Started) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
        }
    }
}

/// A wifi device implementing smoltcp's Device trait.
pub struct WifiDevice<'d> {
    _phantom: PhantomData<&'d ()>,
    mode: WifiDeviceMode,
}

impl WifiDevice<'_> {
    /// Retrieves the MAC address of the Wi-Fi device.
    pub fn mac_address(&self) -> [u8; 6] {
        self.mode.mac_address()
    }

    /// Receives data from the Wi-Fi device (only when `smoltcp` feature is
    /// disabled).
    #[cfg(not(feature = "smoltcp"))]
    pub fn receive(&mut self) -> Option<(WifiRxToken, WifiTxToken)> {
        self.mode.rx_token()
    }

    /// Transmits data through the Wi-Fi device (only when `smoltcp` feature is
    /// disabled).
    #[cfg(not(feature = "smoltcp"))]
    pub fn transmit(&mut self) -> Option<WifiTxToken> {
        self.mode.tx_token()
    }
}

fn convert_ap_info(record: &include::wifi_ap_record_t) -> AccessPointInfo {
    let str_len = record
        .ssid
        .iter()
        .position(|&c| c == 0)
        .unwrap_or(record.ssid.len());
    let ssid_ref = unsafe { core::str::from_utf8_unchecked(&record.ssid[..str_len]) };

    let mut ssid = String::new();
    ssid.push_str(ssid_ref);

    AccessPointInfo {
        ssid,
        bssid: record.bssid,
        channel: record.primary,
        secondary_channel: match record.second {
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => SecondaryChannel::None,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => SecondaryChannel::Above,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => SecondaryChannel::Below,
            _ => panic!(),
        },
        signal_strength: record.rssi,
        auth_method: Some(AuthMethod::from_raw(record.authmode)),
        country: Country::try_from_c(&record.country),
    }
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(not(esp32c6))]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
pub struct RxControlInfo {
    /// Received Signal Strength Indicator (RSSI) of the packet, in dBm.
    pub rssi: i32,
    /// PHY rate encoding of the packet. Only valid for non-HT (802.11b/g)
    /// packets.
    pub rate: u32,
    /// Protocol of the received packet: 0 for non-HT (11bg), 1 for HT (11n), 3
    /// for VHT (11ac).
    pub sig_mode: u32,
    /// Modulation and Coding Scheme (MCS). Indicates modulation for HT (11n)
    /// packets.
    pub mcs: u32,
    /// Channel bandwidth of the packet: 0 for 20MHz, 1 for 40MHz.
    pub cwb: u32,
    /// Channel estimate smoothing: 1 recommends smoothing; 0 recommends
    /// per-carrier-independent estimate.
    pub smoothing: u32,
    /// Sounding indicator: 0 for sounding PPDU (used for channel estimation); 1
    /// for non-sounding PPDU.
    pub not_sounding: u32,
    /// Aggregation status: 0 for MPDU packet, 1 for AMPDU packet.
    pub aggregation: u32,
    /// Space-Time Block Coding (STBC) status: 0 for non-STBC packet, 1 for STBC
    /// packet.
    pub stbc: u32,
    /// Forward Error Correction (FEC) status: indicates if LDPC coding is used
    /// for 11n packets.
    pub fec_coding: u32,
    /// Short Guard Interval (SGI): 0 for long guard interval, 1 for short guard
    /// interval.
    pub sgi: u32,
    /// Number of subframes aggregated in an AMPDU packet.
    pub ampdu_cnt: u32,
    /// Primary channel on which the packet is received.
    pub channel: u32,
    /// Secondary channel on which the packet is received: 0 for none, 1 for
    /// above, 2 for below.
    pub secondary_channel: u32,
    /// Timestamp of when the packet is received, in microseconds. Precise only
    /// if modem sleep or light sleep is not enabled.
    pub timestamp: u32,
    /// Noise floor of the Radio Frequency module, in dBm.
    pub noise_floor: i32,
    /// Antenna number from which the packet is received: 0 for antenna 0, 1 for
    /// antenna 1.
    pub ant: u32,
    /// Length of the packet including the Frame Check Sequence (FCS).
    pub sig_len: u32,
    /// State of the packet: 0 for no error, other values indicate error codes.
    pub rx_state: u32,
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(esp32c6)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
pub struct RxControlInfo {
    /// Received Signal Strength Indicator (RSSI) of the packet, in dBm.
    pub rssi: i32,
    /// PHY rate encoding of the packet. Only valid for non-HT (802.11b/g)
    /// packets.
    pub rate: u32,
    /// Length of the received packet including the Frame Check Sequence (FCS).
    pub sig_len: u32,
    /// Reception state of the packet: 0 for no error, others indicate error
    /// codes.
    pub rx_state: u32,
    /// Length of the dump buffer.
    pub dump_len: u32,
    /// Length of HE-SIG-B field (802.11ax).
    pub he_sigb_len: u32,
    /// Indicates if this is a single MPDU.
    pub cur_single_mpdu: u32,
    /// Current baseband format.
    pub cur_bb_format: u32,
    /// Channel estimation validity.
    pub rx_channel_estimate_info_vld: u32,
    /// Length of the channel estimation.
    pub rx_channel_estimate_len: u32,
    /// Timing information in seconds.
    pub second: u32,
    /// Primary channel on which the packet is received.
    pub channel: u32,
    /// Noise floor of the Radio Frequency module, in dBm.
    pub noise_floor: i32,
    /// Indicates if this is a group-addressed frame.
    pub is_group: u32,
    /// End state of the packet reception.
    pub rxend_state: u32,
    /// Indicate whether the reception frame is from interface 3.
    pub rxmatch3: u32,
    /// Indicate whether the reception frame is from interface 2.
    pub rxmatch2: u32,
    /// Indicate whether the reception frame is from interface 1.
    pub rxmatch1: u32,
    /// Indicate whether the reception frame is from interface 0.
    pub rxmatch0: u32,
}

#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
impl RxControlInfo {
    /// Create an instance from a raw pointer to [wifi_pkt_rx_ctrl_t].
    ///
    /// # Safety
    /// When calling this, you must ensure, that `rx_cntl` points to a valid
    /// instance of [wifi_pkt_rx_ctrl_t].
    pub unsafe fn from_raw(rx_cntl: *const wifi_pkt_rx_ctrl_t) -> Self {
        #[cfg(not(esp32c6))]
        let rx_control_info = unsafe {
            RxControlInfo {
                rssi: (*rx_cntl).rssi(),
                rate: (*rx_cntl).rate(),
                sig_mode: (*rx_cntl).sig_mode(),
                mcs: (*rx_cntl).mcs(),
                cwb: (*rx_cntl).cwb(),
                smoothing: (*rx_cntl).smoothing(),
                not_sounding: (*rx_cntl).not_sounding(),
                aggregation: (*rx_cntl).aggregation(),
                stbc: (*rx_cntl).stbc(),
                fec_coding: (*rx_cntl).fec_coding(),
                sgi: (*rx_cntl).sgi(),
                ampdu_cnt: (*rx_cntl).ampdu_cnt(),
                channel: (*rx_cntl).channel(),
                secondary_channel: (*rx_cntl).secondary_channel(),
                timestamp: (*rx_cntl).timestamp(),
                noise_floor: (*rx_cntl).noise_floor(),
                ant: (*rx_cntl).ant(),
                sig_len: (*rx_cntl).sig_len(),
                rx_state: (*rx_cntl).rx_state(),
            }
        };
        #[cfg(esp32c6)]
        let rx_control_info = unsafe {
            RxControlInfo {
                rssi: (*rx_cntl).rssi(),
                rate: (*rx_cntl).rate(),
                sig_len: (*rx_cntl).sig_len(),
                rx_state: (*rx_cntl).rx_state(),
                dump_len: (*rx_cntl).dump_len(),
                he_sigb_len: (*rx_cntl).he_sigb_len(),
                cur_single_mpdu: (*rx_cntl).cur_single_mpdu(),
                cur_bb_format: (*rx_cntl).cur_bb_format(),
                rx_channel_estimate_info_vld: (*rx_cntl).rx_channel_estimate_info_vld(),
                rx_channel_estimate_len: (*rx_cntl).rx_channel_estimate_len(),
                second: (*rx_cntl).second(),
                channel: (*rx_cntl).channel(),
                noise_floor: (*rx_cntl).noise_floor(),
                is_group: (*rx_cntl).is_group(),
                rxend_state: (*rx_cntl).rxend_state(),
                rxmatch3: (*rx_cntl).rxmatch3(),
                rxmatch2: (*rx_cntl).rxmatch2(),
                rxmatch1: (*rx_cntl).rxmatch1(),
                rxmatch0: (*rx_cntl).rxmatch0(),
            }
        };
        rx_control_info
    }
}
/// Represents a Wi-Fi packet in promiscuous mode.
#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[instability::unstable]
pub struct PromiscuousPkt<'a> {
    /// Control information related to packet reception.
    pub rx_cntl: RxControlInfo,
    /// Frame type of the received packet.
    pub frame_type: wifi_promiscuous_pkt_type_t,
    /// Length of the received packet.
    pub len: usize,
    /// Data contained in the received packet.
    pub data: &'a [u8],
}
#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl PromiscuousPkt<'_> {
    /// # Safety
    /// When calling this, you have to ensure, that `buf` points to a valid
    /// [wifi_promiscuous_pkt_t].
    pub(crate) unsafe fn from_raw(
        buf: *const wifi_promiscuous_pkt_t,
        frame_type: wifi_promiscuous_pkt_type_t,
    ) -> Self {
        let rx_cntl = unsafe { RxControlInfo::from_raw(&(*buf).rx_ctrl) };
        let len = rx_cntl.sig_len as usize;
        PromiscuousPkt {
            rx_cntl,
            frame_type,
            len,
            data: unsafe {
                core::slice::from_raw_parts(
                    (buf as *const u8).add(core::mem::size_of::<wifi_pkt_rx_ctrl_t>()),
                    len,
                )
            },
        }
    }
}

#[cfg(all(feature = "sniffer", feature = "unstable"))]
static SNIFFER_CB: NonReentrantMutex<Option<fn(PromiscuousPkt<'_>)>> = NonReentrantMutex::new(None);

#[cfg(all(feature = "sniffer", feature = "unstable"))]
unsafe extern "C" fn promiscuous_rx_cb(buf: *mut core::ffi::c_void, frame_type: u32) {
    unsafe {
        if let Some(sniffer_callback) = SNIFFER_CB.with(|callback| *callback) {
            let promiscuous_pkt = PromiscuousPkt::from_raw(buf as *const _, frame_type);
            sniffer_callback(promiscuous_pkt);
        }
    }
}

#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[instability::unstable]
/// A Wi-Fi sniffer.
#[non_exhaustive]
pub struct Sniffer<'d> {
    _phantom: PhantomData<&'d ()>,
}

#[cfg(all(feature = "sniffer", feature = "unstable"))]
impl Sniffer<'_> {
    pub(crate) fn new() -> Self {
        // This shouldn't fail, since the way this is created, means that wifi will
        // always be initialized.
        unwrap!(esp_wifi_result!(unsafe {
            esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_cb))
        }));
        Self {
            _phantom: PhantomData,
        }
    }
    /// Set promiscuous mode enabled or disabled.
    #[instability::unstable]
    pub fn set_promiscuous_mode(&self, enabled: bool) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        Ok(())
    }
    /// Transmit a raw frame.
    #[instability::unstable]
    pub fn send_raw_frame(
        &mut self,
        use_sta_interface: bool,
        buffer: &[u8],
        use_internal_seq_num: bool,
    ) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe {
            esp_wifi_80211_tx(
                if use_sta_interface {
                    wifi_interface_t_WIFI_IF_STA
                } else {
                    wifi_interface_t_WIFI_IF_AP
                } as wifi_interface_t,
                buffer.as_ptr() as *const _,
                buffer.len() as i32,
                use_internal_seq_num,
            )
        })
    }
    /// Set the callback for receiving a packet.
    #[instability::unstable]
    pub fn set_receive_cb(&mut self, cb: fn(PromiscuousPkt<'_>)) {
        SNIFFER_CB.with(|callback| *callback = Some(cb));
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
#[cfg(all(feature = "smoltcp", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl Device for WifiDevice<'_> {
    type RxToken<'a>
        = WifiRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = WifiTxToken
    where
        Self: 'a;

    fn receive(
        &mut self,
        _instant: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.mode.rx_token()
    }

    fn transmit(&mut self, _instant: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.mode.tx_token()
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = if esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE") == 0
        {
            None
        } else {
            Some(esp_config_int!(
                usize,
                "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE"
            ))
        };
        caps
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiRxToken {
    mode: WifiDeviceMode,
}

impl WifiRxToken {
    /// Consumes the RX token and applies the callback function to the received
    /// data buffer.
    pub fn consume_token<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut data = self.mode.data_queue_rx().with(|queue| {
            unwrap!(
                queue.pop_front(),
                "unreachable: transmit()/receive() ensures there is a packet to process"
            )
        });

        // We handle the received data outside of the lock because
        // PacketBuffer::drop must not be called in a critical section.
        // Dropping an PacketBuffer will call `esp_wifi_internal_free_rx_buffer`
        // which will try to lock an internal mutex. If the mutex is already
        // taken, the function will try to trigger a context switch, which will
        // fail if we are in an interrupt-free context.
        let buffer = data.as_slice_mut();
        dump_packet_info(buffer);

        f(buffer)
    }
}

#[cfg(all(feature = "smoltcp", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl RxToken for WifiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        self.consume_token(|t| f(t))
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiTxToken {
    mode: WifiDeviceMode,
}

impl WifiTxToken {
    /// Consumes the TX token and applies the callback function to the received
    /// data buffer.
    pub fn consume_token<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.mode.increase_in_flight_counter();

        // (safety): creation of multiple Wi-Fi devices with the same mode is impossible
        // in safe Rust, therefore only smoltcp _or_ embassy-net can be used at
        // one time
        static mut BUFFER: [u8; MTU] = [0u8; MTU];

        let buffer = unsafe { &mut BUFFER[..len] };

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

#[cfg(all(feature = "smoltcp", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl TxToken for WifiTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume_token(len, f)
    }
}

// FIXME data here has to be &mut because of `esp_wifi_internal_tx` signature,
// requiring a *mut ptr to the buffer Casting const to mut is instant UB, even
// though in reality `esp_wifi_internal_tx` copies the buffer into its own
// memory and does not modify
pub(crate) fn esp_wifi_send_data(interface: wifi_interface_t, data: &mut [u8]) {
    trace!("sending... {} bytes", data.len());
    dump_packet_info(data);

    let len = data.len() as u16;
    let ptr = data.as_mut_ptr().cast();

    let res = unsafe { esp_wifi_internal_tx(interface, ptr, len) };

    if res != 0 {
        warn!("esp_wifi_internal_tx {}", res);
        decrement_inflight_counter();
    } else {
        trace!("esp_wifi_internal_tx ok");
    }
}

fn dump_packet_info(_buffer: &mut [u8]) {
    #[cfg(dump_packets)]
    {
        info!("@WIFIFRAME {:?}", _buffer);
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! esp_wifi_result {
    ($value:expr) => {{
        use num_traits::FromPrimitive;
        let result = $value;
        if result != esp_wifi_sys::include::ESP_OK as i32 {
            warn!("{} returned an error: {}", stringify!($value), result);
            Err(WifiError::InternalError(unwrap!(FromPrimitive::from_i32(
                result
            ))))
        } else {
            Ok::<(), WifiError>(())
        }
    }};
}

pub(crate) mod embassy {
    use embassy_net_driver::{Capabilities, Driver, HardwareAddress, RxToken, TxToken};
    use esp_hal::asynch::AtomicWaker;

    use super::*;

    // We can get away with a single tx waker because the transmit queue is shared
    // between interfaces.
    pub(crate) static TRANSMIT_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static AP_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static AP_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static STA_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static STA_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    impl RxToken for WifiRxToken {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(f)
        }
    }

    impl TxToken for WifiTxToken {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(len, f)
        }
    }

    impl Driver for WifiDevice<'_> {
        type RxToken<'a>
            = WifiRxToken
        where
            Self: 'a;
        type TxToken<'a>
            = WifiTxToken
        where
            Self: 'a;

        fn receive(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            self.mode.register_receive_waker(cx);
            self.mode.register_transmit_waker(cx);
            self.mode.rx_token()
        }

        fn transmit(&mut self, cx: &mut core::task::Context<'_>) -> Option<Self::TxToken<'_>> {
            self.mode.register_transmit_waker(cx);
            self.mode.tx_token()
        }

        fn link_state(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> embassy_net_driver::LinkState {
            self.mode.register_link_state_waker(cx);
            self.mode.link_state()
        }

        fn capabilities(&self) -> Capabilities {
            let mut caps = Capabilities::default();
            caps.max_transmission_unit = MTU;
            caps.max_burst_size =
                if esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE") == 0 {
                    None
                } else {
                    Some(esp_config_int!(
                        usize,
                        "ESP_RADIO_CONFIG_WIFI_MAX_BURST_SIZE"
                    ))
                };
            caps
        }

        fn hardware_address(&self) -> HardwareAddress {
            HardwareAddress::Ethernet(self.mac_address())
        }
    }
}

/// Power saving mode settings for the modem.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerSaveMode {
    /// No power saving.
    #[default]
    None,
    /// Minimum power save mode. In this mode, station wakes up to receive beacon every DTIM
    /// period.
    Minimum,
    /// Maximum power save mode. In this mode, interval to receive beacons is determined by the
    /// `listen_interval` config option.
    Maximum,
}

pub(crate) fn apply_power_saving(ps: PowerSaveMode) -> Result<(), WifiError> {
    esp_wifi_result!(unsafe {
        esp_wifi_sys::include::esp_wifi_set_ps(match ps {
            PowerSaveMode::None => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_NONE,
            PowerSaveMode::Minimum => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_MIN_MODEM,
            PowerSaveMode::Maximum => esp_wifi_sys::include::wifi_ps_type_t_WIFI_PS_MAX_MODEM,
        })
    })?;
    Ok(())
}

struct FreeApListOnDrop;
impl FreeApListOnDrop {
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

/// Represents the Wi-Fi controller and its associated interfaces.
#[non_exhaustive]
pub struct Interfaces<'d> {
    /// Station mode Wi-Fi device.
    pub sta: WifiDevice<'d>,
    /// Access Point mode Wi-Fi device.
    pub ap: WifiDevice<'d>,
    /// ESP-NOW interface.
    #[cfg(all(feature = "esp-now", feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    pub esp_now: crate::esp_now::EspNow<'d>,
    /// Wi-Fi sniffer interface.
    #[cfg(all(feature = "sniffer", feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    pub sniffer: Sniffer<'d>,
}

/// Wi-Fi operating class.
///
/// Refer to Annex E of IEEE Std 802.11-2020.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum OperatingClass {
    /// The regulations under which the station/AP is operating encompass all environments for the
    /// current frequency band in the country.
    AllEnvironments,

    /// The regulations under which the station/AP is operating are for an outdoor environment only.
    Outdoors,

    /// The regulations under which the station/AP is operating are for an indoor environment only.
    Indoors,

    /// The station/AP is operating under a noncountry entity. The first two octets of the
    /// noncountry entity is two ASCII ‘XX’ characters.
    NonCountryEntity,

    /// Binary representation of the Operating Class table number currently in use. Refer to Annex E
    /// of IEEE Std 802.11-2020.
    Repr(u8),
}

impl Default for OperatingClass {
    fn default() -> Self {
        OperatingClass::Repr(0) // TODO: is this valid?
    }
}

impl OperatingClass {
    fn into_code(self) -> u8 {
        match self {
            OperatingClass::AllEnvironments => b' ',
            OperatingClass::Outdoors => b'O',
            OperatingClass::Indoors => b'I',
            OperatingClass::NonCountryEntity => b'X',
            OperatingClass::Repr(code) => code,
        }
    }
}

/// Country information.
///
/// Defaults to China (CN) with Operating Class "0".
///
/// To create a [`CountryInfo`] instance, use the `from` method first, then set additional
/// properties using the builder methods.
///
/// ## Example
///
/// ```rust,no_run
/// use esp_radio::wifi::{CountryInfo, OperatingClass};
///
/// let country_info = CountryInfo::from(*b"CN").operating_class(OperatingClass::Indoors);
/// ```
///
/// For more information, see the [Wi-Fi Country Code in the ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-country-code).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CountryInfo {
    /// Country code.
    #[builder_lite(skip)]
    country: [u8; 2],

    /// Operating class.
    #[builder_lite(unstable)]
    operating_class: OperatingClass,
}

impl From<[u8; 2]> for CountryInfo {
    fn from(country: [u8; 2]) -> Self {
        Self {
            country,
            operating_class: OperatingClass::default(),
        }
    }
}

impl CountryInfo {
    fn into_blob(self) -> wifi_country_t {
        wifi_country_t {
            cc: [
                self.country[0],
                self.country[1],
                self.operating_class.into_code(),
            ],
            // TODO: these may be valid defaults, but they should be configurable.
            schan: 1,
            nchan: 13,
            max_tx_power: 20,
            policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        }
    }
}

/// Wi-Fi configuration.
#[derive(Clone, Copy, BuilderLite, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WifiConfig {
    /// Power save mode.
    power_save_mode: PowerSaveMode,

    /// Country code.
    #[builder_lite(into)]
    #[builder_lite(unstable)]
    country_code: CountryInfo,

    /// Size of the RX queue in frames.
    #[builder_lite(unstable)]
    rx_queue_size: usize,
    /// Size of the TX queue in frames.
    #[builder_lite(unstable)]
    tx_queue_size: usize,

    /// Max number of WiFi static RX buffers.
    ///
    /// Each buffer takes approximately 1.6KB of RAM. The static rx buffers are allocated when
    /// esp_wifi_init is called, they are not freed until esp_wifi_deinit is called.
    ///
    /// WiFi hardware use these buffers to receive all 802.11 frames. A higher number may allow
    /// higher throughput but increases memory use. If [`Self::ampdu_rx_enable`] is enabled,
    /// this value is recommended to set equal or bigger than [`Self::rx_ba_win`] in order to
    /// achieve better throughput and compatibility with both stations and APs.
    #[builder_lite(unstable)]
    static_rx_buf_num: u8,

    /// Max number of WiFi dynamic RX buffers
    ///
    /// Set the number of WiFi dynamic RX buffers, 0 means unlimited RX buffers will be allocated
    /// (provided sufficient free RAM). The size of each dynamic RX buffer depends on the size of
    /// the received data frame.
    ///
    /// For each received data frame, the WiFi driver makes a copy to an RX buffer and then
    /// delivers it to the high layer TCP/IP stack. The dynamic RX buffer is freed after the
    /// higher layer has successfully received the data frame.
    ///
    /// For some applications, WiFi data frames may be received faster than the application can
    /// process them. In these cases we may run out of memory if RX buffer number is unlimited
    /// (0).
    ///
    /// If a dynamic RX buffer limit is set, it should be at least the number of
    /// static RX buffers.
    #[builder_lite(unstable)]
    dynamic_rx_buf_num: u16,

    /// Set the number of WiFi static TX buffers.
    ///
    /// Each buffer takes approximately 1.6KB of RAM.
    /// The static RX buffers are allocated when esp_wifi_init() is called, they are not released
    /// until esp_wifi_deinit() is called.
    ///
    /// For each transmitted data frame from the higher layer TCP/IP stack, the WiFi driver makes a
    /// copy of it in a TX buffer.
    ///
    /// For some applications especially UDP applications, the upper layer can deliver frames
    /// faster than WiFi layer can transmit. In these cases, we may run out of TX buffers.
    #[builder_lite(unstable)]
    static_tx_buf_num: u8,

    /// Set the number of WiFi dynamic TX buffers.
    ///
    /// The size of each dynamic TX buffer is not fixed,
    /// it depends on the size of each transmitted data frame.
    ///
    /// For each transmitted frame from the higher layer TCP/IP stack, the WiFi driver makes a copy
    /// of it in a TX buffer.
    ///
    /// For some applications, especially UDP applications, the upper layer can deliver frames
    /// faster than WiFi layer can transmit. In these cases, we may run out of TX buffers.
    #[builder_lite(unstable)]
    dynamic_tx_buf_num: u16,

    /// Select this option to enable AMPDU RX feature.
    #[builder_lite(unstable)]
    ampdu_rx_enable: bool,

    /// Select this option to enable AMPDU TX feature.
    #[builder_lite(unstable)]
    ampdu_tx_enable: bool,

    /// Select this option to enable AMSDU TX feature.
    #[builder_lite(unstable)]
    amsdu_tx_enable: bool,

    /// Set the size of WiFi Block Ack RX window.
    ///
    /// Generally a bigger value means higher throughput and better compatibility but more memory.
    /// Most of time we should NOT change the default value unless special reason, e.g. test
    /// the maximum UDP RX throughput with iperf etc. For iperf test in shieldbox, the
    /// recommended value is 9~12.
    ///
    /// If PSRAM is used and WiFi memory is preferred to allocate in PSRAM first, the default and
    /// minimum value should be 16 to achieve better throughput and compatibility with both
    /// stations and APs.
    #[builder_lite(unstable)]
    rx_ba_win: u8,
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            power_save_mode: PowerSaveMode::default(),
            country_code: CountryInfo::from(*b"CN"),

            rx_queue_size: 5,
            tx_queue_size: 3,

            static_rx_buf_num: 10,
            dynamic_rx_buf_num: 32,

            static_tx_buf_num: 0,
            dynamic_tx_buf_num: 32,

            ampdu_rx_enable: true,
            ampdu_tx_enable: true,
            amsdu_tx_enable: false,

            rx_ba_win: 6,
        }
    }
}

impl WifiConfig {
    fn validate(&self) {
        if self.rx_ba_win as u16 >= self.dynamic_rx_buf_num {
            warn!("RX BA window size should be less than the number of dynamic RX buffers.");
        }
        if self.rx_ba_win as u16 >= 2 * (self.static_rx_buf_num as u16) {
            warn!("RX BA window size should be less than twice the number of static RX buffers.");
        }
    }
}

/// Create a Wi-Fi controller and it's associated interfaces.
///
/// Dropping the controller will deinitialize / stop Wi-Fi.
///
/// Make sure to **not** call this function while interrupts are disabled, or IEEE 802.15.4 is
/// currently in use.
pub fn new<'d>(
    _inited: &'d Controller<'d>,
    device: crate::hal::peripherals::WIFI<'d>,
    config: WifiConfig,
) -> Result<(WifiController<'d>, Interfaces<'d>), WifiError> {
    if crate::is_interrupts_disabled() {
        return Err(WifiError::Unsupported);
    }

    config.validate();

    unsafe {
        internal::G_CONFIG = wifi_init_config_t {
            osi_funcs: (&raw const internal::__ESP_RADIO_G_WIFI_OSI_FUNCS).cast_mut(),

            wpa_crypto_funcs: g_wifi_default_wpa_crypto_funcs,
            static_rx_buf_num: config.static_rx_buf_num as _,
            dynamic_rx_buf_num: config.dynamic_rx_buf_num as _,
            tx_buf_type: esp_wifi_sys::include::CONFIG_ESP_WIFI_TX_BUFFER_TYPE as i32,
            static_tx_buf_num: config.static_tx_buf_num as _,
            dynamic_tx_buf_num: config.dynamic_tx_buf_num as _,
            rx_mgmt_buf_type: esp_wifi_sys::include::CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF as i32,
            rx_mgmt_buf_num: esp_wifi_sys::include::CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF as i32,
            cache_tx_buf_num: esp_wifi_sys::include::WIFI_CACHE_TX_BUFFER_NUM as i32,
            csi_enable: cfg!(feature = "csi") as i32,
            ampdu_rx_enable: config.ampdu_rx_enable as _,
            ampdu_tx_enable: config.ampdu_tx_enable as _,
            amsdu_tx_enable: config.amsdu_tx_enable as _,
            nvs_enable: 0,
            nano_enable: 0,
            rx_ba_win: config.rx_ba_win as _,
            wifi_task_core_id: Cpu::current() as _,
            beacon_max_len: esp_wifi_sys::include::WIFI_SOFTAP_BEACON_MAX_LEN as i32,
            mgmt_sbuf_num: esp_wifi_sys::include::WIFI_MGMT_SBUF_NUM as i32,
            feature_caps: internal::__ESP_RADIO_G_WIFI_FEATURE_CAPS,
            sta_disconnected_pm: false,
            espnow_max_encrypt_num: esp_wifi_sys::include::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM
                as i32,

            tx_hetb_queue_num: 3,
            dump_hesigb_enable: false,

            magic: WIFI_INIT_CONFIG_MAGIC as i32,
        };

        RX_QUEUE_SIZE.store(config.rx_queue_size, Ordering::Relaxed);
        TX_QUEUE_SIZE.store(config.tx_queue_size, Ordering::Relaxed);
    };

    crate::wifi::wifi_init(device)?;

    unsafe {
        let country = config.country_code.into_blob();
        esp_wifi_result!(esp_wifi_set_country(&country))?;
    }

    // At some point the "High-speed ADC" entropy source became available.
    unsafe { esp_hal::rng::TrngSource::increase_entropy_source_counter() };

    // Only create WifiController after we've enabled TRNG - otherwise returning an error from this
    // function will cause panic because WifiController::drop tries to disable the TRNG.
    let mut controller = WifiController {
        _phantom: Default::default(),
        beacon_timeout: 6,
        ap_beacon_timeout: 100,
    };

    controller.set_power_saving(config.power_save_mode)?;

    Ok((
        controller,
        Interfaces {
            sta: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::Sta,
            },
            ap: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::Ap,
            },
            #[cfg(all(feature = "esp-now", feature = "unstable"))]
            esp_now: crate::esp_now::EspNow::new_internal(),
            #[cfg(all(feature = "sniffer", feature = "unstable"))]
            sniffer: Sniffer::new(),
        },
    ))
}

/// Wi-Fi controller.
#[non_exhaustive]
pub struct WifiController<'d> {
    _phantom: PhantomData<&'d ()>,
    // Things we have to remember due to how esp-wifi works:
    beacon_timeout: u16,
    ap_beacon_timeout: u16,
}

impl Drop for WifiController<'_> {
    fn drop(&mut self) {
        if let Err(e) = crate::wifi::wifi_deinit() {
            warn!("Failed to cleanly deinit wifi: {:?}", e);
        }

        esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
            esp_hal::Internal::conjure()
        });
    }
}

impl WifiController<'_> {
    /// Set CSI configuration and register the receiving callback.
    #[cfg(feature = "csi")]
    #[instability::unstable]
    pub fn set_csi(
        &mut self,
        mut csi: CsiConfig,
        cb: impl FnMut(crate::wifi::wifi_csi_info_t) + Send,
    ) -> Result<(), WifiError> {
        csi.apply_config()?;
        csi.set_receive_cb(cb)?;
        csi.set_csi(true)?;

        Ok(())
    }

    /// Set the Wi-Fi protocol.
    ///
    /// This will set the wifi protocol to the desired protocol, the default for
    /// this is: `WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N`
    ///
    /// # Arguments:
    ///
    /// * `protocols` - The desired protocols
    ///
    /// # Example:
    ///
    /// ```
    /// wifi_controller.set_protocol(Protocol::P802D11BGNLR.into());
    /// ```
    /// # Note
    /// Calling this function before `set_config` will return an error.
    pub fn set_protocol(&mut self, protocols: EnumSet<Protocol>) -> Result<(), WifiError> {
        let protocol = protocols
            .into_iter()
            .map(|v| match v {
                Protocol::P802D11B => WIFI_PROTOCOL_11B,
                Protocol::P802D11BG => WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G,
                Protocol::P802D11BGN => WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
                Protocol::P802D11BGNLR => {
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR
                }
                Protocol::P802D11LR => WIFI_PROTOCOL_LR,
                Protocol::P802D11BGNAX => {
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX
                }
            })
            .fold(0, |combined, protocol| combined | protocol) as u8;

        let mode = self.mode()?;
        if mode.is_sta() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_STA, protocol)
            })?;
        }
        if mode.is_ap() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_AP, protocol)
            })?;
        }

        Ok(())
    }

    fn apply_protocols(
        iface: wifi_interface_t,
        protocols: &EnumSet<Protocol>,
    ) -> Result<(), WifiError> {
        let mask = protocols.iter().fold(0, |acc, p| acc | p.to_mask());
        debug!("Setting protocols with mask {:b}", mask);
        esp_wifi_result!(unsafe { esp_wifi_set_protocol(iface, mask as u8) })
    }

    /// Configures modem power saving.
    pub fn set_power_saving(&mut self, ps: PowerSaveMode) -> Result<(), WifiError> {
        apply_power_saving(ps)
    }

    /// A blocking wifi network scan with caller-provided scanning options.
    pub fn scan_with_config_sync(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true, config))?;
        self.scan_results(config.max.unwrap_or(usize::MAX))
    }

    fn scan_results(&mut self, max: usize) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        let mut scanned = alloc::vec::Vec::<AccessPointInfo>::new();
        let mut bss_total: u16 = max as u16;

        // Prevents memory leak on error
        let guard = FreeApListOnDrop;

        unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_num(&mut bss_total))? };

        guard.defuse();

        let mut record: MaybeUninit<include::wifi_ap_record_t> = MaybeUninit::uninit();
        for _ in 0..usize::min(bss_total as usize, max) {
            let record = unsafe { MaybeUninit::assume_init_mut(&mut record) };
            unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_record(record))? };
            let ap_info = convert_ap_info(record);
            scanned.push(ap_info);
        }

        unsafe { esp_wifi_result!(include::esp_wifi_clear_ap_list())? };

        Ok(scanned)
    }

    /// Starts the Wi-Fi controller.
    pub fn start(&mut self) -> Result<(), WifiError> {
        unsafe {
            esp_wifi_result!(esp_wifi_start())?;

            let mode = WifiMode::current()?;

            // This is not an if-else because in AP-STA mode, both are true
            if mode.is_ap() {
                esp_wifi_result!(include::esp_wifi_set_inactive_time(
                    wifi_interface_t_WIFI_IF_AP,
                    self.ap_beacon_timeout
                ))?;
            }
            if mode.is_sta() {
                esp_wifi_result!(include::esp_wifi_set_inactive_time(
                    wifi_interface_t_WIFI_IF_STA,
                    self.beacon_timeout
                ))?;
            }
        }

        Ok(())
    }

    /// Stops the Wi-Fi controller.
    pub fn stop(&mut self) -> Result<(), WifiError> {
        self.stop_impl()
    }

    /// Connect Wi-Fi station to the AP.
    ///
    /// - If station is connected , call [Self::disconnect] to disconnect.
    /// - Scanning will not be effective until connection between device and the AP is established.
    /// - If device is scanning and connecting at the same time, it will abort scanning and return a
    ///   warning message and error
    pub fn connect(&mut self) -> Result<(), WifiError> {
        self.connect_impl()
    }

    /// Disconnect Wi-Fi station from the AP.
    pub fn disconnect(&mut self) -> Result<(), WifiError> {
        self.disconnect_impl()
    }

    /// Get the RSSI information of AP to which the device is associated with.
    /// The value is obtained from the last beacon.
    ///
    /// <div class="warning">
    ///
    /// - This API should be called after station connected to AP.
    /// - Use this API only in STA or AP-STA mode.
    /// </div>
    ///
    /// # Errors
    /// This function returns [WifiError::Unsupported] if the STA side isn't
    /// running. For example, when configured for AP only.
    pub fn rssi(&self) -> Result<i32, WifiError> {
        if self.mode()?.is_sta() {
            let mut rssi: i32 = 0;
            // Will return ESP_FAIL -1 if called in AP mode.
            esp_wifi_result!(unsafe { esp_wifi_sta_get_rssi(&mut rssi) })?;
            Ok(rssi)
        } else {
            Err(WifiError::Unsupported)
        }
    }

    /// Get the supported capabilities of the controller.
    pub fn capabilities(&self) -> Result<EnumSet<crate::wifi::Capability>, WifiError> {
        let caps =
            enumset::enum_set! { Capability::Client | Capability::AccessPoint | Capability::ApSta };

        Ok(caps)
    }

    /// Set the configuration.
    ///
    /// This will set the mode accordingly.
    /// You need to use Wifi::connect() for connecting to an AP.
    ///
    /// Passing [Config::None] will disable both, AP and STA mode.
    ///
    /// If you don't intend to use Wi-Fi anymore at all consider tearing down
    /// Wi-Fi completely.
    pub fn set_config(&mut self, conf: &Config) -> Result<(), WifiError> {
        conf.validate()?;

        let mode = match conf {
            Config::None => wifi_mode_t_WIFI_MODE_NULL,
            Config::Client(_) => wifi_mode_t_WIFI_MODE_STA,
            Config::AccessPoint(_) => wifi_mode_t_WIFI_MODE_AP,
            Config::ApSta(_, _) => wifi_mode_t_WIFI_MODE_APSTA,
            #[cfg(feature = "wifi-eap")]
            Config::EapClient(_) => wifi_mode_t_WIFI_MODE_STA,
        };

        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode) })?;

        match conf {
            Config::None => Ok(()),
            Config::Client(config) => {
                self.apply_sta_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &config.protocols)
            }
            Config::AccessPoint(config) => {
                self.apply_ap_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &config.protocols)
            }
            Config::ApSta(sta_config, ap_config) => {
                self.apply_ap_config(ap_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &ap_config.protocols)?;
                self.apply_sta_config(sta_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &sta_config.protocols)
            }
            #[cfg(feature = "wifi-eap")]
            Config::EapClient(config) => {
                self.apply_sta_eap_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &config.protocols)
            }
        }
        .inspect_err(|_| {
            // we/the driver might have applied a partial configuration
            // so we better disable AP/STA just in case the caller ignores the error we
            // return here - they will run into futher errors this way
            unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL) };
        })?;

        Ok(())
    }

    /// Set the Wi-Fi mode.
    ///
    /// This will override the mode inferred by [Self::set_config].
    pub fn set_mode(&mut self, mode: WifiMode) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode.into()) })?;
        Ok(())
    }

    fn stop_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    fn connect_impl(&mut self) -> Result<(), WifiError> {
        // TODO: implement ROAMING
        esp_wifi_result!(unsafe { esp_wifi_connect_internal() })
    }

    fn disconnect_impl(&mut self) -> Result<(), WifiError> {
        // TODO: implement ROAMING
        esp_wifi_result!(unsafe { esp_wifi_disconnect_internal() })
    }

    /// Checks if the Wi-Fi controller has started. Returns true if STA and/or AP are started.
    ///
    /// This function should be called after the `start` method to verify if the
    /// Wi-Fi has started successfully.
    pub fn is_started(&self) -> Result<bool, WifiError> {
        if matches!(
            crate::wifi::sta_state(),
            WifiStaState::Started | WifiStaState::Connected | WifiStaState::Disconnected
        ) {
            return Ok(true);
        }
        if matches!(crate::wifi::ap_state(), WifiApState::Started) {
            return Ok(true);
        }
        Ok(false)
    }

    /// Checks if the Wi-Fi controller is connected to an AP.
    ///
    /// This function should be called after the `connect` method to verify if
    /// the connection was successful.
    pub fn is_connected(&self) -> Result<bool, WifiError> {
        match crate::wifi::sta_state() {
            crate::wifi::WifiStaState::Connected => Ok(true),
            crate::wifi::WifiStaState::Disconnected => Err(WifiError::Disconnected),
            // FIXME: Should any other enum value trigger an error instead of returning false?
            _ => Ok(false),
        }
    }

    fn mode(&self) -> Result<WifiMode, WifiError> {
        WifiMode::current()
    }

    /// An async Wi-Fi network scan with caller-provided scanning options.
    pub async fn scan_with_config_async(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        Self::clear_events(WifiEvent::ScanDone);
        esp_wifi_result!(wifi_start_scan(false, config))?;

        // Prevents memory leak if `scan_n`'s future is dropped.
        let guard = FreeApListOnDrop;
        WifiEventFuture::new(WifiEvent::ScanDone).await;

        guard.defuse();

        let result = self.scan_results(config.max.unwrap_or(usize::MAX))?;

        Ok(result)
    }

    /// Async version of [`crate::wifi::WifiController`]'s `start` method
    pub async fn start_async(&mut self) -> Result<(), WifiError> {
        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_ap() {
            events |= WifiEvent::ApStart;
        }
        if mode.is_sta() {
            events |= WifiEvent::StaStart;
        }

        Self::clear_events(events);

        self.start()?;

        self.wait_for_all_events(events, false).await;

        Ok(())
    }

    /// Async version of [`crate::wifi::WifiController`]'s `stop` method
    pub async fn stop_async(&mut self) -> Result<(), WifiError> {
        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_ap() {
            events |= WifiEvent::ApStop;
        }
        if mode.is_sta() {
            events |= WifiEvent::StaStop;
        }

        Self::clear_events(events);

        crate::wifi::WifiController::stop_impl(self)?;

        self.wait_for_all_events(events, false).await;

        reset_ap_state();
        reset_sta_state();

        Ok(())
    }

    /// Async version of [`crate::wifi::WifiController`]'s `connect` method
    pub async fn connect_async(&mut self) -> Result<(), WifiError> {
        Self::clear_events(WifiEvent::StaConnected | WifiEvent::StaDisconnected);

        let err = crate::wifi::WifiController::connect_impl(self).err();

        if MultiWifiEventFuture::new(WifiEvent::StaConnected | WifiEvent::StaDisconnected)
            .await
            .contains(WifiEvent::StaDisconnected)
        {
            Err(err.unwrap_or(WifiError::Disconnected))
        } else {
            Ok(())
        }
    }

    /// Async version of [`crate::wifi::WifiController`]'s `Disconnect`
    /// method
    pub async fn disconnect_async(&mut self) -> Result<(), WifiError> {
        // If not connected, this will do nothing.
        // It will also wait forever for a `StaDisconnected` event that will never come.
        // Return early instead of hanging.
        if !matches!(self.is_connected(), Ok(true)) {
            return Ok(());
        }

        Self::clear_events(WifiEvent::StaDisconnected);
        crate::wifi::WifiController::disconnect_impl(self)?;
        WifiEventFuture::new(WifiEvent::StaDisconnected).await;

        Ok(())
    }

    fn clear_events(events: impl Into<EnumSet<WifiEvent>>) {
        WIFI_EVENTS.with(|evts| evts.remove_all(events.into()));
    }

    /// Wait for one [`WifiEvent`].
    pub async fn wait_for_event(&mut self, event: WifiEvent) {
        Self::clear_events(event);
        WifiEventFuture::new(event).await
    }

    /// Wait for one of multiple [`WifiEvent`]s. Returns the events that
    /// occurred while waiting.
    pub async fn wait_for_events(
        &mut self,
        events: EnumSet<WifiEvent>,
        clear_pending: bool,
    ) -> EnumSet<WifiEvent> {
        if clear_pending {
            Self::clear_events(events);
        }
        MultiWifiEventFuture::new(events).await
    }

    /// Wait for multiple [`WifiEvent`]s.
    pub async fn wait_for_all_events(
        &mut self,
        mut events: EnumSet<WifiEvent>,
        clear_pending: bool,
    ) {
        if clear_pending {
            Self::clear_events(events);
        }

        while !events.is_empty() {
            let fired = MultiWifiEventFuture::new(events).await;
            events -= fired;
        }
    }

    fn apply_ap_config(&mut self, config: &AccessPointConfig) -> Result<(), WifiError> {
        self.ap_beacon_timeout = config.beacon_timeout;

        let mut cfg = wifi_config_t {
            ap: wifi_ap_config_t {
                ssid: [0; 32],
                password: [0; 64],
                ssid_len: 0,
                channel: config.channel,
                authmode: config.auth_method.to_raw(),
                ssid_hidden: if config.ssid_hidden { 1 } else { 0 },
                max_connection: config.max_connections as u8,
                beacon_interval: 100,
                pairwise_cipher: wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP,
                ftm_responder: false,
                pmf_cfg: wifi_pmf_config_t {
                    capable: true,
                    required: false,
                },
                sae_pwe_h2e: 0,
                csa_count: 3,
                dtim_period: config.dtim_period,
                transition_disable: 0,
                sae_ext: 0,
                bss_max_idle_cfg: include::wifi_bss_max_idle_config_t {
                    period: 0,
                    protected_keep_alive: false,
                },
                gtk_rekey_interval: 0,
            },
        };

        if config.auth_method == AuthMethod::None && !config.password.is_empty() {
            return Err(WifiError::InternalError(InternalWifiError::InvalidArg));
        }

        unsafe {
            cfg.ap.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
            cfg.ap.ssid_len = config.ssid.len() as u8;
            cfg.ap.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

            esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut cfg))
        }
    }

    fn apply_sta_config(&mut self, config: &ClientConfig) -> Result<(), WifiError> {
        self.beacon_timeout = config.beacon_timeout;

        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: config.scan_method as c_uint,
                bssid_set: config.bssid.is_some(),
                bssid: config.bssid.unwrap_or_default(),
                channel: config.channel.unwrap_or(0),
                listen_interval: config.listen_interval,
                sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                threshold: wifi_scan_threshold_t {
                    rssi: -99,
                    authmode: config.auth_method.to_raw(),
                    rssi_5g_adjustment: 0,
                },
                pmf_cfg: wifi_pmf_config_t {
                    capable: true,
                    required: false,
                },
                sae_pwe_h2e: 3,
                _bitfield_align_1: [0; 0],
                _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
                failure_retry_cnt: config.failure_retry_cnt,
                _bitfield_align_2: [0; 0],
                _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
                sae_pk_mode: 0, // ??
                sae_h2e_identifier: [0; 32],
            },
        };

        if config.auth_method == AuthMethod::None && !config.password.is_empty() {
            return Err(WifiError::InternalError(InternalWifiError::InvalidArg));
        }

        unsafe {
            cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
            cfg.sta.password[0..(config.password.len())]
                .copy_from_slice(config.password.as_bytes());

            esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))
        }
    }

    #[cfg(feature = "wifi-eap")]
    fn apply_sta_eap_config(&mut self, config: &EapClientConfig) -> Result<(), WifiError> {
        self.beacon_timeout = config.beacon_timeout;

        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: config.scan_method as c_uint,
                bssid_set: config.bssid.is_some(),
                bssid: config.bssid.unwrap_or_default(),
                channel: config.channel.unwrap_or(0),
                listen_interval: config.listen_interval,
                sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                threshold: wifi_scan_threshold_t {
                    rssi: -99,
                    authmode: config.auth_method.to_raw(),
                    rssi_5g_adjustment: 0,
                },
                pmf_cfg: wifi_pmf_config_t {
                    capable: true,
                    required: false,
                },
                sae_pwe_h2e: 3,
                _bitfield_align_1: [0; 0],
                _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
                failure_retry_cnt: config.failure_retry_cnt,
                _bitfield_align_2: [0; 0],
                _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
                sae_pk_mode: 0, // ??
                sae_h2e_identifier: [0; 32],
            },
        };

        unsafe {
            cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
            esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))?;

            if let Some(identity) = &config.identity {
                esp_wifi_result!(esp_eap_client_set_identity(
                    identity.as_str().as_ptr(),
                    identity.len() as i32
                ))?;
            } else {
                esp_eap_client_clear_identity();
            }

            if let Some(username) = &config.username {
                esp_wifi_result!(esp_eap_client_set_username(
                    username.as_str().as_ptr(),
                    username.len() as i32
                ))?;
            } else {
                esp_eap_client_clear_username();
            }

            if let Some(password) = &config.password {
                esp_wifi_result!(esp_eap_client_set_password(
                    password.as_str().as_ptr(),
                    password.len() as i32
                ))?;
            } else {
                esp_eap_client_clear_password();
            }

            if let Some(new_password) = &config.new_password {
                esp_wifi_result!(esp_eap_client_set_new_password(
                    new_password.as_str().as_ptr(),
                    new_password.len() as i32
                ))?;
            } else {
                esp_eap_client_clear_new_password();
            }

            if let Some(pac_file) = &config.pac_file {
                esp_wifi_result!(esp_eap_client_set_pac_file(
                    pac_file.as_ptr(),
                    pac_file.len() as i32
                ))?;
            }

            if let Some(phase2_method) = &config.ttls_phase2_method {
                esp_wifi_result!(esp_eap_client_set_ttls_phase2_method(
                    phase2_method.to_raw()
                ))?;
            }

            if let Some(ca_cert) = config.ca_cert {
                esp_wifi_result!(esp_eap_client_set_ca_cert(
                    ca_cert.as_ptr(),
                    ca_cert.len() as i32
                ))?;
            } else {
                esp_eap_client_clear_ca_cert();
            }

            if let Some((cert, key, password)) = config.certificate_and_key {
                let (pwd, pwd_len) = if let Some(pwd) = password {
                    (pwd.as_ptr(), pwd.len() as i32)
                } else {
                    (core::ptr::null(), 0)
                };

                esp_wifi_result!(esp_eap_client_set_certificate_and_key(
                    cert.as_ptr(),
                    cert.len() as i32,
                    key.as_ptr(),
                    key.len() as i32,
                    pwd,
                    pwd_len,
                ))?;
            } else {
                esp_eap_client_clear_certificate_and_key();
            }

            if let Some(cfg) = &config.eap_fast_config {
                let params = esp_eap_fast_config {
                    fast_provisioning: cfg.fast_provisioning as i32,
                    fast_max_pac_list_len: cfg.fast_max_pac_list_len as i32,
                    fast_pac_format_binary: cfg.fast_pac_format_binary,
                };
                esp_wifi_result!(esp_eap_client_set_fast_params(params))?;
            }

            esp_wifi_result!(esp_eap_client_set_disable_time_check(!&config.time_check))?;

            // esp_eap_client_set_suiteb_192bit_certification unsupported because we build
            // without MBEDTLS

            // esp_eap_client_use_default_cert_bundle unsupported because we build without
            // MBEDTLS

            esp_wifi_result!(esp_wifi_sta_enterprise_enable())?;

            Ok(())
        }
    }
}

impl WifiEvent {
    pub(crate) fn waker(&self) -> &'static AtomicWaker {
        // for now use only one waker for all events
        // if that ever becomes a problem we might want to pick some events to use their
        // own
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct WifiEventFuture {
    event: WifiEvent,
}

impl WifiEventFuture {
    /// Creates a new `Future` for the specified Wi-Fi event.
    pub fn new(event: WifiEvent) -> Self {
        Self { event }
    }
}

impl core::future::Future for WifiEventFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        self.event.waker().register(cx.waker());
        if WIFI_EVENTS.with(|events| events.remove(self.event)) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct MultiWifiEventFuture {
    event: EnumSet<WifiEvent>,
}

impl MultiWifiEventFuture {
    /// Creates a new `Future` for the specified set of Wi-Fi events.
    pub fn new(event: EnumSet<WifiEvent>) -> Self {
        Self { event }
    }
}

impl core::future::Future for MultiWifiEventFuture {
    type Output = EnumSet<WifiEvent>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let output = WIFI_EVENTS.with(|events| {
            let active = events.intersection(self.event);
            events.remove_all(active);
            active
        });
        if output.is_empty() {
            for event in self.event.iter() {
                event.waker().register(cx.waker());
            }

            Poll::Pending
        } else {
            Poll::Ready(output)
        }
    }
}
