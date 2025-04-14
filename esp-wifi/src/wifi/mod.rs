//! WiFi

pub mod event;
mod internal;
pub(crate) mod os_adapter;
pub(crate) mod state;
use alloc::{collections::vec_deque::VecDeque, string::String};
use core::{
    fmt::Debug,
    marker::PhantomData,
    mem::{self, MaybeUninit},
    ptr::addr_of,
    task::Poll,
    time::Duration,
};

use enumset::{EnumSet, EnumSetType};
use esp_hal::{asynch::AtomicWaker, sync::Locked};
use esp_wifi_sys::include::{
    WIFI_PROTOCOL_11AX,
    WIFI_PROTOCOL_11B,
    WIFI_PROTOCOL_11G,
    WIFI_PROTOCOL_11N,
    WIFI_PROTOCOL_LR,
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
    wifi_pkt_rx_ctrl_t,
    wifi_scan_channel_bitmap_t,
};
#[cfg(feature = "sniffer")]
use esp_wifi_sys::include::{
    esp_wifi_80211_tx,
    esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_rx_cb,
    wifi_promiscuous_pkt_t,
    wifi_promiscuous_pkt_type_t,
};
use num_derive::FromPrimitive;
#[doc(hidden)]
pub(crate) use os_adapter::*;
use portable_atomic::{AtomicUsize, Ordering};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "smoltcp")]
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
pub use state::*;

use crate::{
    EspWifiController,
    common_adapter::*,
    config::PowerSaveMode,
    esp_wifi_result,
    hal::ram,
    wifi::private::EspWifiPacketBuffer,
};

const MTU: usize = crate::CONFIG.mtu;

#[cfg(all(feature = "csi", esp32c6))]
use crate::binary::include::wifi_csi_acquire_config_t;
#[cfg(feature = "csi")]
pub use crate::binary::include::wifi_csi_info_t;
#[cfg(feature = "csi")]
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
        esp_wifi_connect,
        esp_wifi_deinit_internal,
        esp_wifi_disconnect,
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
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Default)]
#[allow(clippy::upper_case_acronyms)] // FIXME
pub enum AuthMethod {
    /// No authentication (open network).
    None,

    /// Wired Equivalent Privacy (WEP) authentication.
    WEP,

    /// Wi-Fi Protected Access (WPA) authentication.
    WPA,

    /// Wi-Fi Protected Access 2 (WPA2) Personal authentication (default).
    #[default]
    WPA2Personal,

    /// WPA/WPA2 Personal authentication (supports both).
    WPAWPA2Personal,

    /// WPA2 Enterprise authentication.
    WPA2Enterprise,

    /// WPA3 Personal authentication.
    WPA3Personal,

    /// WPA2/WPA3 Personal authentication (supports both).
    WPA2WPA3Personal,

    /// WLAN Authentication and Privacy Infrastructure (WAPI).
    WAPIPersonal,
}

/// Supported Wi-Fi protocols.
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Default)]
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

/// Secondary Wi-Fi channels.
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Default)]
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

/// Information about a detected Wi-Fi access point.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
}

/// Configuration for a Wi-Fi access point.
#[derive(Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccessPointConfiguration {
    /// The SSID of the access point.
    pub ssid: String,

    /// Whether the SSID is hidden or visible.
    pub ssid_hidden: bool,

    /// The channel the access point will operate on.
    pub channel: u8,

    /// The secondary channel configuration.
    pub secondary_channel: Option<u8>,

    /// The set of protocols supported by the access point.
    pub protocols: EnumSet<Protocol>,

    /// The authentication method to be used by the access point.
    pub auth_method: AuthMethod,

    /// The password for securing the access point (if applicable).
    pub password: String,

    /// The maximum number of connections allowed on the access point.
    pub max_connections: u16,
}

impl AccessPointConfiguration {
    fn validate(&self) -> Result<(), WifiError> {
        if self.ssid.len() > 32 {
            return Err(WifiError::InvalidArguments);
        }

        if self.password.len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl Default for AccessPointConfiguration {
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
        }
    }
}

impl core::fmt::Debug for AccessPointConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AccessPointConfiguration")
            .field("ssid", &self.ssid)
            .field("ssid_hidden", &self.ssid_hidden)
            .field("channel", &self.channel)
            .field("secondary_channel", &self.secondary_channel)
            .field("protocols", &self.protocols)
            .field("auth_method", &self.auth_method)
            .field("password", &"**REDACTED**")
            .field("max_connections", &self.max_connections)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AccessPointConfiguration {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        #[derive(Debug, Clone, Copy, Eq, PartialEq, PartialOrd, Default)]
        #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
        pub struct ProtocolSet(EnumSet<Protocol>);

        #[cfg(feature = "defmt")]
        impl defmt::Format for ProtocolSet {
            fn format(&self, fmt: defmt::Formatter<'_>) {
                for (i, p) in self.0.into_iter().enumerate() {
                    if i > 0 {
                        defmt::write!(fmt, " ");
                    }
                    defmt::write!(fmt, "{}", p);
                }
            }
        }

        let protocol_set = ProtocolSet(self.protocols);

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
            }}",
            self.ssid.as_str(),
            self.ssid_hidden,
            self.channel,
            self.secondary_channel,
            protocol_set,
            self.auth_method,
            self.max_connections
        );
    }
}

/// Client configuration for a Wi-Fi connection.
#[derive(Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ClientConfiguration {
    /// The SSID of the Wi-Fi network.
    pub ssid: String,

    /// The BSSID (MAC address) of the client.
    pub bssid: Option<[u8; 6]>,

    // pub protocol: Protocol,
    /// The authentication method for the Wi-Fi connection.
    pub auth_method: AuthMethod,

    /// The password for the Wi-Fi connection.
    pub password: String,

    /// The Wi-Fi channel to connect to.
    pub channel: Option<u8>,
}

impl ClientConfiguration {
    fn validate(&self) -> Result<(), WifiError> {
        if self.ssid.len() > 32 {
            return Err(WifiError::InvalidArguments);
        }

        if self.password.len() > 64 {
            return Err(WifiError::InvalidArguments);
        }

        Ok(())
    }
}

impl core::fmt::Debug for ClientConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ClientConfiguration")
            .field("ssid", &self.ssid)
            .field("bssid", &self.bssid)
            .field("auth_method", &self.auth_method)
            .field("password", &"**REDACTED**")
            .field("channel", &self.channel)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ClientConfiguration {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "ClientConfiguration {{\
            ssid: {}, \
            bssid: {:?}, \
            auth_method: {:?}, \
            password: **REDACTED**, \
            channel: {:?}, \
            }}",
            self.ssid.as_str(),
            self.bssid,
            self.auth_method,
            self.channel
        )
    }
}

/// Configuration for EAP-FAST authentication protocol.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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

/// Configuration for an EAP (Extensible Authentication Protocol) client.
#[derive(Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct EapClientConfiguration {
    /// The SSID of the network the client is connecting to.
    pub ssid: String,

    /// The BSSID (MAC Address) of the specific access point.
    pub bssid: Option<[u8; 6]>,

    // pub protocol: Protocol,
    /// The authentication method used for EAP.
    pub auth_method: AuthMethod,

    /// The identity used during authentication.
    pub identity: Option<String>,

    /// The username used for inner authentication.
    /// Some EAP methods require a username for authentication.
    pub username: Option<String>,

    /// The password used for inner authentication.
    pub password: Option<String>,

    /// A new password to be set during the authentication process.
    /// Some methods support password changes during authentication.
    pub new_password: Option<String>,

    /// Configuration for EAP-FAST.
    pub eap_fast_config: Option<EapFastConfig>,

    /// A PAC (Protected Access Credential) file for EAP-FAST.
    pub pac_file: Option<&'static [u8]>,

    /// A boolean flag indicating whether time checking is enforced during
    /// authentication.
    pub time_check: bool,

    /// A CA (Certificate Authority) certificate for validating the
    /// authentication server's certificate.
    pub ca_cert: Option<&'static [u8]>,

    /// A tuple containing the client's certificate, private key, and an
    /// intermediate certificate.
    #[allow(clippy::type_complexity)]
    pub certificate_and_key: Option<(&'static [u8], &'static [u8], Option<&'static [u8]>)>,

    /// The Phase 2 authentication method used for EAP-TTLS.
    pub ttls_phase2_method: Option<TtlsPhase2Method>,

    /// The specific Wi-Fi channel to use for the connection.
    pub channel: Option<u8>,
}

impl EapClientConfiguration {
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

        Ok(())
    }
}

impl Debug for EapClientConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EapClientConfiguration")
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
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for EapClientConfiguration {
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
        )
    }
}

impl Default for EapClientConfiguration {
    fn default() -> Self {
        EapClientConfiguration {
            ssid: String::new(),
            bssid: None,
            auth_method: AuthMethod::WPA2Enterprise,
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
        }
    }
}

/// Introduces Wi-Fi configuration options.
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub enum Capability {
    /// The device operates as a client, connecting to an existing network.
    Client,

    /// The device operates as an access point, allowing other devices to
    /// connect to it.
    AccessPoint,

    /// The device can operate in both client and access point modes
    /// simultaneously.
    Mixed,
}

/// Configuration of Wi-Fi operation mode.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[allow(clippy::large_enum_variant)]
pub enum Configuration {
    /// No configuration (default).
    #[default]
    None,

    /// Client-only configuration.
    Client(ClientConfiguration),

    /// Access point-only configuration.
    AccessPoint(AccessPointConfiguration),

    /// Simultaneous client and access point configuration.
    Mixed(ClientConfiguration, AccessPointConfiguration),

    /// EAP client configuration for enterprise Wi-Fi.
    #[cfg_attr(feature = "serde", serde(skip))]
    EapClient(EapClientConfiguration),
}

impl Configuration {
    fn validate(&self) -> Result<(), WifiError> {
        match self {
            Configuration::None => Ok(()),
            Configuration::Client(client_configuration) => client_configuration.validate(),
            Configuration::AccessPoint(access_point_configuration) => {
                access_point_configuration.validate()
            }
            Configuration::Mixed(client_configuration, access_point_configuration) => {
                client_configuration.validate()?;
                access_point_configuration.validate()
            }
            Configuration::EapClient(eap_client_configuration) => {
                eap_client_configuration.validate()
            }
        }
    }

    /// Returns a reference to the client configuration if available.
    pub fn as_client_conf_ref(&self) -> Option<&ClientConfiguration> {
        match self {
            Self::Client(client_conf) | Self::Mixed(client_conf, _) => Some(client_conf),
            _ => None,
        }
    }

    /// Returns a reference to the access point configuration if available.
    pub fn as_ap_conf_ref(&self) -> Option<&AccessPointConfiguration> {
        match self {
            Self::AccessPoint(ap_conf) | Self::Mixed(_, ap_conf) => Some(ap_conf),
            _ => None,
        }
    }

    /// Returns a mutable reference to the client configuration, creating it if
    /// necessary.
    pub fn as_client_conf_mut(&mut self) -> &mut ClientConfiguration {
        match self {
            Self::Client(client_conf) => client_conf,
            Self::Mixed(_, _) => {
                let prev = mem::replace(self, Self::None);
                match prev {
                    Self::Mixed(client_conf, _) => {
                        *self = Self::Client(client_conf);
                        self.as_client_conf_mut()
                    }
                    _ => unreachable!(),
                }
            }
            _ => {
                *self = Self::Client(Default::default());
                self.as_client_conf_mut()
            }
        }
    }

    /// Returns a mutable reference to the access point configuration, creating
    /// it if necessary.
    pub fn as_ap_conf_mut(&mut self) -> &mut AccessPointConfiguration {
        match self {
            Self::AccessPoint(ap_conf) => ap_conf,
            Self::Mixed(_, _) => {
                let prev = mem::replace(self, Self::None);
                match prev {
                    Self::Mixed(_, ap_conf) => {
                        *self = Self::AccessPoint(ap_conf);
                        self.as_ap_conf_mut()
                    }
                    _ => unreachable!(),
                }
            }
            _ => {
                *self = Self::AccessPoint(Default::default());
                self.as_ap_conf_mut()
            }
        }
    }

    /// Retrieves mutable references to both the `ClientConfiguration`
    /// and `AccessPointConfiguration`.
    pub fn as_mixed_conf_mut(
        &mut self,
    ) -> (&mut ClientConfiguration, &mut AccessPointConfiguration) {
        match self {
            Self::Mixed(client_conf, ap_conf) => (client_conf, ap_conf),
            Self::AccessPoint(_) => {
                let prev = mem::replace(self, Self::None);
                match prev {
                    Self::AccessPoint(ap_conf) => {
                        *self = Self::Mixed(Default::default(), ap_conf);
                        self.as_mixed_conf_mut()
                    }
                    _ => unreachable!(),
                }
            }
            Self::Client(_) => {
                let prev = mem::replace(self, Self::None);
                match prev {
                    Self::Client(client_conf) => {
                        *self = Self::Mixed(client_conf, Default::default());
                        self.as_mixed_conf_mut()
                    }
                    _ => unreachable!(),
                }
            }
            _ => {
                *self = Self::Mixed(Default::default(), Default::default());
                self.as_mixed_conf_mut()
            }
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
            AuthMethod::WEP => include::wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::WPA => include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::WPA2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WPAWPA2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
            AuthMethod::WPA2Enterprise => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE,
            AuthMethod::WPA3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthMethod::WPA2WPA3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK,
            AuthMethod::WAPIPersonal => include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
        }
    }

    fn from_raw(raw: wifi_auth_mode_t) -> Self {
        match raw {
            include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
            include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::WEP,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthMethod::WPA,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => AuthMethod::WPA2Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => AuthMethod::WPAWPA2Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => AuthMethod::WPA2Enterprise,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthMethod::WPA3Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => AuthMethod::WPA2WPA3Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => AuthMethod::WAPIPersonal,
            _ => unreachable!(),
        }
    }
}

/// Wifi Mode (Sta and/or Ap)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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

impl TryFrom<&Configuration> for WifiMode {
    type Error = WifiError;

    /// Based on the current `Configuration`, derives a `WifiMode` based on it.
    fn try_from(config: &Configuration) -> Result<Self, Self::Error> {
        let mode = match config {
            Configuration::None => return Err(WifiError::UnknownWifiMode),
            Configuration::AccessPoint(_) => Self::Ap,
            Configuration::Client(_) => Self::Sta,
            Configuration::Mixed(_, _) => Self::ApSta,
            Configuration::EapClient(_) => Self::Sta,
        };

        Ok(mode)
    }
}

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

#[derive(Clone, PartialEq, Eq)]
// https://github.com/esp-rs/esp-wifi-sys/blob/main/esp-wifi-sys/headers/local/esp_wifi_types_native.h#L94
/// Channel state information(CSI) configuration
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

#[derive(Clone, PartialEq, Eq)]
#[cfg(all(esp32c6, feature = "csi"))]
// See https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/src/include/esp32c6.rs#L5702-L5705
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

const RX_QUEUE_SIZE: usize = crate::CONFIG.rx_queue_size;
const TX_QUEUE_SIZE: usize = crate::CONFIG.tx_queue_size;

pub(crate) static DATA_QUEUE_RX_AP: Locked<VecDeque<EspWifiPacketBuffer>> =
    Locked::new(VecDeque::new());

pub(crate) static DATA_QUEUE_RX_STA: Locked<VecDeque<EspWifiPacketBuffer>> =
    Locked::new(VecDeque::new());

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

/// Events generated by the WiFi driver.
#[repr(i32)]
#[derive(Debug, FromPrimitive, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    ApStaconnected,
    /// A station disconnected from Soft-AP.
    ApStadisconnected,
    /// Received probe request packet in Soft-AP interface.
    ApProbereqrecved,

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
#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)] // FIXME remove prefix
pub enum InternalWifiError {
    /// Out of memory
    EspErrNoMem          = 0x101,

    /// Invalid argument
    EspErrInvalidArg     = 0x102,

    /// WiFi driver was not installed by esp_wifi_init
    EspErrWifiNotInit    = 0x3001,

    /// WiFi driver was not started by esp_wifi_start
    EspErrWifiNotStarted = 0x3002,

    /// WiFi driver was not stopped by esp_wifi_stop
    EspErrWifiNotStopped = 0x3003,

    /// WiFi interface error
    EspErrWifiIf         = 0x3004,

    /// WiFi mode error
    EspErrWifiMode       = 0x3005,

    /// WiFi internal state error
    EspErrWifiState      = 0x3006,

    /// WiFi internal control block of station or soft-AP error
    EspErrWifiConn       = 0x3007,

    /// WiFi internal NVS module error
    EspErrWifiNvs        = 0x3008,

    /// MAC address is invalid
    EspErrWifiMac        = 0x3009,

    /// SSID is invalid
    EspErrWifiSsid       = 0x300A,

    /// Password is invalid
    EspErrWifiPassword   = 0x300B,

    /// Timeout error
    EspErrWifiTimeout    = 0x300C,

    /// WiFi is in sleep state(RF closed) and wakeup fail
    EspErrWifiWakeFail   = 0x300D,

    /// The caller would block
    EspErrWifiWouldBlock = 0x300E,

    /// Station still in disconnect status
    EspErrWifiNotConnect = 0x300F,

    /// Failed to post the event to WiFi task
    EspErrWifiPost       = 0x3012,

    /// Invalid WiFi state when init/deinit is called
    EspErrWifiInitState  = 0x3013,

    /// Returned when WiFi is stopping
    EspErrWifiStopState  = 0x3014,

    /// The WiFi connection is not associated
    EspErrWifiNotAssoc   = 0x3015,

    /// The WiFi TX is disallowed
    EspErrWifiTxDisallow = 0x3016,
}

/// Get the STA MAC address
pub fn sta_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 0);
    }
}

/// Get the AP MAC address
pub fn ap_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 1);
    }
}

pub(crate) fn wifi_init() -> Result<(), WifiError> {
    unsafe {
        internal::G_CONFIG.wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs;
        internal::G_CONFIG.feature_caps = internal::g_wifi_feature_caps;

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

        #[cfg(any(esp32, esp32s3))]
        {
            static mut NVS_STRUCT: [u32; 12] = [0; 12];
            chip_specific::g_misc_nvs = addr_of!(NVS_STRUCT) as u32;
        }

        crate::flags::WIFI.fetch_add(1, Ordering::SeqCst);

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

pub(crate) fn wifi_deinit() -> Result<(), crate::InitializationError> {
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
    let packet = EspWifiPacketBuffer { buffer, len, eb };
    // We must handle the result outside of the lock because
    // EspWifiPacketBuffer::drop must not be called in a critical section.
    // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    match DATA_QUEUE_RX_STA.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE {
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
    let packet = EspWifiPacketBuffer { buffer, len, eb };
    // We must handle the result outside of the critical section because
    // EspWifiPacketBuffer::drop must not be called in a critical section.
    // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    match DATA_QUEUE_RX_AP.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE {
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

pub(crate) fn wifi_start() -> Result<(), WifiError> {
    unsafe {
        esp_wifi_result!(esp_wifi_start())?;

        let mode = WifiMode::current()?;

        // This is not an if-else because in AP-STA mode, both are true
        if mode.is_ap() {
            esp_wifi_result!(include::esp_wifi_set_inactive_time(
                wifi_interface_t_WIFI_IF_AP,
                crate::CONFIG.ap_beacon_timeout
            ))?;
        }
        if mode.is_sta() {
            esp_wifi_result!(include::esp_wifi_set_inactive_time(
                wifi_interface_t_WIFI_IF_STA,
                crate::CONFIG.beacon_timeout
            ))?;
        };
    }

    Ok(())
}

/// Configuration for active or passive scan. For details see the [WIFI Alliance FAQ](https://www.wi-fi.org/knowledge-center/faq/what-are-passive-and-active-scanning).
///
/// # Comparison of active and passive scan
///
/// |                                      | **Active** | **Passive** |
/// |--------------------------------------|------------|-------------|
/// | **Power consumption**                |    High    |     Low     |
/// | **Time required (typical behavior)** |     Low    |     High    |
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ScanTypeConfig {
    /// Active scan with min and max scan time per channel. This is the default
    /// and recommended if you are unsure.
    ///
    /// # Procedure
    /// 1. Send probe request on each channel.
    /// 2. Wait for probe response. Wait at least `min` time, but if no response
    ///    is received, wait up to `max` time.
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
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct ScanConfig<'a> {
    /// SSID to filter for.
    /// If [`None`] is passed, all SSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given SSID will be
    /// returned.
    pub ssid: Option<&'a str>,
    /// BSSID to filter for.
    /// If [`None`] is passed, all BSSIDs will be returned.
    /// If [`Some`] is passed, only the APs matching the given BSSID will be
    /// returned.
    pub bssid: Option<[u8; 6]>,
    /// Channel to filter for.
    /// If [`None`] is passed, all channels will be returned.
    /// If [`Some`] is passed, only the APs on the given channel will be
    /// returned.
    pub channel: Option<u8>,
    /// Whether to show hidden networks.
    pub show_hidden: bool,
    /// Scan type, active or passive.
    pub scan_type: ScanTypeConfig,
}

pub(crate) fn wifi_start_scan(
    block: bool,
    ScanConfig {
        ssid,
        mut bssid,
        channel,
        show_hidden,
        scan_type,
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
    };

    unsafe { esp_wifi_scan_start(&scan_config, block) }
}

mod private {
    use super::*;

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    /// Take care not to drop this while in a critical section.
    ///
    /// Dropping an EspWifiPacketBuffer will call
    /// `esp_wifi_internal_free_rx_buffer` which will try to lock an
    /// internal mutex. If the mutex is already taken, the function will try
    /// to trigger a context switch, which will fail if we are in a critical
    /// section.
    pub struct EspWifiPacketBuffer {
        pub(crate) buffer: *mut c_types::c_void,
        pub(crate) len: u16,
        pub(crate) eb: *mut c_types::c_void,
    }

    unsafe impl Send for EspWifiPacketBuffer {}

    impl Drop for EspWifiPacketBuffer {
        fn drop(&mut self) {
            trace!("Dropping EspWifiPacketBuffer, freeing memory");
            unsafe { esp_wifi_internal_free_rx_buffer(self.eb) };
        }
    }

    impl EspWifiPacketBuffer {
        pub fn as_slice_mut(&mut self) -> &mut [u8] {
            unsafe { core::slice::from_raw_parts_mut(self.buffer as *mut u8, self.len as usize) }
        }
    }
}

/// Provides methods for retrieving the Wi-Fi mode and MAC address.
#[derive(Debug, Clone, Copy)]
pub enum WifiDeviceMode {
    Sta,
    Ap,
}

impl WifiDeviceMode {
    fn mac_address(&self) -> [u8; 6] {
        match self {
            WifiDeviceMode::Sta => {
                let mut mac = [0; 6];
                sta_mac(&mut mac);
                mac
            }
            WifiDeviceMode::Ap => {
                let mut mac = [0; 6];
                ap_mac(&mut mac);
                mac
            }
        }
    }

    fn data_queue_rx(&self) -> &'static Locked<VecDeque<EspWifiPacketBuffer>> {
        match self {
            WifiDeviceMode::Sta => &DATA_QUEUE_RX_STA,
            WifiDeviceMode::Ap => &DATA_QUEUE_RX_AP,
        }
    }

    fn can_send(&self) -> bool {
        WIFI_TX_INFLIGHT.load(Ordering::SeqCst) < TX_QUEUE_SIZE
    }

    fn increase_in_flight_counter(&self) {
        WIFI_TX_INFLIGHT.fetch_add(1, Ordering::SeqCst);
    }

    fn tx_token(&self) -> Option<WifiTxToken> {
        if !self.can_send() {
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
                if matches!(sta_state(), WifiState::StaConnected) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
            WifiDeviceMode::Ap => {
                if matches!(ap_state(), WifiState::ApStarted) {
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
    }
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(not(any(esp32c6)))]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg(feature = "sniffer")]
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
#[cfg(feature = "sniffer")]
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

#[cfg(feature = "sniffer")]
static SNIFFER_CB: Locked<Option<fn(PromiscuousPkt<'_>)>> = Locked::new(None);

#[cfg(feature = "sniffer")]
unsafe extern "C" fn promiscuous_rx_cb(buf: *mut core::ffi::c_void, frame_type: u32) {
    unsafe {
        if let Some(sniffer_callback) = SNIFFER_CB.with(|callback| *callback) {
            let promiscuous_pkt = PromiscuousPkt::from_raw(buf as *const _, frame_type);
            sniffer_callback(promiscuous_pkt);
        }
    }
}

#[cfg(feature = "sniffer")]
/// A wifi sniffer.
#[non_exhaustive]
pub struct Sniffer {}

#[cfg(feature = "sniffer")]
impl Sniffer {
    pub(crate) fn new() -> Self {
        // This shouldn't fail, since the way this is created, means that wifi will
        // always be initialized.
        unwrap!(esp_wifi_result!(unsafe {
            esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_cb))
        }));
        Self {}
    }
    /// Set promiscuous mode enabled or disabled.
    pub fn set_promiscuous_mode(&self, enabled: bool) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        Ok(())
    }
    /// Transmit a raw frame.
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
    pub fn set_receive_cb(&mut self, cb: fn(PromiscuousPkt<'_>)) {
        SNIFFER_CB.with(|callback| *callback = Some(cb));
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
#[cfg(feature = "smoltcp")]
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
        caps.max_burst_size = if crate::CONFIG.max_burst_size == 0 {
            None
        } else {
            Some(crate::CONFIG.max_burst_size)
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
        // EspWifiPacketBuffer::drop must not be called in a critical section.
        // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
        // which will try to lock an internal mutex. If the mutex is already
        // taken, the function will try to trigger a context switch, which will
        // fail if we are in an interrupt-free context.
        let buffer = data.as_slice_mut();
        dump_packet_info(buffer);

        f(buffer)
    }
}

#[cfg(feature = "smoltcp")]
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

        // (safety): creation of multiple WiFi devices with the same mode is impossible
        // in safe Rust, therefore only smoltcp _or_ embassy-net can be used at
        // one time
        static mut BUFFER: [u8; MTU] = [0u8; MTU];

        let buffer = unsafe { &mut BUFFER[..len] };

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

#[cfg(feature = "smoltcp")]
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

fn apply_ap_config(config: &AccessPointConfiguration) -> Result<(), WifiError> {
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
            dtim_period: 2,
        },
    };

    if config.auth_method == AuthMethod::None && !config.password.is_empty() {
        return Err(WifiError::InternalError(
            InternalWifiError::EspErrInvalidArg,
        ));
    }

    unsafe {
        cfg.ap.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
        cfg.ap.ssid_len = config.ssid.len() as u8;
        cfg.ap.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut cfg))
    }
}

fn apply_sta_config(config: &ClientConfiguration) -> Result<(), WifiError> {
    let mut cfg = wifi_config_t {
        sta: wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: crate::CONFIG.scan_method,
            bssid_set: config.bssid.is_some(),
            bssid: config.bssid.unwrap_or_default(),
            channel: config.channel.unwrap_or(0),
            listen_interval: crate::CONFIG.listen_interval,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {
                rssi: -99,
                authmode: config.auth_method.to_raw(),
            },
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: false,
            },
            sae_pwe_h2e: 3,
            _bitfield_align_1: [0; 0],
            _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
            failure_retry_cnt: crate::CONFIG.failure_retry_cnt,
            _bitfield_align_2: [0; 0],
            _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
            sae_pk_mode: 0, // ??
            sae_h2e_identifier: [0; 32],
        },
    };

    if config.auth_method == AuthMethod::None && !config.password.is_empty() {
        return Err(WifiError::InternalError(
            InternalWifiError::EspErrInvalidArg,
        ));
    }

    unsafe {
        cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
        cfg.sta.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))
    }
}

fn apply_sta_eap_config(config: &EapClientConfiguration) -> Result<(), WifiError> {
    let mut cfg = wifi_config_t {
        sta: wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: crate::CONFIG.scan_method,
            bssid_set: config.bssid.is_some(),
            bssid: config.bssid.unwrap_or_default(),
            channel: config.channel.unwrap_or(0),
            listen_interval: crate::CONFIG.listen_interval,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {
                rssi: -99,
                authmode: config.auth_method.to_raw(),
            },
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: false,
            },
            sae_pwe_h2e: 3,
            _bitfield_align_1: [0; 0],
            _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
            failure_retry_cnt: crate::CONFIG.failure_retry_cnt,
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
            caps.max_burst_size = if crate::CONFIG.max_burst_size == 0 {
                None
            } else {
                Some(crate::CONFIG.max_burst_size)
            };
            caps
        }

        fn hardware_address(&self) -> HardwareAddress {
            HardwareAddress::Ethernet(self.mac_address())
        }
    }
}

pub(crate) fn apply_power_saving(ps: PowerSaveMode) -> Result<(), WifiError> {
    esp_wifi_result!(unsafe { esp_wifi_sys::include::esp_wifi_set_ps(ps.into()) })?;
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

#[non_exhaustive]
pub struct Interfaces<'d> {
    pub sta: WifiDevice<'d>,
    pub ap: WifiDevice<'d>,
    #[cfg(feature = "esp-now")]
    pub esp_now: crate::esp_now::EspNow<'d>,
    #[cfg(feature = "sniffer")]
    pub sniffer: Sniffer,
}

/// Create a WiFi controller and it's associated interfaces.
///
/// Dropping the controller will deinitialize / stop WiFi.
pub fn new<'d>(
    inited: &'d EspWifiController<'d>,
    _device: crate::hal::peripherals::WIFI<'d>,
) -> Result<(WifiController<'d>, Interfaces<'d>), WifiError> {
    let mut controller = WifiController {
        _phantom: Default::default(),
    };

    if !inited.wifi() {
        crate::wifi::wifi_init()?;

        let mut cntry_code = [0u8; 3];
        cntry_code[..crate::CONFIG.country_code.len()]
            .copy_from_slice(crate::CONFIG.country_code.as_bytes());
        cntry_code[2] = crate::CONFIG.country_code_operating_class;

        #[allow(clippy::useless_transmute)]
        unsafe {
            let country = wifi_country_t {
                // FIXME once we bumped the MSRV accordingly (see https://github.com/esp-rs/esp-hal/pull/3027#discussion_r1944718266)
                #[allow(clippy::useless_transmute)]
                cc: core::mem::transmute::<[u8; 3], [core::ffi::c_char; 3]>(cntry_code),
                schan: 1,
                nchan: 13,
                max_tx_power: 20,
                policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
            };
            esp_wifi_result!(esp_wifi_set_country(&country))?;
        }

        #[cfg(initial_power_save_mode_none)]
        const INITIAL_POWER_SAVE_MODE: PowerSaveMode = PowerSaveMode::None;
        #[cfg(initial_power_save_mode_min)]
        const INITIAL_POWER_SAVE_MODE: PowerSaveMode = PowerSaveMode::Minimum;
        #[cfg(initial_power_save_mode_max)]
        const INITIAL_POWER_SAVE_MODE: PowerSaveMode = PowerSaveMode::Maximum;

        controller.set_power_saving(INITIAL_POWER_SAVE_MODE)?;
    }

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
            #[cfg(feature = "esp-now")]
            esp_now: crate::esp_now::EspNow::new_internal(),
            #[cfg(feature = "sniffer")]
            sniffer: Sniffer::new(),
        },
    ))
}

#[non_exhaustive]
pub struct WifiController<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl Drop for WifiController<'_> {
    fn drop(&mut self) {
        if unwrap!(
            crate::flags::WIFI.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
                Some(x.saturating_sub(1))
            })
        ) == 0
        {
            if let Err(e) = crate::wifi::wifi_deinit() {
                warn!("Failed to cleanly deinit wifi: {:?}", e);
            }
        }
    }
}

impl WifiController<'_> {
    /// Set CSI configuration and register the receiving callback.
    #[cfg(feature = "csi")]
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

    /// Set the wifi protocol.
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

    /// Configures modem power saving
    pub fn set_power_saving(&mut self, ps: PowerSaveMode) -> Result<(), WifiError> {
        apply_power_saving(ps)
    }

    /// A blocking wifi network scan with caller-provided scanning options.
    pub fn scan_with_config_sync(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        self.scan_with_config_sync_max(config, usize::MAX)
    }

    pub fn scan_with_config_sync_max(
        &mut self,
        config: ScanConfig<'_>,
        max: usize,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true, config))?;
        let result = self.scan_results(max)?;
        Ok(result)
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

    /// A blocking wifi network scan with default scanning options.
    pub fn scan_n(&mut self, max: usize) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        self.scan_with_config_sync_max(Default::default(), max)
    }

    /// Starts the WiFi controller.
    pub fn start(&mut self) -> Result<(), WifiError> {
        crate::wifi::wifi_start()
    }

    /// Stops the WiFi controller.
    pub fn stop(&mut self) -> Result<(), WifiError> {
        self.stop_impl()
    }

    /// Connect WiFi station to the AP.
    ///
    /// - If station is connected , call [Self::disconnect] to disconnect.
    /// - Scanning will not be effective until connection between device and the
    ///   AP is established.
    /// - If device is scanning and connecting at the same time, it will abort
    ///   scanning and return a warning message and error
    pub fn connect(&mut self) -> Result<(), WifiError> {
        self.connect_impl()
    }

    /// Disconnect WiFi station from the AP.
    pub fn disconnect(&mut self) -> Result<(), WifiError> {
        self.disconnect_impl()
    }

    /// Get the supported capabilities of the controller.
    pub fn capabilities(&self) -> Result<EnumSet<crate::wifi::Capability>, WifiError> {
        let caps =
            enumset::enum_set! { Capability::Client | Capability::AccessPoint | Capability::Mixed };
        Ok(caps)
    }

    /// Set the configuration.
    ///
    /// This will set the mode accordingly.
    /// You need to use Wifi::connect() for connecting to an AP.
    ///
    /// Passing [Configuration::None] will disable both, AP and STA mode.
    ///
    /// If you don't intent to use WiFi anymore at all consider tearing down
    /// WiFi completely.
    pub fn set_configuration(&mut self, conf: &Configuration) -> Result<(), WifiError> {
        conf.validate()?;

        let mode = match conf {
            Configuration::None => wifi_mode_t_WIFI_MODE_NULL,
            Configuration::Client(_) => wifi_mode_t_WIFI_MODE_STA,
            Configuration::AccessPoint(_) => wifi_mode_t_WIFI_MODE_AP,
            Configuration::Mixed(_, _) => wifi_mode_t_WIFI_MODE_APSTA,
            Configuration::EapClient(_) => wifi_mode_t_WIFI_MODE_STA,
        };

        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode) })?;

        match conf {
            Configuration::None => Ok::<(), WifiError>(()),
            Configuration::Client(config) => apply_sta_config(config),
            Configuration::AccessPoint(config) => apply_ap_config(config),
            Configuration::Mixed(sta_config, ap_config) => {
                apply_ap_config(ap_config).and_then(|()| apply_sta_config(sta_config))
            }
            Configuration::EapClient(config) => apply_sta_eap_config(config),
        }
        .inspect_err(|_| {
            // we/the driver might have applied a partial configuration
            // so we better disable AP/STA just in case the caller ignores the error we
            // return here - they will run into futher errors this way
            unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL) };
        })?;

        Ok(())
    }

    /// Set the WiFi mode.
    ///
    /// This will override the mode inferred by [Self::set_configuration].
    pub fn set_mode(&mut self, mode: WifiMode) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode.into()) })?;
        Ok(())
    }

    fn stop_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    fn connect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_connect() })
    }

    fn disconnect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_disconnect() })
    }

    /// Checks if the WiFi controller has started.
    ///
    /// This function should be called after the `start` method to verify if the
    /// WiFi has started successfully.
    pub fn is_started(&self) -> Result<bool, WifiError> {
        if matches!(
            crate::wifi::sta_state(),
            WifiState::StaStarted | WifiState::StaConnected | WifiState::StaDisconnected
        ) {
            return Ok(true);
        }
        if matches!(crate::wifi::ap_state(), WifiState::ApStarted) {
            return Ok(true);
        }
        Ok(false)
    }

    /// Checks if the WiFi controller is connected to an AP.
    ///
    /// This function should be called after the `connect` method to verify if
    /// the connection was successful.
    pub fn is_connected(&self) -> Result<bool, WifiError> {
        match crate::wifi::sta_state() {
            crate::wifi::WifiState::StaConnected => Ok(true),
            crate::wifi::WifiState::StaDisconnected => Err(WifiError::Disconnected),
            // FIXME: Should any other enum value trigger an error instead of returning false?
            _ => Ok(false),
        }
    }

    fn mode(&self) -> Result<WifiMode, WifiError> {
        WifiMode::current()
    }

    /// Async version of [`crate::wifi::WifiController`]'s `scan_n` method
    pub async fn scan_n_async(
        &mut self,
        max: usize,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        self.scan_with_config_async_max(Default::default(), max)
            .await
    }

    /// An async wifi network scan with caller-provided scanning options.
    pub async fn scan_with_config_async(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        self.scan_with_config_async_max(config, usize::MAX).await
    }

    async fn scan_with_config_async_max(
        &mut self,
        config: ScanConfig<'_>,
        max: usize,
    ) -> Result<alloc::vec::Vec<AccessPointInfo>, WifiError> {
        Self::clear_events(WifiEvent::ScanDone);
        esp_wifi_result!(wifi_start_scan(false, config))?;

        // Prevents memory leak if `scan_n`'s future is dropped.
        let guard = FreeApListOnDrop;
        WifiEventFuture::new(WifiEvent::ScanDone).await;

        guard.defuse();

        let result = self.scan_results(max)?;

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

        wifi_start()?;

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
        WIFI_EVENTS.with(|evts| evts.get_mut().remove_all(events.into()));
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
    /// Creates a new `Future` for the specified WiFi event.
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
        if WIFI_EVENTS.with(|events| events.get_mut().remove(self.event)) {
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
    /// Creates a new `Future` for the specified set of WiFi events.
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
            let events = events.get_mut();
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
