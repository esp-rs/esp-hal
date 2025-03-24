use core::{mem, time::Duration};

use enumset::{EnumSet, EnumSetType};
use esp_wifi_sys::include::{
    esp_wifi_get_mode,
    wifi_auth_mode_t,
    wifi_mode_t,
    wifi_mode_t_WIFI_MODE_AP,
    wifi_mode_t_WIFI_MODE_APSTA,
    wifi_mode_t_WIFI_MODE_NULL,
    wifi_mode_t_WIFI_MODE_STA,
};

use super::WifiError;
use crate::esp_wifi_result;

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
    pub ssid: heapless::String<32>,

    /// The BSSID (MAC address) of the access point.
    pub bssid: [u8; 6],

    /// The channel the access point is operating on.
    pub channel: u8,

    /// The secondary channel configuration of the access point.
    pub secondary_channel: SecondaryChannel,

    /// The signal strength of the access point (RSSI).
    pub signal_strength: i8,

    /// The set of protocols supported by the access point.
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub protocols: EnumSet<Protocol>,

    /// The authentication method used by the access point.
    pub auth_method: Option<AuthMethod>,
}

/// Configuration for a Wi-Fi access point.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccessPointConfiguration {
    /// The SSID of the access point.
    pub ssid: heapless::String<32>,

    /// Whether the SSID is hidden or visible.
    pub ssid_hidden: bool,

    /// The channel the access point will operate on.
    pub channel: u8,

    /// The secondary channel configuration.
    pub secondary_channel: Option<u8>,

    /// The set of protocols supported by the access point.
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub protocols: EnumSet<Protocol>,

    /// The authentication method to be used by the access point.
    pub auth_method: AuthMethod,

    /// The password for securing the access point (if applicable).
    pub password: heapless::String<64>,

    /// The maximum number of connections allowed on the access point.
    pub max_connections: u16,
}

impl Default for AccessPointConfiguration {
    fn default() -> Self {
        Self {
            ssid: unwrap!("iot-device".try_into()),
            ssid_hidden: false,
            channel: 1,
            secondary_channel: None,
            protocols: Protocol::P802D11B | Protocol::P802D11BG | Protocol::P802D11BGN,
            auth_method: AuthMethod::None,
            password: heapless::String::new(),
            max_connections: 255,
        }
    }
}

/// Client configuration for a Wi-Fi connection.
#[derive(Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ClientConfiguration {
    /// The SSID of the Wi-Fi network.
    pub ssid: heapless::String<32>,

    /// The BSSID (MAC address) of the client.
    pub bssid: Option<[u8; 6]>,

    // pub protocol: Protocol,
    /// The authentication method for the Wi-Fi connection.
    pub auth_method: AuthMethod,

    /// The password for the Wi-Fi connection.
    pub password: heapless::String<64>,

    /// The Wi-Fi channel to connect to.
    pub channel: Option<u8>,
}

impl core::fmt::Debug for ClientConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ClientConfiguration")
            .field("ssid", &self.ssid)
            .field("bssid", &self.bssid)
            .field("auth_method", &self.auth_method)
            .field("channel", &self.channel)
            .finish()
    }
}

impl Default for ClientConfiguration {
    fn default() -> Self {
        ClientConfiguration {
            ssid: heapless::String::new(),
            bssid: None,
            auth_method: Default::default(),
            password: heapless::String::new(),
            channel: None,
        }
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
    pub(crate) fn to_raw(&self) -> u32 {
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct EapClientConfiguration {
    /// The SSID of the network the client is connecting to.
    pub ssid: heapless::String<32>,

    /// The BSSID (MAC Address) of the specific access point.
    pub bssid: Option<[u8; 6]>,

    // pub protocol: Protocol,
    /// The authentication method used for EAP.
    pub auth_method: AuthMethod,

    /// The identity used during authentication.
    pub identity: Option<heapless::String<128>>,

    /// The username used for inner authentication.
    /// Some EAP methods require a username for authentication.
    pub username: Option<heapless::String<128>>,

    /// The password used for inner authentication.
    pub password: Option<heapless::String<64>>,

    /// A new password to be set during the authentication process.
    /// Some methods support password changes during authentication.
    pub new_password: Option<heapless::String<64>>,

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

impl core::fmt::Debug for EapClientConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EapClientConfiguration")
            .field("ssid", &self.ssid)
            .field("bssid", &self.bssid)
            .field("auth_method", &self.auth_method)
            .field("channel", &self.channel)
            .field("identity", &self.identity)
            .field("username", &self.username)
            .field("eap_fast_config", &self.eap_fast_config)
            .field("time_check", &self.time_check)
            .field("pac_file set", &self.pac_file.is_some())
            .field("ca_cert set", &self.ca_cert.is_some())
            .field(
                "certificate_and_key set",
                &self.certificate_and_key.is_some(),
            )
            .field("ttls_phase2_method", &self.ttls_phase2_method)
            .finish()
    }
}

impl Default for EapClientConfiguration {
    fn default() -> Self {
        EapClientConfiguration {
            ssid: heapless::String::new(),
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
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Default)]
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
            Self::Mixed(client_conf, ref mut ap_conf) => (client_conf, ap_conf),
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

pub(crate) trait AuthMethodExt {
    fn to_raw(&self) -> wifi_auth_mode_t;
    fn from_raw(raw: wifi_auth_mode_t) -> Self;
}

impl AuthMethodExt for AuthMethod {
    fn to_raw(&self) -> wifi_auth_mode_t {
        match self {
            AuthMethod::None => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_OPEN,
            AuthMethod::WEP => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::WPA => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::WPA2Personal => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WPAWPA2Personal => {
                crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK
            }
            AuthMethod::WPA2Enterprise => {
                crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE
            }
            AuthMethod::WPA3Personal => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthMethod::WPA2WPA3Personal => {
                crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK
            }
            AuthMethod::WAPIPersonal => crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
        }
    }

    fn from_raw(raw: wifi_auth_mode_t) -> Self {
        match raw {
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::WEP,
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthMethod::WPA,
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => AuthMethod::WPA2Personal,
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => {
                AuthMethod::WPAWPA2Personal
            }
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => {
                AuthMethod::WPA2Enterprise
            }
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthMethod::WPA3Personal,
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => {
                AuthMethod::WPA2WPA3Personal
            }
            crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => AuthMethod::WAPIPersonal,
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
            crate::binary::include::wifi_mode_t_WIFI_MODE_STA => Ok(Self::Sta),
            crate::binary::include::wifi_mode_t_WIFI_MODE_AP => Ok(Self::Ap),
            crate::binary::include::wifi_mode_t_WIFI_MODE_APSTA => Ok(Self::ApSta),
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
    pub(super) fn validate(&self) {
        if matches!(self, Self::Passive(dur) if *dur > Duration::from_millis(1500)) {
            warn!("Passive scan duration longer than 1500ms may cause a station to disconnect from the AP");
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
    /// Wwhen receiving an STBC applied HE PPDU, 0- acquire the complete
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
impl From<CsiConfig> for crate::binary::include::wifi_csi_config_t {
    fn from(config: CsiConfig) -> Self {
        #[cfg(not(esp32c6))]
        {
            crate::binary::include::wifi_csi_config_t {
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
            crate::binary::include::wifi_csi_acquire_config_t {
                _bitfield_align_1: [0; 0],
                _bitfield_1: crate::binary::include::wifi_csi_acquire_config_t::new_bitfield_1(
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
        let conf: crate::binary::include::wifi_csi_config_t = self.clone().into();

        unsafe {
            esp_wifi_result!(crate::binary::include::esp_wifi_set_csi_config(&conf))?;
        }
        Ok(())
    }

    /// Register the RX callback function of CSI data. Each time a CSI data is
    /// received, the callback function will be called.
    pub(crate) fn set_receive_cb<C: super::CsiCallback>(&mut self, cb: C) -> Result<(), WifiError> {
        let cb = alloc::boxed::Box::new(cb);
        let cb_ptr = alloc::boxed::Box::into_raw(cb) as *mut crate::wifi::c_types::c_void;

        unsafe {
            esp_wifi_result!(crate::binary::include::esp_wifi_set_csi_rx_cb(
                Some(super::csi_rx_cb::<C>),
                cb_ptr
            ))?;
        }
        Ok(())
    }

    /// Enable or disable CSI
    pub(crate) fn set_csi(&self, enable: bool) -> Result<(), WifiError> {
        // https://github.com/esp-rs/esp-wifi-sys/blob/2a466d96fe8119d49852fc794aea0216b106ba7b/esp-wifi-sys/headers/esp_wifi.h#L1241
        unsafe {
            esp_wifi_result!(crate::binary::include::esp_wifi_set_csi(enable))?;
        }
        Ok(())
    }
}
