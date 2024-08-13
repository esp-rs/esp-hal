//! WiFi

pub(crate) mod os_adapter;
pub(crate) mod state;

use core::{
    cell::{RefCell, RefMut},
    fmt::Debug,
    mem::{self, MaybeUninit},
    ptr::addr_of,
    time::Duration,
};

use critical_section::{CriticalSection, Mutex};
use enumset::{EnumSet, EnumSetType};
#[cfg(feature = "sniffer")]
use esp_wifi_sys::include::{
    esp_wifi_80211_tx,
    esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_rx_cb,
    wifi_promiscuous_pkt_t,
    wifi_promiscuous_pkt_type_t,
};
use esp_wifi_sys::include::{
    wifi_pkt_rx_ctrl_t,
    WIFI_PROTOCOL_11AX,
    WIFI_PROTOCOL_11B,
    WIFI_PROTOCOL_11G,
    WIFI_PROTOCOL_11N,
    WIFI_PROTOCOL_LR,
};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
#[doc(hidden)]
pub(crate) use os_adapter::*;
#[cfg(feature = "sniffer")]
use portable_atomic::AtomicBool;
use portable_atomic::{AtomicUsize, Ordering};
#[cfg(feature = "smoltcp")]
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
pub use state::*;

use crate::{
    common_adapter::*,
    esp_wifi_result,
    hal::{
        macros::ram,
        peripheral::{Peripheral, PeripheralRef},
    },
    EspWifiInitialization,
};

const ETHERNET_FRAME_HEADER_SIZE: usize = 18;

const MTU: usize = crate::CONFIG.mtu;

#[cfg(feature = "utils")]
pub mod utils;

#[cfg(coex)]
use include::{coex_adapter_funcs_t, coex_pre_init, esp_coex_adapter_register};

use crate::{
    binary::{
        c_types,
        include::{
            self,
            __BindgenBitfieldUnit,
            esp_err_t,
            esp_interface_t_ESP_IF_WIFI_AP,
            esp_interface_t_ESP_IF_WIFI_STA,
            esp_supplicant_init,
            esp_wifi_connect,
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
            esp_wifi_set_ps,
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
            wifi_init_config_t,
            wifi_interface_t,
            wifi_interface_t_WIFI_IF_AP,
            wifi_interface_t_WIFI_IF_STA,
            wifi_mode_t,
            wifi_mode_t_WIFI_MODE_AP,
            wifi_mode_t_WIFI_MODE_APSTA,
            wifi_mode_t_WIFI_MODE_NULL,
            wifi_mode_t_WIFI_MODE_STA,
            wifi_osi_funcs_t,
            wifi_pmf_config_t,
            wifi_scan_config_t,
            wifi_scan_threshold_t,
            wifi_scan_time_t,
            wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
            wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
            wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            wifi_sta_config_t,
            wpa_crypto_funcs_t,
            ESP_WIFI_OS_ADAPTER_MAGIC,
            ESP_WIFI_OS_ADAPTER_VERSION,
            WIFI_INIT_CONFIG_MAGIC,
        },
    },
    compat::queue::SimpleQueue,
};

#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default)]
pub enum AuthMethod {
    None,
    WEP,
    WPA,
    #[default]
    WPA2Personal,
    WPAWPA2Personal,
    WPA2Enterprise,
    WPA3Personal,
    WPA2WPA3Personal,
    WAPIPersonal,
}

#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default)]
pub enum Protocol {
    P802D11B,
    P802D11BG,
    #[default]
    P802D11BGN,
    P802D11BGNLR,
    P802D11LR,
    P802D11BGNAX,
}

#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default)]
pub enum SecondaryChannel {
    // TODO: Need to extend that for 5GHz
    #[default]
    None,
    Above,
    Below,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccessPointInfo {
    pub ssid: heapless::String<32>,
    pub bssid: [u8; 6],
    pub channel: u8,
    pub secondary_channel: SecondaryChannel,
    pub signal_strength: i8,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub protocols: EnumSet<Protocol>,
    pub auth_method: Option<AuthMethod>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccessPointConfiguration {
    pub ssid: heapless::String<32>,
    pub ssid_hidden: bool,
    pub channel: u8,
    pub secondary_channel: Option<u8>,
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub protocols: EnumSet<Protocol>,
    pub auth_method: AuthMethod,
    pub password: heapless::String<64>,
    pub max_connections: u16,
}

impl Default for AccessPointConfiguration {
    fn default() -> Self {
        Self {
            ssid: "iot-device".try_into().unwrap(),
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

#[derive(Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub struct ClientConfiguration {
    pub ssid: heapless::String<32>,
    pub bssid: Option<[u8; 6]>,
    // pub protocol: Protocol,
    pub auth_method: AuthMethod,
    pub password: heapless::String<64>,
    pub channel: Option<u8>,
}

impl Debug for ClientConfiguration {
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

#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Capability {
    Client,
    AccessPoint,
    Mixed,
}

#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default)]
pub enum Configuration {
    #[default]
    None,
    Client(ClientConfiguration),
    AccessPoint(AccessPointConfiguration),
    Mixed(ClientConfiguration, AccessPointConfiguration),
}

impl Configuration {
    pub fn as_client_conf_ref(&self) -> Option<&ClientConfiguration> {
        match self {
            Self::Client(client_conf) | Self::Mixed(client_conf, _) => Some(client_conf),
            _ => None,
        }
    }

    pub fn as_ap_conf_ref(&self) -> Option<&AccessPointConfiguration> {
        match self {
            Self::AccessPoint(ap_conf) | Self::Mixed(_, ap_conf) => Some(ap_conf),
            _ => None,
        }
    }

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

pub mod ipv4 {
    use core::{fmt::Display, str::FromStr};

    pub use no_std_net::*;

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Mask(pub u8);

    impl FromStr for Mask {
        type Err = &'static str;

        fn from_str(s: &str) -> Result<Self, Self::Err> {
            s.parse::<u8>()
                .map_err(|_| "Invalid subnet mask")
                .map_or_else(Err, |mask| {
                    if (1..=32).contains(&mask) {
                        Ok(Mask(mask))
                    } else {
                        Err("Mask should be a number between 1 and 32")
                    }
                })
        }
    }

    impl Display for Mask {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "{}", self.0)
        }
    }

    impl TryFrom<Ipv4Addr> for Mask {
        type Error = ();

        fn try_from(ip: Ipv4Addr) -> Result<Self, Self::Error> {
            let octets = ip.octets();
            let addr: u32 = ((octets[0] as u32 & 0xff) << 24)
                | ((octets[1] as u32 & 0xff) << 16)
                | ((octets[2] as u32 & 0xff) << 8)
                | (octets[3] as u32 & 0xff);

            if addr.leading_ones() + addr.trailing_zeros() == 32 {
                Ok(Mask(addr.leading_ones() as u8))
            } else {
                Err(())
            }
        }
    }

    impl From<Mask> for Ipv4Addr {
        fn from(mask: Mask) -> Self {
            let addr: u32 = ((1 << (32 - mask.0)) - 1) ^ 0xffffffffu32;

            let (a, b, c, d) = (
                ((addr >> 24) & 0xff) as u8,
                ((addr >> 16) & 0xff) as u8,
                ((addr >> 8) & 0xff) as u8,
                (addr & 0xff) as u8,
            );

            Ipv4Addr::new(a, b, c, d)
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Subnet {
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub gateway: Ipv4Addr,
        pub mask: Mask,
    }

    impl Display for Subnet {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "{}/{}", self.gateway, self.mask)
        }
    }

    impl FromStr for Subnet {
        type Err = &'static str;

        fn from_str(s: &str) -> Result<Self, Self::Err> {
            let mut split = s.split('/');
            if let Some(gateway_str) = split.next() {
                if let Some(mask_str) = split.next() {
                    if split.next().is_none() {
                        if let Ok(gateway) = gateway_str.parse::<Ipv4Addr>() {
                            return mask_str.parse::<Mask>().map(|mask| Self { gateway, mask });
                        } else {
                            return Err("Invalid IP address format, expected XXX.XXX.XXX.XXX");
                        }
                    }
                }
            }

            Err("Expected <gateway-ip-address>/<mask>")
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ClientSettings {
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub ip: Ipv4Addr,
        pub subnet: Subnet,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub dns: Option<Ipv4Addr>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub secondary_dns: Option<Ipv4Addr>,
    }

    impl Default for ClientSettings {
        fn default() -> ClientSettings {
            ClientSettings {
                ip: Ipv4Addr::new(192, 168, 71, 200),
                subnet: Subnet {
                    gateway: Ipv4Addr::new(192, 168, 71, 1),
                    mask: Mask(24),
                },
                dns: Some(Ipv4Addr::new(8, 8, 8, 8)),
                secondary_dns: Some(Ipv4Addr::new(8, 8, 4, 4)),
            }
        }
    }

    #[derive(Default, Clone, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct DHCPClientSettings {
        pub hostname: Option<heapless::String<30>>,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClientConfiguration {
        DHCP(DHCPClientSettings),
        Fixed(ClientSettings),
    }

    impl ClientConfiguration {
        pub fn as_fixed_settings_ref(&self) -> Option<&ClientSettings> {
            match self {
                Self::Fixed(client_settings) => Some(client_settings),
                _ => None,
            }
        }

        pub fn as_fixed_settings_mut(&mut self) -> &mut ClientSettings {
            match self {
                Self::Fixed(client_settings) => client_settings,
                _ => {
                    *self = ClientConfiguration::Fixed(Default::default());
                    self.as_fixed_settings_mut()
                }
            }
        }
    }

    impl Default for ClientConfiguration {
        fn default() -> ClientConfiguration {
            ClientConfiguration::DHCP(Default::default())
        }
    }

    #[derive(Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct RouterConfiguration {
        pub subnet: Subnet,
        pub dhcp_enabled: bool,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub dns: Option<Ipv4Addr>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub secondary_dns: Option<Ipv4Addr>,
    }

    impl Default for RouterConfiguration {
        fn default() -> RouterConfiguration {
            RouterConfiguration {
                subnet: Subnet {
                    gateway: Ipv4Addr::new(192, 168, 71, 1),
                    mask: Mask(24),
                },
                dhcp_enabled: true,
                dns: Some(Ipv4Addr::new(8, 8, 8, 8)),
                secondary_dns: Some(Ipv4Addr::new(8, 8, 4, 4)),
            }
        }
    }

    #[derive(Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Configuration {
        Client(ClientConfiguration),
        Router(RouterConfiguration),
    }

    impl Default for Configuration {
        fn default() -> Self {
            Self::Client(Default::default())
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct IpInfo {
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub ip: Ipv4Addr,
        pub subnet: Subnet,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub dns: Option<Ipv4Addr>,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        pub secondary_dns: Option<Ipv4Addr>,
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
pub enum WifiMode {
    Sta,
    Ap,
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

    fn try_from(config: &Configuration) -> Result<Self, Self::Error> {
        let mode = match config {
            Configuration::None => return Err(WifiError::UnknownWifiMode),
            Configuration::AccessPoint(_) => Self::Ap,
            Configuration::Client(_) => Self::Sta,
            Configuration::Mixed(_, _) => Self::ApSta,
        };

        Ok(mode)
    }
}

impl TryFrom<wifi_mode_t> for WifiMode {
    type Error = WifiError;

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

const DATA_FRAME_SIZE: usize = MTU + ETHERNET_FRAME_HEADER_SIZE;

const RX_QUEUE_SIZE: usize = crate::CONFIG.rx_queue_size;
const TX_QUEUE_SIZE: usize = crate::CONFIG.tx_queue_size;

pub(crate) static DATA_QUEUE_RX_AP: Mutex<
    RefCell<SimpleQueue<EspWifiPacketBuffer, RX_QUEUE_SIZE>>,
> = Mutex::new(RefCell::new(SimpleQueue::new()));

pub(crate) static DATA_QUEUE_RX_STA: Mutex<
    RefCell<SimpleQueue<EspWifiPacketBuffer, RX_QUEUE_SIZE>>,
> = Mutex::new(RefCell::new(SimpleQueue::new()));

/// Common errors
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WifiError {
    NotInitialized,
    InternalError(InternalWifiError),
    Disconnected,
    UnknownWifiMode,
}

/// Events generated by the WiFi driver
#[repr(i32)]
#[derive(Debug, FromPrimitive, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WifiEvent {
    WifiReady = 0,
    ScanDone,
    StaStart,
    StaStop,
    StaConnected,
    StaDisconnected,
    StaAuthmodeChange,
    StaWpsErSuccess,
    StaWpsErFailed,
    StaWpsErTimeout,
    StaWpsErPin,
    StaWpsErPbcOverlap,
    ApStart,
    ApStop,
    ApStaconnected,
    ApStadisconnected,
    ApProbereqrecved,
    FtmReport,
    StaBssRssiLow,
    ActionTxStatus,
    RocDone,
    StaBeaconTimeout,
}

/// Error originating from the underlying drivers
#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[cfg(all(coex, any(esp32, esp32c2, esp32c3, esp32c6, esp32s3)))]
static mut G_COEX_ADAPTER_FUNCS: coex_adapter_funcs_t = coex_adapter_funcs_t {
    _version: include::COEX_ADAPTER_VERSION as i32,
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take_from_isr: Some(semphr_take_from_isr_wrapper),
    _semphr_give_from_isr: Some(semphr_give_from_isr_wrapper),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _is_in_isr: Some(is_in_isr_wrapper),
    _malloc_internal: Some(malloc),
    _free: Some(free),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _env_is_chip: Some(env_is_chip),
    _magic: include::COEX_ADAPTER_MAGIC as i32,
    _timer_disarm: Some(ets_timer_disarm),
    _timer_done: Some(ets_timer_done),
    _timer_setfn: Some(ets_timer_setfn),
    _timer_arm_us: Some(ets_timer_arm_us),

    #[cfg(esp32)]
    _spin_lock_create: Some(spin_lock_create),
    #[cfg(esp32)]
    _spin_lock_delete: Some(spin_lock_delete),
    #[cfg(esp32)]
    _int_disable: Some(wifi_int_disable),
    #[cfg(esp32)]
    _int_enable: Some(wifi_int_restore),

    #[cfg(esp32c2)]
    _slowclk_cal_get: Some(slowclk_cal_get),
};

#[cfg(coex)]
unsafe extern "C" fn semphr_take_from_isr_wrapper(
    semphr: *mut c_types::c_void,
    hptw: *mut c_types::c_void,
) -> i32 {
    crate::common_adapter::semphr_take_from_isr(semphr as *const (), hptw as *const ())
}

#[cfg(coex)]
unsafe extern "C" fn semphr_give_from_isr_wrapper(
    semphr: *mut c_types::c_void,
    hptw: *mut c_types::c_void,
) -> i32 {
    crate::common_adapter::semphr_give_from_isr(semphr as *const (), hptw as *const ())
}

#[cfg(coex)]
unsafe extern "C" fn is_in_isr_wrapper() -> i32 {
    // like original implementation
    0
}

#[cfg(coex)]
pub(crate) fn coex_initialize() -> i32 {
    debug!("call coex-initialize");
    unsafe {
        let res = esp_coex_adapter_register(core::ptr::addr_of_mut!(G_COEX_ADAPTER_FUNCS).cast());
        if res != 0 {
            error!("Error: esp_coex_adapter_register {}", res);
            return res;
        }
        let res = coex_pre_init();
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
        return include::coex_init();
    }

    #[cfg(not(coex))]
    0
}

#[no_mangle]
static g_wifi_osi_funcs: wifi_osi_funcs_t = wifi_osi_funcs_t {
    _version: ESP_WIFI_OS_ADAPTER_VERSION as i32,
    _env_is_chip: Some(env_is_chip),
    _set_intr: Some(set_intr),
    _clear_intr: Some(clear_intr),
    _set_isr: Some(os_adapter_chip_specific::set_isr),
    _ints_on: Some(ints_on),
    _ints_off: Some(ints_off),
    _is_from_isr: Some(is_from_isr),
    _spin_lock_create: Some(spin_lock_create),
    _spin_lock_delete: Some(spin_lock_delete),
    _wifi_int_disable: Some(wifi_int_disable),
    _wifi_int_restore: Some(wifi_int_restore),
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _wifi_thread_semphr_get: Some(wifi_thread_semphr_get),
    _mutex_create: Some(mutex_create),
    _recursive_mutex_create: Some(recursive_mutex_create),
    _mutex_delete: Some(mutex_delete),
    _mutex_lock: Some(mutex_lock),
    _mutex_unlock: Some(mutex_unlock),
    _queue_create: Some(queue_create),
    _queue_delete: Some(queue_delete),
    _queue_send: Some(queue_send),
    _queue_send_from_isr: Some(queue_send_from_isr),
    _queue_send_to_back: Some(queue_send_to_back),
    _queue_send_to_front: Some(queue_send_to_front),
    _queue_recv: Some(queue_recv),
    _queue_msg_waiting: Some(queue_msg_waiting),
    _event_group_create: Some(event_group_create),
    _event_group_delete: Some(event_group_delete),
    _event_group_set_bits: Some(event_group_set_bits),
    _event_group_clear_bits: Some(event_group_clear_bits),
    _event_group_wait_bits: Some(event_group_wait_bits),
    _task_create_pinned_to_core: Some(task_create_pinned_to_core),
    _task_create: Some(task_create),
    _task_delete: Some(task_delete),
    _task_delay: Some(task_delay),
    _task_ms_to_tick: Some(task_ms_to_tick),
    _task_get_current_task: Some(task_get_current_task),
    _task_get_max_priority: Some(task_get_max_priority),
    _malloc: Some(malloc),
    _free: Some(free),
    _event_post: Some(event_post),
    _get_free_heap_size: Some(get_free_heap_size),
    _rand: Some(rand),
    _dport_access_stall_other_cpu_start_wrap: Some(dport_access_stall_other_cpu_start_wrap),
    _dport_access_stall_other_cpu_end_wrap: Some(dport_access_stall_other_cpu_end_wrap),
    _wifi_apb80m_request: Some(wifi_apb80m_request),
    _wifi_apb80m_release: Some(wifi_apb80m_release),
    _phy_disable: Some(phy_disable),
    _phy_enable: Some(phy_enable),
    _phy_update_country_info: Some(phy_update_country_info),
    _read_mac: Some(read_mac),
    _timer_arm: Some(ets_timer_arm),
    _timer_disarm: Some(ets_timer_disarm),
    _timer_done: Some(ets_timer_done),
    _timer_setfn: Some(ets_timer_setfn),
    _timer_arm_us: Some(ets_timer_arm_us),
    _wifi_reset_mac: Some(wifi_reset_mac),
    _wifi_clock_enable: Some(wifi_clock_enable),
    _wifi_clock_disable: Some(wifi_clock_disable),
    _wifi_rtc_enable_iso: Some(wifi_rtc_enable_iso),
    _wifi_rtc_disable_iso: Some(wifi_rtc_disable_iso),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _nvs_set_i8: Some(nvs_set_i8),
    _nvs_get_i8: Some(nvs_get_i8),
    _nvs_set_u8: Some(nvs_set_u8),
    _nvs_get_u8: Some(nvs_get_u8),
    _nvs_set_u16: Some(nvs_set_u16),
    _nvs_get_u16: Some(nvs_get_u16),
    _nvs_open: Some(nvs_open),
    _nvs_close: Some(nvs_close),
    _nvs_commit: Some(nvs_commit),
    _nvs_set_blob: Some(nvs_set_blob),
    _nvs_get_blob: Some(nvs_get_blob),
    _nvs_erase_key: Some(nvs_erase_key),
    _get_random: Some(get_random),
    _get_time: Some(get_time),
    _random: Some(random),
    #[cfg(feature = "wifi-logs")]
    _log_write: Some(log_write),
    #[cfg(not(feature = "wifi-logs"))]
    _log_write: None,
    #[cfg(feature = "wifi-logs")]
    _log_writev: Some(log_writev),
    #[cfg(not(feature = "wifi-logs"))]
    _log_writev: None,
    _log_timestamp: Some(log_timestamp),
    _malloc_internal: Some(malloc_internal),
    _realloc_internal: Some(realloc_internal),
    _calloc_internal: Some(calloc_internal),
    _zalloc_internal: Some(zalloc_internal),
    _wifi_malloc: Some(wifi_malloc),
    _wifi_realloc: Some(wifi_realloc),
    _wifi_calloc: Some(wifi_calloc),
    _wifi_zalloc: Some(wifi_zalloc),
    _wifi_create_queue: Some(wifi_create_queue),
    _wifi_delete_queue: Some(wifi_delete_queue),
    _coex_init: Some(coex_init),
    _coex_deinit: Some(coex_deinit),
    _coex_enable: Some(coex_enable),
    _coex_disable: Some(coex_disable),
    _coex_status_get: Some(coex_status_get),
    _coex_condition_set: None,
    _coex_wifi_request: Some(coex_wifi_request),
    _coex_wifi_release: Some(coex_wifi_release),
    _coex_wifi_channel_set: Some(coex_wifi_channel_set),
    _coex_event_duration_get: Some(coex_event_duration_get),
    _coex_pti_get: Some(coex_pti_get),
    _coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    _coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    _coex_schm_interval_set: Some(coex_schm_interval_set),
    _coex_schm_interval_get: Some(coex_schm_interval_get),
    _coex_schm_curr_period_get: Some(coex_schm_curr_period_get),
    _coex_schm_curr_phase_get: Some(coex_schm_curr_phase_get),
    #[cfg(any(esp32c3, esp32c2, esp32c6, esp32h2, esp32s3, esp32s2))]
    _slowclk_cal_get: Some(slowclk_cal_get),
    #[cfg(any(esp32, esp32s2))]
    _phy_common_clock_disable: Some(os_adapter_chip_specific::phy_common_clock_disable),
    #[cfg(any(esp32, esp32s2))]
    _phy_common_clock_enable: Some(os_adapter_chip_specific::phy_common_clock_enable),
    _coex_register_start_cb: Some(coex_register_start_cb),

    #[cfg(esp32c6)]
    _regdma_link_set_write_wait_content: Some(
        os_adapter_chip_specific::regdma_link_set_write_wait_content_dummy,
    ),
    #[cfg(esp32c6)]
    _sleep_retention_find_link_by_id: Some(
        os_adapter_chip_specific::sleep_retention_find_link_by_id_dummy,
    ),
    #[cfg(esp32c6)]
    _sleep_retention_entries_create: Some(
        os_adapter_chip_specific::sleep_retention_entries_create_dummy,
    ),
    #[cfg(esp32c6)]
    _sleep_retention_entries_destroy: Some(
        os_adapter_chip_specific::sleep_retention_entries_destroy_dummy,
    ),

    _coex_schm_process_restart: Some(coex_schm_process_restart_wrapper),
    _coex_schm_register_cb: Some(coex_schm_register_cb_wrapper),

    _magic: ESP_WIFI_OS_ADAPTER_MAGIC as i32,
};

const CONFIG_FEATURE_WPA3_SAE_BIT: u64 = 1 << 0;

const WIFI_FEATURE_CAPS: u64 = CONFIG_FEATURE_WPA3_SAE_BIT;

#[no_mangle]
static mut g_wifi_feature_caps: u64 = WIFI_FEATURE_CAPS;

static mut G_CONFIG: wifi_init_config_t = wifi_init_config_t {
    osi_funcs: addr_of!(g_wifi_osi_funcs).cast_mut(),

    // dummy for now - populated in init
    wpa_crypto_funcs: wpa_crypto_funcs_t {
        size: 0,
        version: 1,
        aes_wrap: None,
        aes_unwrap: None,
        hmac_sha256_vector: None,
        sha256_prf: None,
        hmac_md5: None,
        hamc_md5_vector: None,
        hmac_sha1: None,
        hmac_sha1_vector: None,
        sha1_prf: None,
        sha1_vector: None,
        pbkdf2_sha1: None,
        rc4_skip: None,
        md5_vector: None,
        aes_encrypt: None,
        aes_encrypt_init: None,
        aes_encrypt_deinit: None,
        aes_decrypt: None,
        aes_decrypt_init: None,
        aes_decrypt_deinit: None,
        aes_128_encrypt: None,
        aes_128_decrypt: None,
        omac1_aes_128: None,
        ccmp_decrypt: None,
        ccmp_encrypt: None,
        aes_gmac: None,
        sha256_vector: None,
        crc32: None,
    },
    static_rx_buf_num: crate::CONFIG.static_rx_buf_num as i32,
    dynamic_rx_buf_num: crate::CONFIG.dynamic_rx_buf_num as i32,
    tx_buf_type: 1,
    static_tx_buf_num: crate::CONFIG.static_tx_buf_num as i32,
    dynamic_tx_buf_num: crate::CONFIG.dynamic_tx_buf_num as i32,
    rx_mgmt_buf_type: 0_i32,
    rx_mgmt_buf_num: 0_i32,
    cache_tx_buf_num: 0,
    csi_enable: 1,
    ampdu_rx_enable: crate::CONFIG.ampdu_rx_enable as i32,
    ampdu_tx_enable: crate::CONFIG.ampdu_tx_enable as i32,
    amsdu_tx_enable: crate::CONFIG.amsdu_tx_enable as i32,
    nvs_enable: 0,
    nano_enable: 0,
    rx_ba_win: crate::CONFIG.rx_ba_win as i32,
    wifi_task_core_id: 0,
    beacon_max_len: 752,
    mgmt_sbuf_num: 32,
    feature_caps: WIFI_FEATURE_CAPS,
    sta_disconnected_pm: false,
    espnow_max_encrypt_num: 7, // 2 for ESP32-C2
    magic: WIFI_INIT_CONFIG_MAGIC as i32,
};

/// Get the STA MAC address
pub fn get_sta_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 0);
    }
}

/// Get the AP MAC address
pub fn get_ap_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 1);
    }
}

pub(crate) fn wifi_init() -> Result<(), WifiError> {
    unsafe {
        G_CONFIG.wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs;
        G_CONFIG.feature_caps = g_wifi_feature_caps;

        #[cfg(coex)]
        esp_wifi_result!(coex_init())?;

        esp_wifi_result!(esp_wifi_init_internal(addr_of!(G_CONFIG)))?;
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

        Ok(())
    }
}

unsafe extern "C" fn recv_cb_sta(
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
    // are in a critical section.
    match critical_section::with(|cs| DATA_QUEUE_RX_STA.borrow_ref_mut(cs).enqueue(packet)) {
        Ok(_) => {
            #[cfg(feature = "embassy-net")]
            embassy::STA_RECEIVE_WAKER.wake();
            include::ESP_OK as esp_err_t
        }
        Err(_) => {
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
    // are in a critical section.
    match critical_section::with(|cs| DATA_QUEUE_RX_AP.borrow_ref_mut(cs).enqueue(packet)) {
        Ok(_) => {
            #[cfg(feature = "embassy-net")]
            embassy::AP_RECEIVE_WAKER.wake();
            include::ESP_OK as esp_err_t
        }
        Err(_) => {
            debug!("RX QUEUE FULL");
            include::ESP_ERR_NO_MEM as esp_err_t
        }
    }
}

pub(crate) static WIFI_TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);

fn decrement_inflight_counter() {
    WIFI_TX_INFLIGHT
        .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
            Some(x.saturating_sub(1))
        })
        .unwrap();
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

    #[cfg(feature = "embassy-net")]
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

        cfg_if::cfg_if! {
            if #[cfg(feature = "ps-min-modem")] {
                let ps_mode = include::wifi_ps_type_t_WIFI_PS_MIN_MODEM;
            } else if #[cfg(feature = "ps-max-modem")] {
                let ps_mode = include::wifi_ps_type_t_WIFI_PS_MAX_MODEM;
            } else if #[cfg(coex)] {
                let ps_mode = include::wifi_ps_type_t_WIFI_PS_MIN_MODEM;
            } else {
                let ps_mode = include::wifi_ps_type_t_WIFI_PS_NONE;
            }
        };

        esp_wifi_result!(esp_wifi_set_ps(ps_mode))?;

        let mut cntry_code = [0u8; 3];
        cntry_code[..crate::CONFIG.country_code.len()]
            .copy_from_slice(crate::CONFIG.country_code.as_bytes());
        cntry_code[2] = crate::CONFIG.country_code_operating_class;

        let country = wifi_country_t {
            cc: core::mem::transmute::<[u8; 3], [i8; 3]>(cntry_code), // [u8] -> [i8] conversion
            schan: 1,
            nchan: 13,
            max_tx_power: 20,
            policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        };
        esp_wifi_result!(esp_wifi_set_country(&country))?;
    }

    Ok(())
}

unsafe extern "C" fn coex_register_start_cb(
    _cb: Option<unsafe extern "C" fn() -> c_types::c_int>,
) -> c_types::c_int {
    #[cfg(coex)]
    return include::coex_register_start_cb(_cb);

    #[cfg(not(coex))]
    0
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
        let mut buf = heapless::Vec::<u8, 33>::from_iter(m.bytes());
        unwrap!(buf.push(b'\0').ok());
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
    };

    unsafe { esp_wifi_scan_start(&scan_config, block) }
}

/// Creates a new [WifiDevice] and [WifiController] in either AP or STA mode
/// with the given configuration.
///
/// This function will panic if the configuration is not
/// [`Configuration::Client`] or [`Configuration::AccessPoint`].
///
/// If you want to use AP-STA mode, use `[new_ap_sta]`.
pub fn new_with_config<'d, MODE: WifiDeviceMode>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    config: MODE::Config,
) -> Result<(WifiDevice<'d, MODE>, WifiController<'d>), WifiError> {
    crate::hal::into_ref!(device);

    Ok((
        WifiDevice::new(unsafe { device.clone_unchecked() }, MODE::new()),
        WifiController::new_with_config(inited, device, MODE::wrap_config(config))?,
    ))
}

/// Creates a new [WifiDevice] and [WifiController] in either AP or STA mode
/// with a default configuration.
///
/// This function will panic if the mode is [`WifiMode::ApSta`].
/// If you want to use AP-STA mode, use `[new_ap_sta]`.
pub fn new_with_mode<'d, MODE: WifiDeviceMode>(
    inited: &EspWifiInitialization,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    _mode: MODE,
) -> Result<(WifiDevice<'d, MODE>, WifiController<'d>), WifiError> {
    new_with_config(inited, device, <MODE as Sealed>::Config::default())
}

/// Creates a new [WifiDevice] and [WifiController] in AP-STA mode, with a
/// default configuration.
///
/// Returns a tuple of `(AP device, STA device, controller)`.
pub fn new_ap_sta<'d>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
) -> Result<
    (
        WifiDevice<'d, WifiApDevice>,
        WifiDevice<'d, WifiStaDevice>,
        WifiController<'d>,
    ),
    WifiError,
> {
    new_ap_sta_with_config(inited, device, Default::default(), Default::default())
}

/// Creates a new Wifi device and controller in AP-STA mode.
///
/// Returns a tuple of `(AP device, STA device, controller)`.
pub fn new_ap_sta_with_config<'d>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    sta_config: crate::wifi::ClientConfiguration,
    ap_config: crate::wifi::AccessPointConfiguration,
) -> Result<
    (
        WifiDevice<'d, WifiApDevice>,
        WifiDevice<'d, WifiStaDevice>,
        WifiController<'d>,
    ),
    WifiError,
> {
    crate::hal::into_ref!(device);

    Ok((
        WifiDevice::new(unsafe { device.clone_unchecked() }, WifiApDevice),
        WifiDevice::new(unsafe { device.clone_unchecked() }, WifiStaDevice),
        WifiController::new_with_config(
            inited,
            device,
            Configuration::Mixed(sta_config, ap_config),
        )?,
    ))
}

mod sealed {
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

    pub trait Sealed: Copy + Sized {
        type Config: Default;

        fn new() -> Self;

        fn wrap_config(config: Self::Config) -> Configuration;

        fn data_queue_rx(
            self,
            cs: CriticalSection,
        ) -> RefMut<'_, SimpleQueue<EspWifiPacketBuffer, RX_QUEUE_SIZE>>;

        fn can_send(self) -> bool {
            WIFI_TX_INFLIGHT.load(Ordering::SeqCst) < TX_QUEUE_SIZE
        }

        fn increase_in_flight_counter(self) {
            WIFI_TX_INFLIGHT.fetch_add(1, Ordering::SeqCst);
        }

        fn tx_token(self) -> Option<WifiTxToken<Self>> {
            if !self.can_send() {
                crate::timer::yield_task();
            }

            if self.can_send() {
                Some(WifiTxToken { mode: self })
            } else {
                None
            }
        }

        fn rx_token(self) -> Option<(WifiRxToken<Self>, WifiTxToken<Self>)> {
            let is_empty = critical_section::with(|cs| self.data_queue_rx(cs).is_empty());
            if is_empty || !self.can_send() {
                crate::timer::yield_task();
            }

            let is_empty =
                is_empty && critical_section::with(|cs| self.data_queue_rx(cs).is_empty());

            if !is_empty {
                self.tx_token().map(|tx| (WifiRxToken { mode: self }, tx))
            } else {
                None
            }
        }

        fn interface(self) -> wifi_interface_t;

        #[cfg(feature = "embassy-net")]
        fn register_transmit_waker(self, cx: &mut core::task::Context) {
            embassy::TRANSMIT_WAKER.register(cx.waker())
        }

        #[cfg(feature = "embassy-net")]
        fn register_receive_waker(self, cx: &mut core::task::Context);

        #[cfg(feature = "embassy-net")]
        fn register_link_state_waker(self, cx: &mut core::task::Context);

        #[cfg(feature = "embassy-net")]
        fn link_state(self) -> embassy_net_driver::LinkState;
    }

    impl Sealed for WifiStaDevice {
        type Config = ClientConfiguration;

        fn new() -> Self {
            Self
        }

        fn wrap_config(config: ClientConfiguration) -> Configuration {
            Configuration::Client(config)
        }

        fn data_queue_rx(
            self,
            cs: CriticalSection,
        ) -> RefMut<'_, SimpleQueue<EspWifiPacketBuffer, RX_QUEUE_SIZE>> {
            DATA_QUEUE_RX_STA.borrow_ref_mut(cs)
        }

        fn interface(self) -> wifi_interface_t {
            wifi_interface_t_WIFI_IF_STA
        }

        #[cfg(feature = "embassy-net")]
        fn register_receive_waker(self, cx: &mut core::task::Context) {
            embassy::STA_RECEIVE_WAKER.register(cx.waker());
        }

        #[cfg(feature = "embassy-net")]
        fn register_link_state_waker(self, cx: &mut core::task::Context) {
            embassy::STA_LINK_STATE_WAKER.register(cx.waker());
        }

        #[cfg(feature = "embassy-net")]
        fn link_state(self) -> embassy_net_driver::LinkState {
            if matches!(get_sta_state(), WifiState::StaConnected) {
                embassy_net_driver::LinkState::Up
            } else {
                embassy_net_driver::LinkState::Down
            }
        }
    }

    impl Sealed for WifiApDevice {
        type Config = AccessPointConfiguration;

        fn new() -> Self {
            Self
        }

        fn wrap_config(config: AccessPointConfiguration) -> Configuration {
            Configuration::AccessPoint(config)
        }

        fn data_queue_rx(
            self,
            cs: CriticalSection,
        ) -> RefMut<'_, SimpleQueue<EspWifiPacketBuffer, RX_QUEUE_SIZE>> {
            DATA_QUEUE_RX_AP.borrow_ref_mut(cs)
        }

        fn interface(self) -> wifi_interface_t {
            wifi_interface_t_WIFI_IF_AP
        }

        #[cfg(feature = "embassy-net")]
        fn register_receive_waker(self, cx: &mut core::task::Context) {
            embassy::AP_RECEIVE_WAKER.register(cx.waker());
        }

        #[cfg(feature = "embassy-net")]
        fn register_link_state_waker(self, cx: &mut core::task::Context) {
            embassy::AP_LINK_STATE_WAKER.register(cx.waker());
        }

        #[cfg(feature = "embassy-net")]
        fn link_state(self) -> embassy_net_driver::LinkState {
            if matches!(get_ap_state(), WifiState::ApStarted) {
                embassy_net_driver::LinkState::Up
            } else {
                embassy_net_driver::LinkState::Down
            }
        }
    }
}

use sealed::*;

pub trait WifiDeviceMode: Sealed {
    fn mode(self) -> WifiMode;

    fn mac_address(self) -> [u8; 6];
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WifiStaDevice;

impl WifiDeviceMode for WifiStaDevice {
    fn mode(self) -> WifiMode {
        WifiMode::Sta
    }

    fn mac_address(self) -> [u8; 6] {
        let mut mac = [0; 6];
        get_sta_mac(&mut mac);
        mac
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WifiApDevice;

impl WifiDeviceMode for WifiApDevice {
    fn mode(self) -> WifiMode {
        WifiMode::Ap
    }

    fn mac_address(self) -> [u8; 6] {
        let mut mac = [0; 6];
        get_ap_mac(&mut mac);
        mac
    }
}

/// A wifi device implementing smoltcp's Device trait.
pub struct WifiDevice<'d, MODE: WifiDeviceMode> {
    _device: PeripheralRef<'d, crate::hal::peripherals::WIFI>,
    mode: MODE,
}

impl<'d, MODE: WifiDeviceMode> WifiDevice<'d, MODE> {
    pub(crate) fn new(
        _device: PeripheralRef<'d, crate::hal::peripherals::WIFI>,
        mode: MODE,
    ) -> Self {
        Self { _device, mode }
    }

    pub fn mac_address(&self) -> [u8; 6] {
        self.mode.mac_address()
    }

    #[cfg(not(feature = "smoltcp"))]
    pub fn receive(&mut self) -> Option<(WifiRxToken<MODE>, WifiTxToken<MODE>)> {
        self.mode.rx_token()
    }

    #[cfg(not(feature = "smoltcp"))]
    pub fn transmit(&mut self) -> Option<WifiTxToken<MODE>> {
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

    let mut ssid = heapless::String::<32>::new();
    unwrap!(ssid.push_str(ssid_ref));

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
        protocols: EnumSet::empty(), // TODO
        auth_method: Some(AuthMethod::from_raw(record.authmode)),
    }
}

#[cfg(not(any(esp32c6)))]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxControlInfo {
    pub rssi: i32,
    pub rate: u32,
    pub sig_mode: u32,
    pub mcs: u32,
    pub cwb: u32,
    pub smoothing: u32,
    pub not_sounding: u32,
    pub aggregation: u32,
    pub stbc: u32,
    pub fec_coding: u32,
    pub sgi: u32,
    pub ampdu_cnt: u32,
    pub channel: u32,
    pub secondary_channel: u32,
    pub timestamp: u32,
    pub noise_floor: i32,
    pub ant: u32,
    pub sig_len: u32,
    pub rx_state: u32,
}

#[cfg(esp32c6)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxControlInfo {
    pub rssi: i32,
    pub rate: u32,
    pub sig_len: u32,
    pub rx_state: u32,
    pub dump_len: u32,
    pub he_sigb_len: u32,
    pub cur_single_mpdu: u32,
    pub cur_bb_format: u32,
    pub rx_channel_estimate_info_vld: u32,
    pub rx_channel_estimate_len: u32,
    pub second: u32,
    pub channel: u32,
    pub data_rssi: i32,
    pub noise_floor: u32,
    pub is_group: u32,
    pub rxend_state: u32,
    pub rxmatch3: u32,
    pub rxmatch2: u32,
    pub rxmatch1: u32,
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
        let rx_control_info = RxControlInfo {
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
        };
        #[cfg(esp32c6)]
        let rx_control_info = RxControlInfo {
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
            data_rssi: (*rx_cntl).data_rssi(),
            noise_floor: (*rx_cntl).noise_floor(),
            is_group: (*rx_cntl).is_group(),
            rxend_state: (*rx_cntl).rxend_state(),
            rxmatch3: (*rx_cntl).rxmatch3(),
            rxmatch2: (*rx_cntl).rxmatch2(),
            rxmatch1: (*rx_cntl).rxmatch1(),
            rxmatch0: (*rx_cntl).rxmatch0(),
        };
        rx_control_info
    }
}
#[cfg(feature = "sniffer")]
pub struct PromiscuousPkt<'a> {
    pub rx_cntl: RxControlInfo,
    pub frame_type: wifi_promiscuous_pkt_type_t,
    pub len: usize,
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
        let rx_cntl = RxControlInfo::from_raw(&(*buf).rx_ctrl);
        let len = rx_cntl.sig_len as usize;
        PromiscuousPkt {
            rx_cntl,
            frame_type,
            len,
            data: core::slice::from_raw_parts(
                (buf as *const u8).add(size_of::<wifi_pkt_rx_ctrl_t>()),
                len,
            ),
        }
    }
}

#[cfg(feature = "sniffer")]
static SNIFFER_CB: Mutex<RefCell<Option<fn(PromiscuousPkt)>>> = Mutex::new(RefCell::new(None));

#[cfg(feature = "sniffer")]
unsafe extern "C" fn promiscuous_rx_cb(buf: *mut core::ffi::c_void, frame_type: u32) {
    critical_section::with(|cs| {
        let Some(sniffer_callback) = *SNIFFER_CB.borrow_ref(cs) else {
            return;
        };
        let promiscuous_pkt = PromiscuousPkt::from_raw(buf as *const _, frame_type);
        sniffer_callback(promiscuous_pkt);
    });
}

#[cfg(feature = "sniffer")]
/// A wifi sniffer.
pub struct Sniffer {
    promiscuous_mode_enabled: AtomicBool,
}
#[cfg(feature = "sniffer")]
impl Sniffer {
    pub(crate) fn new() -> Self {
        // This shouldn't fail, since the way this is created, means that wifi will
        // always be initialized.
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_cb)) })
            .unwrap();
        Self {
            promiscuous_mode_enabled: AtomicBool::new(false),
        }
    }
    /// Set promiscuous mode enabled or disabled.
    pub fn set_promiscuous_mode(&self, enabled: bool) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        self.promiscuous_mode_enabled
            .store(enabled, Ordering::Relaxed);
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
                if use_sta_interface { 0 } else { 1 } as wifi_interface_t,
                buffer.as_ptr() as *const _,
                buffer.len() as i32,
                use_internal_seq_num,
            )
        })
    }
    /// Set the callback for receiving a packet.
    pub fn set_receive_cb(&mut self, cb: fn(PromiscuousPkt)) {
        critical_section::with(|cs| {
            *SNIFFER_CB.borrow_ref_mut(cs) = Some(cb);
        });
    }
}

/// A wifi controller
pub struct WifiController<'d> {
    _device: PeripheralRef<'d, crate::hal::peripherals::WIFI>,
    config: Configuration,
    #[cfg(feature = "sniffer")]
    sniffer_taken: AtomicBool,
}

impl<'d> WifiController<'d> {
    pub(crate) fn new_with_config(
        inited: &EspWifiInitialization,
        _device: PeripheralRef<'d, crate::hal::peripherals::WIFI>,
        config: Configuration,
    ) -> Result<Self, WifiError> {
        if !inited.is_wifi() {
            return Err(WifiError::NotInitialized);
        }

        // We set up the controller with the default config because we need to call
        // `set_configuration` to apply the actual configuration, and it will update the
        // stored configuration anyway.
        let mut this = Self {
            _device,
            config: Default::default(),
            #[cfg(feature = "sniffer")]
            sniffer_taken: AtomicBool::new(false),
        };

        let mode = WifiMode::try_from(&config)?;
        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode.into()) })?;
        debug!("Wifi mode {:?} set", mode);

        this.set_configuration(&config)?;
        Ok(this)
    }
    #[cfg(feature = "sniffer")]
    pub fn take_sniffer(&self) -> Option<Sniffer> {
        if self
            .sniffer_taken
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            == Ok(false)
        {
            Some(Sniffer::new())
        } else {
            None
        }
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
        let mut protocol = 0u8;

        protocols.into_iter().for_each(|v| match v {
            Protocol::P802D11B => protocol |= WIFI_PROTOCOL_11B as u8,
            Protocol::P802D11BG => protocol |= WIFI_PROTOCOL_11B as u8 | WIFI_PROTOCOL_11G as u8,
            Protocol::P802D11BGN => {
                protocol |=
                    WIFI_PROTOCOL_11B as u8 | WIFI_PROTOCOL_11G as u8 | WIFI_PROTOCOL_11N as u8
            }
            Protocol::P802D11BGNLR => {
                protocol |= WIFI_PROTOCOL_11B as u8
                    | WIFI_PROTOCOL_11G as u8
                    | WIFI_PROTOCOL_11N as u8
                    | WIFI_PROTOCOL_LR as u8
            }
            Protocol::P802D11LR => protocol |= WIFI_PROTOCOL_LR as u8,
            Protocol::P802D11BGNAX => {
                protocol |= WIFI_PROTOCOL_11B as u8
                    | WIFI_PROTOCOL_11G as u8
                    | WIFI_PROTOCOL_11N as u8
                    | WIFI_PROTOCOL_11AX as u8
            }
        });

        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut mode) })?;

        if mode == wifi_mode_t_WIFI_MODE_STA || mode == wifi_mode_t_WIFI_MODE_APSTA {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_STA, protocol)
            })?;
        }
        if mode == wifi_mode_t_WIFI_MODE_AP || mode == wifi_mode_t_WIFI_MODE_APSTA {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_AP, protocol)
            })?;
        }

        Ok(())
    }

    pub fn is_sta_enabled(&self) -> Result<bool, WifiError> {
        WifiMode::try_from(&self.config).map(|m| m.is_sta())
    }

    pub fn is_ap_enabled(&self) -> Result<bool, WifiError> {
        WifiMode::try_from(&self.config).map(|m| m.is_ap())
    }

    /// A blocking wifi network scan with caller-provided scanning options.
    pub fn scan_with_config_sync<const N: usize>(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true, config))?;

        let count = self.scan_result_count()?;
        let result = self.scan_results()?;

        Ok((result, count))
    }

    fn scan_result_count(&mut self) -> Result<usize, WifiError> {
        let mut bss_total: u16 = 0;

        // Prevents memory leak on error
        let guard = FreeApListOnDrop;

        unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_num(&mut bss_total))? };

        guard.defuse();

        Ok(bss_total as usize)
    }

    fn scan_results<const N: usize>(
        &mut self,
    ) -> Result<heapless::Vec<AccessPointInfo, N>, WifiError> {
        let mut scanned = heapless::Vec::<AccessPointInfo, N>::new();
        let mut bss_total: u16 = N as u16;

        let mut records: [MaybeUninit<include::wifi_ap_record_t>; N] = [MaybeUninit::uninit(); N];

        // Prevents memory leak on error
        let guard = FreeApListOnDrop;

        unsafe {
            esp_wifi_result!(include::esp_wifi_scan_get_ap_records(
                &mut bss_total,
                records[0].as_mut_ptr(),
            ))?
        };

        guard.defuse();

        for i in 0..bss_total {
            let record = unsafe { MaybeUninit::assume_init_ref(&records[i as usize]) };
            let ap_info = convert_ap_info(record);

            scanned.push(ap_info).ok();
        }

        Ok(scanned)
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
#[cfg(feature = "smoltcp")]
impl<MODE: WifiDeviceMode> Device for WifiDevice<'_, MODE> {
    type RxToken<'a> = WifiRxToken<MODE> where Self: 'a;
    type TxToken<'a> = WifiTxToken<MODE> where Self: 'a;

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
pub struct WifiRxToken<MODE: Sealed> {
    mode: MODE,
}

impl<MODE: Sealed> WifiRxToken<MODE> {
    pub fn consume_token<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut data = critical_section::with(|cs| {
            let mut queue = self.mode.data_queue_rx(cs);

            unwrap!(
                queue.dequeue(),
                "unreachable: transmit()/receive() ensures there is a packet to process"
            )
        });

        // We handle the received data outside of the critical section because
        // EspWifiPacketBuffer::drop must not be called in a critical section.
        // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
        // which will try to lock an internal mutex. If the mutex is already
        // taken, the function will try to trigger a context switch, which will
        // fail if we are in a critical section.
        let buffer = data.as_slice_mut();
        dump_packet_info(buffer);

        f(buffer)
    }
}

#[cfg(feature = "smoltcp")]
impl<MODE: Sealed> RxToken for WifiRxToken<MODE> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume_token(f)
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiTxToken<MODE: Sealed> {
    mode: MODE,
}

impl<MODE: Sealed> WifiTxToken<MODE> {
    pub fn consume_token<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.mode.increase_in_flight_counter();

        // (safety): creation of multiple WiFi devices with the same mode is impossible
        // in safe Rust, therefore only smoltcp _or_ embassy-net can be used at
        // one time
        static mut BUFFER: [u8; DATA_FRAME_SIZE] = [0u8; DATA_FRAME_SIZE];

        let buffer = unsafe { &mut BUFFER[..len] };

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

#[cfg(feature = "smoltcp")]
impl<MODE: Sealed> TxToken for WifiTxToken<MODE> {
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

impl WifiController<'_> {
    /// Get the supported capabilities of the controller.
    pub fn get_capabilities(&self) -> Result<EnumSet<crate::wifi::Capability>, WifiError> {
        let caps = match self.config {
            Configuration::None => unreachable!(),
            Configuration::Client(_) => enumset::enum_set! { Capability::Client },
            Configuration::AccessPoint(_) => enumset::enum_set! { Capability::AccessPoint },
            Configuration::Mixed(_, _) => {
                Capability::Client | Capability::AccessPoint | Capability::Mixed
            }
        };

        Ok(caps)
    }

    /// Get the currently used configuration.
    pub fn get_configuration(&self) -> Result<Configuration, WifiError> {
        Ok(self.config.clone())
    }

    /// Set the configuration, you need to use Wifi::connect() for connecting to
    /// an AP
    pub fn set_configuration(&mut self, conf: &Configuration) -> Result<(), WifiError> {
        match self.config {
            Configuration::None => self.config = conf.clone(), // initial config
            Configuration::Client(ref mut client) => {
                if let Configuration::Client(conf) = conf {
                    *client = conf.clone();
                } else {
                    return Err(WifiError::InternalError(
                        InternalWifiError::EspErrInvalidArg,
                    ));
                }
            }
            Configuration::AccessPoint(ref mut ap) => {
                if let Configuration::AccessPoint(conf) = conf {
                    *ap = conf.clone();
                } else {
                    return Err(WifiError::InternalError(
                        InternalWifiError::EspErrInvalidArg,
                    ));
                }
            }
            Configuration::Mixed(ref mut client, ref mut ap) => match conf {
                Configuration::None => {
                    return Err(WifiError::InternalError(
                        InternalWifiError::EspErrInvalidArg,
                    ));
                }
                Configuration::Mixed(_, _) => self.config = conf.clone(),
                Configuration::Client(conf) => *client = conf.clone(),
                Configuration::AccessPoint(conf) => *ap = conf.clone(),
            },
        }

        match conf {
            Configuration::None => {
                return Err(WifiError::InternalError(
                    InternalWifiError::EspErrInvalidArg,
                ));
            }
            Configuration::Client(config) => apply_sta_config(config)?,
            Configuration::AccessPoint(config) => apply_ap_config(config)?,
            Configuration::Mixed(sta_config, ap_config) => {
                apply_ap_config(ap_config)?;
                apply_sta_config(sta_config)?;
            }
        };

        Ok(())
    }

    pub(crate) fn stop_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    pub(crate) fn connect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_connect() })
    }

    pub(crate) fn disconnect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_disconnect() })
    }

    pub fn is_started(&self) -> Result<bool, WifiError> {
        if matches!(
            crate::wifi::get_sta_state(),
            WifiState::StaStarted | WifiState::StaConnected | WifiState::StaDisconnected
        ) {
            return Ok(true);
        }
        if matches!(crate::wifi::get_ap_state(), WifiState::ApStarted) {
            return Ok(true);
        }
        Ok(false)
    }

    pub fn is_connected(&self) -> Result<bool, WifiError> {
        match crate::wifi::get_sta_state() {
            crate::wifi::WifiState::StaConnected => Ok(true),
            crate::wifi::WifiState::StaDisconnected => Err(WifiError::Disconnected),
            // FIXME: Should any other enum value trigger an error instead of returning false?
            _ => Ok(false),
        }
    }
}

#[cfg(not(feature = "async"))]
impl WifiController<'_> {
    /// A blocking wifi network scan with default scanning options.
    pub fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
        self.scan_with_config_sync(Default::default())
    }

    pub fn start(&mut self) -> Result<(), WifiError> {
        crate::wifi::wifi_start()
    }

    pub fn stop(&mut self) -> Result<(), WifiError> {
        self.stop_impl()
    }

    pub fn connect(&mut self) -> Result<(), WifiError> {
        self.connect_impl()
    }

    pub fn disconnect(&mut self) -> Result<(), WifiError> {
        self.disconnect_impl()
    }
}

fn dump_packet_info(_buffer: &[u8]) {
    #[cfg(feature = "dump-packets")]
    {
        info!("@WIFIFRAME {:?}", _buffer);
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! esp_wifi_result {
    ($value:expr) => {{
        let result = $value;
        if result != include::ESP_OK as i32 {
            warn!("{} returned an error: {}", stringify!($value), result);
            Err(WifiError::InternalError(unwrap!(FromPrimitive::from_i32(
                result
            ))))
        } else {
            Ok::<(), WifiError>(())
        }
    }};
}

#[cfg(feature = "embassy-net")]
pub(crate) mod embassy {
    use embassy_net_driver::{Capabilities, Driver, HardwareAddress, RxToken, TxToken};
    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    // We can get away with a single tx waker because the transmit queue is shared
    // between interfaces.
    pub(crate) static TRANSMIT_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static AP_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static AP_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static STA_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static STA_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    impl<MODE: WifiDeviceMode> RxToken for WifiRxToken<MODE> {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(f)
        }
    }

    impl<MODE: WifiDeviceMode> TxToken for WifiTxToken<MODE> {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(len, f)
        }
    }

    impl<MODE: WifiDeviceMode> Driver for WifiDevice<'_, MODE> {
        type RxToken<'a> = WifiRxToken<MODE> where Self: 'a;
        type TxToken<'a> = WifiTxToken<MODE> where Self: 'a;

        fn receive(
            &mut self,
            cx: &mut core::task::Context,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            self.mode.register_receive_waker(cx);
            self.mode.register_transmit_waker(cx);
            self.mode.rx_token()
        }

        fn transmit(&mut self, cx: &mut core::task::Context) -> Option<Self::TxToken<'_>> {
            self.mode.register_transmit_waker(cx);
            self.mode.tx_token()
        }

        fn link_state(&mut self, cx: &mut core::task::Context) -> embassy_net_driver::LinkState {
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

#[cfg(feature = "async")]
mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    // TODO assumes STA mode only
    impl<'d> WifiController<'d> {
        /// Async version of [`crate::wifi::WifiController`]'s `scan_n` method
        pub async fn scan_n<const N: usize>(
            &mut self,
        ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
            self.scan_with_config(Default::default()).await
        }

        pub async fn scan_with_config<const N: usize>(
            &mut self,
            config: ScanConfig<'_>,
        ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
            Self::clear_events(WifiEvent::ScanDone);
            esp_wifi_result!(wifi_start_scan(false, config))?;

            // Prevents memory leak if `scan_n`'s future is dropped.
            let guard = FreeApListOnDrop;
            WifiEventFuture::new(WifiEvent::ScanDone).await;

            guard.defuse();

            let count = self.scan_result_count()?;
            let result = self.scan_results()?;

            Ok((result, count))
        }

        /// Async version of [`crate::wifi::WifiController`]'s `start` method
        pub async fn start(&mut self) -> Result<(), WifiError> {
            let mode = WifiMode::try_from(&self.config)?;

            let mut events = enumset::enum_set! {};
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
        pub async fn stop(&mut self) -> Result<(), WifiError> {
            let mode = WifiMode::try_from(&self.config)?;

            let mut events = enumset::enum_set! {};
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
        pub async fn connect(&mut self) -> Result<(), WifiError> {
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
        pub async fn disconnect(&mut self) -> Result<(), WifiError> {
            Self::clear_events(WifiEvent::StaDisconnected);
            crate::wifi::WifiController::disconnect_impl(self)?;
            WifiEventFuture::new(WifiEvent::StaDisconnected).await;

            Ok(())
        }

        fn clear_events(events: impl Into<EnumSet<WifiEvent>>) {
            critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove_all(events.into()));
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
            match self {
                WifiEvent::ScanDone => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaStart => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaConnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaDisconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaStop => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::WifiReady => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaAuthmodeChange => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErSuccess => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErFailed => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErTimeout => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErPin => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErPbcOverlap => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStart => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStop => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStaconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStadisconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApProbereqrecved => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::FtmReport => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaBssRssiLow => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ActionTxStatus => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::RocDone => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaBeaconTimeout => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
            }
        }
    }

    pub(crate) struct WifiEventFuture {
        event: WifiEvent,
    }

    impl WifiEventFuture {
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
            if critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove(self.event)) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub(crate) struct MultiWifiEventFuture {
        event: EnumSet<WifiEvent>,
    }

    impl MultiWifiEventFuture {
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
            let output = critical_section::with(|cs| {
                let mut events = WIFI_EVENTS.borrow_ref_mut(cs);
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

#[cfg(feature = "embedded-svc")]
mod embedded_svc_compat {
    use super::*;

    impl From<Capability> for embedded_svc::wifi::Capability {
        fn from(s: Capability) -> embedded_svc::wifi::Capability {
            match s {
                Capability::Client => embedded_svc::wifi::Capability::Client,
                Capability::AccessPoint => embedded_svc::wifi::Capability::AccessPoint,
                Capability::Mixed => embedded_svc::wifi::Capability::Mixed,
            }
        }
    }

    impl From<AuthMethod> for embedded_svc::wifi::AuthMethod {
        fn from(s: AuthMethod) -> embedded_svc::wifi::AuthMethod {
            match s {
                AuthMethod::None => embedded_svc::wifi::AuthMethod::None,
                AuthMethod::WEP => embedded_svc::wifi::AuthMethod::WEP,
                AuthMethod::WPA => embedded_svc::wifi::AuthMethod::WPA,
                AuthMethod::WPA2Personal => embedded_svc::wifi::AuthMethod::WPA2Personal,
                AuthMethod::WPAWPA2Personal => embedded_svc::wifi::AuthMethod::WPAWPA2Personal,
                AuthMethod::WPA2Enterprise => embedded_svc::wifi::AuthMethod::WPA2Enterprise,
                AuthMethod::WPA3Personal => embedded_svc::wifi::AuthMethod::WPA3Personal,
                AuthMethod::WPA2WPA3Personal => embedded_svc::wifi::AuthMethod::WPA2WPA3Personal,
                AuthMethod::WAPIPersonal => embedded_svc::wifi::AuthMethod::WAPIPersonal,
            }
        }
    }

    impl From<embedded_svc::wifi::AuthMethod> for AuthMethod {
        fn from(value: embedded_svc::wifi::AuthMethod) -> Self {
            match value {
                embedded_svc::wifi::AuthMethod::None => AuthMethod::None,
                embedded_svc::wifi::AuthMethod::WEP => AuthMethod::WEP,
                embedded_svc::wifi::AuthMethod::WPA => AuthMethod::WPA,
                embedded_svc::wifi::AuthMethod::WPA2Personal => AuthMethod::WPA2Personal,
                embedded_svc::wifi::AuthMethod::WPAWPA2Personal => AuthMethod::WPAWPA2Personal,
                embedded_svc::wifi::AuthMethod::WPA2Enterprise => AuthMethod::WPA2Enterprise,
                embedded_svc::wifi::AuthMethod::WPA3Personal => AuthMethod::WPA3Personal,
                embedded_svc::wifi::AuthMethod::WPA2WPA3Personal => AuthMethod::WPA2WPA3Personal,
                embedded_svc::wifi::AuthMethod::WAPIPersonal => AuthMethod::WAPIPersonal,
            }
        }
    }

    impl From<Protocol> for embedded_svc::wifi::Protocol {
        fn from(s: Protocol) -> embedded_svc::wifi::Protocol {
            match s {
                Protocol::P802D11B => embedded_svc::wifi::Protocol::P802D11B,
                Protocol::P802D11BG => embedded_svc::wifi::Protocol::P802D11BG,
                Protocol::P802D11BGN => embedded_svc::wifi::Protocol::P802D11BGN,
                Protocol::P802D11BGNLR => embedded_svc::wifi::Protocol::P802D11BGNLR,
                Protocol::P802D11LR => embedded_svc::wifi::Protocol::P802D11LR,
                // not in embedded-svc currently
                Protocol::P802D11BGNAX => embedded_svc::wifi::Protocol::P802D11BGN,
            }
        }
    }

    impl From<embedded_svc::wifi::Protocol> for Protocol {
        fn from(value: embedded_svc::wifi::Protocol) -> Self {
            match value {
                embedded_svc::wifi::Protocol::P802D11B => Protocol::P802D11B,
                embedded_svc::wifi::Protocol::P802D11BG => Protocol::P802D11BG,
                embedded_svc::wifi::Protocol::P802D11BGN => Protocol::P802D11BGN,
                embedded_svc::wifi::Protocol::P802D11BGNLR => Protocol::P802D11BGNLR,
                embedded_svc::wifi::Protocol::P802D11LR => Protocol::P802D11LR,
            }
        }
    }

    impl From<Configuration> for embedded_svc::wifi::Configuration {
        fn from(s: Configuration) -> embedded_svc::wifi::Configuration {
            match s {
                Configuration::None => embedded_svc::wifi::Configuration::None,
                Configuration::Client(conf) => embedded_svc::wifi::Configuration::Client(
                    embedded_svc::wifi::ClientConfiguration {
                        ssid: conf.ssid,
                        bssid: conf.bssid,
                        auth_method: conf.auth_method.into(),
                        password: conf.password,
                        channel: conf.channel,
                    },
                ),
                Configuration::AccessPoint(conf) => embedded_svc::wifi::Configuration::AccessPoint(
                    embedded_svc::wifi::AccessPointConfiguration {
                        ssid: conf.ssid,
                        ssid_hidden: conf.ssid_hidden,
                        channel: conf.channel,
                        secondary_channel: conf.secondary_channel,
                        protocols: {
                            let mut res = EnumSet::<embedded_svc::wifi::Protocol>::new();
                            conf.protocols.into_iter().for_each(|v| {
                                res.insert(v.into());
                            });
                            res
                        },
                        auth_method: conf.auth_method.into(),
                        password: conf.password,
                        max_connections: conf.max_connections,
                    },
                ),
                Configuration::Mixed(client, ap) => embedded_svc::wifi::Configuration::Mixed(
                    embedded_svc::wifi::ClientConfiguration {
                        ssid: client.ssid,
                        bssid: client.bssid,
                        auth_method: client.auth_method.into(),
                        password: client.password,
                        channel: client.channel,
                    },
                    embedded_svc::wifi::AccessPointConfiguration {
                        ssid: ap.ssid,
                        ssid_hidden: ap.ssid_hidden,
                        channel: ap.channel,
                        secondary_channel: ap.secondary_channel,
                        protocols: {
                            let mut res = EnumSet::<embedded_svc::wifi::Protocol>::new();
                            ap.protocols.into_iter().for_each(|v| {
                                res.insert(v.into());
                            });
                            res
                        },

                        auth_method: ap.auth_method.into(),
                        password: ap.password,
                        max_connections: ap.max_connections,
                    },
                ),
            }
        }
    }

    impl From<&embedded_svc::wifi::Configuration> for Configuration {
        fn from(value: &embedded_svc::wifi::Configuration) -> Self {
            match value {
                embedded_svc::wifi::Configuration::None => Configuration::None,
                embedded_svc::wifi::Configuration::Client(conf) => {
                    Configuration::Client(ClientConfiguration {
                        ssid: conf.ssid.clone(),
                        bssid: conf.bssid,
                        auth_method: conf.auth_method.into(),
                        password: conf.password.clone(),
                        channel: conf.channel,
                    })
                }
                embedded_svc::wifi::Configuration::AccessPoint(conf) => {
                    Configuration::AccessPoint(AccessPointConfiguration {
                        ssid: conf.ssid.clone(),
                        ssid_hidden: conf.ssid_hidden,
                        channel: conf.channel,
                        secondary_channel: conf.secondary_channel,
                        protocols: {
                            let mut res = EnumSet::<Protocol>::new();
                            conf.protocols.into_iter().for_each(|v| {
                                res.insert(v.into());
                            });
                            res
                        },
                        auth_method: conf.auth_method.into(),
                        password: conf.password.clone(),
                        max_connections: conf.max_connections,
                    })
                }
                embedded_svc::wifi::Configuration::Mixed(client, ap) => Configuration::Mixed(
                    ClientConfiguration {
                        ssid: client.ssid.clone(),
                        bssid: client.bssid,
                        auth_method: client.auth_method.into(),
                        password: client.password.clone(),
                        channel: client.channel,
                    },
                    AccessPointConfiguration {
                        ssid: ap.ssid.clone(),
                        ssid_hidden: ap.ssid_hidden,
                        channel: ap.channel,
                        secondary_channel: ap.secondary_channel,
                        protocols: {
                            let mut res = EnumSet::<Protocol>::new();
                            ap.protocols.into_iter().for_each(|v| {
                                res.insert(v.into());
                            });
                            res
                        },
                        auth_method: ap.auth_method.into(),
                        password: ap.password.clone(),
                        max_connections: ap.max_connections,
                    },
                ),
            }
        }
    }

    impl From<AccessPointInfo> for embedded_svc::wifi::AccessPointInfo {
        fn from(s: AccessPointInfo) -> embedded_svc::wifi::AccessPointInfo {
            embedded_svc::wifi::AccessPointInfo {
                ssid: s.ssid.clone(),
                bssid: s.bssid,
                channel: s.channel,
                secondary_channel: s.secondary_channel.into(),
                signal_strength: s.signal_strength,
                protocols: {
                    let mut res = EnumSet::<embedded_svc::wifi::Protocol>::new();
                    s.protocols.into_iter().for_each(|v| {
                        res.insert(v.into());
                    });
                    res
                },
                auth_method: s.auth_method.map(|v| v.into()),
            }
        }
    }

    impl From<SecondaryChannel> for embedded_svc::wifi::SecondaryChannel {
        fn from(s: SecondaryChannel) -> embedded_svc::wifi::SecondaryChannel {
            match s {
                SecondaryChannel::None => embedded_svc::wifi::SecondaryChannel::None,
                SecondaryChannel::Above => embedded_svc::wifi::SecondaryChannel::Above,
                SecondaryChannel::Below => embedded_svc::wifi::SecondaryChannel::Below,
            }
        }
    }

    impl From<crate::wifi::ipv4::Subnet> for embedded_svc::ipv4::Subnet {
        fn from(s: crate::wifi::ipv4::Subnet) -> embedded_svc::ipv4::Subnet {
            embedded_svc::ipv4::Subnet {
                gateway: embedded_svc::ipv4::Ipv4Addr::from(s.gateway.octets()),
                mask: embedded_svc::ipv4::Mask(s.mask.0),
            }
        }
    }

    impl From<embedded_svc::ipv4::Subnet> for crate::wifi::ipv4::Subnet {
        fn from(value: embedded_svc::ipv4::Subnet) -> Self {
            Self {
                gateway: super::ipv4::Ipv4Addr::from(value.gateway.octets()),
                mask: super::ipv4::Mask(value.mask.0),
            }
        }
    }

    impl From<super::ipv4::IpInfo> for embedded_svc::ipv4::IpInfo {
        fn from(s: super::ipv4::IpInfo) -> embedded_svc::ipv4::IpInfo {
            embedded_svc::ipv4::IpInfo {
                ip: embedded_svc::ipv4::Ipv4Addr::from(s.ip.octets()),
                subnet: s.subnet.into(),
                dns: s
                    .dns
                    .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
                secondary_dns: s
                    .secondary_dns
                    .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
            }
        }
    }

    impl From<&embedded_svc::ipv4::Configuration> for super::ipv4::Configuration {
        fn from(value: &embedded_svc::ipv4::Configuration) -> Self {
            match value {
                embedded_svc::ipv4::Configuration::Client(client) => {
                    let config = match client {
                        embedded_svc::ipv4::ClientConfiguration::DHCP(dhcp) => {
                            super::ipv4::ClientConfiguration::DHCP(
                                super::ipv4::DHCPClientSettings {
                                    hostname: dhcp.hostname.clone(),
                                },
                            )
                        }
                        embedded_svc::ipv4::ClientConfiguration::Fixed(fixed) => {
                            super::ipv4::ClientConfiguration::Fixed(super::ipv4::ClientSettings {
                                ip: super::ipv4::Ipv4Addr::from(fixed.ip.octets()),
                                subnet: fixed.subnet.into(),
                                dns: fixed.dns.map(|v| super::ipv4::Ipv4Addr::from(v.octets())),
                                secondary_dns: fixed
                                    .secondary_dns
                                    .map(|v| super::ipv4::Ipv4Addr::from(v.octets())),
                            })
                        }
                    };
                    super::ipv4::Configuration::Client(config)
                }
                embedded_svc::ipv4::Configuration::Router(router) => {
                    let config = super::ipv4::RouterConfiguration {
                        subnet: router.subnet.into(),
                        dhcp_enabled: router.dhcp_enabled,
                        dns: router.dns.map(|v| super::ipv4::Ipv4Addr::from(v.octets())),
                        secondary_dns: router
                            .secondary_dns
                            .map(|v| super::ipv4::Ipv4Addr::from(v.octets())),
                    };
                    super::ipv4::Configuration::Router(config)
                }
            }
        }
    }

    impl From<super::ipv4::Configuration> for embedded_svc::ipv4::Configuration {
        fn from(s: super::ipv4::Configuration) -> embedded_svc::ipv4::Configuration {
            match s {
                super::ipv4::Configuration::Client(client) => {
                    let config = match client {
                        super::ipv4::ClientConfiguration::DHCP(dhcp) => {
                            embedded_svc::ipv4::ClientConfiguration::DHCP(
                                embedded_svc::ipv4::DHCPClientSettings {
                                    hostname: dhcp.hostname.clone(),
                                },
                            )
                        }
                        super::ipv4::ClientConfiguration::Fixed(fixed) => {
                            embedded_svc::ipv4::ClientConfiguration::Fixed(
                                embedded_svc::ipv4::ClientSettings {
                                    ip: embedded_svc::ipv4::Ipv4Addr::from(fixed.ip.octets()),
                                    subnet: fixed.subnet.into(),
                                    dns: fixed
                                        .dns
                                        .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
                                    secondary_dns: fixed
                                        .secondary_dns
                                        .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
                                },
                            )
                        }
                    };
                    embedded_svc::ipv4::Configuration::Client(config)
                }
                super::ipv4::Configuration::Router(router) => {
                    let config = embedded_svc::ipv4::RouterConfiguration {
                        subnet: router.subnet.into(),
                        dhcp_enabled: router.dhcp_enabled,
                        dns: router
                            .dns
                            .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
                        secondary_dns: router
                            .secondary_dns
                            .map(|v| embedded_svc::ipv4::Ipv4Addr::from(v.octets())),
                    };
                    embedded_svc::ipv4::Configuration::Router(config)
                }
            }
        }
    }

    #[cfg(not(feature = "async"))]
    impl embedded_svc::wifi::Wifi for WifiController<'_> {
        type Error = WifiError;

        fn get_capabilities(&self) -> Result<EnumSet<embedded_svc::wifi::Capability>, Self::Error> {
            self.get_capabilities().map(|v| {
                let mut res = EnumSet::<embedded_svc::wifi::Capability>::new();
                v.into_iter().for_each(|v| {
                    res.insert(v.into());
                });
                res
            })
        }

        fn get_configuration(&self) -> Result<embedded_svc::wifi::Configuration, Self::Error> {
            self.get_configuration().map(|v| v.into())
        }

        fn set_configuration(
            &mut self,
            conf: &embedded_svc::wifi::Configuration,
        ) -> Result<(), Self::Error> {
            let conf = conf.into();
            self.set_configuration(&conf)
        }

        fn start(&mut self) -> Result<(), Self::Error> {
            self.start()
        }

        fn stop(&mut self) -> Result<(), Self::Error> {
            self.stop()
        }

        fn connect(&mut self) -> Result<(), Self::Error> {
            self.connect()
        }

        fn disconnect(&mut self) -> Result<(), Self::Error> {
            self.disconnect()
        }

        fn is_started(&self) -> Result<bool, Self::Error> {
            self.is_started()
        }

        fn is_connected(&self) -> Result<bool, Self::Error> {
            self.is_connected()
        }

        fn scan_n<const N: usize>(
            &mut self,
        ) -> Result<(heapless::Vec<embedded_svc::wifi::AccessPointInfo, N>, usize), Self::Error>
        {
            self.scan_n::<N>().map(|(v, l)| {
                let mut res: heapless::Vec<embedded_svc::wifi::AccessPointInfo, N> =
                    heapless::Vec::new();
                for ap in v {
                    res.push(ap.into()).ok();
                }
                (res, l)
            })
        }
    }

    #[cfg(feature = "async")]
    impl embedded_svc::wifi::asynch::Wifi for WifiController<'_> {
        type Error = WifiError;

        async fn get_capabilities(
            &self,
        ) -> Result<EnumSet<embedded_svc::wifi::Capability>, Self::Error> {
            self.get_capabilities().map(|v| {
                let mut res = EnumSet::<embedded_svc::wifi::Capability>::new();
                v.into_iter().for_each(|v| {
                    res.insert(v.into());
                });
                res
            })
        }

        async fn get_configuration(
            &self,
        ) -> Result<embedded_svc::wifi::Configuration, Self::Error> {
            WifiController::get_configuration(self).map(|v| v.into())
        }

        async fn set_configuration(
            &mut self,
            conf: &embedded_svc::wifi::Configuration,
        ) -> Result<(), Self::Error> {
            let conf = conf.into();
            self.set_configuration(&conf)
        }

        async fn start(&mut self) -> Result<(), Self::Error> {
            self.start().await
        }

        async fn stop(&mut self) -> Result<(), Self::Error> {
            self.stop().await
        }

        async fn connect(&mut self) -> Result<(), Self::Error> {
            self.connect().await
        }

        async fn disconnect(&mut self) -> Result<(), Self::Error> {
            self.disconnect().await
        }

        async fn is_started(&self) -> Result<bool, Self::Error> {
            self.is_started()
        }

        async fn is_connected(&self) -> Result<bool, Self::Error> {
            self.is_connected()
        }

        async fn scan_n<const N: usize>(
            &mut self,
        ) -> Result<(heapless::Vec<embedded_svc::wifi::AccessPointInfo, N>, usize), Self::Error>
        {
            self.scan_n::<N>().await.map(|(v, l)| {
                let mut res: heapless::Vec<embedded_svc::wifi::AccessPointInfo, N> =
                    heapless::Vec::new();
                for ap in v {
                    res.push(ap.into()).ok();
                }
                (res, l)
            })
        }
    }

    impl<'a, MODE: WifiDeviceMode> embedded_svc::ipv4::Interface
        for crate::wifi_interface::WifiStack<'a, MODE>
    {
        type Error = crate::wifi_interface::WifiStackError;

        fn get_iface_configuration(
            &self,
        ) -> Result<embedded_svc::ipv4::Configuration, Self::Error> {
            self.get_iface_configuration().map(|v| v.into())
        }

        fn set_iface_configuration(
            &mut self,
            conf: &embedded_svc::ipv4::Configuration,
        ) -> Result<(), Self::Error> {
            self.set_iface_configuration(&super::ipv4::Configuration::from(conf))
        }

        fn is_iface_up(&self) -> bool {
            self.is_iface_up()
        }

        fn get_ip_info(&self) -> Result<embedded_svc::ipv4::IpInfo, Self::Error> {
            self.get_ip_info().map(|v| v.into())
        }
    }
}
