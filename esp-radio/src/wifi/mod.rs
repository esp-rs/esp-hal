//! Wi-Fi

#![deny(missing_docs)]

use alloc::{collections::vec_deque::VecDeque, vec::Vec};
use core::{fmt::Debug, marker::PhantomData, mem::MaybeUninit, ptr::addr_of, task::Poll};

use enumset::{EnumSet, EnumSetType};
use esp_config::esp_config_int;
use esp_hal::{asynch::AtomicWaker, system::Cpu};
use esp_sync::NonReentrantMutex;
use num_derive::FromPrimitive;
use portable_atomic::{AtomicUsize, Ordering};
use procmacros::BuilderLite;
#[cfg(all(feature = "smoltcp", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};

pub(crate) use self::os_adapter::*;
#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use self::sniffer::Sniffer;
#[cfg(feature = "wifi-eap")]
use self::sta::eap::EapStationConfig;
pub use self::state::*;
use self::{
    ap::{AccessPointConfig, AccessPointInfo, convert_ap_info},
    private::PacketBuffer,
    scan::{FreeApListOnDrop, ScanConfig, ScanResults, ScanTypeConfig},
    sta::StationConfig,
};
use crate::{
    InitializationError,
    RadioRefGuard,
    common_adapter::*,
    esp_wifi_result,
    hal::ram,
    sys::{
        c_types,
        include::{self, *},
    },
};

pub mod ap;
#[cfg(all(feature = "csi", feature = "unstable"))]
pub mod csi;
pub mod event;
pub mod scan;
#[cfg(all(feature = "sniffer", feature = "unstable"))]
pub mod sniffer;
pub mod sta;

pub(crate) mod os_adapter;
pub(crate) mod state;

mod internal;

const MTU: usize = esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MTU");

/// Supported Wi-Fi authentication methods.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// Opportunistic Wireless Encryption (OWE)
    Owe,

    /// WPA3 Enterprise Suite B 192-bit Encryption
    Wpa3EntSuiteB192Bit,

    /// This authentication mode will yield same result as [AuthMethod::Wpa3Personal] and is not
    /// recommended to be used. It will be deprecated in future, please use
    /// [AuthMethod::Wpa3Personal] instead.
    Wpa3ExtPsk,

    /// This authentication mode will yield same result as [AuthMethod::Wpa3Personal] and is not
    /// recommended to be used. It will be deprecated in future, please use
    /// [AuthMethod::Wpa3Personal] instead.
    Wpa3ExtPskMixed,

    /// WiFi DPP / Wi-Fi Easy Connect
    Dpp,

    /// WPA3-Enterprise Only Mode
    Wpa3Enterprise,

    /// WPA3-Enterprise Transition Mode
    Wpa2Wpa3Enterprise,

    /// WPA-Enterprise security
    WpaEnterprise,
}

/// Supported Wi-Fi protocols.
#[derive(Debug, Default, PartialOrd, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// Introduces Wi-Fi configuration options.
#[derive(EnumSetType, Debug, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Capability {
    /// The device operates as a station, connecting to an existing network.
    Station,

    /// The device operates as an access point, allowing other devices to
    /// connect to it.
    AccessPoint,

    /// The device can operate in both station and access point modes
    /// simultaneously.
    AccessPointStation,
}

/// Configuration of Wi-Fi operation mode.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ModeConfig {
    /// No configuration (default).
    #[default]
    None,

    /// Station configuration.
    Station(StationConfig),

    /// Access point configuration.
    AccessPoint(AccessPointConfig),

    /// Simultaneous station and access point configuration.
    AccessPointStation(StationConfig, AccessPointConfig),

    /// EAP station configuration for enterprise Wi-Fi.
    #[cfg(feature = "wifi-eap")]
    EapStation(EapStationConfig),
}

impl ModeConfig {
    fn validate(&self) -> Result<(), WifiError> {
        match self {
            ModeConfig::None => Ok(()),
            ModeConfig::Station(station_configuration) => station_configuration.validate(),
            ModeConfig::AccessPoint(access_point_configuration) => {
                access_point_configuration.validate()
            }
            ModeConfig::AccessPointStation(station_configuration, access_point_configuration) => {
                station_configuration.validate()?;
                access_point_configuration.validate()
            }
            #[cfg(feature = "wifi-eap")]
            ModeConfig::EapStation(eap_station_configuration) => {
                eap_station_configuration.validate()
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
            AuthMethod::Wep => include::wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::Wpa => include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::Wpa2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WpaWpa2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
            AuthMethod::Wpa2Enterprise => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE,
            AuthMethod::Wpa3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthMethod::Wpa2Wpa3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK,
            AuthMethod::WapiPersonal => include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
            AuthMethod::Owe => include::wifi_auth_mode_t_WIFI_AUTH_OWE,
            AuthMethod::Wpa3EntSuiteB192Bit => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENT_192,
            AuthMethod::Wpa3ExtPsk => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK,
            AuthMethod::Wpa3ExtPskMixed => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE
            }
            AuthMethod::Dpp => include::wifi_auth_mode_t_WIFI_AUTH_DPP,
            AuthMethod::Wpa3Enterprise => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENTERPRISE,
            AuthMethod::Wpa2Wpa3Enterprise => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_ENTERPRISE
            }
            AuthMethod::WpaEnterprise => include::wifi_auth_mode_t_WIFI_AUTH_WPA_ENTERPRISE,
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
            include::wifi_auth_mode_t_WIFI_AUTH_OWE => AuthMethod::Owe,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENT_192 => AuthMethod::Wpa3EntSuiteB192Bit,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK => AuthMethod::Wpa3ExtPsk,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE => {
                AuthMethod::Wpa3ExtPskMixed
            }
            include::wifi_auth_mode_t_WIFI_AUTH_DPP => AuthMethod::Dpp,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENTERPRISE => AuthMethod::Wpa3Enterprise,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_ENTERPRISE => {
                AuthMethod::Wpa2Wpa3Enterprise
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_ENTERPRISE => AuthMethod::WpaEnterprise,
            // we const-assert we know all the auth-methods the wifi driver knows and it shouldn't
            // return anything else.
            //
            // In fact from observation the drivers will return
            // `wifi_auth_mode_t_WIFI_AUTH_OPEN` if the method is unsupported (e.g. any WPA3 in our
            // case, since the supplicant isn't compiled to support it)
            _ => AuthMethod::None,
        }
    }
}

/// Wi-Fi Mode (Station and/or AccessPoint).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WifiMode {
    /// Station mode.
    Station,
    /// Access Point mode.
    AccessPoint,
    /// Both Access Point and Station modes.
    AccessPointStation,
}

impl WifiMode {
    pub(crate) fn current() -> Result<Self, WifiError> {
        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut mode) })?;

        Self::try_from(mode)
    }

    /// Returns true if this mode works as a station.
    pub fn is_station(&self) -> bool {
        match self {
            Self::Station | Self::AccessPointStation => true,
            Self::AccessPoint => false,
        }
    }

    /// Returns true if this mode works as an access point
    pub fn is_access_point(&self) -> bool {
        match self {
            Self::Station => false,
            Self::AccessPoint | Self::AccessPointStation => true,
        }
    }
}

impl TryFrom<&ModeConfig> for WifiMode {
    type Error = WifiError;

    /// Based on the current `ModeConfig`, derives a `WifiMode` based on it.
    fn try_from(config: &ModeConfig) -> Result<Self, Self::Error> {
        let mode = match config {
            ModeConfig::None => return Err(WifiError::UnknownWifiMode),
            ModeConfig::AccessPoint(_) => Self::AccessPoint,
            ModeConfig::Station(_) => Self::Station,
            ModeConfig::AccessPointStation(_, _) => Self::AccessPointStation,
            #[cfg(feature = "wifi-eap")]
            ModeConfig::EapStation(_) => Self::Station,
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
            include::wifi_mode_t_WIFI_MODE_STA => Ok(Self::Station),
            include::wifi_mode_t_WIFI_MODE_AP => Ok(Self::AccessPoint),
            include::wifi_mode_t_WIFI_MODE_APSTA => Ok(Self::AccessPointStation),
            _ => Err(WifiError::UnknownWifiMode),
        }
    }
}

#[doc(hidden)]
impl From<WifiMode> for wifi_mode_t {
    fn from(val: WifiMode) -> Self {
        #[allow(non_upper_case_globals)]
        match val {
            WifiMode::Station => wifi_mode_t_WIFI_MODE_STA,
            WifiMode::AccessPoint => wifi_mode_t_WIFI_MODE_AP,
            WifiMode::AccessPointStation => wifi_mode_t_WIFI_MODE_APSTA,
        }
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
    /// The device disconnected from the network or failed to connect to it.
    Disconnected,

    /// Unknown Wi-Fi mode (not Station/AccessPoint/AccessPointStation).
    UnknownWifiMode,

    /// Unsupported operation or mode.
    Unsupported,

    /// Passed arguments are invalid.
    InvalidArguments,

    /// Generic failure - not further specified
    Failed,

    /// Out of memory
    OutOfMemory,

    /// Wi-Fi driver was not initialized
    NotInitialized,

    /// Wi-Fi driver was not started by [esp_wifi_start]
    NotStarted,

    /// Wi-Fi driver was not stopped by [esp_wifi_stop]
    NotStopped,

    /// Wi-Fi interface error
    Interface,

    /// Wi-Fi mode error
    Mode,

    /// Wi-Fi internal state error
    State,

    /// Wi-Fi internal control block of station or soft-AccessPoint error
    ControlBlock,

    /// Wi-Fi internal NVS module error
    Nvs,

    /// MAC address is invalid
    InvalidMac,

    /// SSID is invalid
    InvalidSsid,

    /// Password is invalid
    InvalidPassword,

    /// Timeout error
    Timeout,

    /// Wi-Fi is in sleep state (RF closed) and wakeup failed
    WakeFailed,

    /// The operation would block
    WouldBlock,

    /// Station still in disconnect status
    NotConnected,

    /// Failed to post the event to Wi-Fi task
    PostFail,

    /// Invalid Wi-Fi state when init/deinit is called
    InvalidInitState,

    /// Returned when Wi-Fi is stopping
    StopState,

    /// The Wi-Fi connection is not associated
    NotAssociated,

    /// The Wi-Fi TX is disallowed
    TxDisallowed,

    /// An unknown error was reported by the Wi-Fi driver.
    // This is here just in case we encounter an unmapped error - there doesn't seem to be a
    // definitive and exhausting list of errors we should expect and panicking because of an
    // unmapped error.
    Unknown(i32),

    /// The current CPU clock frequency is too low.
    WrongClockConfig,

    /// The scheduler is not initialized.
    SchedulerNotInitialized,

    #[cfg(esp32)]
    /// ADC2 is required by esp-radio, but it is in use by esp-hal.
    Adc2IsUsed,
}

impl WifiError {
    fn from_error_code(code: i32) -> Self {
        if crate::sys::include::ESP_FAIL == code {
            return WifiError::Failed;
        }

        match code as u32 {
            crate::sys::include::ESP_ERR_NO_MEM => WifiError::OutOfMemory,
            crate::sys::include::ESP_ERR_INVALID_ARG => WifiError::InvalidArguments,
            crate::sys::include::ESP_ERR_WIFI_NOT_INIT => WifiError::NotInitialized,
            crate::sys::include::ESP_ERR_WIFI_NOT_STARTED => WifiError::NotStarted,
            crate::sys::include::ESP_ERR_WIFI_NOT_STOPPED => WifiError::NotStopped,
            crate::sys::include::ESP_ERR_WIFI_IF => WifiError::Interface,
            crate::sys::include::ESP_ERR_WIFI_MODE => WifiError::Mode,
            crate::sys::include::ESP_ERR_WIFI_STATE => WifiError::State,
            crate::sys::include::ESP_ERR_WIFI_CONN => WifiError::ControlBlock,
            crate::sys::include::ESP_ERR_WIFI_NVS => WifiError::Nvs,
            crate::sys::include::ESP_ERR_WIFI_MAC => WifiError::InvalidMac,
            crate::sys::include::ESP_ERR_WIFI_SSID => WifiError::InvalidSsid,
            crate::sys::include::ESP_ERR_WIFI_PASSWORD => WifiError::InvalidPassword,
            crate::sys::include::ESP_ERR_WIFI_TIMEOUT => WifiError::Timeout,
            crate::sys::include::ESP_ERR_WIFI_WAKE_FAIL => WifiError::WakeFailed,
            crate::sys::include::ESP_ERR_WIFI_WOULD_BLOCK => WifiError::WouldBlock,
            crate::sys::include::ESP_ERR_WIFI_NOT_CONNECT => WifiError::NotConnected,
            crate::sys::include::ESP_ERR_WIFI_POST => WifiError::PostFail,
            crate::sys::include::ESP_ERR_WIFI_INIT_STATE => WifiError::InvalidInitState,
            crate::sys::include::ESP_ERR_WIFI_STOP_STATE => WifiError::StopState,
            crate::sys::include::ESP_ERR_WIFI_NOT_ASSOC => WifiError::NotAssociated,
            crate::sys::include::ESP_ERR_WIFI_TX_DISALLOW => WifiError::TxDisallowed,
            _ => WifiError::Unknown(code),
        }
    }
}

/// Events generated by the Wi-Fi driver.
impl core::fmt::Display for WifiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WifiError::Disconnected => write!(f, "Wi-Fi disconnected."),
            WifiError::UnknownWifiMode => write!(f, "Unknown Wi-Fi mode."),
            WifiError::Unsupported => write!(f, "Unsupported operation or mode."),
            WifiError::InvalidArguments => write!(f, "Invalid arguments."),
            WifiError::Failed => write!(f, "Generic unspecified failure."),
            WifiError::NotInitialized => write!(
                f,
                "Wi-Fi module is not initialized or not initialized for `Wi-Fi` operations."
            ),
            WifiError::OutOfMemory => write!(f, "Out of memory."),
            WifiError::NotStarted => write!(f, "Wi-Fi driver was not started."),
            WifiError::NotStopped => write!(f, "Wi-Fi driver was not stopped."),
            WifiError::Interface => write!(f, "Wi-Fi interface error."),
            WifiError::Mode => write!(f, "Wi-Fi mode error."),
            WifiError::State => write!(f, "Wi-Fi internal state error."),
            WifiError::ControlBlock => write!(
                f,
                "Wi-Fi internal control block of station or soft-AccessPoint error."
            ),
            WifiError::Nvs => write!(f, "Wi-Fi internal NVS module error."),
            WifiError::InvalidMac => write!(f, "MAC address is invalid."),
            WifiError::InvalidSsid => write!(f, "SSID is invalid."),
            WifiError::InvalidPassword => write!(f, "Password is invalid."),
            WifiError::Timeout => write!(f, "Timeout error."),
            WifiError::WakeFailed => {
                write!(f, "Wi-Fi is in sleep state (RF closed) and wakeup failed.")
            }
            WifiError::WouldBlock => write!(f, "The operation would block."),
            WifiError::NotConnected => write!(f, "Station still in disconnect status."),
            WifiError::PostFail => write!(f, "Failed to post the event to Wi-Fi task."),
            WifiError::InvalidInitState => {
                write!(f, "Invalid Wi-Fi state when init/deinit is called.")
            }
            WifiError::StopState => write!(f, "Returned when Wi-Fi is stopping."),
            WifiError::NotAssociated => write!(f, "The Wi-Fi connection is not associated."),
            WifiError::TxDisallowed => write!(f, "The Wi-Fi TX is disallowed."),
            WifiError::Unknown(_) => {
                write!(f, "An unknown error was reported by the Wi-Fi driver.")
            }
            WifiError::WrongClockConfig => {
                write!(f, "The current CPU clock frequency is too low")
            }
            WifiError::SchedulerNotInitialized => {
                write!(f, "The scheduler is not initialized")
            }
            #[cfg(esp32)]
            WifiError::Adc2IsUsed => write!(
                f,
                "ADC2 cannot be used with `radio` functionality on `esp32`"
            ),
        }
    }
}

impl core::error::Error for WifiError {}

impl From<InitializationError> for WifiError {
    fn from(err: InitializationError) -> Self {
        match err {
            InitializationError::WifiError(e) => e,
            InitializationError::SchedulerNotInitialized => WifiError::SchedulerNotInitialized,
            InitializationError::WrongClockConfig => WifiError::WrongClockConfig,
            #[cfg(esp32)]
            InitializationError::Adc2IsUsed => WifiError::Adc2IsUsed,
        }
    }
}

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
    StationStart,
    /// Station mode stopped.
    StationStop,
    /// Station connected to a network.
    StationConnected,
    /// Station disconnected from a network.
    StationDisconnected,
    /// Station authentication mode changed.
    StationAuthenticationModeChange,

    /// Station WiFi-Protected-Status succeeds in enrollee mode.
    StationWifiProtectedStatusEnrolleeSuccess,
    /// Station WiFi-Protected-Status fails in enrollee mode.
    StationWifiProtectedStatusEnrolleeFailed,
    /// Station WiFi-Protected-Status timeout in enrollee mode.
    StationWifiProtectedStatusEnrolleeTimeout,
    /// Station WiFi-Protected-Status pin code in enrollee mode.
    StationWifiProtectedStatusEnrolleePin,
    /// Station WiFi-Protected-Status overlap in enrollee mode.
    StationWifiProtectedStatusEnrolleePushButtonConfigurationOverlap,

    /// Soft-AccessPoint start.
    AccessPointStart,
    /// Soft-AccessPoint stop.
    AccessPointStop,
    /// A station connected to Soft-AccessPoint.
    AccessPointStationConnected,
    /// A station disconnected from Soft-AccessPoint.
    AccessPointStationDisconnected,
    /// Received probe request packet in Soft-AccessPoint interface.
    AccessPointProbeRequestReceived,

    /// Received report of Fine-Timing-Measurement procedure.
    FineTimingMeasurementReport,

    /// Station Receive-Signal-Strenght-Indicator goes below the configured threshold.
    StationBasicServiceSetReceivedSignalStrengthIndicatorLow,
    /// Status indication of Action Transmission operation.
    ActionTransmissionStatus,
    /// Remain-on-Channel operation complete.
    RemainOnChannelDone,

    /// Station beacon timeout.
    StationBeaconTimeout,

    /// Connectionless module wake interval has started.
    ConnectionlessModuleWakeIntervalStart,

    /// Soft-AccessPoint WiFi-Protected-Status succeeded in registrar mode.
    AccessPointWifiProtectedStatusRegistrarSuccess,
    /// Soft-AccessPoint WiFi-Protected-Status failed in registrar mode.
    AccessPointWifiProtectedStatusRegistrarFailed,
    /// Soft-AccessPoint WiFi-Protected-Status timed out in registrar mode.
    AccessPointWifiProtectedStatusRegistrarTimeout,
    /// Soft-AccessPoint WiFi-Protected-Status pin code in registrar mode.
    AccessPointWifiProtectedStatusRegistrarPin,
    /// Soft-AccessPoint WiFi-Protected-Status overlap in registrar mode.
    AccessPointWifiProtectedStatusRegistrarPushButtonConfigurationOverlap,

    /// Individual Target-Wake-Time setup.
    IndividualTargetWakeTimeSetup,
    /// Individual Target-Wake-Time teardown.
    IndividualTargetWakeTimeTeardown,
    /// Individual Target-Wake-Time probe.
    IndividualTargetWakeTimeProbe,
    /// Individual Target-Wake-Time suspended.
    IndividualTargetWakeTimeSuspend,
    /// Target-Wake-Wakeup event.
    TargetWakeTimeWakeup,
    /// Broadcast-Target-Wake-Time setup.
    BroadcastTargetWakeTimeSetup,
    /// Broadcast-Target-Wake-Time teardown.
    BroadcastTargetWakeTimeTeardown,

    /// Neighbor-Awareness-Networking discovery has started.
    NeighborAwarenessNetworkingStarted,
    /// Neighbor-Awareness-Networking discovery has stopped.
    NeighborAwarenessNetworkingStopped,
    /// Neighbor-Awareness-Networking service discovery match found.
    NeighborAwarenessNetworkingServiceMatch,
    /// Replied to a Neighbor-Awareness-Networking peer with service discovery match.
    NeighborAwarenessNetworkingReplied,
    /// Received a follow-up message in Neighbor-Awareness-Networking.
    NeighborAwarenessNetworkingReceive,
    /// Received NDP (Neighbor Discovery Protocol) request from a Neighbor-Awareness-Networking
    /// peer.
    NeighborDiscoveryProtocolIndication,
    /// NDP confirm indication.
    NeighborDiscoveryProtocolConfirmation,
    /// Neighbor-Awareness-Networking datapath terminated indication.
    NeighborDiscoveryProtocolTerminated,
    /// Wi-Fi home channel change, doesn't occur when scanning.
    HomeChannelChange,

    /// Received Neighbor Report response.
    StationNeighborRep,
}

/// Get the access point MAC address of the device.
pub fn access_point_mac() -> [u8; 6] {
    let mut mac = [0u8; 6];
    unsafe {
        read_mac(mac.as_mut_ptr(), 1);
    }
    mac
}

/// Get the station MAC address of the device.
pub fn station_mac() -> [u8; 6] {
    let mut mac = [0u8; 6];
    unsafe {
        read_mac(mac.as_mut_ptr(), 0);
    }
    mac
}

#[cfg(esp32)]
fn set_mac_time_update_cb(wifi: crate::hal::peripherals::WIFI<'_>) {
    use esp_phy::MacTimeExt;

    use crate::sys::include::esp_wifi_internal_update_mac_time;
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

        // until we support APSTA we just register the same callback for AP and station
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
        let res = crate::sys::include::esp_coex_adapter_register(
            core::ptr::addr_of_mut!(internal::G_COEX_ADAPTER_FUNCS).cast(),
        );
        if res != 0 {
            error!("Error: esp_coex_adapter_register {}", res);
            return res;
        }
        let res = crate::sys::include::coex_pre_init();
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
        return unsafe { crate::sys::include::coex_init() };
    }

    #[cfg(not(coex))]
    0
}

fn wifi_deinit() -> Result<(), crate::WifiError> {
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
        let mut buf = Vec::from_iter(m.bytes());
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
    Station,
    /// Access Point mode.
    AccessPoint,
}

impl WifiDeviceMode {
    fn mac_address(&self) -> [u8; 6] {
        match self {
            WifiDeviceMode::Station => station_mac(),
            WifiDeviceMode::AccessPoint => access_point_mac(),
        }
    }

    fn data_queue_rx(&self) -> &'static NonReentrantMutex<VecDeque<PacketBuffer>> {
        match self {
            WifiDeviceMode::Station => &DATA_QUEUE_RX_STA,
            WifiDeviceMode::AccessPoint => &DATA_QUEUE_RX_AP,
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
            WifiDeviceMode::Station => wifi_interface_t_WIFI_IF_STA,
            WifiDeviceMode::AccessPoint => wifi_interface_t_WIFI_IF_AP,
        }
    }

    fn register_transmit_waker(&self, cx: &mut core::task::Context<'_>) {
        embassy::TRANSMIT_WAKER.register(cx.waker())
    }

    fn register_receive_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Station => embassy::STA_RECEIVE_WAKER.register(cx.waker()),
            WifiDeviceMode::AccessPoint => embassy::AP_RECEIVE_WAKER.register(cx.waker()),
        }
    }

    fn register_link_state_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Station => embassy::STA_LINK_STATE_WAKER.register(cx.waker()),
            WifiDeviceMode::AccessPoint => embassy::AP_LINK_STATE_WAKER.register(cx.waker()),
        }
    }

    fn link_state(&self) -> embassy_net_driver::LinkState {
        match self {
            WifiDeviceMode::Station => {
                if matches!(station_state(), WifiStationState::Connected) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
            WifiDeviceMode::AccessPoint => {
                if matches!(access_point_state(), WifiAccessPointState::Started) {
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

    let res = unsafe { esp_wifi_result!(esp_wifi_internal_tx(interface, ptr, len)) };

    if res.is_err() {
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
        if result != $crate::sys::include::ESP_OK as i32 {
            let error = unwrap!(FromPrimitive::from_i32(result));
            warn!(
                "{} returned an error: {:?} ({})",
                stringify!($value),
                error,
                result
            );
            Err(WifiError::from_error_code(error))
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
        crate::sys::include::esp_wifi_set_ps(match ps {
            PowerSaveMode::None => crate::sys::include::wifi_ps_type_t_WIFI_PS_NONE,
            PowerSaveMode::Minimum => crate::sys::include::wifi_ps_type_t_WIFI_PS_MIN_MODEM,
            PowerSaveMode::Maximum => crate::sys::include::wifi_ps_type_t_WIFI_PS_MAX_MODEM,
        })
    })?;
    Ok(())
}

/// Represents the Wi-Fi controller and its associated interfaces.
#[non_exhaustive]
pub struct Interfaces<'d> {
    /// Station mode Wi-Fi device.
    pub station: WifiDevice<'d>,
    /// Access Point mode Wi-Fi device.
    pub access_point: WifiDevice<'d>,
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
    /// The regulations under which the Station/Access Point is operating encompass all environments
    /// for the current frequency band in the country.
    AllEnvironments,

    /// The regulations under which the Station/Access Point is operating are for an outdoor
    /// environment only.
    Outdoors,

    /// The regulations under which the Station/Access Point is operating are for an indoor
    /// environment only.
    Indoors,

    /// The Station/Access Point is operating under a noncountry entity. The first two octets of the
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

#[procmacros::doc_replace]
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
/// # {before_snippet}
/// use esp_radio::wifi::{CountryInfo, OperatingClass};
///
/// let country_info = CountryInfo::from(*b"CN").with_operating_class(OperatingClass::Indoors);
/// # {after_snippet}
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
pub struct Config {
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

impl Default for Config {
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

impl Config {
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
    device: crate::hal::peripherals::WIFI<'d>,
    config: Config,
) -> Result<(WifiController<'d>, Interfaces<'d>), WifiError> {
    let _guard = RadioRefGuard::new()?;

    // TODO: Re-check, if not having interrupts disabled pre-condition is still true
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
            tx_buf_type: crate::sys::include::CONFIG_ESP_WIFI_TX_BUFFER_TYPE as i32,
            static_tx_buf_num: config.static_tx_buf_num as _,
            dynamic_tx_buf_num: config.dynamic_tx_buf_num as _,
            rx_mgmt_buf_type: crate::sys::include::CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF as i32,
            rx_mgmt_buf_num: crate::sys::include::CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF as i32,
            cache_tx_buf_num: crate::sys::include::WIFI_CACHE_TX_BUFFER_NUM as i32,
            csi_enable: cfg!(feature = "csi") as i32,
            ampdu_rx_enable: config.ampdu_rx_enable as _,
            ampdu_tx_enable: config.ampdu_tx_enable as _,
            amsdu_tx_enable: config.amsdu_tx_enable as _,
            nvs_enable: 0,
            nano_enable: 0,
            rx_ba_win: config.rx_ba_win as _,
            wifi_task_core_id: Cpu::current() as _,
            beacon_max_len: crate::sys::include::WIFI_SOFTAP_BEACON_MAX_LEN as i32,
            mgmt_sbuf_num: crate::sys::include::WIFI_MGMT_SBUF_NUM as i32,
            feature_caps: internal::__ESP_RADIO_G_WIFI_FEATURE_CAPS,
            sta_disconnected_pm: false,
            espnow_max_encrypt_num: crate::sys::include::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM
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
        _guard,
        _phantom: Default::default(),
        beacon_timeout: 6,
        ap_beacon_timeout: 100,
    };

    controller.set_power_saving(config.power_save_mode)?;

    Ok((
        controller,
        Interfaces {
            station: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::Station,
            },
            access_point: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::AccessPoint,
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
    _guard: RadioRefGuard,
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
    #[cfg(all(feature = "csi", feature = "unstable"))]
    #[instability::unstable]
    pub fn set_csi(
        &mut self,
        mut csi: csi::CsiConfig,
        cb: impl FnMut(crate::wifi::csi::WifiCsiInfo<'_>) + Send,
    ) -> Result<(), WifiError> {
        csi.apply_config()?;
        csi.set_receive_cb(cb)?;
        csi.set_csi(true)?;

        Ok(())
    }

    #[procmacros::doc_replace]
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
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{ap::AccessPointConfig, ModeConfig};
    /// use esp_radio::wifi::Protocol;
    ///
    /// let (mut wifi_controller, _interfaces) =
    ///     esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    ///
    /// wifi_controller.set_config(&ModeConfig::AccessPoint(
    ///     AccessPointConfig::default().with_ssid("esp-radio".into()),
    /// ))?;
    ///
    /// wifi_controller.set_protocol(Protocol::P802D11BGNLR.into());
    /// # {after_snippet}
    /// ```
    ///
    /// # Note
    ///
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
        if mode.is_station() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_STA, protocol)
            })?;
        }
        if mode.is_access_point() {
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
    pub fn scan_with_config(
        &mut self,
        config: ScanConfig<'_>,
    ) -> Result<Vec<AccessPointInfo>, WifiError> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true, config))?;
        self.scan_results(config.max.unwrap_or(usize::MAX))
    }

    fn scan_results_iter(&mut self) -> Result<ScanResults<'_>, WifiError> {
        ScanResults::new(self)
    }

    fn scan_results(&mut self, max: usize) -> Result<Vec<AccessPointInfo>, WifiError> {
        Ok(self.scan_results_iter()?.take(max).collect::<Vec<_>>())
    }

    /// Starts the Wi-Fi controller.
    ///
    /// This method is not blocking. To check if the controller has started, use the
    /// [`Self::is_started`] method.
    pub fn start(&mut self) -> Result<(), WifiError> {
        unsafe {
            esp_wifi_result!(esp_wifi_start())?;

            let mode = WifiMode::current()?;

            // This is not an if-else because in AccessPoint-Station mode, both are true
            if mode.is_access_point() {
                esp_wifi_result!(include::esp_wifi_set_inactive_time(
                    wifi_interface_t_WIFI_IF_AP,
                    self.ap_beacon_timeout
                ))?;
            }
            if mode.is_station() {
                esp_wifi_result!(include::esp_wifi_set_inactive_time(
                    wifi_interface_t_WIFI_IF_STA,
                    self.beacon_timeout
                ))?;
            }
        }

        Ok(())
    }

    /// Stops the Wi-Fi controller.
    ///
    /// This method is not blocking. Use the [`Self::is_started`] method to see if the controller is
    /// still running.
    pub fn stop(&mut self) -> Result<(), WifiError> {
        self.stop_impl()
    }

    /// Connect Wi-Fi station to the Access Point.
    ///
    /// This method is not blocking. Use the [`Self::is_connected`] method to see if the station is
    /// connected.
    ///
    /// - If station is connected, call [`Self::disconnect`] to disconnect.
    /// - Calling [`Self::scan_with_config`] or [`Self::scan_with_config_async`] will not be
    ///   effective until connection between device and the access point is established.
    /// - If device is scanning and connecting at the same time, it will abort scanning and return a
    ///   warning message and error.
    pub fn connect(&mut self) -> Result<(), WifiError> {
        self.connect_impl()
    }

    /// Disconnect Wi-Fi station from the access point.
    ///
    /// This method is not blocking. Use the [`Self::is_connected`] method to see if the station is
    /// still connected.
    pub fn disconnect(&mut self) -> Result<(), WifiError> {
        self.disconnect_impl()
    }

    /// Get the RSSI information of access point to which the device is associated with.
    /// The value is obtained from the last beacon.
    ///
    /// <div class="warning">
    ///
    /// - Use this API only in Station or AccessPoint-Station mode.
    /// - This API should be called after the station has connected to an access point.
    /// </div>
    ///
    /// # Errors
    /// This function returns [`WifiError::Unsupported`] if the Station side isn't
    /// running. For example, when configured for access point only.
    pub fn rssi(&self) -> Result<i32, WifiError> {
        if self.mode()?.is_station() {
            let mut rssi: i32 = 0;
            // Will return ESP_FAIL -1 if called in access point mode.
            esp_wifi_result!(unsafe { esp_wifi_sta_get_rssi(&mut rssi) })?;
            Ok(rssi)
        } else {
            Err(WifiError::Unsupported)
        }
    }

    /// Get the Access Point information of access point to which the device is associated with.
    /// The value is obtained from the last beacon.
    ///
    /// <div class="warning">
    ///
    /// - Use this API only in Station or AccessPoint-Station mode.
    /// - This API should be called after the station has connected to an access point.
    /// </div>
    ///
    /// # Errors
    /// This function returns [`WifiError::Unsupported`] if the Station side isn't
    /// running. For example, when configured for access point only.
    pub fn ap_info(&self) -> Result<AccessPointInfo, WifiError> {
        if self.mode()?.is_station() {
            let mut record: MaybeUninit<include::wifi_ap_record_t> = MaybeUninit::uninit();
            esp_wifi_result!(unsafe { esp_wifi_sta_get_ap_info(record.as_mut_ptr()) })?;

            let record = unsafe { MaybeUninit::assume_init(record) };
            let ap_info = convert_ap_info(&record);
            Ok(ap_info)
        } else {
            Err(WifiError::Unsupported)
        }
    }

    /// Get the supported capabilities of the controller.
    pub fn capabilities(&self) -> Result<EnumSet<crate::wifi::Capability>, WifiError> {
        let caps = enumset::enum_set! { Capability::Station | Capability::AccessPoint | Capability::AccessPointStation };

        Ok(caps)
    }

    /// Set the configuration.
    ///
    /// This will set the mode accordingly.
    /// You need to use [`Self::connect`] for connecting to an access point.
    ///
    /// Passing [`ModeConfig::None`] will disable both access point and station modes.
    ///
    /// If you don't intend to use Wi-Fi anymore at all consider tearing down
    /// Wi-Fi completely.
    pub fn set_config(&mut self, conf: &ModeConfig) -> Result<(), WifiError> {
        conf.validate()?;

        let mode = match conf {
            ModeConfig::None => wifi_mode_t_WIFI_MODE_NULL,
            ModeConfig::Station(_) => wifi_mode_t_WIFI_MODE_STA,
            ModeConfig::AccessPoint(_) => wifi_mode_t_WIFI_MODE_AP,
            ModeConfig::AccessPointStation(_, _) => wifi_mode_t_WIFI_MODE_APSTA,
            #[cfg(feature = "wifi-eap")]
            ModeConfig::EapStation(_) => wifi_mode_t_WIFI_MODE_STA,
        };

        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode) })?;

        match conf {
            ModeConfig::None => Ok(()),
            ModeConfig::Station(config) => {
                self.apply_sta_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &config.protocols)
            }
            ModeConfig::AccessPoint(config) => {
                self.apply_ap_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &config.protocols)
            }
            ModeConfig::AccessPointStation(sta_config, ap_config) => {
                self.apply_ap_config(ap_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &ap_config.protocols)?;
                self.apply_sta_config(sta_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &sta_config.protocols)
            }
            #[cfg(feature = "wifi-eap")]
            ModeConfig::EapStation(config) => {
                self.apply_sta_eap_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &config.protocols)
            }
        }
        .inspect_err(|_| {
            // we/the driver might have applied a partial configuration
            // so we better disable AccessPoint/Station just in case the caller ignores the error we
            // return here - they will run into futher errors this way
            unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL) };
        })?;

        Ok(())
    }

    /// Set the Wi-Fi mode.
    ///
    /// This will override the mode inferred by [`Self::set_config`].
    #[instability::unstable]
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

    /// Checks if the Wi-Fi controller has started. Returns true if Station and/or AccessPoint are
    /// started.
    ///
    /// This function should be called after the [`Self::start`] method to verify if the
    /// Wi-Fi controller has started successfully.
    pub fn is_started(&self) -> Result<bool, WifiError> {
        if matches!(
            crate::wifi::station_state(),
            WifiStationState::Started
                | WifiStationState::Connected
                | WifiStationState::Disconnected
        ) {
            return Ok(true);
        }
        if matches!(
            crate::wifi::access_point_state(),
            WifiAccessPointState::Started
        ) {
            return Ok(true);
        }
        Ok(false)
    }

    /// Checks if the Wi-Fi controller is connected to an access point.
    ///
    /// This function should be called after the [`Self::connect`] method to verify if
    /// the connection was successful.
    pub fn is_connected(&self) -> Result<bool, WifiError> {
        match crate::wifi::station_state() {
            crate::wifi::WifiStationState::Connected => Ok(true),
            crate::wifi::WifiStationState::Disconnected => Err(WifiError::Disconnected),
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
    ) -> Result<Vec<AccessPointInfo>, WifiError> {
        Self::clear_events(WifiEvent::ScanDone);
        esp_wifi_result!(wifi_start_scan(false, config))?;

        // Prevents memory leak if `scan_with_config_async`'s future is dropped.
        let guard = FreeApListOnDrop;
        WifiEventFuture::new(WifiEvent::ScanDone).await;
        guard.defuse();

        let result = self.scan_results(config.max.unwrap_or(usize::MAX))?;

        Ok(result)
    }

    /// Async version of [`Self::start`].
    ///
    /// This function will wait for the Wi-Fi controller to start before returning.
    pub async fn start_async(&mut self) -> Result<(), WifiError> {
        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_access_point() {
            events |= WifiEvent::AccessPointStart;
        }
        if mode.is_station() {
            events |= WifiEvent::StationStart;
        }

        Self::clear_events(events);

        self.start()?;

        self.wait_for_all_events(events, false).await;

        Ok(())
    }

    /// Async version of [`Self::stop`].
    ///
    /// This function will wait for the Wi-Fi controller to stop before returning.
    pub async fn stop_async(&mut self) -> Result<(), WifiError> {
        // TODO: This might be racey when there is a start operation in progress but it didn't made
        // it to the state where the driver emits the Started event?
        // https://github.com/esp-rs/esp-hal/pull/4504#discussion_r2533184425
        if !self.is_started()? {
            return Err(WifiError::NotStarted);
        }

        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_access_point() {
            events |= WifiEvent::AccessPointStop;
        }
        if mode.is_station() {
            events |= WifiEvent::StationStop;
        }

        Self::clear_events(events);

        self.stop_impl()?;

        self.wait_for_all_events(events, false).await;

        reset_access_point_state();
        reset_station_state();

        Ok(())
    }

    /// Async version of [`Self::connect`].
    ///
    /// This function will wait for the connection to be established before returning.
    pub async fn connect_async(&mut self) -> Result<(), WifiError> {
        Self::clear_events(WifiEvent::StationConnected | WifiEvent::StationDisconnected);

        let err = self.connect_impl().err();

        if MultiWifiEventFuture::new(WifiEvent::StationConnected | WifiEvent::StationDisconnected)
            .await
            .contains(WifiEvent::StationDisconnected)
        {
            Err(err.unwrap_or(WifiError::Disconnected))
        } else {
            Ok(())
        }
    }

    /// Async version of [`Self::disconnect`].
    ///
    /// This function will wait for the connection to be closed before returning.
    pub async fn disconnect_async(&mut self) -> Result<(), WifiError> {
        // If not connected, this will do nothing.
        // It will also wait forever for a `StationDisconnected` event that will never come.
        // Return early instead of hanging.
        if !matches!(self.is_connected(), Ok(true)) {
            return Ok(());
        }

        Self::clear_events(WifiEvent::StationDisconnected);
        self.disconnect_impl()?;
        WifiEventFuture::new(WifiEvent::StationDisconnected).await;

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
            return Err(WifiError::InvalidArguments);
        }

        unsafe {
            cfg.ap.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
            cfg.ap.ssid_len = config.ssid.len() as u8;
            cfg.ap.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

            esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut cfg))
        }
    }

    fn apply_sta_config(&mut self, config: &StationConfig) -> Result<(), WifiError> {
        self.beacon_timeout = config.beacon_timeout;

        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: config.scan_method as c_types::c_uint,
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
            return Err(WifiError::InvalidArguments);
        }

        unsafe {
            cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
            cfg.sta.password[0..(config.password.len())]
                .copy_from_slice(config.password.as_bytes());

            esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))
        }
    }

    #[cfg(feature = "wifi-eap")]
    fn apply_sta_eap_config(&mut self, config: &EapStationConfig) -> Result<(), WifiError> {
        self.beacon_timeout = config.beacon_timeout;

        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: config.scan_method as c_types::c_uint,
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
