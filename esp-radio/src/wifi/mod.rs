//! Wi-Fi

use alloc::{borrow::ToOwned, collections::vec_deque::VecDeque, str, vec::Vec};
use core::{
    fmt::{Debug, Write},
    marker::PhantomData,
    mem::MaybeUninit,
    ptr::addr_of,
};

use docsplay::Display;
use enumset::{EnumSet, EnumSetType};
use esp_config::esp_config_int;
use esp_hal::system::Cpu;
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
use esp_hal::time::{Duration, Instant};
use esp_sync::NonReentrantMutex;
use event::EVENT_CHANNEL;
use portable_atomic::{AtomicUsize, Ordering};
use procmacros::BuilderLite;

pub(crate) use self::os_adapter::*;
#[cfg(all(feature = "sniffer", feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use self::sniffer::Sniffer;
#[cfg(feature = "wifi-eap")]
use self::sta::eap::EapStationConfig;
use self::{
    ap::{AccessPointConfig, AccessPointInfo, convert_ap_info},
    private::PacketBuffer,
    scan::{FreeApListOnDrop, ScanConfig, ScanResults, ScanTypeConfig},
    sta::StationConfig,
    state::*,
};
use crate::{
    RadioRefGuard,
    esp_wifi_result,
    hal::ram,
    sys::{
        c_types,
        include::{self, *},
    },
    wifi::event::{EventInfo, WifiEvent},
};
pub mod ap;

unstable_module!(
    #[cfg(feature = "csi")]
    #[cfg_attr(docsrs, doc(cfg(feature = "csi")))]
    pub mod csi;
    pub mod event;
    #[cfg(feature = "sniffer")]
    #[cfg_attr(docsrs, doc(cfg(feature = "sniffer")))]
    pub mod sniffer;
);

pub mod scan;
pub mod sta;

pub(crate) mod os_adapter;
pub(crate) mod state;

mod internal;

const MTU: usize = esp_config_int!(usize, "ESP_RADIO_CONFIG_WIFI_MTU");

/// Supported Wi-Fi authentication methods.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AuthenticationMethod {
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

    /// This authentication mode will yield same result as [AuthenticationMethod::Wpa3Personal] and
    /// is not recommended to be used. It will be deprecated in future, please use
    /// [AuthenticationMethod::Wpa3Personal] instead.
    Wpa3ExtPsk,

    /// This authentication mode will yield same result as [AuthenticationMethod::Wpa3Personal] and
    /// is not recommended to be used. It will be deprecated in future, please use
    /// [AuthenticationMethod::Wpa3Personal] instead.
    Wpa3ExtPskMixed,

    /// Wi-Fi DPP / Wi-Fi Easy Connect
    Dpp,

    /// WPA3-Enterprise Only Mode
    Wpa3Enterprise,

    /// WPA3-Enterprise Transition Mode
    Wpa2Wpa3Enterprise,

    /// WPA-Enterprise security
    WpaEnterprise,
}

/// Supported Wi-Fi protocols for each band.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Protocols {
    /// Protocol for 2.4 GHz band.
    _2_4: EnumSet<Protocol>,
    /// Protocol for 5 GHz band.
    #[cfg(wifi_has_5g)]
    _5: EnumSet<Protocol>,
}

impl Default for Protocols {
    fn default() -> Self {
        Self {
            _2_4: Protocol::B | Protocol::G | Protocol::N,
            #[cfg(wifi_has_5g)]
            _5: Protocol::AC | Protocol::A | Protocol::AX,
        }
    }
}

impl Protocols {
    fn to_raw(self) -> wifi_protocols_t {
        wifi_protocols_t {
            ghz_2g: to_mask(self._2_4),
            #[cfg(wifi_has_5g)]
            ghz_5g: to_mask(self._5),
            #[cfg(not(wifi_has_5g))]
            ghz_5g: 0,
        }
    }
}

#[cfg_attr(docsrs, procmacros::doc_replace(
    "hint_5g" => {
        cfg(wifi_has_5g) => "The default protocol is AC/A/AX for band mode 5G.",
        _ => ""
    },
))]
/// Supported Wi-Fi protocols.
///
/// The default protocol is B/G/N for band mode 2.4G.
/// # {hint_5g}
#[derive(Debug, PartialOrd, Hash, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Protocol {
    /// 802.11b protocol
    B,

    /// 802.11g protocol
    G,

    /// 802.11n protocol
    N,

    /// Low Rate protocol
    LR,

    /// 802.11a protocol
    A,

    /// 802.11ac protocol
    AC,

    /// 802.11ax protocol
    AX,
}

impl Protocol {
    fn to_mask(self) -> u16 {
        let mask = match self {
            Protocol::B => WIFI_PROTOCOL_11B,
            Protocol::G => WIFI_PROTOCOL_11G,
            Protocol::N => WIFI_PROTOCOL_11N,
            Protocol::LR => WIFI_PROTOCOL_LR,
            Protocol::A => WIFI_PROTOCOL_11A,
            Protocol::AC => WIFI_PROTOCOL_11AC,
            Protocol::AX => WIFI_PROTOCOL_11AX,
        };
        mask as _
    }
}

fn to_mask(protocols: EnumSet<Protocol>) -> u16 {
    protocols.iter().fold(0, |acc, p| acc | p.to_mask())
}

/// Secondary Wi-Fi channels.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SecondaryChannel {
    /// No secondary channel (default).
    #[default]
    None,

    /// Secondary channel is above the primary channel.
    Above,

    /// Secondary channel is below the primary channel.
    Below,
}

impl SecondaryChannel {
    fn from_raw(raw: u32) -> Self {
        match raw {
            0 => SecondaryChannel::None,
            1 => SecondaryChannel::Above,
            2 => SecondaryChannel::Below,
            _ => panic!("Invalid secondary channel value: {}", raw),
        }
    }
}

#[cfg_attr(docsrs, procmacros::doc_replace(
    "hint_5g" => {
        cfg(wifi_has_5g) => "The default is [BandMode::Auto].",
        _ => "The default is [BandMode::_2_4G]"
    },
))]
/// Wi-Fi band mode.
///
/// # {hint_5g}
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BandMode {
    /// Wi-Fi band mode is 2.4 GHz only.
    #[cfg_attr(not(wifi_has_5g), default)]
    _2_4G,
    /// Wi-Fi band mode is 5 GHz only.
    #[cfg(wifi_has_5g)]
    _5G,
    /// Wi-Fi band mode is 2.4 GHz + 5 GHz.
    #[cfg_attr(wifi_has_5g, default)]
    #[cfg(wifi_has_5g)]
    Auto,
}

impl BandMode {
    fn to_raw(&self) -> u32 {
        match self {
            BandMode::_2_4G => wifi_band_mode_t_WIFI_BAND_MODE_2G_ONLY,
            #[cfg(wifi_has_5g)]
            BandMode::_5G => wifi_band_mode_t_WIFI_BAND_MODE_5G_ONLY,
            #[cfg(wifi_has_5g)]
            BandMode::Auto => wifi_band_mode_t_WIFI_BAND_MODE_AUTO,
        }
    }
}

/// Configuration of Wi-Fi operation mode.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Config {
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

impl Config {
    fn validate(&self) -> Result<(), WifiError> {
        match self {
            Config::Station(station_configuration) => station_configuration.validate(),
            Config::AccessPoint(access_point_configuration) => {
                access_point_configuration.validate()
            }
            Config::AccessPointStation(station_configuration, access_point_configuration) => {
                station_configuration.validate()?;
                access_point_configuration.validate()
            }
            #[cfg(feature = "wifi-eap")]
            Config::EapStation(eap_station_configuration) => eap_station_configuration.validate(),
        }
    }
}

impl AuthenticationMethod {
    fn to_raw(self) -> wifi_auth_mode_t {
        match self {
            AuthenticationMethod::None => include::wifi_auth_mode_t_WIFI_AUTH_OPEN,
            AuthenticationMethod::Wep => include::wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthenticationMethod::Wpa => include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthenticationMethod::Wpa2Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthenticationMethod::WpaWpa2Personal => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK
            }
            AuthenticationMethod::Wpa2Enterprise => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE
            }
            AuthenticationMethod::Wpa3Personal => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthenticationMethod::Wpa2Wpa3Personal => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK
            }
            AuthenticationMethod::WapiPersonal => include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
            AuthenticationMethod::Owe => include::wifi_auth_mode_t_WIFI_AUTH_OWE,
            AuthenticationMethod::Wpa3EntSuiteB192Bit => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENT_192
            }
            AuthenticationMethod::Wpa3ExtPsk => include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK,
            AuthenticationMethod::Wpa3ExtPskMixed => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE
            }
            AuthenticationMethod::Dpp => include::wifi_auth_mode_t_WIFI_AUTH_DPP,
            AuthenticationMethod::Wpa3Enterprise => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENTERPRISE
            }
            AuthenticationMethod::Wpa2Wpa3Enterprise => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_ENTERPRISE
            }
            AuthenticationMethod::WpaEnterprise => {
                include::wifi_auth_mode_t_WIFI_AUTH_WPA_ENTERPRISE
            }
        }
    }

    fn from_raw(raw: wifi_auth_mode_t) -> Self {
        match raw {
            include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthenticationMethod::None,
            include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthenticationMethod::Wep,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthenticationMethod::Wpa,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => AuthenticationMethod::Wpa2Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => {
                AuthenticationMethod::WpaWpa2Personal
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => {
                AuthenticationMethod::Wpa2Enterprise
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthenticationMethod::Wpa3Personal,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => {
                AuthenticationMethod::Wpa2Wpa3Personal
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => AuthenticationMethod::WapiPersonal,
            include::wifi_auth_mode_t_WIFI_AUTH_OWE => AuthenticationMethod::Owe,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENT_192 => {
                AuthenticationMethod::Wpa3EntSuiteB192Bit
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK => AuthenticationMethod::Wpa3ExtPsk,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE => {
                AuthenticationMethod::Wpa3ExtPskMixed
            }
            include::wifi_auth_mode_t_WIFI_AUTH_DPP => AuthenticationMethod::Dpp,
            include::wifi_auth_mode_t_WIFI_AUTH_WPA3_ENTERPRISE => {
                AuthenticationMethod::Wpa3Enterprise
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_ENTERPRISE => {
                AuthenticationMethod::Wpa2Wpa3Enterprise
            }
            include::wifi_auth_mode_t_WIFI_AUTH_WPA_ENTERPRISE => {
                AuthenticationMethod::WpaEnterprise
            }
            // we const-assert we know all the auth-methods the wifi driver knows and it shouldn't
            // return anything else.
            //
            // In fact from observation the drivers will return
            // `wifi_auth_mode_t_WIFI_AUTH_OPEN` if the method is unsupported (e.g. any WPA3 in our
            // case, since the supplicant isn't compiled to support it)
            _ => AuthenticationMethod::None,
        }
    }
}

/// Wi-Fi Mode (Station and/or AccessPoint).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
enum WifiMode {
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

        Ok(Self::from_raw(mode))
    }

    /// Returns true if this mode works as a station.
    fn is_station(&self) -> bool {
        match self {
            Self::Station | Self::AccessPointStation => true,
            Self::AccessPoint => false,
        }
    }

    /// Returns true if this mode works as an access point.
    fn is_access_point(&self) -> bool {
        match self {
            Self::Station => false,
            Self::AccessPoint | Self::AccessPointStation => true,
        }
    }

    /// Creates a `WifiMode` from a raw `wifi_mode_t` value.
    fn from_raw(value: wifi_mode_t) -> Self {
        #[allow(non_upper_case_globals)]
        match value {
            include::wifi_mode_t_WIFI_MODE_STA => Self::Station,
            include::wifi_mode_t_WIFI_MODE_AP => Self::AccessPoint,
            include::wifi_mode_t_WIFI_MODE_APSTA => Self::AccessPointStation,
            _ => panic!("Invalid wifi mode value: {}", value),
        }
    }
}

impl From<&Config> for WifiMode {
    fn from(config: &Config) -> Self {
        match config {
            Config::AccessPoint(_) => Self::AccessPoint,
            Config::Station(_) => Self::Station,
            Config::AccessPointStation(_, _) => Self::AccessPointStation,
            #[cfg(feature = "wifi-eap")]
            Config::EapStation(_) => Self::Station,
        }
    }
}

/// Reason for disconnection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum DisconnectReason {
    /// Unspecified reason
    Unspecified,
    /// Authentication expired
    AuthenticationExpired,
    /// Deauthentication due to leaving
    AuthenticationLeave,
    /// Disassociated due to inactivity
    DisassociatedDueToInactivity,
    /// Too many associated stations
    AssociationTooMany,
    /// Class 2 frame received from non authenticated station
    Class2FrameFromNonAuthenticatedStation,
    /// Class 3 frame received from non associated station
    Class3FrameFromNonAssociatedStation,
    /// Disassociated due to leaving
    AssociationLeave,
    /// Association but not authenticated
    AssociationNotAuthenticated,
    /// Disassociated due to poor power capability
    DisassociatedPowerCapabilityBad,
    /// Disassociated due to unsupported channel
    DisassociatedUnsupportedChannel,
    /// Disassociated due to BSS transition
    BssTransitionDisassociated,
    /// Invalid Information Element (IE)
    IeInvalid,
    /// MIC failure
    MicFailure,
    /// 4-way handshake timeout
    FourWayHandshakeTimeout,
    /// Group key update timeout
    GroupKeyUpdateTimeout,
    /// IE differs in 4-way handshake
    IeIn4wayDiffers,
    /// Invalid group cipher
    GroupCipherInvalid,
    /// Invalid pairwise cipher
    PairwiseCipherInvalid,
    /// Invalid AKMP
    AkmpInvalid,
    /// Unsupported RSN IE version
    UnsupportedRsnIeVersion,
    /// Invalid RSN IE capabilities
    InvalidRsnIeCapabilities,
    /// 802.1X authentication failed
    _802_1xAuthenticationFailed,
    /// Cipher suite rejected
    CipherSuiteRejected,
    /// TDLS peer unreachable
    TdlsPeerUnreachable,
    /// TDLS unspecified
    TdlsUnspecified,
    /// SSP requested disassociation
    SspRequestedDisassociation,
    /// No SSP roaming agreement
    NoSspRoamingAgreement,
    /// Bad cipher or AKM
    BadCipherOrAkm,
    /// Not authorized in this location
    NotAuthorizedThisLocation,
    /// Service change precludes TS
    ServiceChangePercludesTs,
    /// Unspecified QoS reason
    UnspecifiedQos,
    /// Not enough bandwidth
    NotEnoughBandwidth,
    /// Missing ACKs
    MissingAcks,
    /// Exceeded TXOP
    ExceededTxOp,
    /// Station leaving
    StationLeaving,
    /// End of Block Ack (BA)
    EndBlockAck,
    /// Unknown Block Ack (BA)
    UnknownBlockAck,
    /// Timeout
    Timeout,
    /// Peer initiated disassociation
    PeerInitiated,
    /// Access point initiated disassociation
    AccessPointInitiatedDisassociation,
    /// Invalid FT action frame count
    InvalidFtActionFrameCount,
    /// Invalid PMKID
    InvalidPmkid,
    /// Invalid MDE
    InvalidMde,
    /// Invalid FTE
    InvalidFte,
    /// Transmission link establishment failed
    TransmissionLinkEstablishmentFailed,
    /// Alternative channel occupied
    AlterativeChannelOccupied,
    /// Beacon timeout
    BeaconTimeout,
    /// No access point found
    NoAccessPointFound,
    /// Authentication failed
    AuthenticationFailed,
    /// Association failed
    AssociationFailed,
    /// Handshake timeout
    HandshakeTimeout,
    /// Connection failed
    ConnectionFailed,
    /// AP TSF reset
    AccessPointTsfReset,
    /// Roaming
    Roaming,
    /// Association comeback time too long
    AssociationComebackTimeTooLong,
    /// SA query timeout
    SaQueryTimeout,
    /// No AP found with compatible security
    NoAccessPointFoundWithCompatibleSecurity,
    /// No AP found in auth mode threshold
    NoAccessPointFoundInAuthmodeThreshold,
    /// No AP found in RSSI threshold
    NoAccessPointFoundInRssiThreshold,
}

impl DisconnectReason {
    fn from_raw(id: u16) -> Self {
        match id {
            1 => Self::Unspecified,
            2 => Self::AuthenticationExpired,
            3 => Self::AuthenticationLeave,
            4 => Self::DisassociatedDueToInactivity,
            5 => Self::AssociationTooMany,
            6 => Self::Class2FrameFromNonAuthenticatedStation,
            7 => Self::Class3FrameFromNonAssociatedStation,
            8 => Self::AssociationLeave,
            9 => Self::AssociationNotAuthenticated,
            10 => Self::DisassociatedPowerCapabilityBad,
            11 => Self::DisassociatedUnsupportedChannel,
            12 => Self::BssTransitionDisassociated,
            13 => Self::IeInvalid,
            14 => Self::MicFailure,
            15 => Self::FourWayHandshakeTimeout,
            16 => Self::GroupKeyUpdateTimeout,
            17 => Self::IeIn4wayDiffers,
            18 => Self::GroupCipherInvalid,
            19 => Self::PairwiseCipherInvalid,
            20 => Self::AkmpInvalid,
            21 => Self::UnsupportedRsnIeVersion,
            22 => Self::InvalidRsnIeCapabilities,
            23 => Self::_802_1xAuthenticationFailed,
            24 => Self::CipherSuiteRejected,
            25 => Self::TdlsPeerUnreachable,
            26 => Self::TdlsUnspecified,
            27 => Self::SspRequestedDisassociation,
            28 => Self::NoSspRoamingAgreement,
            29 => Self::BadCipherOrAkm,
            30 => Self::NotAuthorizedThisLocation,
            31 => Self::ServiceChangePercludesTs,
            32 => Self::UnspecifiedQos,
            33 => Self::NotEnoughBandwidth,
            34 => Self::MissingAcks,
            35 => Self::ExceededTxOp,
            36 => Self::StationLeaving,
            37 => Self::EndBlockAck,
            38 => Self::UnknownBlockAck,
            39 => Self::Timeout,
            46 => Self::PeerInitiated,
            47 => Self::AccessPointInitiatedDisassociation,
            48 => Self::InvalidFtActionFrameCount,
            49 => Self::InvalidPmkid,
            50 => Self::InvalidMde,
            51 => Self::InvalidFte,
            67 => Self::TransmissionLinkEstablishmentFailed,
            68 => Self::AlterativeChannelOccupied,
            200 => Self::BeaconTimeout,
            201 => Self::NoAccessPointFound,
            202 => Self::AuthenticationFailed,
            203 => Self::AssociationFailed,
            204 => Self::HandshakeTimeout,
            205 => Self::ConnectionFailed,
            206 => Self::AccessPointTsfReset,
            207 => Self::Roaming,
            208 => Self::AssociationComebackTimeTooLong,
            209 => Self::SaQueryTimeout,
            210 => Self::NoAccessPointFoundWithCompatibleSecurity,
            211 => Self::NoAccessPointFoundInAuthmodeThreshold,
            212 => Self::NoAccessPointFoundInRssiThreshold,
            _ => Self::Unspecified,
        }
    }
}

/// Information about a connected station.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ssid {
    ssid: [u8; 32],
    len: u8,
}

impl Ssid {
    pub(crate) fn new(ssid: &str) -> Self {
        let mut ssid_bytes = [0u8; 32];
        let bytes = ssid.as_bytes();
        let len = usize::min(32, bytes.len());
        ssid_bytes[..len].copy_from_slice(bytes);

        Self::from_raw(&ssid_bytes, len as u8)
    }

    pub(crate) fn from_raw(ssid: &[u8], len: u8) -> Self {
        let mut ssid_bytes = [0u8; 32];
        let len = usize::min(32, len as usize);
        ssid_bytes[..len].copy_from_slice(&ssid[..len]);

        Self {
            ssid: ssid_bytes,
            len: len as u8,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.ssid[..self.len as usize]
    }

    /// The length (in bytes) of the SSID.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns true if the SSID is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The SSID as a string slice.
    pub fn as_str(&self) -> &str {
        let part = &self.ssid[..self.len as usize];
        match str::from_utf8(part) {
            Ok(s) => s,
            Err(e) => {
                let (valid, _) = part.split_at(e.valid_up_to());
                unsafe { str::from_utf8_unchecked(valid) }
            }
        }
    }
}

impl Debug for Ssid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_char('"')?;
        f.write_str(self.as_str())?;
        f.write_char('"')
    }
}

impl From<alloc::string::String> for Ssid {
    fn from(ssid: alloc::string::String) -> Self {
        Self::new(&ssid)
    }
}

impl From<&str> for Ssid {
    fn from(ssid: &str) -> Self {
        Self::new(ssid)
    }
}

impl From<&[u8]> for Ssid {
    fn from(ssid: &[u8]) -> Self {
        Self::from_raw(ssid, ssid.len() as u8)
    }
}

/// Information about a connected station.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ConnectedStationInfo {
    /// The SSID of the connected station.
    pub ssid: Ssid,
    /// The BSSID of the connected station.
    pub bssid: [u8; 6],
    /// The channel of the connected station.
    pub channel: u8,
    /// The authmode of the connected station.
    pub authmode: AuthenticationMethod,
    /// The Association ID (AID) of the connected station.
    pub aid: u16,
}

/// Information about a disconnected station.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct DisconnectedStationInfo {
    /// The SSID of the disconnected station.
    pub ssid: Ssid,
    /// The BSSID of the disconnected station.
    pub bssid: [u8; 6],
    /// The disconnect reason.
    // should we introduce an enum?
    pub reason: DisconnectReason,
    /// The RSSI.
    pub rssi: i8,
}

/// Information about a station connected to the access point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct AccessPointStationConnectedInfo {
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
pub struct AccessPointStationDisconnectedInfo {
    /// The MAC address.
    pub mac: [u8; 6],
    /// The Association ID (AID) of the connected station.
    pub aid: u16,
    /// If this is a mesh child.
    pub is_mesh_child: bool,
    /// The disconnect reason.
    pub reason: DisconnectReason,
}

/// Either the [AccessPointStationConnectedInfo] or [AccessPointStationDisconnectedInfo].
pub enum AccessPointStationEventInfo {
    /// Information about a station connected to the access point.
    Connected(AccessPointStationConnectedInfo),
    /// Information about a station disconnected from the access point.
    Disconnected(AccessPointStationDisconnectedInfo),
}

static RX_QUEUE_SIZE: AtomicUsize = AtomicUsize::new(0);
static TX_QUEUE_SIZE: AtomicUsize = AtomicUsize::new(0);

pub(crate) static DATA_QUEUE_RX_AP: NonReentrantMutex<VecDeque<PacketBuffer>> =
    NonReentrantMutex::new(VecDeque::new());

pub(crate) static DATA_QUEUE_RX_STA: NonReentrantMutex<VecDeque<PacketBuffer>> =
    NonReentrantMutex::new(VecDeque::new());

/// Common errors.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WifiError {
    /// The device disconnected from the network or failed to connect to it.
    Disconnected(DisconnectedStationInfo),

    /// Unsupported operation or mode.
    Unsupported,

    /// Passed arguments are invalid.
    InvalidArguments,

    /// Generic failure - not further specified.
    Failed,

    /// Out of memory.
    OutOfMemory,

    /// Wi-Fi driver was not started by [esp_wifi_start].
    NotStarted,

    /// SSID is invalid.
    InvalidSsid,

    /// Password is invalid.
    InvalidPassword,

    /// Station still in disconnect status.
    NotConnected,
}

impl WifiError {
    fn from_error_code(code: i32) -> Self {
        if crate::sys::include::ESP_FAIL == code {
            return WifiError::Failed;
        }

        match code as u32 {
            crate::sys::include::ESP_ERR_NO_MEM => WifiError::OutOfMemory,
            crate::sys::include::ESP_ERR_INVALID_ARG => WifiError::InvalidArguments,
            crate::sys::include::ESP_ERR_WIFI_NOT_STARTED => WifiError::NotStarted,
            crate::sys::include::ESP_ERR_WIFI_SSID => WifiError::InvalidSsid,
            crate::sys::include::ESP_ERR_WIFI_PASSWORD => WifiError::InvalidPassword,
            crate::sys::include::ESP_ERR_WIFI_NOT_CONNECT => WifiError::NotConnected,
            _ => panic!("Unknown error code: {}", code),
        }
    }
}

impl core::error::Error for WifiError {}

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
    }: ScanConfig,
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
        let mut buf = Vec::from_iter(m.as_bytes().to_owned());
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

/// Wi-Fi interface mode.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum InterfaceType {
    /// Station mode.
    Station,
    /// Access Point mode.
    AccessPoint,
}

impl InterfaceType {
    fn mac_address(&self) -> [u8; 6] {
        use esp_hal::efuse::{Efuse, InterfaceMacAddress};
        let mac = match self {
            InterfaceType::Station => Efuse::interface_mac_address(InterfaceMacAddress::Station),
            InterfaceType::AccessPoint => {
                Efuse::interface_mac_address(InterfaceMacAddress::AccessPoint)
            }
        };

        let mut out = [0u8; 6];
        out.copy_from_slice(mac.as_bytes());
        out
    }

    fn data_queue_rx(&self) -> &'static NonReentrantMutex<VecDeque<PacketBuffer>> {
        match self {
            InterfaceType::Station => &DATA_QUEUE_RX_STA,
            InterfaceType::AccessPoint => &DATA_QUEUE_RX_AP,
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
            // even checking for !Uninitialized would be enough to not crash
            match self {
                InterfaceType::Station => {
                    if !matches!(station_state(), WifiStationState::Connected) {
                        return None;
                    }
                }
                InterfaceType::AccessPoint => {
                    if !matches!(access_point_state(), WifiAccessPointState::Started) {
                        return None;
                    }
                }
            }

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
            InterfaceType::Station => wifi_interface_t_WIFI_IF_STA,
            InterfaceType::AccessPoint => wifi_interface_t_WIFI_IF_AP,
        }
    }

    fn register_transmit_waker(&self, cx: &mut core::task::Context<'_>) {
        embassy::TRANSMIT_WAKER.register(cx.waker())
    }

    fn register_receive_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            InterfaceType::Station => embassy::STA_RECEIVE_WAKER.register(cx.waker()),
            InterfaceType::AccessPoint => embassy::AP_RECEIVE_WAKER.register(cx.waker()),
        }
    }

    fn register_link_state_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            InterfaceType::Station => embassy::STA_LINK_STATE_WAKER.register(cx.waker()),
            InterfaceType::AccessPoint => embassy::AP_LINK_STATE_WAKER.register(cx.waker()),
        }
    }

    fn link_state(&self) -> embassy_net_driver::LinkState {
        match self {
            InterfaceType::Station => {
                if matches!(station_state(), WifiStationState::Connected) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
            InterfaceType::AccessPoint => {
                if matches!(access_point_state(), WifiAccessPointState::Started) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
        }
    }
}

/// Wi-Fi interface.
pub struct Interface<'d> {
    _phantom: PhantomData<&'d ()>,
    mode: InterfaceType,
}

impl Interface<'_> {
    #[procmacros::doc_replace]
    /// Retrieves the MAC address of the Wi-Fi device.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// let (_controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    ///
    /// let station = interfaces.station;
    /// let mac = station.mac_address();
    ///
    /// println!("Station MAC: {:02x?}", mac);
    /// # {after_snippet}
    /// ```
    pub fn mac_address(&self) -> [u8; 6] {
        self.mode.mac_address()
    }

    #[doc(hidden)]
    /// Receives data from the Wi-Fi device.
    pub fn receive(&mut self) -> Option<(WifiRxToken, WifiTxToken)> {
        self.mode.rx_token()
    }

    #[doc(hidden)]
    /// Transmits data through the Wi-Fi device.
    pub fn transmit(&mut self) -> Option<WifiTxToken> {
        self.mode.tx_token()
    }
}

/// Supported Wi-Fi protocols for each band.
#[derive(Debug, Clone, Copy, PartialEq, Hash, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Bandwidths {
    /// Bandwidth for 2.4 GHz band.
    _2_4: Bandwidth,
    /// Bandwidth for 5 GHz band.
    #[cfg(wifi_has_5g)]
    _5: Bandwidth,
}

impl Bandwidths {
    fn to_raw(self) -> wifi_bandwidths_t {
        wifi_bandwidths_t {
            ghz_2g: self._2_4.to_raw(),
            #[cfg(wifi_has_5g)]
            ghz_5g: self._5.to_raw(),
            #[cfg(not(wifi_has_5g))]
            ghz_5g: 0,
        }
    }
}

/// Wi-Fi bandwidth options.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum Bandwidth {
    /// 20 MHz bandwidth.
    _20MHz,
    /// 40 MHz bandwidth.
    _40MHz,
    /// 80 MHz bandwidth.
    _80MHz,
    /// 160 MHz bandwidth.
    _160MHz,
    /// 80+80 MHz bandwidth.
    _80_80MHz,
}

impl Bandwidth {
    fn to_raw(self) -> wifi_bandwidth_t {
        match self {
            Bandwidth::_20MHz => wifi_bandwidth_t_WIFI_BW_HT20,
            Bandwidth::_40MHz => wifi_bandwidth_t_WIFI_BW_HT40,
            Bandwidth::_80MHz => wifi_bandwidth_t_WIFI_BW80,
            Bandwidth::_160MHz => wifi_bandwidth_t_WIFI_BW160,
            Bandwidth::_80_80MHz => wifi_bandwidth_t_WIFI_BW80_BW80,
        }
    }

    fn from_raw(raw: wifi_bandwidth_t) -> Self {
        match raw {
            raw if raw == wifi_bandwidth_t_WIFI_BW_HT20 => Bandwidth::_20MHz,
            raw if raw == wifi_bandwidth_t_WIFI_BW_HT40 => Bandwidth::_40MHz,
            raw if raw == wifi_bandwidth_t_WIFI_BW80 => Bandwidth::_80MHz,
            raw if raw == wifi_bandwidth_t_WIFI_BW160 => Bandwidth::_160MHz,
            raw if raw == wifi_bandwidth_t_WIFI_BW80_BW80 => Bandwidth::_80_80MHz,
            _ => Bandwidth::_20MHz,
        }
    }
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(wifi_mac_version = "1")]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
#[instability::unstable]
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
    pub secondary_channel: SecondaryChannel,
    /// Timestamp of when the packet is received, in microseconds. Precise only
    /// if modem sleep or light sleep is not enabled.
    pub timestamp: Instant,
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
#[cfg(wifi_mac_version = "2")]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
#[instability::unstable]
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
    /// The secondary channel if in HT40.
    pub secondary_channel: SecondaryChannel,
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
    /// The local time when this packet is received. It is precise only if modem sleep or light
    /// sleep is not enabled. unit: microsecond.
    pub timestamp: Instant,
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(wifi_mac_version = "3")]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
#[instability::unstable]
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
    /// Current baseband format.
    pub cur_bb_format: u32,
    /// Channel estimation validity.
    pub rx_channel_estimate_info_vld: u32,
    /// Length of the channel estimation.
    pub rx_channel_estimate_len: u32,
    /// The secondary channel if in HT40.
    pub secondary_channel: SecondaryChannel,
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
    /// The local time when this packet is received. It is precise only if modem sleep or light
    /// sleep is not enabled. unit: microsecond.
    pub timestamp: Instant,
}

#[cfg(all(any(feature = "esp-now", feature = "sniffer"), feature = "unstable"))]
impl RxControlInfo {
    /// Create an instance from a raw pointer to [wifi_pkt_rx_ctrl_t].
    ///
    /// # Safety
    /// When calling this, you must ensure, that `rx_cntl` points to a valid
    /// instance of [wifi_pkt_rx_ctrl_t].
    pub(super) unsafe fn from_raw(rx_cntl: *const wifi_pkt_rx_ctrl_t) -> Self {
        #[cfg(wifi_mac_version = "1")]
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
                secondary_channel: SecondaryChannel::from_raw((*rx_cntl).secondary_channel()),
                timestamp: Instant::EPOCH + Duration::from_micros((*rx_cntl).timestamp() as u64),
                noise_floor: (*rx_cntl).noise_floor(),
                ant: (*rx_cntl).ant(),
                sig_len: (*rx_cntl).sig_len(),
                rx_state: (*rx_cntl).rx_state(),
            }
        };
        #[cfg(wifi_mac_version = "2")]
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
                secondary_channel: SecondaryChannel::from_raw((*rx_cntl).second()),
                channel: (*rx_cntl).channel(),
                noise_floor: (*rx_cntl).noise_floor() as _,
                is_group: (*rx_cntl).is_group(),
                rxend_state: (*rx_cntl).rxend_state(),
                rxmatch3: (*rx_cntl).rxmatch3(),
                rxmatch2: (*rx_cntl).rxmatch2(),
                rxmatch1: (*rx_cntl).rxmatch1(),
                rxmatch0: (*rx_cntl).rxmatch0(),
                timestamp: Instant::EPOCH + Duration::from_micros((*rx_cntl).timestamp() as u64),
            }
        };
        #[cfg(wifi_mac_version = "3")]
        let rx_control_info = unsafe {
            RxControlInfo {
                rssi: (*rx_cntl).rssi(),
                rate: (*rx_cntl).rate(),
                sig_len: (*rx_cntl).sig_len(),
                rx_state: (*rx_cntl).rx_state(),
                dump_len: (*rx_cntl).dump_len(),
                he_sigb_len: (*rx_cntl).sigb_len(),
                cur_bb_format: (*rx_cntl).cur_bb_format(),
                rx_channel_estimate_info_vld: (*rx_cntl).rx_channel_estimate_info_vld(),
                rx_channel_estimate_len: (*rx_cntl).rx_channel_estimate_len(),
                secondary_channel: SecondaryChannel::from_raw((*rx_cntl).second()),
                channel: (*rx_cntl).channel(),
                noise_floor: (*rx_cntl).noise_floor() as _,
                is_group: (*rx_cntl).is_group(),
                rxend_state: (*rx_cntl).rxend_state(),
                rxmatch3: (*rx_cntl).rxmatch3(),
                rxmatch2: (*rx_cntl).rxmatch2(),
                rxmatch1: (*rx_cntl).rxmatch1(),
                rxmatch0: (*rx_cntl).rxmatch0(),
                timestamp: Instant::EPOCH + Duration::from_micros((*rx_cntl).timestamp() as u64),
            }
        };
        rx_control_info
    }
}

#[doc(hidden)]
// These token doesn't need the debug trait, they aren't needed publicly.
pub struct WifiRxToken {
    mode: InterfaceType,
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

#[doc(hidden)]
// These token doesn't need the debug trait, they aren't needed publicly.
pub struct WifiTxToken {
    mode: InterfaceType,
}

impl WifiTxToken {
    /// Consumes the TX token and applies the callback function to the received
    /// data buffer.
    pub fn consume_token<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.mode.increase_in_flight_counter();

        let mut buffer: [u8; MTU] = [0u8; MTU];
        let buffer = &mut buffer[..len];

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

// FIXME data here has to be &mut because of `esp_wifi_internal_tx` signature,
// requiring a *mut ptr to the buffer Casting const to mut is instant UB, even
// though in reality `esp_wifi_internal_tx` copies the buffer into its own
// memory and does not modify
pub(crate) fn esp_wifi_send_data(interface: wifi_interface_t, data: &mut [u8]) {
    // `esp_wifi_internal_tx` will crash if wifi is uninitialized or de-inited

    state::locked(|| {
        // even checking for !Uninitialized would be enough to not crash
        if (interface == wifi_interface_t_WIFI_IF_STA
            && !matches!(station_state(), WifiStationState::Connected))
            || (interface == wifi_interface_t_WIFI_IF_AP
                && !matches!(access_point_state(), WifiAccessPointState::Started))
        {
            return;
        }

        trace!("sending... {} bytes", data.len());
        dump_packet_info(data);

        let len = data.len() as u16;
        let ptr = data.as_mut_ptr().cast();

        let res = unsafe { esp_wifi_internal_tx(interface, ptr, len) };

        if res != include::ESP_OK as i32 {
            warn!("esp_wifi_internal_tx returned error: {}", res);
            decrement_inflight_counter();
        }
    })
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
                "{} returned an error: {:?} ({}). If this error is unmapped, please open an issue at <https://github.com/esp-rs/esp-hal/issues>.",
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

    impl Driver for Interface<'_> {
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
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
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
    pub station: Interface<'d>,
    /// Access Point mode Wi-Fi device.
    pub access_point: Interface<'d>,
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

    fn from_code(code: u8) -> Option<Self> {
        match code {
            b' ' => Some(OperatingClass::AllEnvironments),
            b'O' => Some(OperatingClass::Outdoors),
            b'I' => Some(OperatingClass::Indoors),
            b'X' => Some(OperatingClass::NonCountryEntity),
            code => Some(OperatingClass::Repr(code)),
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
#[instability::unstable]
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

            #[cfg(wifi_has_5g)]
            wifi_5g_channel_mask: 0,
        }
    }

    #[cfg_attr(not(feature = "unstable"), expect(dead_code))]
    fn try_from_c(info: &wifi_country_t) -> Option<Self> {
        let cc = &info.cc;
        let operating_class = OperatingClass::from_code(cc[2])?;

        Some(Self {
            country: [cc[0], cc[1]],
            operating_class,
        })
    }
}

/// Wi-Fi configuration.
#[derive(Clone, Copy, BuilderLite, Debug, Hash, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ControllerConfig {
    /// Country info.
    #[builder_lite(into)]
    #[builder_lite(unstable)]
    country_info: CountryInfo,
    /// Size of the RX queue in frames.
    #[builder_lite(unstable)]
    rx_queue_size: usize,
    /// Size of the TX queue in frames.
    #[builder_lite(unstable)]
    tx_queue_size: usize,

    /// Max number of Wi-Fi static RX buffers.
    ///
    /// Each buffer takes approximately 1.6KB of RAM. The static rx buffers are allocated when
    /// esp_wifi_init is called, they are not freed until esp_wifi_deinit is called.
    ///
    /// Wi-Fi hardware use these buffers to receive all 802.11 frames. A higher number may allow
    /// higher throughput but increases memory use. If [`Self::ampdu_rx_enable`] is enabled,
    /// this value is recommended to set equal or bigger than [`Self::rx_ba_win`] in order to
    /// achieve better throughput and compatibility with both stations and APs.
    #[builder_lite(unstable)]
    static_rx_buf_num: u8,

    /// Max number of Wi-Fi dynamic RX buffers
    ///
    /// Set the number of Wi-Fi dynamic RX buffers, 0 means unlimited RX buffers will be allocated
    /// (provided sufficient free RAM). The size of each dynamic RX buffer depends on the size of
    /// the received data frame.
    ///
    /// For each received data frame, the Wi-Fi driver makes a copy to an RX buffer and then
    /// delivers it to the high layer TCP/IP stack. The dynamic RX buffer is freed after the
    /// higher layer has successfully received the data frame.
    ///
    /// For some applications, Wi-Fi data frames may be received faster than the application can
    /// process them. In these cases we may run out of memory if RX buffer number is unlimited
    /// (0).
    ///
    /// If a dynamic RX buffer limit is set, it should be at least the number of
    /// static RX buffers.
    #[builder_lite(unstable)]
    dynamic_rx_buf_num: u16,

    /// Set the number of Wi-Fi static TX buffers.
    ///
    /// Each buffer takes approximately 1.6KB of RAM.
    /// The static RX buffers are allocated when esp_wifi_init() is called, they are not released
    /// until esp_wifi_deinit() is called.
    ///
    /// For each transmitted data frame from the higher layer TCP/IP stack, the Wi-Fi driver makes
    /// a copy of it in a TX buffer.
    ///
    /// For some applications especially UDP applications, the upper layer can deliver frames
    /// faster than Wi-Fi layer can transmit. In these cases, we may run out of TX buffers.
    #[builder_lite(unstable)]
    static_tx_buf_num: u8,

    /// Set the number of Wi-Fi dynamic TX buffers.
    ///
    /// The size of each dynamic TX buffer is not fixed,
    /// it depends on the size of each transmitted data frame.
    ///
    /// For each transmitted frame from the higher layer TCP/IP stack, the Wi-Fi driver makes a
    /// copy of it in a TX buffer.
    ///
    /// For some applications, especially UDP applications, the upper layer can deliver frames
    /// faster than Wi-Fi layer can transmit. In these cases, we may run out of TX buffers.
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

    /// Set the size of Wi-Fi Block Ack RX window.
    ///
    /// Generally a bigger value means higher throughput and better compatibility but more memory.
    /// Most of time we should NOT change the default value unless special reason, e.g. test
    /// the maximum UDP RX throughput with iperf etc. For iperf test in shieldbox, the
    /// recommended value is 9~12.
    ///
    /// If PSRAM is used and Wi-Fi memory is preferred to allocate in PSRAM first, the default and
    /// minimum value should be 16 to achieve better throughput and compatibility with both
    /// stations and APs.
    #[builder_lite(unstable)]
    rx_ba_win: u8,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
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

            country_info: CountryInfo::from(*b"CN"),
        }
    }
}

impl ControllerConfig {
    fn validate(&self) {
        if self.rx_ba_win as u16 >= self.dynamic_rx_buf_num {
            warn!("RX BA window size should be less than the number of dynamic RX buffers.");
        }
        if self.rx_ba_win as u16 >= 2 * (self.static_rx_buf_num as u16) {
            warn!("RX BA window size should be less than twice the number of static RX buffers.");
        }
    }
}

#[procmacros::doc_replace]
/// Create a Wi-Fi controller and it's associated interfaces.
///
/// Dropping the controller will deinitialize / stop Wi-Fi.
///
/// Make sure to **not** call this function while interrupts are disabled, or IEEE 802.15.4 is
/// currently in use.
///
/// ## Example
///
/// ```rust,no_run
/// # {before_snippet}
/// let (controller, interfaces) =
///     esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();
/// # {after_snippet}
/// ```
pub fn new<'d>(
    device: crate::hal::peripherals::WIFI<'d>,
    config: ControllerConfig,
) -> Result<(WifiController<'d>, Interfaces<'d>), WifiError> {
    let _guard = RadioRefGuard::new();

    // TODO: Re-check, if not having interrupts disabled pre-condition is still true
    if crate::is_interrupts_disabled() {
        return Err(WifiError::Unsupported);
    }

    config.validate();

    event::enable_wifi_events(
        WifiEvent::StationStart
            | WifiEvent::StationStop
            | WifiEvent::StationConnected
            | WifiEvent::StationDisconnected
            | WifiEvent::AccessPointStart
            | WifiEvent::AccessPointStop
            | WifiEvent::AccessPointStationConnected
            | WifiEvent::AccessPointStationDisconnected
            | WifiEvent::ScanDone,
    );

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

    // At some point the "High-speed ADC" entropy source became available.
    #[cfg(not(esp32c5))]
    unsafe {
        esp_hal::rng::TrngSource::increase_entropy_source_counter()
    };

    // Only create WifiController after we've enabled TRNG - otherwise returning an error from this
    // function will cause panic because WifiController::drop tries to disable the TRNG.
    let mut controller = WifiController {
        _guard,
        _phantom: Default::default(),
        beacon_timeout: 6,
        ap_beacon_timeout: 100,
    };

    controller.set_country_info(&config.country_info)?;
    // Set a sane default power saving mode. The blob default is not the best for bandwidth.
    controller.set_power_saving(PowerSaveMode::default())?;

    Ok((
        controller,
        Interfaces {
            station: Interface {
                _phantom: Default::default(),
                mode: InterfaceType::Station,
            },
            access_point: Interface {
                _phantom: Default::default(),
                mode: InterfaceType::AccessPoint,
            },
            #[cfg(all(feature = "esp-now", feature = "unstable"))]
            esp_now: crate::esp_now::EspNow::new_internal(),
            #[cfg(all(feature = "sniffer", feature = "unstable"))]
            sniffer: Sniffer::new(),
        },
    ))
}

/// Wi-Fi controller.
///
/// After initial creation via [new] the controller is in stopped state.
/// Use [WifiController::set_config] to set the configuration and start the controller.
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
        state::locked(|| {
            if let Err(e) = crate::wifi::wifi_deinit() {
                warn!("Failed to cleanly deinit wifi: {:?}", e);
            }

            set_access_point_state(WifiAccessPointState::Uninitialized);
            set_station_state(WifiStationState::Uninitialized);

            #[cfg(not(esp32c5))]
            esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
                esp_hal::Internal::conjure()
            });
        })
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
    /// This will set the desired protocols.
    ///
    /// # Arguments:
    ///
    /// * `protocols` - The desired protocols
    ///
    /// # Example:
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{ap::AccessPointConfig, Config};
    /// use esp_radio::wifi::Protocols;
    ///
    /// let (mut wifi_controller, _interfaces) =
    ///     esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    ///
    /// wifi_controller.set_config(&Config::AccessPoint(
    ///     AccessPointConfig::default().with_ssid("esp-radio"),
    /// ))?;
    ///
    /// wifi_controller.set_protocols(Protocols::default());
    /// # {after_snippet}
    /// ```
    ///
    /// # Note
    ///
    /// Calling this function before `set_config` will return an error.
    #[instability::unstable]
    pub fn set_protocols(&mut self, protocols: Protocols) -> Result<(), WifiError> {
        let mode = self.mode()?;
        if mode.is_station() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocols(wifi_interface_t_WIFI_IF_STA, &mut protocols.to_raw())
            })?;
        }
        if mode.is_access_point() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocols(wifi_interface_t_WIFI_IF_AP, &mut protocols.to_raw())
            })?;
        }

        Ok(())
    }

    fn apply_protocols(iface: wifi_interface_t, protocols: &Protocols) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_protocols(iface, &mut protocols.to_raw()) })?;
        Ok(())
    }

    #[procmacros::doc_replace]
    /// Configures modem power saving.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::PowerSaveMode;
    /// let (mut controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// controller.set_power_saving(PowerSaveMode::Maximum)?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn set_power_saving(&mut self, ps: PowerSaveMode) -> Result<(), WifiError> {
        apply_power_saving(ps)
    }

    fn set_country_info(&mut self, country: &CountryInfo) -> Result<(), WifiError> {
        unsafe {
            let country = country.into_blob();
            esp_wifi_result!(esp_wifi_set_country(&country))?;
        }
        Ok(())
    }

    #[procmacros::doc_replace]
    /// Get the RSSI information of access point to which the device is associated with.
    /// The value is obtained from the last beacon.
    ///
    /// <div class="warning">
    ///
    /// - Use this API only in Station or AccessPoint-Station mode.
    /// - This API should be called after the station has connected to an access point.
    /// </div>
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # let (controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// // Assume the station has already been started and connected
    /// match controller.rssi() {
    ///     Ok(rssi) => {
    ///         println!("RSSI: {} dBm", rssi);
    ///     }
    ///     Err(e) => {
    ///         println!("Failed to get RSSI: {e:?}");
    ///     }
    /// }
    /// # {after_snippet}
    /// ```
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

    #[procmacros::doc_replace]
    /// Get the Access Point information of access point to which the device is associated with.
    /// The value is obtained from the last beacon.
    ///
    /// <div class="warning">
    ///
    /// - Use this API only in Station or AccessPoint-Station mode.
    /// - This API should be called after the station has connected to an access point.
    /// </div>
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # let (controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// // Assume the station has already been started and connected
    /// match controller.ap_info() {
    ///     Ok(info) => {
    ///         println!("BSSID: {}", info.bssid);
    ///     }
    ///     Err(e) => {
    ///         println!("Failed to get AP info: {e:?}");
    ///     }
    /// }
    /// # {after_snippet}
    /// ```
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

    #[procmacros::doc_replace]
    /// Set the configuration and (re)start the controller as needed.
    ///
    /// This will set the mode accordingly.
    /// You need to use [`Self::connect_async`] for connecting to an access point.
    ///
    /// If you don't intend to use Wi-Fi anymore at all consider tearing down
    /// Wi-Fi completely.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{Config, sta::StationConfig};
    /// # let (mut controller, _interfaces) =
    /// #    esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// let station_config = Config::Station(
    ///     StationConfig::default()
    ///         .with_ssid("SSID")
    ///         .with_password("PASSWORD".into()),
    /// );
    ///
    /// controller.set_config(&station_config)?;
    /// # {after_snippet}
    pub fn set_config(&mut self, conf: &Config) -> Result<(), WifiError> {
        conf.validate()?;

        let mut previous_mode = 0u32;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut previous_mode) })?;

        let mode = match conf {
            Config::Station(_) => wifi_mode_t_WIFI_MODE_STA,
            Config::AccessPoint(_) => wifi_mode_t_WIFI_MODE_AP,
            Config::AccessPointStation(_, _) => wifi_mode_t_WIFI_MODE_APSTA,
            #[cfg(feature = "wifi-eap")]
            Config::EapStation(_) => wifi_mode_t_WIFI_MODE_STA,
        };

        if previous_mode != mode {
            self.stop_impl()?;
        }

        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode) })?;

        match conf {
            Config::Station(config) => {
                self.apply_sta_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &config.protocols)
            }
            Config::AccessPoint(config) => {
                self.apply_ap_config(config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &config.protocols)
            }
            Config::AccessPointStation(sta_config, ap_config) => {
                self.apply_ap_config(ap_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_AP, &ap_config.protocols)?;
                self.apply_sta_config(sta_config)?;
                Self::apply_protocols(wifi_interface_t_WIFI_IF_STA, &sta_config.protocols)
            }
            #[cfg(feature = "wifi-eap")]
            Config::EapStation(config) => {
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

        if previous_mode != mode {
            set_access_point_state(WifiAccessPointState::Starting);
            set_station_state(WifiStationState::Starting);

            // `esp_wifi_start` is actually not async - i.e. we get the even before it returns
            let res = esp_wifi_result!(unsafe { esp_wifi_start() });
            if res.is_err() {
                unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL) };
                return res;
            }
        }

        Ok(())
    }

    #[cfg_attr(docsrs, procmacros::doc_replace(
        "hint_5g" => {
            cfg(wifi_has_5g) => "
When the WiFi band mode is set to [BandMode::_5G], it operates exclusively on the 5GHz channels.

When the WiFi band mode is set to [BandMode::Auto], it can operate on both the 2.4GHz and
5GHz channels.

When a WiFi band mode change triggers a band change, if no channel is set for the current
band, a default channel will be assigned: channel 1 for 2.4G band and channel 36 for 5G
band.

When a WiFi band mode change triggers a band change, if no channel is set for the current
band, a default channel will be assigned: channel 1 for 2.4G band and channel 36 for 5G
band.
            ",
            _ => ""
        },
    ))]
    /// Set WiFi band mode.
    ///
    /// When the WiFi band mode is set to [BandMode::_2_4G], it operates exclusively on the 2.4GHz
    /// channels.
    ///
    /// # {hint_5g}
    ///
    /// The controller needs to be configured and started before setting the band mode.
    #[instability::unstable]
    pub fn set_band_mode(&mut self, band_mode: BandMode) -> Result<(), WifiError> {
        // Wi-Fi needs to be started in order to set the band mode
        if !self.is_started() {
            return Err(WifiError::NotStarted);
        }

        esp_wifi_result!(unsafe { esp_wifi_set_band_mode(band_mode.to_raw()) })
    }

    /// Sets the Wi-Fi channel bandwidth for the currently active interface(s).
    ///
    /// If the device is operating in station mode, the bandwidth is applied to the
    /// Station interface. If operating in access point mode, it is applied to the Access Point
    /// interface. In Station+Access Point mode, the bandwidth is set for both interfaces.
    #[instability::unstable]
    pub fn set_bandwidths(&mut self, bandwidths: Bandwidths) -> Result<(), WifiError> {
        let mode = self.mode()?;
        if mode.is_station() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_bandwidths(wifi_interface_t_WIFI_IF_STA, &mut bandwidths.to_raw())
            })?;
        }
        if mode.is_access_point() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_bandwidths(wifi_interface_t_WIFI_IF_AP, &mut bandwidths.to_raw())
            })?;
        }

        Ok(())
    }

    /// Returns the Wi-Fi channel bandwidth of the active interface.
    ///
    /// If the device is operating in station mode, the bandwidth of the Station
    /// interface is returned. If operating in access point mode, the bandwidth
    /// of the Access Point interface is returned. In Station+Access Point mode, the bandwidth of
    /// the Access Point interface is returned.
    #[instability::unstable]
    pub fn bandwidths(&self) -> Result<Bandwidths, WifiError> {
        let mut bw = wifi_bandwidths_t {
            ghz_2g: 0,
            ghz_5g: 0,
        };

        let mode = self.mode()?;
        if mode.is_station() {
            esp_wifi_result!(unsafe {
                esp_wifi_get_bandwidths(wifi_interface_t_WIFI_IF_STA, &mut bw)
            })?;
        }
        if mode.is_access_point() {
            esp_wifi_result!(unsafe {
                esp_wifi_get_bandwidths(wifi_interface_t_WIFI_IF_AP, &mut bw)
            })?;
        }

        Ok(Bandwidths {
            _2_4: Bandwidth::from_raw(bw.ghz_2g),
            #[cfg(wifi_has_5g)]
            _5: Bandwidth::from_raw(bw.ghz_5g),
        })
    }

    /// Returns the current Wi-Fi channel configuration.
    #[instability::unstable]
    pub fn channel(&self) -> Result<(u8, SecondaryChannel), WifiError> {
        let mut primary = 0;
        let mut secondary = 0;

        esp_wifi_result!(unsafe { esp_wifi_get_channel(&mut primary, &mut secondary) })?;

        Ok((primary, SecondaryChannel::from_raw(secondary)))
    }

    #[cfg_attr(docsrs, procmacros::doc_replace(
        "hint_5g" => {
            cfg(wifi_has_5g) => "
When operating in 5 GHz band, the second channel is automatically determined by the primary
channel according to the 802.11 standard. Any manually configured second channel will be
ignored.",
            _ => ""
        },
    ))]
    /// Sets the primary and secondary Wi-Fi channel.
    ///
    /// # {hint_5g}
    #[instability::unstable]
    pub fn set_channel(
        &mut self,
        primary: u8,
        secondary: SecondaryChannel,
    ) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_channel(primary, secondary as u32) })?;

        Ok(())
    }

    /// Set maximum transmitting power after WiFi start.
    ///
    /// Power unit is 0.25dBm, range is [8, 84] corresponding to 2dBm - 20dBm.
    #[instability::unstable]
    pub fn set_max_tx_power(&mut self, power: i8) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_max_tx_power(power) })
    }

    fn stop_impl(&mut self) -> Result<(), WifiError> {
        set_access_point_state(WifiAccessPointState::Stopping);
        set_station_state(WifiStationState::Stopping);

        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    fn connect_impl(&mut self) -> Result<(), WifiError> {
        set_station_state(WifiStationState::Connecting);

        // TODO: implement ROAMING
        esp_wifi_result!(unsafe { esp_wifi_connect_internal() })
    }

    fn disconnect_impl(&mut self) -> Result<(), WifiError> {
        set_station_state(WifiStationState::Disconnecting);

        // TODO: implement ROAMING
        esp_wifi_result!(unsafe { esp_wifi_disconnect_internal() })
    }

    #[procmacros::doc_replace]
    /// Checks if the Wi-Fi controller is started. Returns true if Station and/or AccessPoint are
    /// started.
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # let (controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// if controller.is_started() {
    ///     println!("Wi-Fi is started");
    /// } else {
    ///     println!("Wi-Fi is not started");
    /// }
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn is_started(&self) -> bool {
        if matches!(
            crate::wifi::station_state(),
            WifiStationState::Started
                | WifiStationState::Connected
                | WifiStationState::Disconnected
        ) {
            return true;
        }
        if matches!(
            crate::wifi::access_point_state(),
            WifiAccessPointState::Started
        ) {
            return true;
        }
        false
    }

    #[procmacros::doc_replace]
    /// Checks if the Wi-Fi controller is currently connected to an access point.
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::WifiError;
    /// # let (controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// if controller.is_connected() {
    ///     println!("Station is connected");
    /// } else {
    ///     println!("Station is not connected yet");
    /// }
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn is_connected(&self) -> bool {
        matches!(
            crate::wifi::station_state(),
            crate::wifi::WifiStationState::Connected
        )
    }

    fn mode(&self) -> Result<WifiMode, WifiError> {
        WifiMode::current()
    }

    #[procmacros::doc_replace]
    /// An async Wi-Fi network scan with caller-provided scanning options.
    ///
    /// Scanning is not supported in AcessPoint-only mode.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{WifiController, scan::ScanConfig};
    /// # let (mut controller, _interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// // Create a scan configuration (e.g., scan up to 10 APs)
    /// let scan_config = ScanConfig::default().with_max(10);
    /// let result = controller
    ///     .scan_async(&scan_config)
    ///     .await
    ///     .unwrap();
    /// for ap in result {
    ///     println!("{:?}", ap);
    /// }
    /// # {after_snippet}
    /// ```
    pub async fn scan_async(
        &mut self,
        config: &ScanConfig,
    ) -> Result<Vec<AccessPointInfo>, WifiError> {
        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing the internal event channel subscriber count");

        esp_wifi_result!(wifi_start_scan(false, *config))?;

        // Prevents memory leak if `scan_async`'s future is dropped.
        let guard = FreeApListOnDrop;

        loop {
            let event = subscriber.next_message_pure().await;
            if let EventInfo::ScanDone {
                status: _status,
                number: _number,
                scan_id: _scan_id,
            } = event
            {
                break;
            }
        }

        guard.defuse();

        Ok(ScanResults::new(self)?.collect::<Vec<_>>())
    }

    #[procmacros::doc_replace]
    /// Connect Wi-Fi station to the AP.
    ///
    /// Use [Self::disconnect_async] to disconnect.
    ///
    /// Calling [Self::scan_async] will not be effective until
    /// connection between device and the AP is established.
    ///
    /// If device is scanning and connecting at the same time, it will abort scanning and return a
    /// warning message and error.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{Config, sta::StationConfig};
    ///
    /// # let (mut controller, _interfaces) =
    /// #   esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    ///
    /// match controller.connect_async().await {
    ///     Ok(_) => {
    ///         println!("Wifi connected!");
    ///     }
    ///     Err(e) => {
    ///       println!("Failed to connect to wifi: {e:?}");
    ///     }
    /// }
    /// # {after_snippet}
    pub async fn connect_async(&mut self) -> Result<ConnectedStationInfo, WifiError> {
        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing the internal event channel subscriber count");

        self.connect_impl()?;

        let result = loop {
            let event = subscriber.next_message().await;
            if let embassy_sync::pubsub::WaitResult::Message(event) = event {
                match event {
                    EventInfo::StationConnected { .. } => {
                        break event;
                    }
                    EventInfo::StationDisconnected { .. } => {
                        break event;
                    }
                    _ => (),
                }
            }
        };

        match result {
            event::EventInfo::StationConnected {
                ssid,
                bssid,
                channel,
                authmode,
                aid,
            } => Ok(ConnectedStationInfo {
                ssid,
                bssid,
                channel,
                authmode: AuthenticationMethod::from_raw(authmode),
                aid,
            }),
            event::EventInfo::StationDisconnected {
                ssid,
                bssid,
                reason,
                rssi,
            } => Err(WifiError::Disconnected(DisconnectedStationInfo {
                ssid,
                bssid,
                reason: DisconnectReason::from_raw(reason),
                rssi,
            })),
            _ => unreachable!(),
        }
    }

    #[procmacros::doc_replace]
    /// Disconnect Wi-Fi station from the AP.
    ///
    /// This function will wait for the connection to be closed before returning.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::{Config, sta::StationConfig};
    ///
    /// # let (mut controller, _interfaces) =
    /// #    esp_radio::wifi::new(peripherals.WIFI, Default::default())?;
    /// match controller.disconnect_async().await {
    ///     Ok(info) => {
    ///         println!("Station disconnected successfully. {info:?}");
    ///     }
    ///     Err(e) => {
    ///         println!("Failed to disconnect: {e:?}");
    ///     }
    /// }
    /// # {after_snippet}
    pub async fn disconnect_async(&mut self) -> Result<DisconnectedStationInfo, WifiError> {
        // If not connected it would wait forever for a `StationDisconnected` event that will never
        // happen. Return early instead of hanging.
        if !self.is_connected() {
            return Err(WifiError::NotConnected);
        }

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing the internal event channel subscriber count");

        self.disconnect_impl()?;

        loop {
            let event = subscriber.next_message_pure().await;

            if let event::EventInfo::StationDisconnected {
                ssid,
                bssid,
                reason,
                rssi,
            } = event
            {
                break Ok(DisconnectedStationInfo {
                    ssid,
                    bssid,
                    reason: DisconnectReason::from_raw(reason),
                    rssi,
                });
            }
        }
    }

    /// Wait until the station gets disconnected from the AP.
    pub async fn wait_for_disconnect_async(&self) -> Result<DisconnectedStationInfo, WifiError> {
        // If not connected it would wait forever for a `StationDisconnected` event that will never
        // happen. Return early instead of hanging.
        if !self.is_connected() {
            return Err(WifiError::NotConnected);
        }

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing the internal event channel subscriber count");

        loop {
            let event = subscriber.next_message_pure().await;

            if let event::EventInfo::StationDisconnected {
                ssid,
                bssid,
                reason,
                rssi,
            } = event
            {
                break Ok(DisconnectedStationInfo {
                    ssid,
                    bssid,
                    reason: DisconnectReason::from_raw(reason),
                    rssi,
                });
            }
        }
    }

    /// Wait for connected / disconnected events.
    pub async fn wait_for_access_point_connected_event_async(
        &self,
    ) -> Result<AccessPointStationEventInfo, WifiError> {
        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing the internal event channel subscriber count");

        loop {
            let event = subscriber.next_message_pure().await;

            match event {
                event::EventInfo::AccessPointStationConnected {
                    mac,
                    aid,
                    is_mesh_child,
                } => {
                    break Ok(AccessPointStationEventInfo::Connected(
                        AccessPointStationConnectedInfo {
                            mac,
                            aid,
                            is_mesh_child,
                        },
                    ));
                }
                event::EventInfo::AccessPointStationDisconnected {
                    mac,
                    aid,
                    is_mesh_child,
                    reason,
                } => {
                    break Ok(AccessPointStationEventInfo::Disconnected(
                        AccessPointStationDisconnectedInfo {
                            mac,
                            aid: aid as u16,
                            is_mesh_child,
                            reason: DisconnectReason::from_raw(reason),
                        },
                    ));
                }
                _ => (),
            }
        }
    }

    /// Subscribe to events.
    ///
    /// # Errors
    /// This returns [WifiError::Failed] if no more subscriptions are available.
    /// Consider increasing the internal event channel subscriber count in this case.
    #[instability::unstable]
    pub fn subscribe<'a>(&'a self) -> Result<event::EventSubscriber<'a>, WifiError> {
        if let Ok(subscriber) = EVENT_CHANNEL.subscriber() {
            return Ok(event::EventSubscriber::new(subscriber));
        }

        Err(WifiError::Failed)
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

        if config.auth_method == AuthenticationMethod::None && !config.password.is_empty() {
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

        if config.auth_method == AuthenticationMethod::None && !config.password.is_empty() {
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
