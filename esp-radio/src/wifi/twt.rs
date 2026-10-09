//! Target Wake Time (TWT) types and configuration.
//!
//! TWT is a Wi-Fi 6 (802.11ax) feature that allows a station to negotiate
//! wake intervals with an access point, enabling significant power savings.
//!
//! This module provides types for configuring individual TWT (iTWT) agreements.
//!
//! # Packet buffering
//!
//! When TWT is active, WiFi firmware buffers outgoing packets and only transmits
//! them during TWT wake windows. Sending packets outside a wake window does
//! **not** immediately wake up the modem -- the packet is held until the next
//! scheduled window.
//!
//! For best latency, send packets in response to
//! [`WifiController::wait_for_next_twt_wakeup`](crate::wifi::WifiController::wait_for_next_twt_wakeup)
//! events. An independent timer can desync from TWT wakeups, causing
//! packets to miss their window.
//!
//! # Event subscribers
//!
//! The async TWT methods on
//! [`WifiController`](crate::wifi::WifiController)
//! wait for driver events. Each pending call holds one event channel
//! subscriber and needs queue capacity in the event channel. Concurrent
//! waiters (or other tasks listening for events) require increasing the
//! `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
//! `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options; running out
//! of subscribers panics. The calls do not apply timeouts of their own.
//!
//! # Multiple flows
//!
//! Up to 8 simultaneous iTWT flows (flow IDs 0–7) can be active.
//! Whenever any flow's wake window opens, **all** buffered packets are flushed.

use core::fmt;

use docsplay::Display;
use enumset::{EnumSet, EnumSetType};
use esp_hal::time::Duration;
use portable_atomic::Ordering;
use procmacros::BuilderLite;

use super::{WifiController, WifiError, WifiEvent, esp_wifi_result, event, event::EVENT_CHANNEL};
use crate::sys::include::{
    wifi_itwt_probe_status_t,
    wifi_itwt_probe_status_t_ITWT_PROBE_FAIL,
    wifi_itwt_probe_status_t_ITWT_PROBE_STA_DISCONNECTED,
    wifi_itwt_probe_status_t_ITWT_PROBE_SUCCESS,
    wifi_itwt_probe_status_t_ITWT_PROBE_TIMEOUT,
    wifi_itwt_teardown_status_t,
    wifi_itwt_teardown_status_t_ITWT_TEARDOWN_FAIL,
    wifi_itwt_teardown_status_t_ITWT_TEARDOWN_SUCCESS,
    wifi_twt_config_t,
    wifi_twt_setup_cmds_t,
    wifi_twt_setup_cmds_t_TWT_ACCEPT,
    wifi_twt_setup_cmds_t_TWT_ALTERNATE,
    wifi_twt_setup_cmds_t_TWT_DEMAND,
    wifi_twt_setup_cmds_t_TWT_DICTATE,
    wifi_twt_setup_cmds_t_TWT_GROUPING,
    wifi_twt_setup_cmds_t_TWT_REJECT,
    wifi_twt_setup_cmds_t_TWT_REQUEST,
    wifi_twt_setup_cmds_t_TWT_SUGGEST,
    wifi_twt_setup_config_t,
    wifi_twt_type_t,
    wifi_twt_type_t_TWT_TYPE_BROADCAST,
    wifi_twt_type_t_TWT_TYPE_INDIVIDUAL,
};

/// TWT setup command type.
///
/// Indicates the type of TWT command used during negotiation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TwtSetupCommand {
    /// Station requests a TWT agreement.
    Request   = 0,
    /// Station suggests TWT parameters.
    Suggest   = 1,
    /// Station demands specific TWT parameters.
    Demand    = 2,
    /// TWT grouping.
    Grouping  = 3,
    /// AP accepts the TWT agreement.
    Accept    = 4,
    /// AP suggests alternate TWT parameters.
    Alternate = 5,
    /// AP dictates TWT parameters.
    Dictate   = 6,
    /// AP rejects the TWT agreement.
    Reject    = 7,
}

impl TwtSetupCommand {
    #[allow(non_upper_case_globals)]
    fn from_raw(val: wifi_twt_setup_cmds_t) -> Self {
        match val {
            wifi_twt_setup_cmds_t_TWT_REQUEST => Self::Request,
            wifi_twt_setup_cmds_t_TWT_SUGGEST => Self::Suggest,
            wifi_twt_setup_cmds_t_TWT_DEMAND => Self::Demand,
            wifi_twt_setup_cmds_t_TWT_GROUPING => Self::Grouping,
            wifi_twt_setup_cmds_t_TWT_ACCEPT => Self::Accept,
            wifi_twt_setup_cmds_t_TWT_ALTERNATE => Self::Alternate,
            wifi_twt_setup_cmds_t_TWT_DICTATE => Self::Dictate,
            wifi_twt_setup_cmds_t_TWT_REJECT => Self::Reject,
            _ => panic!("Invalid TWT setup command: {}", val),
        }
    }
}

/// TWT flow type.
///
/// Determines whether the station must announce its wakeup to the AP.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TwtFlowType {
    /// The station sends a trigger frame to the AP when it wakes up.
    Announced   = 0,
    /// The station does not need to announce its wakeup.
    Unannounced = 1,
}

/// Unit for the minimum wake duration field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TwtWakeDurationUnit {
    /// 256 microseconds per unit.
    Us256 = 0,
    /// 1 TU (1024 microseconds) per unit.
    Tu    = 1,
}

/// A TWT flow identifier (0-7).
///
/// Each individual TWT agreement is identified by a flow ID in the range 0-7.
/// A [`FlowId`] always refers to exactly one flow; use [`FlowTarget`] to
/// address "all flows" in requests.
#[derive(Debug, EnumSetType, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[enumset(repr = "u8")]
pub enum FlowId {
    /// Flow 0.
    Flow0 = 0,
    /// Flow 1.
    Flow1 = 1,
    /// Flow 2.
    Flow2 = 2,
    /// Flow 3.
    Flow3 = 3,
    /// Flow 4.
    Flow4 = 4,
    /// Flow 5.
    Flow5 = 5,
    /// Flow 6.
    Flow6 = 6,
    /// Flow 7.
    Flow7 = 7,
}

impl FlowId {
    /// Return the numeric flow ID (0-7).
    #[instability::unstable]
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    pub(crate) fn from_raw(val: u8) -> Self {
        match val {
            0 => Self::Flow0,
            1 => Self::Flow1,
            2 => Self::Flow2,
            3 => Self::Flow3,
            4 => Self::Flow4,
            5 => Self::Flow5,
            6 => Self::Flow6,
            7 => Self::Flow7,
            _ => panic!("Invalid TWT flow ID: {}", val),
        }
    }
}

/// Raw flow ID meaning "all flows" in teardown/suspend operations and events.
pub(crate) const FLOW_ID_ALL: u8 = 8;

/// Target for a TWT operation — either a specific flow or all active flows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum FlowTarget {
    /// A single flow identified by its [`FlowId`].
    Id(FlowId),
    /// All active flows.
    All,
}

impl From<FlowId> for FlowTarget {
    fn from(id: FlowId) -> Self {
        FlowTarget::Id(id)
    }
}

impl FlowTarget {
    pub(crate) fn as_raw(self) -> u8 {
        match self {
            FlowTarget::Id(id) => id.as_u8(),
            FlowTarget::All => FLOW_ID_ALL,
        }
    }
}

#[procmacros::doc_replace]
/// Configuration for an individual TWT (iTWT) setup.
///
/// The recommended way to create a configuration is to start from
/// [`ITwtSetupConfig::default`] and use the builder methods:
///
/// ```rust,no_run
/// # {before_snippet}
/// # use core::default::Default;
/// # use esp_hal::time::Duration;
/// # use esp_radio::wifi::twt::*;
/// let config = ITwtSetupConfig::default()
///     .with_setup_cmd(TwtSetupCommand::Request)
///     .with_trigger(true)
///     .with_flow_type(TwtFlowType::Announced)
///     .with_wake_interval(Duration::from_millis(20))
///     .with_min_wake_duration(Duration::from_micros(2048))
///     .with_timeout(Duration::from_secs(5));
/// # {after_snippet}
/// ```
#[derive(BuilderLite, Clone, Copy, PartialEq, Eq, Hash)]
#[instability::unstable]
pub struct ITwtSetupConfig {
    /// The type of TWT command.
    pub setup_cmd: TwtSetupCommand,
    /// Whether this is a trigger-enabled TWT.
    pub trigger: bool,
    /// The flow type (announced or unannounced).
    pub flow_type: TwtFlowType,
    /// The flow ID.
    ///
    /// The value in the request is typically ignored.
    /// The actual flow ID assigned by the AP must be obtained from the response.
    #[builder_lite(skip_setter)]
    pub flow_id: FlowId,
    /// Internal correlation ID used to match setup responses to requests.
    ///
    /// Automatically assigned by
    /// [`WifiController::itwt_setup`](crate::wifi::WifiController::itwt_setup).
    #[builder_lite(skip_setter)]
    pub twt_id: u16,
    /// The wake interval exponent (set via
    /// [`with_wake_interval`](Self::with_wake_interval)).
    #[builder_lite(skip_setter)]
    pub wake_interval_exponent: u8,
    /// The unit for the minimum wake duration (set via
    /// [`with_min_wake_duration`](Self::with_min_wake_duration)).
    #[builder_lite(skip_setter)]
    pub wake_duration_unit: TwtWakeDurationUnit,
    /// Nominal minimum wake duration in units of
    /// [`wake_duration_unit`](Self::wake_duration_unit) (set via
    /// [`with_min_wake_duration`](Self::with_min_wake_duration)).
    #[builder_lite(skip_setter)]
    pub min_wake_duration: u8,
    /// Individual TWT wake interval mantissa (set via
    /// [`with_wake_interval`](Self::with_wake_interval)).
    #[builder_lite(skip_setter)]
    pub wake_interval_mantissa: u16,
    /// Timeout for receiving the setup action frame response.
    pub timeout: Duration,
}

impl Default for ITwtSetupConfig {
    fn default() -> Self {
        Self {
            setup_cmd: TwtSetupCommand::Request,
            trigger: true,
            flow_type: TwtFlowType::Announced,
            flow_id: FlowId::Flow0,
            twt_id: 0,
            wake_interval_exponent: 0,
            wake_duration_unit: TwtWakeDurationUnit::Us256,
            min_wake_duration: 1,
            wake_interval_mantissa: 1,
            timeout: Duration::from_millis(5000),
        }
    }
}

impl fmt::Debug for ITwtSetupConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ITwtSetupConfig")
            .field("setup_cmd", &self.setup_cmd)
            .field("trigger", &self.trigger)
            .field("flow_type", &self.flow_type)
            .field("flow_id", &self.flow_id)
            .field("wake_interval", &self.wake_interval())
            .field("wake_duration", &self.wake_duration())
            .field("timeout", &self.timeout)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ITwtSetupConfig {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "ITwtSetupConfig {{ \
            setup_cmd: {}, trigger: {}, flow_type: {}, flow_id: {}, \
            wake_interval: {}, wake_duration: {}, \
            timeout: {} }}",
            self.setup_cmd,
            self.trigger,
            self.flow_type,
            self.flow_id,
            self.wake_interval(),
            self.wake_duration(),
            self.timeout,
        );
    }
}

impl ITwtSetupConfig {
    /// Set the TWT wake interval.
    ///
    /// This is how often the station wakes up. The protocol encodes this as
    /// `mantissa * 2^exponent` microseconds; this method picks the optimal
    /// encoding automatically.
    ///
    /// The interval must be nonzero. Furthermore, very small or large values
    /// are rejected by the firmware as well.
    /// Some APs have also been found to silently mangle very large (hours) values.
    #[instability::unstable]
    pub fn with_wake_interval(mut self, interval: Duration) -> Self {
        let us = interval.as_micros();
        debug_assert!(us > 0, "wake interval must be > 0");

        let mut exponent = 0u8;
        let mut mantissa = us;
        while mantissa > 65535 && exponent < 31 {
            exponent += 1;
            mantissa = us >> exponent;
        }

        self.wake_interval_mantissa = mantissa as u16;
        self.wake_interval_exponent = exponent;
        self
    }

    /// Set the nominal minimum wake duration.
    ///
    /// This is the minimum time the station stays awake during each TWT
    /// service period. The protocol encodes this in 256µs or 1024µs units;
    /// this method picks the best unit automatically.
    ///
    /// The duration must be nonzero. Furthermore, very small or large values
    /// are rejected by the firmware as well.
    /// Some APs have also been found to silently mangle the unit, effectively limiting
    /// this to a maximum duration of approximately 65ms.
    #[instability::unstable]
    pub fn with_min_wake_duration(mut self, duration: Duration) -> Self {
        let us = duration.as_micros();
        debug_assert!(us > 0, "wake duration must be > 0");

        // Try 256µs units first (finer granularity)
        let units_256 = us.div_ceil(256);
        if units_256 <= 255 {
            self.wake_duration_unit = TwtWakeDurationUnit::Us256;
            self.min_wake_duration = units_256 as u8;
        } else {
            // Fall back to TU (1024µs) units
            let units_tu = us.div_ceil(1024);
            self.wake_duration_unit = TwtWakeDurationUnit::Tu;
            debug_assert!(
                units_tu <= 255,
                "wake duration exceeds maximum possible value"
            );
            self.min_wake_duration = units_tu.min(255) as u8;
        }
        self
    }

    /// The negotiated wake interval as a [`Duration`].
    #[instability::unstable]
    pub fn wake_interval(&self) -> Duration {
        let us = (self.wake_interval_mantissa as u64) << self.wake_interval_exponent;
        Duration::from_micros(us)
    }

    /// The nominal minimum wake duration as a [`Duration`].
    #[instability::unstable]
    pub fn wake_duration(&self) -> Duration {
        let us = match self.wake_duration_unit {
            TwtWakeDurationUnit::Us256 => self.min_wake_duration as u64 * 256,
            TwtWakeDurationUnit::Tu => self.min_wake_duration as u64 * 1024,
        };
        Duration::from_micros(us)
    }

    pub(crate) fn to_raw(self) -> wifi_twt_setup_config_t {
        debug_assert!(self.wake_interval_exponent <= 31);
        debug_assert!(self.min_wake_duration >= 1);
        debug_assert!(self.wake_interval_mantissa >= 1);
        let raw = wifi_twt_setup_config_t {
            setup_cmd: self.setup_cmd as wifi_twt_setup_cmds_t,
            _bitfield_align_1: Default::default(),
            _bitfield_1: wifi_twt_setup_config_t::new_bitfield_1(
                self.trigger as u16,
                self.flow_type as u16,
                self.flow_id.as_u8() as u16,
                self.wake_interval_exponent as u16,
                self.wake_duration_unit as u16,
                0, // reserved
            ),
            min_wake_dura: self.min_wake_duration,
            wake_invl_mant: self.wake_interval_mantissa,
            twt_id: self.twt_id,
            timeout_time_ms: self.timeout.as_millis() as u16,
        };
        trace!(
            "to_raw: setup_cmd={} trigger={} flow_type={} flow_id={} wake_invl_expn={} wake_duration_unit={} min_wake_dura={} wake_invl_mant={} twt_id={} timeout_time_ms={}",
            raw.setup_cmd,
            raw.trigger(),
            raw.flow_type(),
            raw.flow_id(),
            raw.wake_invl_expn(),
            raw.wake_duration_unit(),
            raw.min_wake_dura,
            raw.wake_invl_mant,
            raw.twt_id,
            raw.timeout_time_ms
        );
        raw
    }

    pub(crate) fn from_raw(raw: &wifi_twt_setup_config_t) -> Self {
        trace!(
            "from_raw: setup_cmd={} trigger={} flow_type={} flow_id={} wake_invl_expn={} wake_duration_unit={} min_wake_dura={} wake_invl_mant={} twt_id={} timeout_time_ms={}",
            raw.setup_cmd,
            raw.trigger(),
            raw.flow_type(),
            raw.flow_id(),
            raw.wake_invl_expn(),
            raw.wake_duration_unit(),
            raw.min_wake_dura,
            raw.wake_invl_mant,
            raw.twt_id,
            raw.timeout_time_ms
        );
        Self {
            setup_cmd: TwtSetupCommand::from_raw(raw.setup_cmd),
            trigger: raw.trigger() != 0,
            flow_type: if raw.flow_type() != 0 {
                TwtFlowType::Unannounced
            } else {
                TwtFlowType::Announced
            },
            flow_id: FlowId::from_raw(raw.flow_id() as u8),
            twt_id: raw.twt_id,
            wake_interval_exponent: raw.wake_invl_expn() as u8,
            wake_duration_unit: if raw.wake_duration_unit() != 0 {
                TwtWakeDurationUnit::Tu
            } else {
                TwtWakeDurationUnit::Us256
            },
            min_wake_duration: raw.min_wake_dura,
            wake_interval_mantissa: raw.wake_invl_mant,
            timeout: Duration::from_millis(raw.timeout_time_ms as u64),
        }
    }
}

/// Status value used by the iTWT setup event to indicate success.
pub(crate) const ITWT_SETUP_SUCCESS: i32 = 1;

/// Status of an iTWT teardown operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ITwtTeardownStatus {
    /// Teardown failed (station failed to send teardown frame).
    Fail    = 0,
    /// Teardown succeeded (station sent teardown frame or received one from AP).
    Success = 1,
}

impl ITwtTeardownStatus {
    #[allow(non_upper_case_globals)]
    pub(crate) fn from_raw(val: wifi_itwt_teardown_status_t) -> Self {
        match val {
            wifi_itwt_teardown_status_t_ITWT_TEARDOWN_SUCCESS => Self::Success,
            wifi_itwt_teardown_status_t_ITWT_TEARDOWN_FAIL => Self::Fail,
            _ => panic!("Invalid iTWT teardown status: {}", val),
        }
    }
}

/// Status of an iTWT probe operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ITwtProbeStatus {
    /// Probe failed.
    Fail         = 0,
    /// Probe succeeded.
    Success      = 1,
    /// Probe timed out.
    Timeout      = 2,
    /// Station disconnected during probe.
    Disconnected = 3,
}

impl ITwtProbeStatus {
    #[allow(non_upper_case_globals)]
    pub(crate) fn from_raw(val: wifi_itwt_probe_status_t) -> Self {
        match val {
            wifi_itwt_probe_status_t_ITWT_PROBE_SUCCESS => Self::Success,
            wifi_itwt_probe_status_t_ITWT_PROBE_TIMEOUT => Self::Timeout,
            wifi_itwt_probe_status_t_ITWT_PROBE_STA_DISCONNECTED => Self::Disconnected,
            wifi_itwt_probe_status_t_ITWT_PROBE_FAIL => Self::Fail,
            _ => panic!("Invalid iTWT probe status: {}", val),
        }
    }
}

pub(crate) fn teardown_result(status: ITwtTeardownStatus) -> Result<(), WifiError> {
    match status {
        ITwtTeardownStatus::Success => Ok(()),
        ITwtTeardownStatus::Fail => Err(WifiError::Twt(TwtError::TeardownFailed)),
    }
}

pub(crate) fn probe_result(status: ITwtProbeStatus) -> Result<(), WifiError> {
    match status {
        ITwtProbeStatus::Success => Ok(()),
        ITwtProbeStatus::Fail => Err(WifiError::Twt(TwtError::ProbeFailed)),
        ITwtProbeStatus::Timeout => Err(WifiError::Twt(TwtError::ProbeTimeout)),
        ITwtProbeStatus::Disconnected => Err(WifiError::NotConnected),
    }
}

/// Errors specific to Target Wake Time (TWT) operations.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum TwtError {
    /// All TWT flow IDs are in use.
    Full,
    /// TWT setup action frame response timed out.
    SetupTimeout,
    /// TWT setup action frame transmission failed.
    SetupTxFail,
    /// TWT setup was rejected by the AP.
    SetupRejected,
    /// The station failed to send the teardown frame.
    TeardownFailed,
    /// Suspending the TWT agreement failed.
    SuspendFailed,
    /// The probe request failed.
    ProbeFailed,
    /// The probe request timed out.
    ProbeTimeout,
}

impl core::error::Error for TwtError {}

/// TWT type (individual or broadcast).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TwtType {
    /// Individual TWT agreement.
    Individual = 0,
    /// Broadcast TWT agreement.
    Broadcast  = 1,
}

impl TwtType {
    #[allow(non_upper_case_globals)]
    pub(crate) fn from_raw(val: wifi_twt_type_t) -> Self {
        match val {
            wifi_twt_type_t_TWT_TYPE_BROADCAST => Self::Broadcast,
            wifi_twt_type_t_TWT_TYPE_INDIVIDUAL => Self::Individual,
            _ => panic!("Invalid TWT type: {}", val),
        }
    }
}

/// General TWT configuration options.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct TwtConfig {
    /// Whether to post a wakeup event when TWT wakes up.
    post_wakeup_event: bool,
    /// Whether to send QoS Null frames to keep the connection alive.
    enable_keep_alive: bool,
}

impl TwtConfig {
    pub(crate) fn to_raw(self) -> wifi_twt_config_t {
        wifi_twt_config_t {
            post_wakeup_event: self.post_wakeup_event,
            twt_enable_keep_alive: self.enable_keep_alive,
        }
    }
}

/// Information about a TWT wakeup event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct TwtWakeupInfo {
    /// The TWT type (individual or broadcast).
    pub twt_type: TwtType,
    /// The flow ID that woke up.
    pub flow_id: FlowId,
}

/// Error returned by
/// [`WifiController::wait_for_next_twt_wakeup`](crate::wifi::WifiController::wait_for_next_twt_wakeup).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum TwtWaitError {
    /// An active TWT flow was torn down (by the station or the AP).
    FlowTornDown {
        /// The flow that was torn down.
        flow_id: FlowId,
        /// Teardown status.
        status: ITwtTeardownStatus,
    },
    /// The station disconnected from the AP.
    Disconnected,
}

impl fmt::Display for TwtWaitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FlowTornDown { flow_id, .. } => {
                write!(f, "TWT flow {:?} torn down", flow_id)
            }
            Self::Disconnected => write!(f, "station disconnected"),
        }
    }
}

impl core::error::Error for TwtWaitError {}

/// Information about a successfully negotiated iTWT agreement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct ITwtSetupInfo {
    /// The negotiated iTWT setup configuration (may differ from requested).
    pub config: ITwtSetupConfig,
    /// TWT service period start time.
    pub target_wake_time: u64,
}

/// Information about a failed iTWT setup.
///
/// `config` holds the AP's response (`setup_cmd` and the proposed
/// parameters), which can be used to re-request a setup with the AP's
/// parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct ITwtSetupFailedInfo {
    /// The configuration returned in the failure event.
    pub config: ITwtSetupConfig,
    /// Setup status code (non-1 value indicates failure).
    pub status: i32,
    /// Failure reason code.
    pub reason: u8,
}

/// Errors that can occur during an iTWT setup.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum ITwtSetupError {
    /// The AP responded with a non-success status.
    Rejected(ITwtSetupFailedInfo),

    /// A Wi-Fi error occurred.
    Wifi(WifiError),
}

impl From<WifiError> for ITwtSetupError {
    fn from(error: WifiError) -> Self {
        ITwtSetupError::Wifi(error)
    }
}

impl core::error::Error for ITwtSetupError {}

impl WifiController<'_> {
    #[procmacros::doc_replace]
    /// Negotiate an individual TWT (Target Wake Time) agreement with the AP.
    ///
    /// The AP may accept, modify, or reject the requested parameters. On
    /// success, returns [`ITwtSetupInfo`] with the negotiated configuration.
    /// Up to 8 simultaneous agreements are supported.
    ///
    /// Each call is assigned a unique `twt_id` for correlating the response.
    /// The call completes with [`ITwtSetupError::Rejected`] if the AP
    /// rejects the request and with [`WifiError::NotConnected`] (wrapped in
    /// [`ITwtSetupError::Wifi`]) if the station is or becomes
    /// disconnected while waiting. There is no timeout on the driver side.
    ///
    /// # Event subscribers
    ///
    /// Each pending call holds one event channel subscriber and relies on the
    /// event channel having enough queue capacity. Running several TWT calls
    /// concurrently (or alongside other tasks that wait for events) requires
    /// increasing the `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options.
    /// Running out of subscribers panics.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_hal::time::Duration;
    /// # use esp_radio::wifi::twt::ITwtSetupConfig;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// let setup = ITwtSetupConfig::default()
    ///     .with_wake_interval(Duration::from_millis(500))
    ///     .with_min_wake_duration(Duration::from_millis(65));
    ///
    /// let info = controller.itwt_setup(setup).await?;
    /// println!(
    ///     "flow_id={:?}, interval={:?}",
    ///     info.config.flow_id,
    ///     info.config.wake_interval()
    /// );
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub async fn itwt_setup(
        &self,
        mut config: ITwtSetupConfig,
    ) -> Result<ITwtSetupInfo, ITwtSetupError> {
        use portable_atomic::AtomicU16;
        /// Monotonic counter for assigning unique `twt_id` values to iTWT setup
        /// requests.
        static NEXT_TWT_ID: AtomicU16 = AtomicU16::new(0);
        const TWT_ID_MASK: u16 = 0x7FFF;

        event::enable_wifi_events(
            WifiEvent::IndividualTargetWakeTimeSetup | WifiEvent::StationDisconnected,
        );

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS (esp-radio config option `event_channel_subscribers`)");

        if !self.is_connected() {
            return Err(WifiError::NotConnected.into());
        }

        // ESP-IDF accepts a `twt_id` in [0, 32767]. Wrap within 15 bits.
        let twt_id = NEXT_TWT_ID.fetch_add(1, Ordering::Relaxed) & TWT_ID_MASK;
        config.twt_id = twt_id;

        let mut raw = config.to_raw();
        esp_wifi_result!(unsafe { crate::sys::include::esp_wifi_sta_itwt_setup(&mut raw) })?;

        loop {
            match subscriber.next_message_pure().await {
                event::EventInfo::IndividualTargetWakeTimeSetup {
                    config: negotiated_config,
                    status,
                    reason,
                    target_wake_time,
                } => {
                    if negotiated_config.twt_id != twt_id {
                        continue;
                    }
                    if status == ITWT_SETUP_SUCCESS {
                        break Ok(ITwtSetupInfo {
                            config: negotiated_config,
                            target_wake_time,
                        });
                    } else {
                        break Err(ITwtSetupError::Rejected(ITwtSetupFailedInfo {
                            config: negotiated_config,
                            status,
                            reason,
                        }));
                    }
                }
                event::EventInfo::StationDisconnected { .. } => {
                    break Err(WifiError::NotConnected.into());
                }
                _ => {}
            }
        }
    }

    #[procmacros::doc_replace]
    /// Tear down an individual TWT agreement.
    ///
    /// Use [`FlowTarget::All`] to tear down all active agreements.
    ///
    /// Returns `Ok(())` once the teardown completed and
    /// [`WifiError::Twt`]`(`[`TwtError::TeardownFailed`]`)` if the station
    /// failed to send the teardown frame. Returns
    /// [`WifiError::NotConnected`] if the station is or becomes disconnected.
    ///
    /// The call completes on the first teardown event covering the flow, so it
    /// may also complete because of an AP-initiated teardown of the same flow.
    /// A [`FlowTarget::All`] request completes on an event covering all
    /// flows. There is no timeout on the driver side.
    ///
    /// Concurrent calls must use different flow IDs. Otherwise incorrect
    /// results may be returned.
    ///
    /// # Event subscribers
    ///
    /// Each pending call holds one event channel subscriber and relies on the
    /// event channel having enough queue capacity. Concurrent callers (or
    /// other tasks waiting for events) require increasing the
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options.
    /// Running out of subscribers panics.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::twt::FlowId;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// controller.itwt_teardown(FlowId::Flow0).await?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub async fn itwt_teardown(&self, target: impl Into<FlowTarget>) -> Result<(), WifiError> {
        self.itwt_teardown_internal(target.into()).await
    }
    async fn itwt_teardown_internal(&self, target: FlowTarget) -> Result<(), WifiError> {
        event::enable_wifi_events(
            WifiEvent::IndividualTargetWakeTimeTeardown | WifiEvent::StationDisconnected,
        );

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS (esp-radio config option `event_channel_subscribers`)");

        if !self.is_connected() {
            return Err(WifiError::NotConnected);
        }

        esp_wifi_result!(unsafe {
            crate::sys::include::esp_wifi_sta_itwt_teardown(target.as_raw() as i32)
        })?;

        loop {
            match subscriber.next_message_pure().await {
                event::EventInfo::IndividualTargetWakeTimeTeardown { flows, status } => {
                    match target {
                        // A "teardown all" completes when the event covers all flows.
                        FlowTarget::All => {
                            if flows != EnumSet::all() {
                                continue;
                            }
                        }
                        FlowTarget::Id(id) => {
                            if !flows.contains(id) {
                                continue;
                            }
                        }
                    }
                    break teardown_result(status);
                }
                event::EventInfo::StationDisconnected { .. } => {
                    break Err(WifiError::NotConnected);
                }
                _ => {}
            }
        }
    }

    #[procmacros::doc_replace]
    /// Suspend an individual TWT agreement for the given duration.
    ///
    /// The station temporarily stops following the TWT schedule, then
    /// automatically resumes.
    /// Use [`FlowTarget::All`] to suspend all active agreements.
    ///
    /// Returns [`WifiError::NotConnected`] if the station is or becomes
    /// disconnected. The call completes on the first suspend event covering
    /// the flow, so it may also complete because of an AP-initiated suspend of
    /// the same flow. There is no timeout on the driver side.
    ///
    /// Concurrent calls must use different flow IDs. Otherwise incorrect
    /// results may be returned.
    ///
    /// # Event subscribers
    ///
    /// Each pending call holds one event channel subscriber and relies on the
    /// event channel having enough queue capacity. Concurrent callers (or
    /// other tasks waiting for events) require increasing the
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options.
    /// Running out of subscribers panics.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_hal::time::Duration;
    /// # use esp_radio::wifi::twt::FlowId;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// // Suspend flow 0 for 5 seconds
    /// controller
    ///     .itwt_suspend(FlowId::Flow0, Duration::from_secs(5))
    ///     .await?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub async fn itwt_suspend(
        &self,
        target: impl Into<FlowTarget>,
        suspend_time: Duration,
    ) -> Result<(), WifiError> {
        self.itwt_suspend_internal(target.into(), suspend_time)
            .await
    }
    async fn itwt_suspend_internal(
        &self,
        target: FlowTarget,
        suspend_time: Duration,
    ) -> Result<(), WifiError> {
        event::enable_wifi_events(
            WifiEvent::IndividualTargetWakeTimeSuspend | WifiEvent::StationDisconnected,
        );

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS (esp-radio config option `event_channel_subscribers`)");

        if !self.is_connected() {
            return Err(WifiError::NotConnected);
        }

        esp_wifi_result!(unsafe {
            crate::sys::include::esp_wifi_sta_itwt_suspend(
                target.as_raw() as i32,
                suspend_time.as_millis() as i32,
            )
        })?;

        loop {
            match subscriber.next_message_pure().await {
                event::EventInfo::IndividualTargetWakeTimeSuspend {
                    status,
                    suspended_flows,
                    actual_suspend_time_ms: _,
                } => {
                    match target {
                        FlowTarget::All => {}
                        FlowTarget::Id(id) => {
                            if !suspended_flows.contains(id) {
                                continue;
                            }
                        }
                    }
                    if status == 0 {
                        break Ok(());
                    } else {
                        break Err(WifiError::Twt(TwtError::SuspendFailed));
                    }
                }
                event::EventInfo::StationDisconnected { .. } => {
                    break Err(WifiError::NotConnected);
                }
                _ => {}
            }
        }
    }

    #[procmacros::doc_replace]
    /// Send a probe request to resynchronize the station's TSF clock with the
    /// AP.
    ///
    /// During iTWT the station misses beacons while sleeping, causing its
    /// local clock to drift. Periodic probes keep the clocks aligned and
    /// prevent the AP from tearing down the agreement.
    ///
    /// Returns [`TwtError::ProbeTimeout`] or [`TwtError::ProbeFailed`]
    /// (wrapped in [`WifiError::Twt`]) if the probe did not succeed, and
    /// [`WifiError::NotConnected`] if the station disconnected.
    ///
    /// Probe events carry no identifier, so only one probe should be in flight
    /// at a time. There is no additional timeout on the driver side.
    ///
    /// # Event subscribers
    ///
    /// Each pending call holds one event channel subscriber and relies on the
    /// event channel having enough queue capacity. Concurrent callers (or
    /// other tasks waiting for events) require increasing the
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options.
    /// Running out of subscribers panics.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_hal::time::Duration;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// // Send a probe with a 50ms timeout
    /// controller
    ///     .itwt_send_probe(Duration::from_millis(50))
    ///     .await?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub async fn itwt_send_probe(&self, timeout: Duration) -> Result<(), WifiError> {
        event::enable_wifi_events(
            WifiEvent::IndividualTargetWakeTimeProbe | WifiEvent::StationDisconnected,
        );

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS (esp-radio config option `event_channel_subscribers`)");

        if !self.is_connected() {
            return Err(WifiError::NotConnected);
        }

        esp_wifi_result!(unsafe {
            crate::sys::include::esp_wifi_sta_itwt_send_probe_req(timeout.as_millis() as i32)
        })?;

        loop {
            match subscriber.next_message_pure().await {
                event::EventInfo::IndividualTargetWakeTimeProbe { status, reason: _ } => {
                    break probe_result(status);
                }
                event::EventInfo::StationDisconnected { .. } => {
                    break Err(WifiError::NotConnected);
                }
                _ => {}
            }
        }
    }

    #[procmacros::doc_replace]
    /// Wait for the next TWT wakeup event on the given flow(s).
    ///
    /// Returns when the station wakes up for a TWT service period on one
    /// of the specified flows, or returns an error if a watched flow is
    /// torn down or the station disconnects.
    ///
    /// [`TwtConfig::post_wakeup_event`](TwtConfig::post_wakeup_event)
    /// must be `true` for wakeup events to be generated.
    ///
    /// For latency-sensitive applications, send outgoing packets in
    /// response to this event rather than on an independent timer. The
    /// WiFi firmware buffers TX until the next wake window, so a packet
    /// queued just after a window closes must wait for the next one.
    ///
    /// Multiple concurrent callers will each receive every wakeup event.
    ///
    /// # Event subscribers
    ///
    /// Each pending call holds one event channel subscriber and relies on the
    /// event channel having enough queue capacity. Concurrent callers (or
    /// other tasks waiting for events) require increasing the
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS` and
    /// `ESP_RADIO_CONFIG_EVENT_CHANNEL_CAPACITY` configuration options.
    /// Running out of subscribers panics.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::twt::FlowId;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// loop {
    ///     match controller.wait_for_next_twt_wakeup(FlowId::Flow0).await {
    ///         Ok(wakeup) => {
    ///             println!(
    ///                 "woke up: type={:?}, flow={:?}",
    ///                 wakeup.twt_type, wakeup.flow_id
    ///             );
    ///             // ... do work during the service period ...
    ///         }
    ///         Err(e) => {
    ///             println!("TWT wait failed: {e:?}");
    ///             break;
    ///         }
    ///     }
    /// }
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub async fn wait_for_next_twt_wakeup(
        &self,
        flows: impl Into<EnumSet<FlowId>>,
    ) -> Result<TwtWakeupInfo, TwtWaitError> {
        self.wait_for_next_twt_wakeup_internal(flows.into()).await
    }
    async fn wait_for_next_twt_wakeup_internal(
        &self,
        flows: EnumSet<FlowId>,
    ) -> Result<TwtWakeupInfo, TwtWaitError> {
        event::enable_wifi_events(
            WifiEvent::TargetWakeTimeWakeup
                | WifiEvent::IndividualTargetWakeTimeTeardown
                | WifiEvent::StationDisconnected,
        );

        let mut subscriber = EVENT_CHANNEL
            .subscriber()
            .expect("Unable to subscribe to events - consider increasing ESP_RADIO_CONFIG_EVENT_CHANNEL_SUBSCRIBERS (esp-radio config option `event_channel_subscribers`)");

        if !self.is_connected() {
            return Err(TwtWaitError::Disconnected);
        }

        if let Ok(active) = self.itwt_flow_id_status()
            && active.intersection(flows).is_empty()
        {
            return Err(TwtWaitError::FlowTornDown {
                flow_id: flows.iter().next().unwrap(),
                status: ITwtTeardownStatus::Success,
            });
        }

        loop {
            let event = subscriber.next_message_pure().await;
            match event {
                event::EventInfo::TargetWakeTimeWakeup { twt_type, flow_id }
                    if flows.contains(flow_id) =>
                {
                    break Ok(TwtWakeupInfo { twt_type, flow_id });
                }
                event::EventInfo::IndividualTargetWakeTimeTeardown {
                    flows: torn_down,
                    status,
                } if !torn_down.is_disjoint(flows) => {
                    break Err(TwtWaitError::FlowTornDown {
                        // Not empty, checked by the guard above.
                        flow_id: (torn_down & flows).iter().next().unwrap(),
                        status,
                    });
                }
                event::EventInfo::StationDisconnected { .. } => {
                    break Err(TwtWaitError::Disconnected);
                }
                _ => {}
            }
        }
    }

    #[procmacros::doc_replace]
    /// Get the set of active iTWT flow IDs.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// let active = controller.itwt_flow_id_status()?;
    /// println!("active flows: {:?}", active);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn itwt_flow_id_status(&self) -> Result<EnumSet<FlowId>, WifiError> {
        let mut bitmap: i32 = 0;
        esp_wifi_result!(unsafe {
            crate::sys::include::esp_wifi_sta_itwt_get_flow_id_status(&mut bitmap)
        })?;
        Ok(EnumSet::from_repr(bitmap as u8))
    }

    /// Adjust the wake-up time relative to the negotiated target wake time.
    #[instability::unstable]
    pub fn itwt_set_target_wake_time_offset(&self, offset_us: u32) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe {
            crate::sys::include::esp_wifi_sta_itwt_set_target_wake_time_offset(offset_us as i32)
        })
    }

    #[procmacros::doc_replace]
    /// Apply general TWT configuration.
    ///
    /// ## Example
    ///
    /// ```rust,no_run
    /// # {before_snippet}
    /// # use esp_radio::wifi::twt::TwtConfig;
    /// # let controller =
    /// #   esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
    /// controller.twt_config(&TwtConfig::default().with_post_wakeup_event(true))?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn twt_config(&self, config: &TwtConfig) -> Result<(), WifiError> {
        let mut raw = config.to_raw();
        esp_wifi_result!(unsafe { crate::sys::include::esp_wifi_sta_twt_config(&mut raw) })
    }
}
