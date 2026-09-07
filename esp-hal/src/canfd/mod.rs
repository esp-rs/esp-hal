//! # Controller Area Network Flexible Data-Rate (CAN FD)
//!
//! ## Overview
//!
//! The CAN FD controller is a CTU CAN FD core. It is a different peripheral
//! from the classic two-wire automotive interface (TWAI) controller. It
//! supports classic CAN 2.0 frames and CAN FD frames. A CAN FD frame carries up
//! to 64 bytes of payload and can switch to a faster bit rate for its data
//! phase.
//!
//! The peripheral needs an external transceiver to drive a real bus.
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/peripherals/twai.html)")]
//! ## Configuration
//!
//! [`Config`] selects the bus participation [`Mode`], the function
//! [`ClockSource`], and the bit [`Timing`] of the arbitration and data phases.
//! The controller stays off the bus until [`CanFd::start`] is called, so the
//! pins can be assigned first.
//!
//! Received frames pass through three mask filters and one range filter. Each
//! filter accepts a configurable set of [`FrameKinds`], so classic and CAN FD
//! frames can be filtered separately.
//!
//! ## Usage
//!
//! [`CanFd::into_async`] converts the driver into an async driver. Received
//! frames stay in the hardware RX buffer until they are read, so
//! [`CanFd::receive_async`] is cancel-safe and the driver needs no software
//! frame queue.
//!
//! [`ClassicFrame`] implements [`embedded_can::Frame`] and [`BusErrorKind`]
//! implements [`embedded_can::Error`], for code written against the
//! `embedded-can` traits.
//!
//! ## Examples
//!
//! ### Sending a CAN FD frame
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::canfd::{CanFd, Config, Frame};
//!
//! let mut canfd = CanFd::new(peripherals.TWAI0, Config::default())?
//!     .with_rx(peripherals.GPIO9)
//!     .with_tx(peripherals.GPIO8);
//! canfd.start()?;
//!
//! // A 64-byte frame, sent with the data phase at the FD bit rate.
//! let frame = Frame::new_fd(0x123, false, true, &[0xAA; 64])?;
//! canfd.transmit(&frame)?;
//! # Ok(())
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! - Time-triggered transmission is not exposed.
//! - TX buffer backup mode and RAM parity protection are not exposed.
//!
//! Bare "TRM" references in this module mean the ESP32-C5 TRM v1.1, chapter 38.
//! Section numbers are specific to that manual.
#![doc = crate::trm_markdown_link!("#canfd")]

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    DriverMode,
    clock::ll::{ClockTree, TwaiInstance},
    gpio::{
        DriveMode,
        InputConfig,
        InputSignal,
        OutputConfig,
        OutputSignal,
        Pull,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    interrupt::InterruptHandler,
    peripherals::Interrupt,
    system::{Cpu, Peripheral, PeripheralGuard},
    time::{Duration, Instant},
};

mod asynch;
mod ll;

pub use ll::{
    BusErrorKind,
    ErrorCapture,
    ErrorPosition,
    ErrorState,
    FD_TIMING_LIMITS,
    FrameKinds,
    MaskFilter,
    NOMINAL_TIMING_LIMITS,
    Timing,
    TimingLimits,
    TxBufferState,
    dlc_to_len,
    len_to_dlc,
};
use ll::{
    CLASSIC_MAX_DATA_LEN,
    EXT_ID_MASK,
    FrameBuffer,
    Ll,
    MAX_DATA_LEN,
    STD_ID_MASK,
    SspSource,
    TimestampPoint,
};

use crate::rtc_cntl::WakeLock;
/// Clock source for the CAN FD peripheral.
pub use crate::soc::clocks::TwaiFunctionClockConfig as ClockSource;

/// The device ID every CTU CAN FD core reports in `TWAIFD_DEVICE_ID_VERSION_REG`.
pub const CANFD_DEVICE_ID: u16 = 0xCAFD;

/// An interrupt source of the controller (TRM 38.4).
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CanFdInterrupt {
    /// A frame was received.
    RxFrame,
    /// A frame was transmitted.
    TxFrame,
    /// The error warning limit was reached.
    ErrorWarning,
    /// The RX buffer overran.
    DataOverrun,
    /// The fault confinement state changed.
    FaultStateChanged,
    /// Arbitration was lost.
    ArbitrationLost,
    /// A bus error was detected.
    BusError,
    /// An overload frame was seen.
    Overload,
    /// The RX buffer is full.
    RxFull,
    /// A frame switched to the data bit rate.
    ///
    /// This is `BSI`, the bit rate shifted interrupt of TRM 38.4. It fires on
    /// both sides of a bit rate switched frame. It does not report a change
    /// between transmitting and receiving.
    BitRateShifted,
    /// The RX buffer is no longer empty.
    RxNotEmpty,
    /// A TX buffer finished, successfully or not.
    TxDone,
}

impl CanFdInterrupt {
    /// The bit this source occupies in `TWAIFD_INT_STAT_REG`.
    const fn bit(self) -> u32 {
        1 << match self {
            Self::RxFrame => 0,
            Self::TxFrame => 1,
            Self::ErrorWarning => 2,
            Self::DataOverrun => 3,
            Self::FaultStateChanged => 4,
            Self::ArbitrationLost => 5,
            Self::BusError => 6,
            Self::Overload => 7,
            Self::RxFull => 8,
            Self::BitRateShifted => 9,
            Self::RxNotEmpty => 10,
            Self::TxDone => 11,
        }
    }

    fn mask(set: EnumSet<Self>) -> u32 {
        set.iter().fold(0, |acc, source| acc | source.bit())
    }

    fn from_mask(bits: u32) -> EnumSet<Self> {
        EnumSet::all()
            .iter()
            .filter(|source: &Self| bits & source.bit() != 0)
            .collect()
    }
}

/// Identity of a CAN FD controller, read out of the hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Identity {
    /// Device ID. Reads `0xCAFD` on a working core.
    pub device_id: u16,
    /// IP core major version.
    pub version_major: u8,
    /// IP core minor version.
    pub version_minor: u8,
}

/// How the controller participates on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Transmits and receives normally.
    #[default]
    Normal,
    /// Receives only, never driving a dominant bit (TRM 38.3.12.4).
    ListenOnly,
    /// Treats a frame as transmitted even without an ACK (TRM 38.3.12.2).
    ///
    /// Needed to transmit as the only node on a bus.
    SelfTest,
    /// Routes transmitted frames back into the RX buffer (TRM 38.3.12.1).
    ///
    /// Loopback alone still requires an external ACK, so this mode also enables
    /// self test. Together they let a single node verify its own operation.
    LoopbackSelfTest,
}

/// Errors that can occur when the configuration is applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The nominal bit timing is outside the limits of the hardware.
    UnsupportedNominalTiming,
    /// The data phase bit timing is outside the limits of the hardware.
    UnsupportedFdTiming,
    /// The timestamp resolution cannot be reached from the function clock.
    UnsupportedTimestampResolution,
    /// The timestamp counter did not start ticking.
    TimestampTimerStalled,
    /// The retransmission limit is larger than the hardware field can hold.
    UnsupportedRetransmitLimit,
    /// The secondary sample point cannot be placed where it was asked for.
    ///
    /// The final position must stay inside four data bit times and inside the
    /// eight-bit offset field (TRM 38.3.7.3).
    UnsupportedSecondarySamplePoint,
    /// A transmission was still in flight and did not settle, so the
    /// configuration was not applied.
    BusBusy,
    /// The transceiver setting cannot change after a pin is assigned, because
    /// it decides how that pin is driven.
    TransceiverModeLocked,
    /// A filter was given an identifier that does not fit the frame format.
    ///
    /// A base identifier is 11 bits and an extended one 29.
    FilterIdTooLarge,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::UnsupportedNominalTiming => {
                "The nominal bit timing is outside the hardware limits"
            }
            Self::UnsupportedFdTiming => "The data phase bit timing is outside the hardware limits",
            Self::UnsupportedTimestampResolution => {
                "The timestamp resolution cannot be reached from the function clock"
            }
            Self::TimestampTimerStalled => "The timestamp counter did not start ticking",
            Self::UnsupportedRetransmitLimit => {
                "The retransmission limit is larger than the hardware field can hold"
            }
            Self::UnsupportedSecondarySamplePoint => {
                "The secondary sample point cannot be placed where it was asked for"
            }
            Self::BusBusy => "A transmission was still in flight and did not settle",
            Self::TransceiverModeLocked => {
                "Whether a transceiver is in the way cannot be changed once a pin is assigned"
            }
            Self::FilterIdTooLarge => "A filter identifier does not fit the frame format",
        };
        f.write_str(message)
    }
}

/// Errors that can occur while the driver operates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Every TX buffer holds a frame that is not yet finished.
    NoFreeTxBuffer,
    /// The RX buffer holds no complete frame.
    RxBufferEmpty,
    /// The controller did not become error-active in time.
    ///
    /// The bus must be idle-recessive for 11 bit times before the controller
    /// can join it.
    BusIntegrationTimeout,
    /// The controller gave up on the frame after its retransmission limit.
    TransmitFailed,
    /// The transmission was aborted.
    TransmitAborted,
    /// The operation needs the controller to be off the bus first.
    ControllerRunning,
    /// The operation needs the controller to be on the bus; see
    /// [`CanFd::start`].
    ControllerStopped,
    /// A transmission did not settle in time after being aborted.
    AbortTimeout,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::NoFreeTxBuffer => "Every TX buffer holds a frame that is not yet finished",
            Self::RxBufferEmpty => "The RX buffer holds no complete frame",
            Self::BusIntegrationTimeout => "The controller did not become error-active in time",
            Self::TransmitFailed => "The controller gave up on the frame",
            Self::TransmitAborted => "The transmission was aborted",
            Self::ControllerRunning => "The operation needs the controller to be off the bus",
            Self::ControllerStopped => "The operation needs the controller to be on the bus",
            Self::AbortTimeout => "A transmission did not settle in time after being aborted",
        };
        f.write_str(message)
    }
}

/// Errors that can occur when a frame is created.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FrameError {
    /// The payload is longer than the frame format can carry.
    ///
    /// Classic CAN frames carry up to 8 bytes, CAN FD frames up to 64.
    PayloadTooLong,
    /// The data length code is larger than 15.
    InvalidDataLengthCode,
    /// The identifier does not fit the frame format.
    ///
    /// A base identifier is 11 bits and an extended one 29.
    IdTooLarge,
    /// The frame is a CAN FD frame, which a [`ClassicFrame`] cannot hold.
    NotClassic,
}

impl core::error::Error for FrameError {}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::PayloadTooLong => "The payload is longer than the frame format can carry",
            Self::InvalidDataLengthCode => "The data length code is larger than 15",
            Self::IdTooLarge => "The identifier does not fit the frame format",
            Self::NotClassic => "The frame is a CAN FD frame",
        };
        f.write_str(message)
    }
}

/// CAN FD driver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Bus participation mode.
    ///
    /// Default value: [`Mode::Normal`].
    mode: Mode,

    /// Function clock source.
    ///
    /// The TRM recommends [`ClockSource::Xtal`] up to 5 Mbit/s and
    /// [`ClockSource::PllF80m`] above that.
    ///
    /// Default value: [`ClockSource::PllF80m`].
    clock_source: ClockSource,

    /// Bit timing of the arbitration phase.
    ///
    /// Default value: 500 kbit/s from an 80 MHz function clock.
    nominal_timing: Timing,

    /// Bit timing of the data phase, used by bit-rate-switched frames.
    ///
    /// Default value: 2 Mbit/s from an 80 MHz function clock.
    fd_timing: Timing,

    /// Number of retransmission attempts, or `None` to retry forever.
    ///
    /// The first attempt is not a retransmission.
    ///
    /// Default value: `Some(3)`.
    retransmit_limit: Option<u8>,

    /// Secondary sample point offset in time quanta, or `None` to disable it.
    ///
    /// Default value: `None`.
    secondary_sample_point_offset: Option<u8>,

    /// Whether received request frames are dropped instead of stored.
    ///
    /// Default value: `false`.
    drop_request_frames: bool,

    /// Whether protocol exception handling is enabled.
    ///
    /// Default value: `false`.
    protocol_exception: bool,

    /// Whether the TX pin is driven open-drain with a pull-up.
    ///
    /// Lets two nodes be wired together directly, without transceivers. Binding
    /// the RX and TX signals to the same pin then forms a single-node bus.
    ///
    /// Default value: `false`.
    no_transceiver: bool,
}

impl Default for Config {
    fn default() -> Self {
        // 500 kbit/s nominal and 2 Mbit/s data from an 80 MHz function clock,
        // sampling at 81.2% and 75%.
        Self {
            mode: Mode::Normal,
            clock_source: ClockSource::PllF80m,
            nominal_timing: Timing {
                baud_rate_prescaler: 10,
                propagation_segment: 7,
                phase_segment_1: 5,
                phase_segment_2: 3,
                sync_jump_width: 3,
            },
            fd_timing: Timing {
                baud_rate_prescaler: 10,
                propagation_segment: 1,
                phase_segment_1: 1,
                phase_segment_2: 1,
                sync_jump_width: 1,
            },
            retransmit_limit: Some(3),
            secondary_sample_point_offset: None,
            drop_request_frames: false,
            protocol_exception: false,
            no_transceiver: false,
        }
    }
}

impl Config {
    /// Checks the configuration against the limits of the hardware.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnsupportedNominalTiming`] when the arbitration phase
    /// timing does not fit [`NOMINAL_TIMING_LIMITS`].
    ///
    /// [`ConfigError::UnsupportedFdTiming`] when the data phase timing does not
    /// fit [`FD_TIMING_LIMITS`].
    fn validate(&self) -> Result<(), ConfigError> {
        if !self.nominal_timing.is_valid(&NOMINAL_TIMING_LIMITS) {
            return Err(ConfigError::UnsupportedNominalTiming);
        }
        if !self.fd_timing.is_valid(&FD_TIMING_LIMITS) {
            return Err(ConfigError::UnsupportedFdTiming);
        }
        // `RTRTH` is four bits wide (TRM 38.3.8.4). A larger value would be
        // truncated on the way in, and 16 in particular becomes zero, turning
        // "retry sixteen times" into "never retry".
        if self
            .retransmit_limit
            .is_some_and(|limit| limit > ll::MAX_RETRANSMIT_LIMIT)
        {
            return Err(ConfigError::UnsupportedRetransmitLimit);
        }

        if let Some(offset) = self.secondary_sample_point_offset {
            // The hardware counts the offset in function clock periods, not in
            // time quanta, so the prescaler is part of the conversion.
            let cycles = u32::from(offset) * u32::from(self.fd_timing.baud_rate_prescaler);

            // TRM 38.3.7.3: the field is eight bits and a larger position is
            // silently saturated to 255, which would place the sample somewhere
            // other than asked for. Refuse instead.
            if cycles > u32::from(u8::MAX) {
                return Err(ConfigError::UnsupportedSecondarySamplePoint);
            }

            // "Users should not configure the secondary sample point position
            // later than 4 data bit times." The limit applies to the final
            // position, which is the offset plus the delay the hardware
            // measures, so the offset alone has to leave room for that delay.
            // How much room is unknowable here — it depends on the transceiver
            // — but the core contributes two clock periods of input delay on
            // its own, so an offset that only fits with a delay of zero cannot
            // fit in practice. Reserving those two cycles turns the boundary
            // case from "accepted, then every FD transmission fails" into a
            // configuration error. Read `CanFd::transmitter_delay` after an FD
            // transmission for the delay actually measured.
            let data_bit_cycles =
                u32::from(self.fd_timing.baud_rate_prescaler) * self.fd_timing.total_quanta();
            if cycles + MIN_TRANSMITTER_DELAY_CYCLES > data_bit_cycles * 4 {
                return Err(ConfigError::UnsupportedSecondarySamplePoint);
            }

            // A position below three cannot transmit FD frames without flagging
            // bit errors against the core's own output.
            if cycles <= MIN_TRANSMITTER_DELAY_CYCLES {
                return Err(ConfigError::UnsupportedSecondarySamplePoint);
            }
        }

        Ok(())
    }
}

/// Configuration of one mask filter.
///
/// A frame is accepted when its identifier matches `id` in every bit set in
/// `mask`. A `mask` of zero therefore accepts every identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MaskFilterConfig {
    /// Identifier to match.
    pub id: u32,
    /// Which identifier bits are compared. Zero matches everything.
    pub mask: u32,
    /// Whether `id` and `mask` are 29-bit extended identifiers.
    pub extended: bool,
    /// Which frame kinds this filter accepts.
    pub accepts: FrameKinds,
}

/// Configuration of the range filter.
///
/// Accepts identifiers in `low..=high`. A `low` above `high` is an empty
/// range, which accepts nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangeFilterConfig {
    /// Lower bound, included.
    pub low: u32,
    /// Upper bound, included.
    pub high: u32,
    /// Whether the bounds are 29-bit extended identifiers.
    pub extended: bool,
    /// Which frame kinds this filter accepts.
    pub accepts: FrameKinds,
}

/// A CAN FD frame.
#[derive(Clone, Copy)]
pub struct Frame {
    // Private, so `len` cannot exceed `data` and the format flags cannot
    // contradict each other. Every value is reachable through a constructor,
    // and reading is what callers actually need.
    id: u32,
    extended: bool,
    request: bool,
    fd: bool,
    bit_rate_switch: bool,
    error_state_indicator: bool,
    data: [u8; MAX_DATA_LEN],
    /// Bytes actually carried. Always zero for a request frame.
    len: usize,
    /// Bytes a request frame asks for. Zero for a data frame.
    ///
    /// Kept apart from `len` because a request frame has a data length code but
    /// no data: folding the two together either invents a payload it never
    /// carried or loses the length it asked for, depending on which way it is
    /// folded.
    requested_len: usize,
    timestamp: u64,
}

impl Frame {
    /// Creates a new classic CAN data frame.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `payload` is longer than 8 bytes.
    ///
    /// [`FrameError::IdTooLarge`] when `id` does not fit the identifier of the
    /// chosen format.
    pub fn new(id: u32, extended: bool, payload: &[u8]) -> Result<Self, FrameError> {
        if payload.len() > CLASSIC_MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        Self::build(id, extended, false, false, false, payload)
    }

    /// Creates a new classic CAN request frame.
    ///
    /// A request frame carries no payload. `len` is the payload length that the
    /// frame asks for.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `len` is greater than 8.
    ///
    /// [`FrameError::IdTooLarge`] when `id` does not fit the identifier of the
    /// chosen format.
    pub fn new_request(id: u32, extended: bool, len: usize) -> Result<Self, FrameError> {
        if len > CLASSIC_MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        let mut frame = Self::build(id, extended, true, false, false, &[])?;
        // A request frame carries no data; `len` stays zero and the length it
        // asks for lives on its own.
        frame.requested_len = len;
        Ok(frame)
    }

    /// Creates a new CAN FD data frame.
    ///
    /// Set `bit_rate_switch` to send the data phase at the FD bit rate.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `payload` is longer than 64 bytes.
    ///
    /// [`FrameError::IdTooLarge`] when `id` does not fit the identifier of the
    /// chosen format.
    pub fn new_fd(
        id: u32,
        extended: bool,
        bit_rate_switch: bool,
        payload: &[u8],
    ) -> Result<Self, FrameError> {
        if payload.len() > MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        Self::build(id, extended, false, true, bit_rate_switch, payload)
    }

    fn build(
        id: u32,
        extended: bool,
        request: bool,
        fd: bool,
        brs: bool,
        payload: &[u8],
    ) -> Result<Self, FrameError> {
        // The frame buffer holds 11 or 29 identifier bits and the rest is
        // dropped on the way in. Accepting a wider identifier here would send a
        // frame under a different one than the caller asked for and checked,
        // which changes both which filters accept it and how it arbitrates.
        let limit = if extended { EXT_ID_MASK } else { STD_ID_MASK };
        if id > limit {
            return Err(FrameError::IdTooLarge);
        }

        let mut data = [0u8; MAX_DATA_LEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            extended,
            request,
            fd,
            bit_rate_switch: brs,
            error_state_indicator: false,
            data,
            len: payload.len(),
            requested_len: 0,
            timestamp: 0,
        })
    }

    /// Returns the payload the frame carries.
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Returns the payload length in bytes.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns whether the frame carries no payload.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the payload length a request frame asks for, in bytes.
    ///
    /// Zero for a data frame, whose length is [`Frame::len`].
    pub fn requested_len(&self) -> usize {
        self.requested_len
    }

    /// Returns the arbitration identifier.
    pub fn id(&self) -> u32 {
        self.id
    }

    /// Returns whether the identifier is a 29-bit extended one.
    pub fn is_extended(&self) -> bool {
        self.extended
    }

    /// Returns whether this is a request frame, which carries no payload.
    pub fn is_request(&self) -> bool {
        self.request
    }

    /// Returns whether the frame uses the CAN FD format.
    pub fn is_fd(&self) -> bool {
        self.fd
    }

    /// Returns whether the data phase uses the FD bit rate.
    pub fn is_bit_rate_switched(&self) -> bool {
        self.bit_rate_switch
    }

    /// Returns the error state indicator of the transmitting node.
    pub fn error_state_indicator(&self) -> bool {
        self.error_state_indicator
    }

    /// Returns the timestamp captured when the frame was received.
    ///
    /// Zero for a frame that was not received; see
    /// [`CanFd::start_timestamp_timer`].
    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    fn to_buffer(self) -> FrameBuffer {
        // A request frame's length code is the length it asks for, not a
        // payload it holds. A data payload that is not a valid FD length is
        // padded up to the next code.
        let dlc = if self.request {
            len_to_dlc(self.requested_len as u8)
        } else {
            len_to_dlc(self.len as u8)
        };
        FrameBuffer::build(
            self.id,
            self.extended,
            self.request,
            self.fd,
            self.bit_rate_switch,
            dlc,
            &self.data[..self.len],
            0,
        )
    }

    fn from_buffer(buffer: &FrameBuffer) -> Self {
        let mut data = [0u8; MAX_DATA_LEN];
        let len = buffer.data(&mut data);
        let request = buffer.is_request();
        Self {
            id: buffer.id(),
            extended: buffer.is_extended(),
            request,
            fd: buffer.is_fd(),
            bit_rate_switch: buffer.is_bit_rate_switched(),
            error_state_indicator: buffer.error_state_indicator(),
            data,
            len,
            // Preserved separately, so a received request can be answered or
            // forwarded with the length it actually asked for.
            requested_len: if request { buffer.requested_len() } else { 0 },
            timestamp: buffer.timestamp(),
        }
    }
}

/// A frame that is known to be classic CAN, for code written against
/// `embedded-can`.
///
/// [`embedded_can::Frame`] describes CAN 2.0. It promises that `dlc` and
/// `data` never exceed 8 bytes, and generic code can size its buffers by that
/// promise. A [`Frame`] cannot keep that promise: a 64-byte FD frame is a
/// `Frame` too, and one can arrive through [`CanFd::receive`]. The trait is
/// therefore implemented on this type, which can only hold a classic frame.
/// The trait constructors build one directly, and `TryFrom<Frame>` refuses an
/// FD frame with [`FrameError::NotClassic`].
///
/// The type dereferences to the [`Frame`] it wraps, so the timestamp and the
/// other accessors are available. It converts back into a [`Frame`] for
/// [`CanFd::transmit`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClassicFrame(Frame);

impl ClassicFrame {
    /// Returns the frame this wraps.
    pub fn into_frame(self) -> Frame {
        self.0
    }
}

impl core::ops::Deref for ClassicFrame {
    type Target = Frame;

    fn deref(&self) -> &Frame {
        &self.0
    }
}

impl TryFrom<Frame> for ClassicFrame {
    type Error = FrameError;

    /// # Errors
    ///
    /// [`FrameError::NotClassic`] when the frame uses the CAN FD format.
    fn try_from(frame: Frame) -> Result<Self, FrameError> {
        if frame.fd {
            return Err(FrameError::NotClassic);
        }
        // Everything else the trait promises follows: a classic frame carries
        // at most 8 bytes, and a request frame asks for at most 8.
        Ok(Self(frame))
    }
}

impl From<ClassicFrame> for Frame {
    fn from(frame: ClassicFrame) -> Self {
        frame.0
    }
}

#[instability::unstable]
impl embedded_can::Frame for ClassicFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let (id, extended) = split_id(id.into());
        Frame::new(id, extended, data).ok().map(Self)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        let (id, extended) = split_id(id.into());
        Frame::new_request(id, extended, dlc).ok().map(Self)
    }

    fn is_extended(&self) -> bool {
        self.0.extended
    }

    fn is_remote_frame(&self) -> bool {
        self.0.request
    }

    fn id(&self) -> embedded_can::Id {
        // The constructors refuse an identifier that does not fit its format,
        // so neither conversion can fail here.
        if self.0.extended {
            embedded_can::Id::Extended(
                embedded_can::ExtendedId::new(self.0.id).expect("29-bit identifier"),
            )
        } else {
            embedded_can::Id::Standard(
                embedded_can::StandardId::new(self.0.id as u16).expect("11-bit identifier"),
            )
        }
    }

    fn dlc(&self) -> usize {
        // For a data frame the trait defines this as the payload length; a
        // request frame carries none and the code is the length it asks for.
        if self.0.request {
            self.0.requested_len
        } else {
            self.0.len
        }
    }

    fn data(&self) -> &[u8] {
        self.0.payload()
    }
}

#[instability::unstable]
impl From<BusErrorKind> for embedded_can::ErrorKind {
    fn from(kind: BusErrorKind) -> Self {
        match kind {
            BusErrorKind::Bit => Self::Bit,
            BusErrorKind::Crc => Self::Crc,
            BusErrorKind::Form => Self::Form,
            BusErrorKind::Ack => Self::Acknowledge,
            BusErrorKind::Stuff => Self::Stuff,
            BusErrorKind::Unknown(_) => Self::Other,
        }
    }
}

#[instability::unstable]
impl embedded_can::Error for BusErrorKind {
    fn kind(&self) -> embedded_can::ErrorKind {
        (*self).into()
    }
}

/// Takes an `embedded-can` identifier apart into its value and format.
fn split_id(id: embedded_can::Id) -> (u32, bool) {
    match id {
        embedded_can::Id::Standard(id) => (u32::from(id.as_raw()), false),
        embedded_can::Id::Extended(id) => (id.as_raw(), true),
    }
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Frame")
            .field("id", &format_args!("{:#x}", self.id))
            .field("extended", &self.extended)
            .field("request", &self.request)
            .field("fd", &self.fd)
            .field("bit_rate_switch", &self.bit_rate_switch)
            .field("error_state_indicator", &self.error_state_indicator)
            .field("dlc", &len_to_dlc(self.len as u8))
            .field("len", &self.len)
            .field("timestamp", &self.timestamp)
            .field("data", &self.payload())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Frame {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "Frame {{ id: {=u32:#x}, extended: {}, request: {}, fd: {}, brs: {}, esi: {}, dlc: {=u8}, len: {=usize}, timestamp: {=u64}, data: {=[u8]:#x} }}",
            self.id,
            self.extended,
            self.request,
            self.fd,
            self.bit_rate_switch,
            self.error_state_indicator,
            len_to_dlc(self.len as u8),
            self.len,
            self.timestamp,
            self.payload()
        )
    }
}

/// A CAN FD controller.
pub struct CanFd<'d, Dm: DriverMode = Blocking> {
    // Fields drop in declaration order, and the order here is load-bearing:
    // the controller must leave the bus before its clocks are released, and
    // release the function clock while the register clock is still up.
    bus: BusGuard,
    config: Config,
    _mode: PhantomData<Dm>,
    /// Accepted frame kinds for filters A, B, C and the range filter, mirrored
    /// here because they share one register.
    filter_kinds: [FrameKinds; 4],
    /// Whether a pin has been assigned, and so whether the output stage the
    /// configuration asks for has already been committed to hardware.
    pins_bound: bool,
    twai: AnyCanFd<'d>,
    _function_clock: FunctionClockGuard,
    _peripheral: PeripheralGuard,
}

impl<Dm: DriverMode> core::fmt::Debug for CanFd<'_, Dm> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CanFd")
            .field("twai", &self.twai)
            .field("config", &self.config)
            .field("started", &self.is_started())
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "defmt")]
impl<Dm: DriverMode> defmt::Format for CanFd<'_, Dm> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "CanFd {{ twai: {:?}, config: {:?}, started: {} }}",
            self.twai,
            self.config,
            self.is_started()
        )
    }
}

/// Receiving half of a split controller.
///
/// It borrows the driver. The driver keeps the configuration and the teardown,
/// and the halves cannot outlive it. See [`CanFd::split`].
pub struct CanFdRx<'a, Dm: DriverMode> {
    ll: Ll,
    state: &'static asynch::State,
    _driver: PhantomData<&'a mut ()>,
    _mode: PhantomData<Dm>,
}

/// Transmitting half of a split controller.
///
/// See [`CanFd::split`].
pub struct CanFdTx<'a, Dm: DriverMode> {
    ll: Ll,
    state: &'static asynch::State,
    tx_buffers: u8,
    _driver: PhantomData<&'a mut ()>,
    _mode: PhantomData<Dm>,
}

impl<Dm: DriverMode> core::fmt::Debug for CanFdRx<'_, Dm> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CanFdRx")
            .field("rx_frame_count", &self.rx_frame_count())
            .finish_non_exhaustive()
    }
}

impl<Dm: DriverMode> core::fmt::Debug for CanFdTx<'_, Dm> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CanFdTx")
            .field("tx_buffers", &self.tx_buffers)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "defmt")]
impl<Dm: DriverMode> defmt::Format for CanFdRx<'_, Dm> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "CanFdRx {{ rx_frame_count: {=u16} }}",
            self.rx_frame_count()
        )
    }
}

#[cfg(feature = "defmt")]
impl<Dm: DriverMode> defmt::Format for CanFdTx<'_, Dm> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "CanFdTx {{ tx_buffers: {=u8} }}", self.tx_buffers)
    }
}

impl<Dm: DriverMode> CanFdRx<'_, Dm> {
    /// Returns the number of complete frames waiting in the RX buffer.
    pub fn rx_frame_count(&self) -> u16 {
        self.ll.rx_frame_count()
    }

    /// Reads one frame from the RX buffer.
    ///
    /// # Errors
    ///
    /// [`Error::RxBufferEmpty`] when no complete frame is waiting.
    pub fn receive(&mut self) -> Result<Frame, Error> {
        // The same predicate the async path waits on: complete frames, counted
        // by the hardware. Reading from a buffer that only holds part of a
        // frame would return that part as a frame.
        if self.ll.rx_frame_count() == 0 {
            return Err(Error::RxBufferEmpty);
        }
        Ok(Frame::from_buffer(&self.ll.read_rx_frame()))
    }

    /// Discards everything in the RX buffer.
    pub fn flush_rx(&mut self) {
        self.ll.flush_rx();
    }
}

impl CanFdRx<'_, Async> {
    /// Waits until a frame is received, then reads it.
    ///
    /// # Cancellation Safety
    ///
    /// This future is cancel-safe. A frame stays in the hardware RX buffer
    /// until it is read, so dropping the future loses nothing.
    pub async fn receive_async(&mut self) -> Frame {
        core::future::poll_fn(|cx| {
            self.state.rx_waker.register(cx.waker());
            // Arm the interrupt before looking at the buffer. Checking first
            // would lose a frame that arrives between the check and the arming,
            // because nothing would be left to raise the interrupt afterwards.
            self.ll.enable_interrupts(CanFdInterrupt::RxNotEmpty.bit());

            if self.ll.rx_frame_count() > 0 {
                self.ll.disable_interrupts(CanFdInterrupt::RxNotEmpty.bit());
                return core::task::Poll::Ready(());
            }

            core::task::Poll::Pending
        })
        .await;

        Frame::from_buffer(&self.ll.read_rx_frame())
    }
}

impl<Dm: DriverMode> CanFdTx<'_, Dm> {
    /// Returns the number of TX buffers the hardware provides.
    pub fn tx_buffer_count(&self) -> u8 {
        self.tx_buffers
    }

    /// Queues a frame in the first free TX buffer and arms it.
    ///
    /// Returns the index of the buffer used.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerStopped`] when the controller is not on the bus. A
    /// stopped controller ignores the command that arms a buffer, so the frame
    /// would be lost.
    ///
    /// [`Error::NoFreeTxBuffer`] when every TX buffer is occupied.
    pub fn transmit(&mut self, frame: &Frame) -> Result<u8, Error> {
        // Checked before touching a buffer: with ENA clear the hardware leaves
        // the buffer empty (TRM 38.3.4), and nothing later replays the arming.
        // Not before `start`, not after `stop` and not after `apply_config`,
        // which all leave the controller off the bus.
        if !self.ll.is_enabled() {
            return Err(Error::ControllerStopped);
        }

        let index = (0..self.tx_buffers)
            .find(|&i| self.ll.tx_buffer_state(i).is_writable())
            .ok_or(Error::NoFreeTxBuffer)?;

        self.ll.write_tx_buffer(index, &frame.to_buffer());
        self.ll.set_tx_ready(index);
        Ok(index)
    }

    /// Returns the state of one TX buffer.
    ///
    /// Returns [`TxBufferState::NotExist`] for an index the hardware does not
    /// have.
    pub fn tx_buffer_state(&self, index: u8) -> TxBufferState {
        if index >= self.tx_buffers {
            return TxBufferState::NotExist;
        }
        self.ll.tx_buffer_state(index)
    }

    /// Requests that a queued transmission be aborted.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn abort_transmit(&mut self, index: u8) {
        if index < self.tx_buffers {
            self.ll.set_tx_abort(index);
        }
    }

    /// Returns a TX buffer to the empty state, releasing it for reuse.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn release_tx_buffer(&mut self, index: u8) {
        if index < self.tx_buffers {
            self.ll.set_tx_empty(index);
        }
    }
}

impl CanFdTx<'_, Async> {
    /// Queues a frame and waits until the hardware finishes with it.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerStopped`] when the controller is not on the bus, and
    /// [`Error::NoFreeTxBuffer`] when every TX buffer is occupied; see
    /// [`CanFdTx::transmit`].
    ///
    /// [`Error::TransmitFailed`] when the controller gave up on the frame.
    ///
    /// [`Error::TransmitAborted`] when the transmission was aborted.
    ///
    /// # Cancellation Safety
    ///
    /// This future is not cancel-safe. Dropping it leaves the frame queued, and
    /// the hardware may still transmit it.
    pub async fn transmit_async(&mut self, frame: &Frame) -> Result<(), Error> {
        let index = self.transmit(frame)?;

        core::future::poll_fn(|cx| {
            self.state.tx_waker.register(cx.waker());
            // Arm before reading the state, so a transmission that finishes
            // between the two still raises the interrupt.
            self.ll.enable_interrupts(CanFdInterrupt::TxDone.bit());

            let state = self.ll.tx_buffer_state(index);
            if !matches!(
                state,
                TxBufferState::Ready | TxBufferState::InProgress | TxBufferState::AbortInProgress
            ) {
                self.ll.disable_interrupts(CanFdInterrupt::TxDone.bit());
            }

            match state {
                TxBufferState::Ready
                | TxBufferState::InProgress
                | TxBufferState::AbortInProgress => core::task::Poll::Pending,
                TxBufferState::Ok => core::task::Poll::Ready(Ok(())),
                TxBufferState::Failed => core::task::Poll::Ready(Err(Error::TransmitFailed)),
                TxBufferState::Aborted => core::task::Poll::Ready(Err(Error::TransmitAborted)),
                // A buffer that is empty or absent was never armed.
                _ => core::task::Poll::Ready(Err(Error::NoFreeTxBuffer)),
            }
        })
        .await
    }
}

/// Owns the controller's participation in the bus.
///
/// Exists to make leaving the bus part of teardown rather than something the
/// caller has to remember. Gating the clock, or clearing `ENA`, while a frame is
/// on the wire freezes the transmission mid-frame, and the other nodes see a
/// corrupt frame rather than a node that went away. A dropped driver would
/// otherwise inject errors into a working bus.
struct BusGuard {
    ll: Ll,
    tx_buffers: u8,
    /// The controller's interrupt, so teardown can stop it from being delivered
    /// without holding the peripheral handle.
    interrupt: Interrupt,
    /// How long a frame in flight is given to finish, for the configured bit
    /// rate. Kept here so `Drop` has it without reaching for the clock tree.
    abort_timeout: Duration,
    /// Held for exactly as long as the controller is on the bus.
    ///
    /// Light sleep gates the function clock, and a controller whose clock
    /// stops mid-frame corrupts what it was sending and misses what it was
    /// receiving. Neither is tied to a future: a frame keeps going after the
    /// future that queued it is dropped, and a frame can arrive while nothing
    /// is waiting for it. So the lock follows `ENA`, through [`Self::enable`]
    /// and [`Self::disable`], which are the only places that touch that bit.
    wake_lock: Option<WakeLock>,
}

impl BusGuard {
    /// Puts the controller on the bus and keeps the chip awake while it is.
    fn enable(&mut self) {
        self.ll.enable(true);
        if self.wake_lock.is_none() {
            self.wake_lock = Some(WakeLock::new());
        }
    }

    /// Takes the controller off the bus and lets the chip sleep again.
    ///
    /// The caller is responsible for the wire being free; see [`Self::quiesce`].
    fn disable(&mut self) {
        self.ll.enable(false);
        self.wake_lock = None;
    }

    /// Aborts anything queued and waits for the wire to be free.
    ///
    /// Follows TRM 38.3.6 step 1: aborting a frame already being transmitted
    /// only moves its buffer to "abort in progress", which is not yet a
    /// finished transmission.
    ///
    /// # Errors
    ///
    /// [`Error::AbortTimeout`] when a buffer does not settle within the bound
    /// computed by [`abort_timeout`].
    fn quiesce(&self) -> Result<(), Error> {
        for index in 0..self.tx_buffers {
            self.ll.set_tx_abort(index);
        }

        let deadline = Instant::now() + self.abort_timeout;
        for index in 0..self.tx_buffers {
            while matches!(
                self.ll.tx_buffer_state(index),
                TxBufferState::Ready | TxBufferState::InProgress | TxBufferState::AbortInProgress
            ) {
                if Instant::now() > deadline {
                    return Err(Error::AbortTimeout);
                }
            }
        }

        Ok(())
    }
}

impl Drop for BusGuard {
    fn drop(&mut self) {
        // Silence the controller before anything else. An interrupt source left
        // armed keeps the peripheral's request line asserted, and the clocks go
        // away with the guards that drop after this one: the handler can then no
        // longer reach the registers to clear what it is being woken for, and
        // the CPU spins in the trap for a request nothing can retire. Masking
        // the sources drops the line, and disabling delivery means a handler
        // cannot run against a peripheral that is no longer clocked.
        self.ll.disable_interrupts(u32::MAX);
        self.ll.clear_interrupts(u32::MAX);
        crate::interrupt::disable(Cpu::current(), self.interrupt);

        // Best effort: a wedged transmission must not hang teardown, but the
        // ordinary case must not corrupt the bus either.
        let _ = self.quiesce();
        self.disable();
    }
}

impl crate::private::Sealed for CanFd<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for CanFd<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_interrupt_handler(handler);
    }
}

impl<'d> CanFd<'d, Blocking> {
    /// Registers an interrupt handler for this controller.
    ///
    /// Replaces any previously registered handler. The handler runs for the
    /// sources enabled with [`CanFd::listen`]. It must clear them with
    /// [`CanFd::clear_interrupts`], or it runs again as soon as it returns.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_interrupt_handler(handler);
    }

    /// Enables the given interrupt sources.
    ///
    /// Without a handler from [`CanFd::set_interrupt_handler`] this only makes
    /// [`CanFd::interrupts`] record what happened.
    ///
    /// Blocking only: the async driver owns the sources it needs, and a caller
    /// that disabled them would stop its futures from waking.
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .ll
            .enable_interrupts(CanFdInterrupt::mask(interrupts.into()));
    }

    /// Disables the given interrupt sources.
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .ll
            .disable_interrupts(CanFdInterrupt::mask(interrupts.into()));
    }

    /// Returns the interrupt sources that are currently asserted.
    pub fn interrupts(&self) -> EnumSet<CanFdInterrupt> {
        CanFdInterrupt::from_mask(self.bus.ll.interrupt_status())
    }

    /// Clears the given interrupt sources.
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .ll
            .clear_interrupts(CanFdInterrupt::mask(interrupts.into()));
    }

    /// Creates a new CAN FD driver in [`Blocking`] mode.
    ///
    /// The controller is configured but stays off the bus. Assign the pins with
    /// [`CanFd::with_rx`] and [`CanFd::with_tx`], then call [`CanFd::start`].
    ///
    /// # Errors
    ///
    /// [`ConfigError`] when the configuration does not fit the hardware.
    pub fn new(twai: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let twai = twai.degrade();
        let info = twai.info();
        let peripheral = PeripheralGuard::new(info.peripheral);
        let function_clock = FunctionClockGuard::new(info.clock_instance, config.clock_source);

        let ll = Ll::new(info.register_block);
        ll.reset();
        let tx_buffers = ll.tx_buffer_count();

        let abort_timeout = abort_timeout(
            &config.nominal_timing,
            &config.fd_timing,
            info.clock_instance.function_clock_frequency(),
        );

        let mut this = Self {
            bus: BusGuard {
                ll,
                tx_buffers,
                abort_timeout,
                interrupt: info.interrupt,
                wake_lock: None,
            },
            config,
            // Matches the reset value: filter A accepts everything, rest off.
            filter_kinds: [
                FrameKinds::ALL,
                FrameKinds::NONE,
                FrameKinds::NONE,
                FrameKinds::NONE,
            ],
            pins_bound: false,
            _mode: PhantomData,
            twai,
            _function_clock: function_clock,
            _peripheral: peripheral,
        };
        this.configure()?;

        Ok(this)
    }

    /// Converts the driver into an async driver.
    pub fn into_async(self) -> CanFd<'d, Async> {
        let mut this = CanFd {
            bus: self.bus,

            config: self.config,
            filter_kinds: self.filter_kinds,
            pins_bound: self.pins_bound,
            _mode: PhantomData,
            twai: self.twai,
            _function_clock: self._function_clock,
            _peripheral: self._peripheral,
        };
        let handler = this.twai.info().async_handler;
        this.bind_interrupt_handler(handler);

        this
    }
}

impl<'d> CanFd<'d, Async> {
    /// Waits until a frame is received, then reads it.
    ///
    /// # Cancellation Safety
    ///
    /// This future is cancel-safe. A frame stays in the hardware RX buffer
    /// until it is read, so dropping the future loses nothing.
    pub async fn receive_async(&mut self) -> Frame {
        self.rx_half().receive_async().await
    }

    /// Queues a frame and waits until the hardware finishes with it.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerStopped`] when the controller is not on the bus, and
    /// [`Error::NoFreeTxBuffer`] when every TX buffer is occupied; see
    /// [`CanFdTx::transmit`].
    ///
    /// [`Error::TransmitFailed`] when the controller gave up on the frame.
    ///
    /// [`Error::TransmitAborted`] when the transmission was aborted.
    ///
    /// # Cancellation Safety
    ///
    /// This future is not cancel-safe. Dropping it leaves the frame queued, and
    /// the hardware may still transmit it.
    pub async fn transmit_async(&mut self, frame: &Frame) -> Result<(), Error> {
        self.tx_half().transmit_async(frame).await
    }

    /// Converts the driver back into a blocking driver.
    pub fn into_blocking(self) -> CanFd<'d, Blocking> {
        self.bus.ll.disable_interrupts(u32::MAX);
        crate::interrupt::disable(Cpu::current(), self.twai.info().interrupt);

        CanFd {
            bus: self.bus,

            config: self.config,
            filter_kinds: self.filter_kinds,
            pins_bound: self.pins_bound,
            _mode: PhantomData,
            twai: self.twai,
            _function_clock: self._function_clock,
            _peripheral: self._peripheral,
        }
    }
}

impl<'d, Dm: DriverMode> CanFd<'d, Dm> {
    fn bind_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt = self.twai.info().interrupt;
        crate::interrupt::disable(Cpu::current(), interrupt);
        self.bus.ll.disable_interrupts(u32::MAX);
        self.bus.ll.clear_interrupts(u32::MAX);
        crate::interrupt::bind_handler(interrupt, handler);
        crate::interrupt::enable(interrupt, handler.priority());
    }

    /// Assigns the RX pin.
    ///
    /// The pin is driven according to [`Config::with_no_transceiver`]. That
    /// setting is committed to the hardware here and cannot change afterwards.
    pub fn with_rx(mut self, rx: impl PeripheralInput<'d>) -> Self {
        let rx = rx.into();
        rx.apply_input_config(&InputConfig::default().with_pull(self.pin_pull()));
        rx.set_input_enable(true);
        self.twai.info().rx_signal.connect_to(&rx);
        self.pins_bound = true;

        self
    }

    /// Assigns the TX pin.
    ///
    /// With [`Config::with_no_transceiver`] set, the pin is driven open-drain
    /// with a pull-up. Assigning the same pin to [`CanFd::with_rx`] then forms
    /// a single-node bus that needs no external wiring. That setting is
    /// committed to the hardware here and cannot change afterwards.
    pub fn with_tx(mut self, tx: impl PeripheralOutput<'d>) -> Self {
        let tx = tx.into();
        let mut output = OutputConfig::default();
        if self.config.no_transceiver {
            output = output
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up);
        }
        tx.apply_output_config(&output);
        tx.set_output_enable(true);
        self.twai.info().tx_signal.connect_to(&tx);
        self.pins_bound = true;

        self
    }

    fn pin_pull(&self) -> Pull {
        if self.config.no_transceiver {
            Pull::Up
        } else {
            Pull::None
        }
    }

    /// Applies a new configuration.
    ///
    /// The controller leaves the bus while the configuration is applied. A
    /// running driver must be started again with [`CanFd::start`] afterwards.
    /// A change of clock source also stops a running timestamp counter,
    /// because its resolution is derived from that clock. Start it again with
    /// [`CanFd::start_timestamp_timer`] and use the resolution that it returns.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] when the configuration does not fit the hardware.
    ///
    /// [`ConfigError::TransceiverModeLocked`] when the configuration changes
    /// [`Config::with_no_transceiver`] after a pin has been assigned.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Validate before storing, so a rejected configuration leaves `config()`
        // describing the hardware rather than a state never applied.
        config.validate()?;

        // Whether a transceiver is in the way decides how the pins are driven,
        // and the pins are configured when they are assigned. The driver does
        // not keep them afterwards, so this can no longer be applied — and
        // accepting it would leave `config()` describing an output stage the
        // hardware does not have.
        if self.pins_bound && config.no_transceiver != self.config.no_transceiver {
            return Err(ConfigError::TransceiverModeLocked);
        }

        // The timestamp prescaler divides the function clock, so pointing that
        // clock somewhere else silently rescales a running counter: the
        // resolution `start_timestamp_timer` reported would quietly stop being
        // the resolution it counts at. Stop it instead, and let the caller ask
        // for a resolution again against the new clock.
        if config.clock_source != self.config.clock_source {
            self.stop_timestamp_timer();
        }

        // `configure` can still refuse, on a bus that will not go quiet, and it
        // refuses before writing anything. Put the old configuration back in
        // that case, for the same reason: `config()` must describe the hardware.
        let previous = self.config;
        self.config = *config;
        self.configure().inspect_err(|_| self.config = previous)
    }

    /// Returns the current configuration.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Writes the stored configuration to the hardware, leaving the bus.
    fn configure(&mut self) -> Result<(), ConfigError> {
        self.config.validate()?;

        // Leave the bus cleanly before touching anything. Both the clock switch
        // and clearing ENA cut a transmission in flight, and the peers see a
        // corrupt frame rather than a node reconfiguring itself.
        if self.bus.ll.is_enabled() {
            self.bus.quiesce().map_err(|_| ConfigError::BusBusy)?;
        }

        // Re-point the clock mux next. Every timing parameter below is expressed
        // in periods of this clock (TRM 38.3.1), so programming the timing while
        // the mux still selects the previous source yields a bit rate that
        // silently differs from the configured one.
        self._function_clock.set_source(self.config.clock_source);

        // Mode bits can only be changed while the controller is disabled.
        self.bus.disable();

        let config = self.config;
        let (listen_only, self_test, loopback) = match config.mode {
            Mode::Normal => (false, false, false),
            Mode::ListenOnly => (true, false, false),
            Mode::SelfTest => (false, true, false),
            Mode::LoopbackSelfTest => (false, true, true),
        };

        self.bus.ll.apply_mode_settings(&ll::ModeSettings {
            listen_only,
            self_test,
            loopback,
            fd_enabled: true,
            protocol_exception: config.protocol_exception,
            rx_auto_increment: true,
            filters_enabled: true,
            bus_off_tx_fail: true,
            drop_request_frames: config.drop_request_frames,
            time_triggered_tx: false,
            retransmit_limit: config.retransmit_limit,
        });

        self.bus.ll.set_timestamp_point(TimestampPoint::EndOfFrame);

        self.bus.ll.set_nominal_timing(&config.nominal_timing);
        self.bus.ll.set_fd_timing(&config.fd_timing);

        // How long a frame takes is a function of the timing just programmed,
        // so the next teardown waits for a frame at the new bit rate.
        let timeout = abort_timeout(
            &config.nominal_timing,
            &config.fd_timing,
            self.function_clock_frequency(),
        );
        self.bus.abort_timeout = timeout;

        match config.secondary_sample_point_offset {
            // The hardware counts the offset in function clock cycles, not quanta.
            Some(offset) => self.bus.ll.set_secondary_sample_point(
                SspSource::MeasuredPlusOffset,
                offset.saturating_mul(config.fd_timing.baud_rate_prescaler),
            ),
            None => self
                .bus
                .ll
                .set_secondary_sample_point(SspSource::Disabled, 0),
        }

        Ok(())
    }

    /// Joins the bus and waits for the controller to become error-active.
    ///
    /// Follows the initialization sequence in TRM 38.3.5.
    ///
    /// # Errors
    ///
    /// [`Error::BusIntegrationTimeout`] when the bus does not go idle-recessive
    /// long enough for the controller to join it. The controller stays off the
    /// bus in that case, unless it was on the bus before the call.
    pub fn start(&mut self) -> Result<(), Error> {
        let timeout = integration_timeout(
            &self.config.nominal_timing,
            &self.config.fd_timing,
            self.function_clock_frequency(),
        );

        let was_enabled = self.bus.ll.is_enabled();
        self.bus.enable();

        // TRM 38.3.5 step 4: integration finishes when the controller becomes
        // error-active, which takes 11 consecutive recessive bits.
        let deadline = Instant::now() + timeout;
        loop {
            if self.bus.ll.error_state() == ErrorState::Active {
                return Ok(());
            }
            if Instant::now() > deadline {
                // Undo exactly what this call did. Leaving a controller enabled
                // behind a returned error lets it join the bus later on its own,
                // after the caller has already taken the failure branch.
                if !was_enabled {
                    self.bus.disable();
                }
                return Err(Error::BusIntegrationTimeout);
            }
        }
    }

    /// Returns the identity of the core.
    pub fn identity(&self) -> Identity {
        let (major, minor) = self.bus.ll.version();
        Identity {
            device_id: self.bus.ll.device_id(),
            version_major: major,
            version_minor: minor,
        }
    }

    /// Returns the number of TX buffers the hardware provides.
    pub fn tx_buffer_count(&self) -> u8 {
        self.bus.tx_buffers
    }

    /// Splits the controller into a receiving and a transmitting half.
    ///
    /// The two halves can be used at the same time, for example to wait for a
    /// frame while a transmission is in flight. The whole-controller methods
    /// cannot do that, because each of them borrows the whole driver. The
    /// halves arm different interrupt sources through the separate set and
    /// clear registers of the hardware, so neither disturbs the other.
    ///
    /// The halves borrow the driver. The driver keeps the configuration and the
    /// teardown: everything that reconfigures the controller or takes it off
    /// the bus stays on [`CanFd`] and is unavailable while a half is alive. For
    /// two independent tasks, split a driver that lives in a `static`; the
    /// halves then borrow it for `'static` as well.
    pub fn split(&mut self) -> (CanFdRx<'_, Dm>, CanFdTx<'_, Dm>) {
        let (info, state) = self.twai.parts();
        (
            CanFdRx {
                ll: Ll::new(info.register_block),
                state,
                _driver: PhantomData,
                _mode: PhantomData,
            },
            CanFdTx {
                ll: Ll::new(info.register_block),
                state,
                tx_buffers: self.bus.tx_buffers,
                _driver: PhantomData,
                _mode: PhantomData,
            },
        )
    }

    fn rx_half(&mut self) -> CanFdRx<'_, Dm> {
        self.split().0
    }

    fn tx_half(&mut self) -> CanFdTx<'_, Dm> {
        self.split().1
    }

    /// Returns the frequency of the controller's function clock, in Hz.
    pub fn function_clock_frequency(&self) -> u32 {
        self.twai.info().clock_instance.function_clock_frequency()
    }

    /// Leaves the bus, following the de-initialization sequence in TRM 38.3.6.
    ///
    /// Every TX buffer returns to the empty state without its memory being
    /// cleared, and the RX buffer is flushed.
    ///
    /// While the controller is on the bus, the driver holds a [`WakeLock`],
    /// because Light-sleep mode gates the function clock. Leaving the bus
    /// releases the lock, so the chip can sleep afterwards. A running timestamp
    /// counter does not advance during sleep.
    ///
    /// # Errors
    ///
    /// [`Error::AbortTimeout`] when a transmission does not settle. The
    /// controller stays on the bus in that case.
    pub fn stop(&mut self) -> Result<(), Error> {
        self.bus.quiesce()?;
        self.bus.disable();
        Ok(())
    }

    /// Returns whether the controller is on the bus.
    ///
    /// This is true between a successful [`CanFd::start`] and the next
    /// [`CanFd::stop`] or [`CanFd::apply_config`]. Transmitting needs it to be
    /// true, and changing the error warning limit needs it to be false.
    pub fn is_started(&self) -> bool {
        self.bus.ll.is_enabled()
    }

    /// Queues a frame in the first free TX buffer and arms it.
    ///
    /// Returns the index of the buffer used.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerStopped`] when the controller is not on the bus, and
    /// [`Error::NoFreeTxBuffer`] when every TX buffer is occupied; see
    /// [`CanFdTx::transmit`].
    pub fn transmit(&mut self, frame: &Frame) -> Result<u8, Error> {
        self.tx_half().transmit(frame)
    }

    /// Returns the state of one TX buffer.
    ///
    /// Returns [`TxBufferState::NotExist`] for an index the hardware does not
    /// have.
    pub fn tx_buffer_state(&self, index: u8) -> TxBufferState {
        // Checked rather than passed through: the states share one register,
        // four bits each, so an unchecked index reads a neighbor's field or
        // runs off the end of the word entirely.
        if index >= self.bus.tx_buffers {
            return TxBufferState::NotExist;
        }
        self.bus.ll.tx_buffer_state(index)
    }

    /// Requests that a queued transmission be aborted.
    ///
    /// The buffer moves to "aborted", or to "abort in progress" if the frame is
    /// already on the wire. Does nothing for an index the hardware does not
    /// have.
    pub fn abort_transmit(&mut self, index: u8) {
        if index < self.bus.tx_buffers {
            self.bus.ll.set_tx_abort(index);
        }
    }

    /// Returns a TX buffer to the empty state, releasing it for reuse.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn release_tx_buffer(&mut self, index: u8) {
        if index < self.bus.tx_buffers {
            self.bus.ll.set_tx_empty(index);
        }
    }

    /// Sets a TX buffer's arbitration priority.
    ///
    /// Higher values win. Equal priorities are resolved in favor of the lower
    /// buffer index (TRM 38.3.8.1). The hardware field holds up to
    /// [`MAX_TX_PRIORITY`], and a larger value counts as that maximum. Does
    /// nothing for an index the hardware does not have.
    pub fn set_tx_priority(&mut self, index: u8, priority: u8) {
        if index < self.bus.tx_buffers {
            // Saturated rather than truncated: the field is three bits, and
            // letting the write mask the value would turn 8 into 0 — the
            // buffer the caller ranked highest would go last.
            self.bus
                .ll
                .set_tx_priority(index, priority.min(MAX_TX_PRIORITY));
        }
    }

    /// Returns the size and the free space of the RX buffer, as `(size, free)`
    /// in 32-bit words.
    pub fn rx_buffer_words(&self) -> (u16, u16) {
        (self.bus.ll.rx_buffer_size(), self.bus.ll.rx_free_words())
    }

    /// Returns whether the RX buffer has overrun and dropped a frame
    /// (TRM 38.3.9.4).
    pub fn rx_overrun(&self) -> bool {
        self.bus.ll.rx_overrun()
    }

    /// Clears the RX buffer overrun flag.
    pub fn clear_rx_overrun(&mut self) {
        self.bus.ll.clear_overrun();
    }

    /// Returns the number of complete frames waiting in the RX buffer.
    pub fn rx_frame_count(&self) -> u16 {
        self.bus.ll.rx_frame_count()
    }

    /// Reads one frame from the RX buffer.
    ///
    /// # Errors
    ///
    /// [`Error::RxBufferEmpty`] when no complete frame is waiting.
    pub fn receive(&mut self) -> Result<Frame, Error> {
        self.rx_half().receive()
    }

    /// Returns the current fault confinement state.
    ///
    /// Returns [`ErrorState::BusOff`] while the controller is off the bus, both
    /// before the first [`CanFd::start`] and after [`CanFd::stop`]. That is how
    /// the hardware reports a disabled controller. [`CanFd::is_started`] tells
    /// the two apart.
    pub fn error_state(&self) -> ErrorState {
        self.bus.ll.error_state()
    }

    /// Returns whether an error counter has reached the error warning limit.
    ///
    /// A node at the limit is still [`ErrorState::Active`]: the limit raises a
    /// flag and, when enabled, [`CanFdInterrupt::ErrorWarning`], but it is not a
    /// fault confinement state of its own (TRM 38.3.10).
    pub fn error_warning(&self) -> bool {
        self.bus.ll.error_warning()
    }

    /// Returns the receive and transmit error counters, as `(rec, tec)`.
    pub fn error_counters(&self) -> (u16, u16) {
        (self.bus.ll.rec(), self.bus.ll.tec())
    }

    /// Requests that a bus-off controller rejoin the bus.
    ///
    /// Does nothing unless the controller is bus-off, so it cannot clear the
    /// counters of a running node and cannot arm a later reintegration.
    /// Rejoining takes 128 occurrences of 11 recessive bits, as ISO 11898-1
    /// requires.
    ///
    /// Leaving the bus with [`CanFd::stop`] and joining it again with
    /// [`CanFd::start`] also clears the error counters and the bus-off state.
    /// Measured on an ESP32-C5, that path rejoins after the 11 recessive bits
    /// of ordinary integration rather than the 128 × 11 of recovery.
    ///
    /// The state is checked here rather than left to the hardware, because the
    /// hardware does not behave the way the register description says. TRM
    /// register 38.3 documents `ERCRST` as having no effect outside bus-off,
    /// while TRM 38.3.4 calls the error state sticky. Measured on an ESP32-C5,
    /// the sticky reading wins: a request issued while the controller is
    /// error-active is remembered, and the controller then rejoins the bus on
    /// its own the next time it goes bus-off. A caller that polled this method
    /// would silently turn a one-shot recovery into an automatic one.
    pub fn request_bus_off_recovery(&mut self) {
        if self.bus.ll.error_state() == ErrorState::BusOff {
            self.bus.ll.request_bus_off_recovery();
        }
    }

    /// Sets the error warning limit, which defaults to 96.
    ///
    /// Moving the limit takes the controller outside ISO 11898-1, so the
    /// hardware only accepts the write in test mode (TRM 38.3.10). Test mode is
    /// entered for the write and left again, because it also makes the error
    /// counters writable.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerRunning`] when the controller is still on the bus.
    /// Test mode can only be entered while it is off.
    pub fn set_error_warning_limit(&mut self, limit: u8) -> Result<(), Error> {
        if self.bus.ll.is_enabled() {
            return Err(Error::ControllerRunning);
        }
        self.bus.ll.enable_test_mode(true);
        self.bus.ll.set_error_warning_limit(limit);
        self.bus.ll.enable_test_mode(false);
        Ok(())
    }

    /// Resets the received and transmitted frame counters.
    pub fn reset_traffic_counters(&mut self) {
        self.bus.ll.reset_traffic_counters();
    }

    /// Returns the numbers of frames received and transmitted since the
    /// counters were last reset, as `(rx, tx)`.
    pub fn traffic_counters(&self) -> (u32, u32) {
        (
            self.bus.ll.rx_traffic_counter(),
            self.bus.ll.tx_traffic_counter(),
        )
    }

    /// Discards everything in the RX buffer.
    pub fn flush_rx(&mut self) {
        self.bus.ll.flush_rx();
    }

    // ----------------------------------------------------------------- filters

    /// Configures one mask filter.
    ///
    /// A frame reaches the RX buffer if it passes at least one enabled filter,
    /// so filters combine as a logical OR (TRM 38.3.9.8).
    ///
    /// Out of reset, filter A accepts every frame kind with a zero mask, so the
    /// controller receives everything until a filter is configured.
    ///
    /// # Errors
    ///
    /// [`ConfigError::FilterIdTooLarge`] when the identifier or the mask does
    /// not fit the identifier of the chosen format.
    pub fn set_mask_filter(
        &mut self,
        filter: MaskFilter,
        config: &MaskFilterConfig,
    ) -> Result<(), ConfigError> {
        // The registers hold 11 or 29 bits, so a wider value would be masked on
        // the way in and the filter would quietly match something other than
        // what it was configured with — a filter for 0x800 would become a filter
        // for 0x000.
        check_filter_id(config.id, config.extended)?;
        check_filter_id(config.mask, config.extended)?;

        self.write_mask_filter(filter, config);
        Ok(())
    }

    fn write_mask_filter(&mut self, filter: MaskFilter, config: &MaskFilterConfig) {
        self.bus
            .ll
            .set_mask_filter(filter, config.extended, config.id, config.mask);
        self.filter_kinds[Self::mask_filter_index(filter)] = config.accepts;
        self.apply_filter_kinds();
    }

    /// Disables one mask filter.
    pub fn disable_mask_filter(&mut self, filter: MaskFilter) {
        self.filter_kinds[Self::mask_filter_index(filter)] = FrameKinds::NONE;
        self.apply_filter_kinds();
    }

    /// Configures the range filter.
    ///
    /// # Errors
    ///
    /// [`ConfigError::FilterIdTooLarge`] when either bound does not fit the
    /// identifier of the chosen format.
    pub fn set_range_filter(&mut self, config: &RangeFilterConfig) -> Result<(), ConfigError> {
        check_filter_id(config.low, config.extended)?;
        check_filter_id(config.high, config.extended)?;

        self.bus
            .ll
            .set_range_filter(config.extended, config.low, config.high);
        self.filter_kinds[RANGE_FILTER_INDEX] = config.accepts;
        self.apply_filter_kinds();
        Ok(())
    }

    /// Disables the range filter.
    pub fn disable_range_filter(&mut self) {
        self.filter_kinds[RANGE_FILTER_INDEX] = FrameKinds::NONE;
        self.apply_filter_kinds();
    }

    /// Accepts every frame, by giving filter A a zero mask and disabling the rest.
    pub fn accept_all(&mut self) {
        // Written directly rather than through the checked setter: zero fits
        // every format, so there is nothing here for a caller to handle.
        self.write_mask_filter(
            MaskFilter::A,
            &MaskFilterConfig {
                id: 0,
                mask: 0,
                extended: false,
                accepts: FrameKinds::ALL,
            },
        );
        self.disable_mask_filter(MaskFilter::B);
        self.disable_mask_filter(MaskFilter::C);
        self.disable_range_filter();
    }

    /// Returns which filters this core implements, as `(a, b, c, range)`.
    pub fn filters_supported(&self) -> (bool, bool, bool, bool) {
        self.bus.ll.filters_supported()
    }

    fn mask_filter_index(filter: MaskFilter) -> usize {
        match filter {
            MaskFilter::A => 0,
            MaskFilter::B => 1,
            MaskFilter::C => 2,
        }
    }

    fn apply_filter_kinds(&mut self) {
        let [a, b, c, range] = self.filter_kinds;
        self.bus.ll.set_filter_kinds(a, b, c, range);
    }

    // ------------------------------------------------------------ error detail

    /// Returns details of the last bus error the core captured.
    ///
    /// Only meaningful after the core has reported a bus error; see
    /// [`ErrorCapture`].
    pub fn error_capture(&self) -> ErrorCapture {
        self.bus.ll.error_capture()
    }

    /// Returns the number of retransmission attempts made for the frame
    /// currently being sent.
    pub fn retransmit_count(&self) -> u8 {
        self.bus.ll.retransmit_count()
    }

    /// Returns the error counters of the nominal and data phases, as
    /// `(nominal, fd)`.
    pub fn phase_error_counters(&self) -> (u16, u16) {
        self.bus.ll.special_error_counters()
    }

    /// Returns the transmitter delay the core measured, in function clock
    /// periods.
    ///
    /// The core measures the delay during every FD frame it sends, whether or
    /// not the frame switches bit rate (TRM 38.3.7.2). The value reads zero
    /// until the first FD frame has been transmitted. The secondary sample
    /// point sits at this delay plus the configured offset, and the
    /// four-bit-time limit of TRM 38.3.7.3 applies to that sum. The driver can
    /// only check the offset, so the sum must be checked against the hardware
    /// in use.
    pub fn transmitter_delay(&self) -> u8 {
        self.bus.ll.transmitter_delay()
    }

    // -------------------------------------------------------------- timestamps

    /// Starts the timestamp counter from zero, at the requested resolution.
    ///
    /// Received frames are stamped at the sixth bit of their end-of-frame
    /// field, which is when a frame becomes valid. The achievable resolution is
    /// the function clock divided by an integer, so the returned value is the
    /// resolution actually programmed, in Hz.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnsupportedTimestampResolution`] when the resolution
    /// cannot be reached from the function clock.
    ///
    /// [`ConfigError::TimestampTimerStalled`] when the counter does not start.
    pub fn start_timestamp_timer(&mut self, resolution_hz: u32) -> Result<u32, ConfigError> {
        let clock = self.function_clock_frequency();
        if resolution_hz == 0 || resolution_hz > clock {
            return Err(ConfigError::UnsupportedTimestampResolution);
        }

        let divider = clock / resolution_hz;
        let divider =
            u16::try_from(divider).map_err(|_| ConfigError::UnsupportedTimestampResolution)?;

        // Reprogram while the counter is stopped and clear it before starting,
        // so a restart at another resolution begins from a known value instead
        // of carrying over counts made in the previous unit, and so the first
        // interval measured against it is not short by whatever the prescaler
        // had already accumulated.
        self.bus.ll.timer_enable(false);
        self.bus.ll.timer_enable_config_clock(true);

        self.bus.ll.timer_set_divider(divider);
        self.bus.ll.timer_count_up(true);
        self.bus.ll.timer_set_free_running();
        self.bus.ll.timer_enable(true);
        self.bus.ll.timer_clear();

        // Wait for the counter to actually start.
        //
        // The prescaler keeps its phase across a reprogram, and it matches the
        // divider rather than reloading from it, so a divider below the phase it
        // happens to hold is missed until the 16-bit prescaler wraps all the way
        // around: up to 65536 function clock periods, 819 us at 80 MHz, during
        // which the counter does not move. Measured on an ESP32-C5, this catches
        // roughly a third of the restarts that lower the divider, and nothing
        // resets that phase — clearing the counter, disabling the timer and
        // gating its configuration clock all leave it untouched. Returning
        // during the stall would hand back a timer that reports the right
        // resolution and stamps the next millisecond of frames as if no time
        // had passed, so wait it out instead.
        let cycles = u64::from(PRESCALER_PERIOD) + 2 * u64::from(divider);
        let deadline = Instant::now()
            + Duration::from_micros(cycles * 1_000_000 / u64::from(clock))
            + Duration::from_millis(1);
        while self.bus.ll.timer_count() == 0 {
            if Instant::now() > deadline {
                return Err(ConfigError::TimestampTimerStalled);
            }
        }

        Ok(clock / u32::from(divider))
    }

    /// Stops the timestamp counter.
    pub fn stop_timestamp_timer(&mut self) {
        self.bus.ll.timer_enable(false);
        self.bus.ll.timer_enable_config_clock(false);
    }

    /// Returns the current value of the timestamp counter.
    pub fn timestamp(&self) -> u64 {
        self.bus.ll.timer_count()
    }

    /// Returns the width of the timestamp counter in bits.
    pub fn timestamp_bit_width(&self) -> u8 {
        self.bus.ll.timer_bit_width()
    }
}

crate::any_peripheral! {
    /// Any CAN FD peripheral.
    pub peripheral AnyCanFd<'d> {
        Twai0(crate::peripherals::TWAI0<'d>),
        Twai1(crate::peripherals::TWAI1<'d>),
    }
}

/// Peripheral data describing one CAN FD controller.
#[doc(hidden)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block of this controller.
    pub register_block: *const crate::pac::twai0::RegisterBlock,
    /// The system peripheral marker.
    pub peripheral: Peripheral,
    /// Clock tree node of this controller.
    pub clock_instance: TwaiInstance,
    /// The controller's interrupt.
    pub interrupt: Interrupt,
    /// Interrupt handler for the asynchronous operations of this controller.
    pub async_handler: InterruptHandler,
    /// RX signal.
    pub rx_signal: InputSignal,
    /// TX signal.
    pub tx_signal: OutputSignal,
}

// SAFETY: `Info` only holds a pointer to a register block that exists for the
// whole life of the program, plus plain data.
unsafe impl Sync for Info {}

/// A peripheral singleton compatible with the CAN FD driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    /// Returns the peripheral data and state describing this controller.
    #[doc(hidden)]
    fn parts(&self) -> (&'static Info, &'static asynch::State);

    /// Returns the peripheral data describing this controller.
    #[doc(hidden)]
    #[inline(always)]
    fn info(&self) -> &'static Info {
        self.parts().0
    }

    /// Returns the peripheral state of this controller.
    #[doc(hidden)]
    #[inline(always)]
    fn state(&self) -> &'static asynch::State {
        self.parts().1
    }
}

for_each_canfd! {
    ($instance:ident, $sys:ident, $rx:ident, $tx:ident) => {
        impl Instance for crate::peripherals::$instance<'_> {
            fn parts(&self) -> (&'static Info, &'static asynch::State) {
                #[crate::handler]
                fn irq_handler() {
                    asynch::handle(&INFO, &STATE);
                }

                static STATE: asynch::State = asynch::State::new();

                static INFO: Info = Info {
                    register_block: crate::peripherals::$instance::ptr(),
                    peripheral: Peripheral::$sys,
                    clock_instance: TwaiInstance::$sys,
                    interrupt: Interrupt::$instance,
                    async_handler: irq_handler,
                    rx_signal: InputSignal::$rx,
                    tx_signal: OutputSignal::$tx,
                };

                (&INFO, &STATE)
            }
        }
    };
}

impl Instance for AnyCanFd<'_> {
    #[inline]
    fn parts(&self) -> (&'static Info, &'static asynch::State) {
        any::delegate!(self, twai => { twai.parts() })
    }
}

/// Index of the range filter in `CanFd::filter_kinds`.
const RANGE_FILTER_INDEX: usize = 3;

/// Highest TX buffer priority the hardware can hold; see
/// [`CanFd::set_tx_priority`].
pub const MAX_TX_PRIORITY: u8 = 7;

/// Function clock periods the timestamp prescaler takes to wrap.
///
/// Not documented as a width; measured on an ESP32-C5, where a divider missed
/// by the prescaler's phase costs exactly this many periods before the counter
/// moves again.
const PRESCALER_PERIOD: u32 = 1 << 16;

/// Transmitter delay the core adds on its own, in function clock periods.
///
/// The measured delay (TRM 38.3.7.3) includes two periods of the core's input
/// delay before any transceiver is in the loop, so this is the floor of what a
/// secondary sample point has to leave room for.
const MIN_TRANSMITTER_DELAY_CYCLES: u32 = 2;

/// Bit times to allow for the controller to join the bus.
///
/// The eleven consecutive recessive bits integration waits for cannot appear
/// until whatever is on the wire has finished, so this has to cover a frame
/// already in flight when the controller joins, and then the integration
/// itself. The longest CAN FD frame is under 800 bit times. Anything smaller
/// turns an ordinary exchange between two other nodes into a failure to start:
/// the payload of a 64-byte frame is 512 bits on its own.
const INTEGRATION_TIMEOUT_BITS: u64 = 2048;

/// Shortest wait for bus integration, whatever the bit rate.
///
/// At megabit rates the derived bound is a matter of microseconds, which says
/// more about the speed of the polling loop than about the bus.
const INTEGRATION_TIMEOUT_FLOOR: Duration = Duration::from_millis(50);

/// Bits to allow for one frame while waiting for a transmission to settle.
///
/// An abort only takes effect once the frame in flight has finished, so the
/// bound has to cover a whole frame. The longest CAN FD frame, with an extended
/// identifier, 64 data bytes and worst-case stuffing, is under 800 bit times.
const ABORT_TIMEOUT_BITS: u64 = 1024;

/// How long a transmission may take to finish or abort while leaving the bus.
///
/// A deadline rather than a spin count, so the bound means the same thing
/// whatever the CPU clock is, and derived from the bit rate rather than fixed:
/// at 5 kbit/s a full frame takes over a hundred milliseconds, so any constant
/// short enough to be useful at 1 Mbit/s would cut a legal frame short.
///
/// Both phases are considered, and the slower one wins. A bit rate switched
/// frame spends its data field at the FD timing, the two prescalers are
/// independent (TRM 38.3.7.1), and nothing in the hardware or in
/// [`Config::validate`] requires the data phase to be the faster of the two. A
/// bound taken from the arbitration timing alone gives up in the middle of a
/// legal frame whenever it is not.
fn abort_timeout(nominal: &Timing, fd: &Timing, function_clock_hz: u32) -> Duration {
    // The added millisecond keeps a fast bit rate from timing out on the
    // register accesses of the polling loop itself.
    bit_times(
        slowest_cycles_per_bit(nominal, fd),
        ABORT_TIMEOUT_BITS,
        function_clock_hz,
    ) + Duration::from_millis(1)
}

/// How long the controller may take to join the bus.
///
/// A deadline rather than a spin count: how far a spin count gets depends on the
/// CPU clock, so the same loop that is generous at one CPU frequency can give up
/// early at another, and at the slowest configurable bit rate joining takes over
/// a hundred milliseconds of real time either way.
///
/// Both phases count, for the same reason they do in [`abort_timeout`]. What is
/// being waited for is the wire going quiet, and the frame occupying it belongs
/// to whichever node is transmitting: its data field runs at its own FD timing,
/// which the local arbitration timing says nothing about.
fn integration_timeout(nominal: &Timing, fd: &Timing, function_clock_hz: u32) -> Duration {
    let derived = bit_times(
        slowest_cycles_per_bit(nominal, fd),
        INTEGRATION_TIMEOUT_BITS,
        function_clock_hz,
    );

    if derived < INTEGRATION_TIMEOUT_FLOOR {
        INTEGRATION_TIMEOUT_FLOOR
    } else {
        derived
    }
}

/// Checks that a filter's identifier fits the format it is configured for.
fn check_filter_id(id: u32, extended: bool) -> Result<(), ConfigError> {
    let limit = if extended { EXT_ID_MASK } else { STD_ID_MASK };
    if id > limit {
        return Err(ConfigError::FilterIdTooLarge);
    }
    Ok(())
}

/// Function clock periods one bit takes in the slower of the two phases.
///
/// Every wait for a frame to pass is bounded by this: a bit rate switched frame
/// spends its payload at the FD timing, the two prescalers are independent (TRM
/// 38.3.7.1), and nothing requires the data phase to be the faster of the two.
fn slowest_cycles_per_bit(nominal: &Timing, fd: &Timing) -> u64 {
    let cycles_per_bit =
        |timing: &Timing| u64::from(timing.baud_rate_prescaler) * u64::from(timing.total_quanta());
    cycles_per_bit(nominal).max(cycles_per_bit(fd))
}

/// How long `bits` bit times last, given the clock periods one bit takes.
///
/// Scales by the whole count before dividing: a bit at 2 Mbit/s is half a
/// microsecond, and rounding each bit to whole microseconds first would round
/// the whole span away.
fn bit_times(cycles_per_bit: u64, bits: u64, function_clock_hz: u32) -> Duration {
    let micros = cycles_per_bit
        .saturating_mul(bits)
        .saturating_mul(1_000_000)
        / u64::from(function_clock_hz).max(1);

    Duration::from_micros(micros)
}

struct FunctionClockGuard {
    instance: TwaiInstance,
}

impl FunctionClockGuard {
    fn new(instance: TwaiInstance, clock_source: ClockSource) -> Self {
        ClockTree::with(|clocks| {
            // The mux has no configuration until a driver selects one, and
            // `request_function_clock` requires it to be set.
            instance.configure_function_clock(clocks, clock_source);
            instance.request_function_clock(clocks);
        });
        Self { instance }
    }

    /// Re-points the mux at another source, keeping the request refcount.
    fn set_source(&self, clock_source: ClockSource) {
        ClockTree::with(|clocks| {
            self.instance.configure_function_clock(clocks, clock_source);
        });
    }
}

impl Drop for FunctionClockGuard {
    fn drop(&mut self) {
        ClockTree::with(|clocks| self.instance.release_function_clock(clocks));
    }
}
