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
//! Identifiers are the [`embedded_can`] types. [`ClassicFrame`] implements
//! [`embedded_can::Frame`] and [`BusErrorKind`] implements
//! [`embedded_can::Error`], for code written against those traits.
//!
//! ## Examples
//!
//! ### Sending a CAN FD frame
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::canfd::{CanFd, Config, Frame, StandardId};
//!
//! let mut canfd = CanFd::new(peripherals.TWAI0, Config::default())?
//!     .with_rx(peripherals.GPIO9)
//!     .with_tx(peripherals.GPIO8);
//! canfd.start()?;
//!
//! // A 64-byte frame, sent with the data phase at the FD bit rate.
//! let id = StandardId::new(0x123).unwrap();
//! let frame = Frame::new_fd(id, &[0xAA; 64])?.with_bit_rate_switch(true);
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
//! "TRM" in this module means the ESP32-C5 TRM v1.1, chapter 38.
#![doc = crate::trm_markdown_link!("#canfd")]

use core::marker::PhantomData;

pub use embedded_can::{ExtendedId, Id, StandardId};
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
    rtc_cntl::WakeLock,
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
    MAX_RETRANSMIT_LIMIT,
    MAX_TX_PRIORITY,
    MaskFilter,
    NOMINAL_TIMING_LIMITS,
    Timing,
    TimingLimits,
    TxBufferState,
};
use ll::{
    CLASSIC_MAX_DATA_LEN,
    Driver,
    EXT_ID_MASK,
    FrameBuffer,
    FrameHeader,
    MAX_DATA_LEN,
    STD_ID_MASK,
    SspSource,
    TimestampPoint,
    len_to_dlc,
};

/// Clock source for the CAN FD peripheral.
pub use crate::soc::clocks::TwaiFunctionClockConfig as ClockSource;

/// The device ID every CTU CAN FD core reports.
const CTU_CAN_FD_DEVICE_ID: u16 = 0xCAFD;

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
    /// A frame switched to the data bit rate, on either side of the transfer.
    BitRateShifted,
    /// The RX buffer is no longer empty.
    RxNotEmpty,
    /// A TX buffer finished, successfully or not.
    TxDone,
}

impl CanFdInterrupt {
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

impl Identity {
    /// Returns whether the device ID is the one a CTU CAN FD core reports.
    pub fn is_ctu_can_fd(&self) -> bool {
        self.device_id == CTU_CAN_FD_DEVICE_ID
    }
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
    /// Also enables self test, because loopback alone still requires an ACK.
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
    /// The retransmission limit is larger than [`MAX_RETRANSMIT_LIMIT`].
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
    /// The frame is a CAN FD frame, which a [`ClassicFrame`] cannot hold.
    NotClassic,
}

impl core::error::Error for FrameError {}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::PayloadTooLong => "The payload is longer than the frame format can carry",
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
    /// The first attempt is not a retransmission. The hardware holds at most
    /// [`MAX_RETRANSMIT_LIMIT`].
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
        // 500 kbit/s nominal and 2 Mbit/s data from 80 MHz, sampling at 81.2%
        // and 75%.
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
    fn validate(&self) -> Result<(), ConfigError> {
        if !self.nominal_timing.is_valid(&NOMINAL_TIMING_LIMITS) {
            return Err(ConfigError::UnsupportedNominalTiming);
        }
        if !self.fd_timing.is_valid(&FD_TIMING_LIMITS) {
            return Err(ConfigError::UnsupportedFdTiming);
        }
        if self
            .retransmit_limit
            .is_some_and(|limit| limit > MAX_RETRANSMIT_LIMIT)
        {
            return Err(ConfigError::UnsupportedRetransmitLimit);
        }

        if let Some(offset) = self.secondary_sample_point_offset {
            // The hardware counts the offset in function clock periods.
            let cycles = u32::from(offset) * u32::from(self.fd_timing.baud_rate_prescaler);

            // The field is eight bits; the hardware would saturate silently.
            if cycles > u32::from(u8::MAX) {
                return Err(ConfigError::UnsupportedSecondarySamplePoint);
            }

            // TRM 38.3.7.3 limits the final position, offset plus measured
            // delay, to four data bit times. The core adds at least two cycles
            // of delay on its own, so an offset that only fits with zero delay
            // cannot fit in practice.
            let data_bit_cycles =
                u32::from(self.fd_timing.baud_rate_prescaler) * self.fd_timing.total_quanta();
            if cycles + MIN_TRANSMITTER_DELAY_CYCLES > data_bit_cycles * 4 {
                return Err(ConfigError::UnsupportedSecondarySamplePoint);
            }

            // Below three cycles the core flags bit errors against its own
            // output.
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
    id: Id,
    request: bool,
    fd: bool,
    bit_rate_switch: bool,
    error_state_indicator: bool,
    data: [u8; MAX_DATA_LEN],
    /// Bytes carried. Always zero for a request frame.
    len: usize,
    /// Bytes a request frame asks for. Zero for a data frame.
    requested_len: usize,
    timestamp: u64,
}

impl Frame {
    /// Creates a new classic CAN data frame.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `payload` is longer than 8 bytes.
    pub fn new(id: impl Into<Id>, payload: &[u8]) -> Result<Self, FrameError> {
        if payload.len() > CLASSIC_MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        Ok(Self::build(id.into(), false, false, payload))
    }

    /// Creates a new classic CAN request frame.
    ///
    /// A request frame carries no payload. `len` is the payload length that the
    /// frame asks for.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `len` is greater than 8.
    pub fn new_request(id: impl Into<Id>, len: usize) -> Result<Self, FrameError> {
        if len > CLASSIC_MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        let mut frame = Self::build(id.into(), true, false, &[]);
        frame.requested_len = len;
        Ok(frame)
    }

    /// Creates a new CAN FD data frame.
    ///
    /// The data phase uses the nominal bit rate unless
    /// [`Frame::with_bit_rate_switch`] is set.
    ///
    /// # Errors
    ///
    /// [`FrameError::PayloadTooLong`] when `payload` is longer than 64 bytes.
    pub fn new_fd(id: impl Into<Id>, payload: &[u8]) -> Result<Self, FrameError> {
        if payload.len() > MAX_DATA_LEN {
            return Err(FrameError::PayloadTooLong);
        }
        Ok(Self::build(id.into(), false, true, payload))
    }

    /// Sets whether the data phase is sent at the FD bit rate.
    ///
    /// Only a CAN FD frame can switch bit rate; the setting is ignored on a
    /// classic frame.
    pub fn with_bit_rate_switch(mut self, bit_rate_switch: bool) -> Self {
        self.bit_rate_switch = bit_rate_switch && self.fd;
        self
    }

    fn build(id: Id, request: bool, fd: bool, payload: &[u8]) -> Self {
        let mut data = [0u8; MAX_DATA_LEN];
        data[..payload.len()].copy_from_slice(payload);
        Self {
            id,
            request,
            fd,
            bit_rate_switch: false,
            error_state_indicator: false,
            data,
            len: payload.len(),
            requested_len: 0,
            timestamp: 0,
        }
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

    /// Returns the data length code the frame is sent with.
    ///
    /// A payload length that no code expresses is padded up to the next one,
    /// so a received copy of the frame can be longer than [`Frame::len`].
    pub fn dlc(&self) -> u8 {
        if self.request {
            len_to_dlc(self.requested_len as u8)
        } else {
            len_to_dlc(self.len as u8)
        }
    }

    /// Returns the arbitration identifier.
    pub fn id(&self) -> Id {
        self.id
    }

    /// Returns whether the identifier is a 29-bit extended one.
    pub fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
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
        FrameBuffer::build(
            FrameHeader {
                id: self.id,
                request: self.request,
                fd: self.fd,
                bit_rate_switch: self.bit_rate_switch,
                dlc: self.dlc(),
            },
            self.payload(),
        )
    }

    fn from_buffer(buffer: &FrameBuffer) -> Self {
        let payload = buffer.data();
        let mut data = [0u8; MAX_DATA_LEN];
        data[..payload.len()].copy_from_slice(payload);
        let request = buffer.is_request();
        Self {
            id: buffer.id(),
            request,
            fd: buffer.is_fd(),
            bit_rate_switch: buffer.is_bit_rate_switched(),
            error_state_indicator: buffer.error_state_indicator(),
            data,
            len: payload.len(),
            requested_len: if request { buffer.requested_len() } else { 0 },
            timestamp: buffer.timestamp(),
        }
    }

    fn raw_id(&self) -> u32 {
        match self.id {
            Id::Standard(id) => u32::from(id.as_raw()),
            Id::Extended(id) => id.as_raw(),
        }
    }
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Frame")
            .field("id", &format_args!("{:#x}", self.raw_id()))
            .field("extended", &self.is_extended())
            .field("request", &self.request)
            .field("fd", &self.fd)
            .field("bit_rate_switch", &self.bit_rate_switch)
            .field("error_state_indicator", &self.error_state_indicator)
            .field("dlc", &self.dlc())
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
            self.raw_id(),
            self.is_extended(),
            self.request,
            self.fd,
            self.bit_rate_switch,
            self.error_state_indicator,
            self.dlc(),
            self.len,
            self.timestamp,
            self.payload()
        )
    }
}

/// A frame that is known to be classic CAN, for code written against
/// `embedded-can`.
///
/// [`embedded_can::Frame`] promises at most 8 bytes of data, which a [`Frame`]
/// cannot keep: a 64-byte FD frame is a `Frame` too. The trait is therefore
/// implemented on this type, which the trait constructors build directly and
/// which `TryFrom<Frame>` refuses to make from an FD frame.
///
/// The type dereferences to the [`Frame`] it wraps and converts back into one
/// for [`CanFd::transmit`].
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
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        Frame::new(id, data).ok().map(Self)
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        Frame::new_request(id, dlc).ok().map(Self)
    }

    fn is_extended(&self) -> bool {
        self.0.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        self.0.request
    }

    fn id(&self) -> Id {
        self.0.id
    }

    fn dlc(&self) -> usize {
        // The trait defines this as the payload length of a data frame and the
        // requested length of a remote frame.
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

/// A CAN FD controller.
pub struct CanFd<'d, Dm: DriverMode = Blocking> {
    // Drop order matters: the controller leaves the bus before its clocks are
    // released, and the function clock goes before the register clock.
    bus: BusGuard,
    config: Config,
    _mode: PhantomData<Dm>,
    /// Accepted frame kinds for filters A, B, C and the range filter. They
    /// share one register.
    filter_kinds: [FrameKinds; 4],
    /// Whether a pin has been assigned, which commits `no_transceiver`.
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
/// Borrows the driver, which keeps the configuration and the teardown. See
/// [`CanFd::split`].
pub struct CanFdRx<'a, Dm: DriverMode> {
    driver: Driver,
    state: &'static asynch::State,
    _driver: PhantomData<&'a mut ()>,
    _mode: PhantomData<Dm>,
}

/// Transmitting half of a split controller.
///
/// See [`CanFd::split`].
pub struct CanFdTx<'a, Dm: DriverMode> {
    driver: Driver,
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
        self.driver.rx_frame_count()
    }

    /// Reads one frame from the RX buffer.
    ///
    /// # Errors
    ///
    /// [`Error::RxBufferEmpty`] when no complete frame is waiting.
    pub fn receive(&mut self) -> Result<Frame, Error> {
        if self.driver.rx_frame_count() == 0 {
            return Err(Error::RxBufferEmpty);
        }
        Ok(Frame::from_buffer(&self.driver.read_rx_frame()))
    }

    /// Discards everything in the RX buffer.
    pub fn flush_rx(&mut self) {
        self.driver.flush_rx();
    }

    /// Returns the size and the free space of the RX buffer, as `(size, free)`
    /// in 32-bit words.
    pub fn rx_buffer_words(&self) -> (u16, u16) {
        (self.driver.rx_buffer_size(), self.driver.rx_free_words())
    }

    /// Returns whether the RX buffer has overrun and dropped a frame
    /// (TRM 38.3.9.4).
    pub fn rx_overrun(&self) -> bool {
        self.driver.rx_overrun()
    }

    /// Clears the RX buffer overrun flag.
    pub fn clear_rx_overrun(&mut self) {
        self.driver.clear_overrun();
    }

    /// Returns the current fault confinement state; see [`CanFd::error_state`].
    pub fn error_state(&self) -> ErrorState {
        self.driver.error_state()
    }

    /// Returns the receive and transmit error counters, as `(rec, tec)`.
    pub fn error_counters(&self) -> (u16, u16) {
        (self.driver.rec(), self.driver.tec())
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
            // Armed before the check, so a frame arriving in between still
            // raises the interrupt.
            self.driver
                .enable_interrupts(CanFdInterrupt::RxNotEmpty.bit());

            if self.driver.rx_frame_count() > 0 {
                self.driver
                    .disable_interrupts(CanFdInterrupt::RxNotEmpty.bit());
                return core::task::Poll::Ready(());
            }

            core::task::Poll::Pending
        })
        .await;

        Frame::from_buffer(&self.driver.read_rx_frame())
    }
}

impl<Dm: DriverMode> CanFdTx<'_, Dm> {
    /// Returns the number of TX buffers the hardware provides.
    pub fn tx_buffer_count(&self) -> u8 {
        self.tx_buffers
    }

    /// Queues a frame in the first free TX buffer and arms it.
    ///
    /// Returns the index of the buffer used. Up to [`CanFdTx::tx_buffer_count`]
    /// frames can be queued at once.
    ///
    /// The hardware picks the next frame to send among the armed buffers by
    /// priority, and among equal priorities by the lower buffer index; see
    /// [`CanFdTx::set_tx_priority`]. The order in which frames were queued
    /// plays no part in that choice. With equal priorities a frame queued into
    /// a buffer that a finished frame freed up is sent before frames still
    /// waiting in higher-numbered buffers. To keep frames in the order they
    /// were queued, wait for each buffer to leave [`TxBufferState::Ready`]
    /// before queueing the next frame, or give later frames lower priorities.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerStopped`] when the controller is not on the bus. A
    /// stopped controller ignores the command that arms a buffer, so the frame
    /// would be lost.
    ///
    /// [`Error::NoFreeTxBuffer`] when every TX buffer is occupied.
    pub fn transmit(&mut self, frame: &Frame) -> Result<u8, Error> {
        if !self.driver.is_enabled() {
            return Err(Error::ControllerStopped);
        }

        let index = (0..self.tx_buffers)
            .find(|&i| self.driver.tx_buffer_state(i).is_writable())
            .ok_or(Error::NoFreeTxBuffer)?;

        self.driver.write_tx_buffer(index, &frame.to_buffer());
        self.driver.set_tx_ready(index);
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
        self.driver.tx_buffer_state(index)
    }

    /// Requests that a queued transmission be aborted.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn abort_transmit(&mut self, index: u8) {
        if index < self.tx_buffers {
            self.driver.set_tx_abort(index);
        }
    }

    /// Returns a TX buffer to the empty state, releasing it for reuse.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn release_tx_buffer(&mut self, index: u8) {
        if index < self.tx_buffers {
            self.driver.set_tx_empty(index);
        }
    }

    /// Sets a TX buffer's arbitration priority.
    ///
    /// Higher values win. Equal priorities are resolved in favor of the lower
    /// buffer index (TRM 38.3.8.1). A value above [`MAX_TX_PRIORITY`] counts as
    /// that maximum. Does nothing for an index the hardware does not have.
    ///
    /// The priority belongs to the buffer, not to the frame in it, and all
    /// buffers start out equal. Set it before arming the buffer with
    /// [`CanFdTx::transmit`], or right after, while the frame is still
    /// [`TxBufferState::Ready`].
    pub fn set_tx_priority(&mut self, index: u8, priority: u8) {
        if index < self.tx_buffers {
            self.driver
                .set_tx_priority(index, priority.min(MAX_TX_PRIORITY));
        }
    }

    /// Returns the current fault confinement state; see [`CanFd::error_state`].
    ///
    /// On bus-off every armed TX buffer fails, so a transmission that ends in
    /// [`Error::TransmitFailed`] can be told apart from an exhausted
    /// retransmission limit here.
    pub fn error_state(&self) -> ErrorState {
        self.driver.error_state()
    }

    /// Returns the receive and transmit error counters, as `(rec, tec)`.
    pub fn error_counters(&self) -> (u16, u16) {
        (self.driver.rec(), self.driver.tec())
    }

    /// Requests that a bus-off controller rejoin the bus.
    ///
    /// Does nothing unless the controller is bus-off. Rejoining takes 128
    /// occurrences of 11 recessive bits, as ISO 11898-1 requires. Leaving the
    /// bus with [`CanFd::stop`] and joining it again with [`CanFd::start`] also
    /// clears the bus-off state, after ordinary integration.
    ///
    /// The state is checked here because the hardware remembers a request made
    /// while error-active and then rejoins on its own the next time it goes
    /// bus-off (measured on an ESP32-C5; TRM register 38.3 says otherwise).
    pub fn request_bus_off_recovery(&mut self) {
        if self.driver.error_state() == ErrorState::BusOff {
            self.driver.request_bus_off_recovery();
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
            // Armed before the check, so a transmission finishing in between
            // still raises the interrupt.
            self.driver.enable_interrupts(CanFdInterrupt::TxDone.bit());

            let state = self.driver.tx_buffer_state(index);
            if !matches!(
                state,
                TxBufferState::Ready | TxBufferState::InProgress | TxBufferState::AbortInProgress
            ) {
                self.driver.disable_interrupts(CanFdInterrupt::TxDone.bit());
            }

            match state {
                TxBufferState::Ready
                | TxBufferState::InProgress
                | TxBufferState::AbortInProgress => core::task::Poll::Pending,
                TxBufferState::Ok => core::task::Poll::Ready(Ok(())),
                TxBufferState::Failed => core::task::Poll::Ready(Err(Error::TransmitFailed)),
                TxBufferState::Aborted => core::task::Poll::Ready(Err(Error::TransmitAborted)),
                _ => core::task::Poll::Ready(Err(Error::NoFreeTxBuffer)),
            }
        })
        .await
    }
}

/// Owns the controller's participation in the bus, so that a dropped driver
/// leaves the bus cleanly instead of cutting a frame short.
struct BusGuard {
    driver: Driver,
    tx_buffers: u8,
    interrupt: Interrupt,
    /// How long a frame in flight is given to finish at the configured bit
    /// rate.
    abort_timeout: Duration,
    /// Held while the controller is on the bus: light sleep gates the function
    /// clock, and a frame keeps going after the future that queued it is
    /// dropped.
    wake_lock: Option<WakeLock>,
}

impl BusGuard {
    fn enable(&mut self) {
        self.driver.enable(true);
        if self.wake_lock.is_none() {
            self.wake_lock = Some(WakeLock::new());
        }
    }

    fn disable(&mut self) {
        self.driver.enable(false);
        self.wake_lock = None;
    }

    /// Aborts anything queued and waits for the wire to be free (TRM 38.3.6
    /// step 1).
    fn quiesce(&self) -> Result<(), Error> {
        for index in 0..self.tx_buffers {
            self.driver.set_tx_abort(index);
        }

        let deadline = Instant::now() + self.abort_timeout;
        for index in 0..self.tx_buffers {
            while matches!(
                self.driver.tx_buffer_state(index),
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
        // An interrupt source left armed keeps the request line asserted after
        // the clocks are gone, and the handler could no longer retire it.
        self.driver.disable_interrupts(u32::MAX);
        self.driver.clear_interrupts(u32::MAX);
        crate::interrupt::disable(Cpu::current(), self.interrupt);

        // Best effort: a wedged transmission must not hang teardown.
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
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .driver
            .enable_interrupts(CanFdInterrupt::mask(interrupts.into()));
    }

    /// Disables the given interrupt sources.
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .driver
            .disable_interrupts(CanFdInterrupt::mask(interrupts.into()));
    }

    /// Returns the interrupt sources that are currently asserted.
    pub fn interrupts(&self) -> EnumSet<CanFdInterrupt> {
        CanFdInterrupt::from_mask(self.bus.driver.interrupt_status())
    }

    /// Clears the given interrupt sources.
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<CanFdInterrupt>>) {
        self.bus
            .driver
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

        let driver = Driver::new(info.register_block);
        driver.reset();
        let tx_buffers = driver.tx_buffer_count();

        let abort_timeout = abort_timeout(
            &config.nominal_timing,
            &config.fd_timing,
            info.clock_instance.function_clock_frequency(),
        );

        let mut this = Self {
            bus: BusGuard {
                driver,
                tx_buffers,
                abort_timeout,
                interrupt: info.interrupt,
                wake_lock: None,
            },
            config,
            // The reset value: filter A accepts everything, the rest are off.
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
        self.bus.driver.disable_interrupts(u32::MAX);
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
        self.bus.driver.disable_interrupts(u32::MAX);
        self.bus.driver.clear_interrupts(u32::MAX);
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
        config.validate()?;

        if self.pins_bound && config.no_transceiver != self.config.no_transceiver {
            return Err(ConfigError::TransceiverModeLocked);
        }

        // The prescaler divides the function clock, so a new source would
        // silently rescale a running counter.
        if config.clock_source != self.config.clock_source {
            self.stop_timestamp_timer();
        }

        // `configure` refuses before writing anything, so `config()` keeps
        // describing the hardware.
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

        if self.bus.driver.is_enabled() {
            self.bus.quiesce().map_err(|_| ConfigError::BusBusy)?;
        }

        // Mode bits can only be changed while the controller is disabled.
        self.bus.disable();

        // The timing below is expressed in periods of the function clock
        // (TRM 38.3.1), so the mux is switched first.
        self._function_clock.set_source(self.config.clock_source);

        let config = self.config;
        let (listen_only, self_test, loopback) = match config.mode {
            Mode::Normal => (false, false, false),
            Mode::ListenOnly => (true, false, false),
            Mode::SelfTest => (false, true, false),
            Mode::LoopbackSelfTest => (false, true, true),
        };

        self.bus.driver.apply_mode_settings(&ll::ModeSettings {
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

        self.bus
            .driver
            .set_timestamp_point(TimestampPoint::EndOfFrame);

        self.bus.driver.set_nominal_timing(&config.nominal_timing);
        self.bus.driver.set_fd_timing(&config.fd_timing);

        self.bus.abort_timeout = abort_timeout(
            &config.nominal_timing,
            &config.fd_timing,
            self.function_clock_frequency(),
        );

        match config.secondary_sample_point_offset {
            // The hardware counts the offset in function clock cycles.
            Some(offset) => self.bus.driver.set_secondary_sample_point(
                SspSource::MeasuredPlusOffset,
                offset.saturating_mul(config.fd_timing.baud_rate_prescaler),
            ),
            None => self
                .bus
                .driver
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

        let was_enabled = self.bus.driver.is_enabled();
        self.bus.enable();

        // Integration finishes when the controller becomes error-active, after
        // 11 consecutive recessive bits.
        let deadline = Instant::now() + timeout;
        loop {
            if self.bus.driver.error_state() == ErrorState::Active {
                return Ok(());
            }
            if Instant::now() > deadline {
                if !was_enabled {
                    self.bus.disable();
                }
                return Err(Error::BusIntegrationTimeout);
            }
        }
    }

    /// Returns the identity of the core.
    pub fn identity(&self) -> Identity {
        let (major, minor) = self.bus.driver.version();
        Identity {
            device_id: self.bus.driver.device_id(),
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
    /// frame while a transmission is in flight. They arm different interrupt
    /// sources, so neither disturbs the other.
    ///
    /// The halves borrow the driver, which keeps the configuration and the
    /// teardown: everything that reconfigures the controller or takes it off
    /// the bus stays on [`CanFd`] and is unavailable while a half is alive. For
    /// two independent tasks, split a driver that lives in a `static`.
    pub fn split(&mut self) -> (CanFdRx<'_, Dm>, CanFdTx<'_, Dm>) {
        let (info, state) = self.twai.parts();
        (
            CanFdRx {
                driver: Driver::new(info.register_block),
                state,
                _driver: PhantomData,
                _mode: PhantomData,
            },
            CanFdTx {
                driver: Driver::new(info.register_block),
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
    /// releases the lock. A running timestamp counter does not advance during
    /// sleep.
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
    /// [`CanFd::stop`] or [`CanFd::apply_config`].
    pub fn is_started(&self) -> bool {
        self.bus.driver.is_enabled()
    }

    /// Queues a frame in the first free TX buffer and arms it.
    ///
    /// Returns the index of the buffer used. The hardware chooses the order in
    /// which armed buffers are sent, not the order in which they were queued;
    /// see [`CanFdTx::transmit`].
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
        if index >= self.bus.tx_buffers {
            return TxBufferState::NotExist;
        }
        self.bus.driver.tx_buffer_state(index)
    }

    /// Requests that a queued transmission be aborted.
    ///
    /// The buffer moves to "aborted", or to "abort in progress" if the frame is
    /// already on the wire. Does nothing for an index the hardware does not
    /// have.
    pub fn abort_transmit(&mut self, index: u8) {
        if index < self.bus.tx_buffers {
            self.bus.driver.set_tx_abort(index);
        }
    }

    /// Returns a TX buffer to the empty state, releasing it for reuse.
    ///
    /// Does nothing for an index the hardware does not have.
    pub fn release_tx_buffer(&mut self, index: u8) {
        if index < self.bus.tx_buffers {
            self.bus.driver.set_tx_empty(index);
        }
    }

    /// Sets a TX buffer's arbitration priority.
    ///
    /// Higher values win, and a value above [`MAX_TX_PRIORITY`] counts as that
    /// maximum; see [`CanFdTx::set_tx_priority`].
    pub fn set_tx_priority(&mut self, index: u8, priority: u8) {
        self.tx_half().set_tx_priority(index, priority);
    }

    /// Returns the size and the free space of the RX buffer, as `(size, free)`
    /// in 32-bit words.
    pub fn rx_buffer_words(&self) -> (u16, u16) {
        (
            self.bus.driver.rx_buffer_size(),
            self.bus.driver.rx_free_words(),
        )
    }

    /// Returns whether the RX buffer has overrun and dropped a frame
    /// (TRM 38.3.9.4).
    pub fn rx_overrun(&self) -> bool {
        self.bus.driver.rx_overrun()
    }

    /// Clears the RX buffer overrun flag.
    pub fn clear_rx_overrun(&mut self) {
        self.bus.driver.clear_overrun();
    }

    /// Returns the number of complete frames waiting in the RX buffer.
    pub fn rx_frame_count(&self) -> u16 {
        self.bus.driver.rx_frame_count()
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
    /// before the first [`CanFd::start`] and after [`CanFd::stop`]; that is how
    /// the hardware reports a disabled controller. [`CanFd::is_started`] tells
    /// the two apart.
    pub fn error_state(&self) -> ErrorState {
        self.bus.driver.error_state()
    }

    /// Returns whether an error counter has reached the error warning limit.
    ///
    /// A node at the limit is still [`ErrorState::Active`] (TRM 38.3.10).
    pub fn error_warning(&self) -> bool {
        self.bus.driver.error_warning()
    }

    /// Returns the receive and transmit error counters, as `(rec, tec)`.
    pub fn error_counters(&self) -> (u16, u16) {
        (self.bus.driver.rec(), self.bus.driver.tec())
    }

    /// Requests that a bus-off controller rejoin the bus; see
    /// [`CanFdTx::request_bus_off_recovery`].
    pub fn request_bus_off_recovery(&mut self) {
        self.tx_half().request_bus_off_recovery();
    }

    /// Sets the error warning limit, which defaults to 96.
    ///
    /// The hardware only accepts the write in test mode (TRM 38.3.10), which
    /// is entered for the write and left again.
    ///
    /// # Errors
    ///
    /// [`Error::ControllerRunning`] when the controller is still on the bus.
    pub fn set_error_warning_limit(&mut self, limit: u8) -> Result<(), Error> {
        if self.bus.driver.is_enabled() {
            return Err(Error::ControllerRunning);
        }
        self.bus.driver.enable_test_mode(true);
        self.bus.driver.set_error_warning_limit(limit);
        self.bus.driver.enable_test_mode(false);
        Ok(())
    }

    /// Resets the received and transmitted frame counters.
    pub fn reset_traffic_counters(&mut self) {
        self.bus.driver.reset_traffic_counters();
    }

    /// Returns the numbers of frames received and transmitted since the
    /// counters were last reset, as `(rx, tx)`.
    pub fn traffic_counters(&self) -> (u32, u32) {
        (
            self.bus.driver.rx_traffic_counter(),
            self.bus.driver.tx_traffic_counter(),
        )
    }

    /// Discards everything in the RX buffer.
    pub fn flush_rx(&mut self) {
        self.bus.driver.flush_rx();
    }

    // ----------------------------------------------------------------- filters

    /// Configures one mask filter.
    ///
    /// A frame reaches the RX buffer if it passes at least one enabled filter
    /// (TRM 38.3.9.8). Out of reset, filter A accepts every frame.
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
        check_filter_id(config.id, config.extended)?;
        check_filter_id(config.mask, config.extended)?;

        self.write_mask_filter(filter, config);
        Ok(())
    }

    fn write_mask_filter(&mut self, filter: MaskFilter, config: &MaskFilterConfig) {
        self.bus
            .driver
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
            .driver
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
        self.bus.driver.filters_supported()
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
        self.bus.driver.set_filter_kinds(a, b, c, range);
    }

    // ------------------------------------------------------------ error detail

    /// Returns details of the last bus error the core captured.
    ///
    /// Only meaningful after the core has reported a bus error; see
    /// [`ErrorCapture`].
    pub fn error_capture(&self) -> ErrorCapture {
        self.bus.driver.error_capture()
    }

    /// Returns the number of retransmission attempts made for the frame
    /// currently being sent.
    pub fn retransmit_count(&self) -> u8 {
        self.bus.driver.retransmit_count()
    }

    /// Returns the error counters of the nominal and data phases, as
    /// `(nominal, fd)`.
    pub fn phase_error_counters(&self) -> (u16, u16) {
        self.bus.driver.special_error_counters()
    }

    /// Returns the transmitter delay the core measured, in function clock
    /// periods.
    ///
    /// The core measures the delay during every FD frame it sends
    /// (TRM 38.3.7.2), so the value reads zero until the first one. The
    /// secondary sample point sits at this delay plus the configured offset,
    /// and the four-bit-time limit of TRM 38.3.7.3 applies to that sum.
    pub fn transmitter_delay(&self) -> u8 {
        self.bus.driver.transmitter_delay()
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

        self.bus.driver.timer_enable(false);
        self.bus.driver.timer_enable_config_clock(true);

        self.bus.driver.timer_set_divider(divider);
        self.bus.driver.timer_count_up(true);
        self.bus.driver.timer_set_free_running();
        self.bus.driver.timer_enable(true);
        self.bus.driver.timer_clear();

        // The prescaler keeps its phase across a reprogram and compares
        // against the divider instead of reloading, so a divider below the
        // phase it holds is missed until the 16-bit prescaler wraps. Measured
        // on an ESP32-C5; nothing resets that phase. Wait it out rather than
        // hand back a counter that stands still for up to 819 us.
        let cycles = u64::from(PRESCALER_PERIOD) + 2 * u64::from(divider);
        let deadline = Instant::now()
            + Duration::from_micros(cycles * 1_000_000 / u64::from(clock))
            + Duration::from_millis(1);
        while self.bus.driver.timer_count() == 0 {
            if Instant::now() > deadline {
                return Err(ConfigError::TimestampTimerStalled);
            }
        }

        Ok(clock / u32::from(divider))
    }

    /// Stops the timestamp counter.
    pub fn stop_timestamp_timer(&mut self) {
        self.bus.driver.timer_enable(false);
        self.bus.driver.timer_enable_config_clock(false);
    }

    /// Returns the current value of the timestamp counter.
    pub fn timestamp(&self) -> u64 {
        self.bus.driver.timer_count()
    }

    /// Returns the width of the timestamp counter in bits.
    pub fn timestamp_bit_width(&self) -> u8 {
        self.bus.driver.timer_bit_width()
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
    pub register_block: *const crate::pac::twai0::RegisterBlock,
    pub peripheral: Peripheral,
    pub clock_instance: TwaiInstance,
    pub interrupt: Interrupt,
    pub async_handler: InterruptHandler,
    pub rx_signal: InputSignal,
    pub tx_signal: OutputSignal,
}

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

/// Function clock periods the timestamp prescaler takes to wrap, measured on
/// an ESP32-C5.
const PRESCALER_PERIOD: u32 = 1 << 16;

/// Transmitter delay the core adds on its own, in function clock periods
/// (TRM 38.3.7.3).
const MIN_TRANSMITTER_DELAY_CYCLES: u32 = 2;

/// Bit times allowed for joining the bus: a frame already in flight (under 800
/// bit times at most), then the 11 recessive bits of integration.
const INTEGRATION_TIMEOUT_BITS: u64 = 2048;

/// Shortest wait for bus integration, so the bound at megabit rates is not
/// about the speed of the polling loop.
const INTEGRATION_TIMEOUT_FLOOR: Duration = Duration::from_millis(50);

/// Bit times allowed for a transmission to settle: an abort only takes effect
/// once the frame in flight has finished.
const ABORT_TIMEOUT_BITS: u64 = 1024;

/// How long a transmission may take to finish or abort while leaving the bus.
///
/// Derived from the slower of the two phases: a bit rate switched frame
/// spends its data field at the FD timing, and nothing requires that to be the
/// faster one.
fn abort_timeout(nominal: &Timing, fd: &Timing, function_clock_hz: u32) -> Duration {
    // The added millisecond covers the polling loop at fast bit rates.
    bit_times(
        slowest_cycles_per_bit(nominal, fd),
        ABORT_TIMEOUT_BITS,
        function_clock_hz,
    ) + Duration::from_millis(1)
}

/// How long the controller may take to join the bus.
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

fn check_filter_id(id: u32, extended: bool) -> Result<(), ConfigError> {
    let limit = if extended { EXT_ID_MASK } else { STD_ID_MASK };
    if id > limit {
        return Err(ConfigError::FilterIdTooLarge);
    }
    Ok(())
}

/// Function clock periods one bit takes in the slower of the two phases.
fn slowest_cycles_per_bit(nominal: &Timing, fd: &Timing) -> u64 {
    let cycles_per_bit =
        |timing: &Timing| u64::from(timing.baud_rate_prescaler) * u64::from(timing.total_quanta());
    cycles_per_bit(nominal).max(cycles_per_bit(fd))
}

/// How long `bits` bit times last, given the clock periods one bit takes.
fn bit_times(cycles_per_bit: u64, bits: u64, function_clock_hz: u32) -> Duration {
    // Scaled before dividing: a bit at 2 Mbit/s is half a microsecond.
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
            // The mux must be configured before the clock is requested.
            instance.configure_function_clock(clocks, clock_source);
            instance.request_function_clock(clocks);
        });
        Self { instance }
    }

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
