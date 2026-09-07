//! Low-level register access for the CTU CAN FD core.
//!
//! Ported from ESP-IDF's [`twaifd_ll.h`](https://github.com/espressif/esp-idf/blob/96f54947e08c196cf71c0588243dfc8e33807acc/components/esp_hal_twai/esp32c5/include/hal/twaifd_ll.h).
//!
//! Register semantics follow the **ESP32-C5 TRM v1.1, chapter 38**, which every
//! bare "TRM" reference below means. Chapter and section numbers are specific to
//! that manual: the ESP32-H4 and ESP32-S31 carry the same core but number their
//! chapters differently. Where the TRM and the ESP-IDF headers disagree, the TRM
//! wins and the difference is noted at the site.

use crate::pac::twai0::RegisterBlock;

/// Offset of the first TX buffer RAM cell from the peripheral base (TRM 38.3.8).
const TXT_BUFFER_BASE: usize = 0x100;
/// Stride between TX buffer RAM cells.
const TXT_BUFFER_STRIDE: usize = 0x100;
/// Words in a frame buffer: 1 format + 1 identifier + 2 timestamp + 16 data.
pub(super) const FRAME_WORDS: usize = 20;

/// Largest DLC code a CAN FD frame can carry.
pub(super) const MAX_DLC: u8 = 15;
/// Largest payload a CAN FD frame can carry, in bytes.
pub(super) const MAX_DATA_LEN: usize = 64;
/// Largest payload a classic CAN frame can carry, in bytes.
pub(super) const CLASSIC_MAX_DATA_LEN: usize = 8;

/// Mask of the 11-bit base identifier.
pub(super) const STD_ID_MASK: u32 = 0x0000_07FF;
/// Mask of the 29-bit extended identifier.
pub(super) const EXT_ID_MASK: u32 = 0x1FFF_FFFF;
/// Bit position of the base identifier inside the frame buffer's identifier word.
const IDENTIFIER_BASE_SHIFT: u32 = 18;

/// Hardware limits for one set of bit timing parameters.
///
/// The maxima are the register field widths from TRM 38.7; the minima come from
/// ESP-IDF's `twaifd_ll.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimingLimits {
    /// Inclusive bounds on the prescaler.
    pub baud_rate_prescaler: (u8, u8),
    /// Inclusive bounds on the propagation segment.
    pub propagation_segment: (u8, u8),
    /// Inclusive bounds on phase segment 1.
    pub phase_segment_1: (u8, u8),
    /// Inclusive bounds on phase segment 2.
    pub phase_segment_2: (u8, u8),
    /// Inclusive bounds on the synchronization jump width.
    pub sync_jump_width: (u8, u8),
}

/// Limits for the nominal (arbitration phase) bit timing.
pub const NOMINAL_TIMING_LIMITS: TimingLimits = TimingLimits {
    baud_rate_prescaler: (1, 255),
    propagation_segment: (1, 127),
    phase_segment_1: (0, 63),
    phase_segment_2: (1, 63),
    sync_jump_width: (1, 31),
};

/// Limits for the data phase bit timing.
///
/// The FD phase segments are narrower fields than the nominal ones.
pub const FD_TIMING_LIMITS: TimingLimits = TimingLimits {
    baud_rate_prescaler: (1, 255),
    propagation_segment: (1, 63),
    phase_segment_1: (0, 31),
    phase_segment_2: (1, 31),
    sync_jump_width: (1, 31),
};

/// Bit timing parameters for one phase (nominal or data).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    /// Bit rate prescaler. Divides the function clock to produce the time quantum.
    pub baud_rate_prescaler: u8,
    /// Propagation segment, in time quanta.
    pub propagation_segment: u8,
    /// Phase segment 1, in time quanta.
    pub phase_segment_1: u8,
    /// Phase segment 2, in time quanta.
    pub phase_segment_2: u8,
    /// Synchronization jump width, in time quanta.
    pub sync_jump_width: u8,
}

/// Largest retransmission limit the four-bit `RTRTH` field can hold
/// (TRM 38.3.8.4).
pub const MAX_RETRANSMIT_LIMIT: u8 = 15;

impl Timing {
    /// Returns whether every parameter fits the given limits.
    ///
    /// Covers the per-field ranges and the two combined constraints of TRM
    /// 38.3.7.7. Those constraints are expressed in minimal time quanta, that
    /// is, in system clock periods, so the prescaler is part of them. A bit
    /// timing that satisfies every individual range can still violate them, and
    /// the hardware would then sample wrongly.
    pub const fn is_valid(&self, limits: &TimingLimits) -> bool {
        // Phase_Seg2 >= 2 minimal time quanta.
        if (self.baud_rate_prescaler as u32) * (self.phase_segment_2 as u32) < 2 {
            return false;
        }
        // Sync_Seg + Prop_Seg + Phase_Seg1 > 2 minimal time quanta.
        if (self.baud_rate_prescaler as u32)
            * (1 + self.propagation_segment as u32 + self.phase_segment_1 as u32)
            <= 2
        {
            return false;
        }

        self.baud_rate_prescaler >= limits.baud_rate_prescaler.0
            && self.baud_rate_prescaler <= limits.baud_rate_prescaler.1
            && self.propagation_segment >= limits.propagation_segment.0
            && self.propagation_segment <= limits.propagation_segment.1
            && self.phase_segment_1 >= limits.phase_segment_1.0
            && self.phase_segment_1 <= limits.phase_segment_1.1
            && self.phase_segment_2 >= limits.phase_segment_2.0
            && self.phase_segment_2 <= limits.phase_segment_2.1
            && self.sync_jump_width >= limits.sync_jump_width.0
            && self.sync_jump_width <= limits.sync_jump_width.1
    }

    /// Returns the total bit time in time quanta, including the sync segment.
    pub const fn total_quanta(&self) -> u32 {
        1 + self.propagation_segment as u32
            + self.phase_segment_1 as u32
            + self.phase_segment_2 as u32
    }

    /// Returns the bit rate in bits per second for a given function clock.
    pub const fn bitrate(&self, clock_hz: u32) -> u32 {
        clock_hz / (self.baud_rate_prescaler as u32 * self.total_quanta())
    }

    /// Returns the sample point in per mille of the bit time.
    pub const fn sample_point_permille(&self) -> u32 {
        (1 + self.propagation_segment as u32 + self.phase_segment_1 as u32) * 1000
            / self.total_quanta()
    }
}

/// Source used to place the secondary sample point (TRM 38.3.7.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SspSource {
    /// Measured transmitter delay plus the configured offset.
    MeasuredPlusOffset = 0,
    /// Secondary sampling is disabled.
    Disabled           = 1,
    /// The configured offset alone.
    ///
    /// Not used by the driver: without the measured delay, the offset would
    /// have to include the transceiver loop delay, which the hardware can
    /// measure on its own. Kept so the enum covers the whole field.
    #[allow(dead_code)]
    OffsetOnly         = 2,
}

/// State of a single TX buffer (TRM figure 38.3-8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxBufferState {
    /// The buffer is not implemented by this core.
    NotExist,
    /// Armed for transmission.
    Ready,
    /// Currently being transmitted.
    InProgress,
    /// Abort requested while a transmission was in progress.
    AbortInProgress,
    /// Last transmission succeeded.
    Ok,
    /// Last transmission failed.
    Failed,
    /// Last transmission was aborted.
    Aborted,
    /// Ready to be filled by software.
    Empty,
    /// Hardware reported a state this driver does not model.
    Unknown(u8),
}

impl TxBufferState {
    fn from_bits(bits: u8) -> Self {
        // TRM 38.22, TWAIFD_TX_STATUS_REG. Note that "empty" is 0x8 and 0x0
        // means the buffer is not implemented; ESP-IDF's header only names
        // 0x4/0x6/0x7, so these codes come from the TRM.
        match bits {
            0x0 => Self::NotExist,
            0x1 => Self::Ready,
            0x2 => Self::InProgress,
            0x3 => Self::AbortInProgress,
            0x4 => Self::Ok,
            0x6 => Self::Failed,
            0x7 => Self::Aborted,
            0x8 => Self::Empty,
            other => Self::Unknown(other),
        }
    }

    /// Returns whether software can write to a buffer in this state (TRM 38.3.8).
    pub fn is_writable(self) -> bool {
        matches!(self, Self::Empty | Self::Ok | Self::Failed | Self::Aborted)
    }
}

/// Fault confinement state (TRM 38.3.10).
///
/// These are the three states the hardware reports, matching ISO 11898-1. The
/// error warning limit is deliberately not among them: reaching it does not
/// change the confinement state, it only raises a flag, which
/// [`super::CanFd::error_warning`] reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorState {
    /// Takes part in the bus normally and signals errors actively.
    Active,
    /// Still communicates, but signals errors passively.
    Passive,
    /// Off the bus after TEC passed 255.
    BusOff,
}

/// Point in the frame at which a received frame is timestamped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TimestampPoint {
    /// Sixth bit of the end-of-frame field, when the frame becomes valid.
    EndOfFrame   = 0,
    /// Start-of-frame bit.
    StartOfFrame = 1,
}

/// Every bit of `MODE_SETTINGS` the driver configures, applied in one write.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub(super) struct ModeSettings {
    /// Receives only, never driving a dominant bit.
    pub listen_only: bool,
    /// Treats a frame as transmitted even without an ACK.
    pub self_test: bool,
    /// Routes transmitted frames back into the RX buffer.
    pub loopback: bool,
    /// Accepts CAN FD frames.
    pub fd_enabled: bool,
    /// Handles protocol exceptions.
    pub protocol_exception: bool,
    /// Advances the RX read pointer on each read of `RX_DATA`.
    pub rx_auto_increment: bool,
    /// Applies the acceptance filters.
    pub filters_enabled: bool,
    /// Moves every TX buffer to "TX failed" on bus-off.
    pub bus_off_tx_fail: bool,
    /// Drops received request frames.
    pub drop_request_frames: bool,
    /// Transmits only when the time base reaches a frame's trigger time.
    pub time_triggered_tx: bool,
    /// Retransmission attempts, or `None` to retry forever.
    pub retransmit_limit: Option<u8>,
}

/// One of the three mask filter instances.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MaskFilter {
    /// Filter A.
    A,
    /// Filter B.
    B,
    /// Filter C.
    C,
}

/// Which kinds of frame a filter accepts.
///
/// A filter with none of these set is disabled (TRM 38.3.9.8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameKinds {
    /// Classic CAN frames with an 11-bit identifier.
    pub classic_standard: bool,
    /// Classic CAN frames with a 29-bit identifier.
    pub classic_extended: bool,
    /// CAN FD frames with an 11-bit identifier.
    pub fd_standard: bool,
    /// CAN FD frames with a 29-bit identifier.
    pub fd_extended: bool,
}

impl FrameKinds {
    /// Accepts every kind of frame.
    pub const ALL: Self = Self {
        classic_standard: true,
        classic_extended: true,
        fd_standard: true,
        fd_extended: true,
    };

    /// Accepts nothing, which disables the filter.
    pub const NONE: Self = Self {
        classic_standard: false,
        classic_extended: false,
        fd_standard: false,
        fd_extended: false,
    };

    fn bits(self) -> u32 {
        (self.classic_standard as u32)
            | ((self.classic_extended as u32) << 1)
            | ((self.fd_standard as u32) << 2)
            | ((self.fd_extended as u32) << 3)
    }
}

/// Reason the core last flagged a bus error (TRM 38.3.13.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusErrorKind {
    /// The transmitted bit did not match the bit sampled back.
    Bit,
    /// The frame's CRC did not match.
    Crc,
    /// A fixed-form field held an illegal value.
    Form,
    /// No node acknowledged the frame.
    Ack,
    /// A stuffing rule was violated.
    Stuff,
    /// Hardware reported a code this driver does not model.
    Unknown(u8),
}

impl BusErrorKind {
    fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Bit,
            1 => Self::Crc,
            2 => Self::Form,
            3 => Self::Ack,
            4 => Self::Stuff,
            other => Self::Unknown(other),
        }
    }
}

/// Where in a frame the core last flagged an error (TRM 38.3.13.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorPosition {
    /// Start of frame.
    StartOfFrame,
    /// Arbitration field.
    Arbitration,
    /// Control field.
    Control,
    /// Data field.
    Data,
    /// CRC field.
    Crc,
    /// CRC delimiter, ACK field or ACK delimiter.
    Ack,
    /// End of frame field.
    EndOfFrame,
    /// During an error frame.
    ErrorFrame,
    /// During an overload frame.
    OverloadFrame,
    /// Somewhere else. Also what the register reads out of reset.
    Other,
    /// Hardware reported a code this driver does not model.
    Unknown(u8),
}

impl ErrorPosition {
    fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::StartOfFrame,
            1 => Self::Arbitration,
            2 => Self::Control,
            3 => Self::Data,
            4 => Self::Crc,
            5 => Self::Ack,
            6 => Self::EndOfFrame,
            7 => Self::ErrorFrame,
            8 => Self::OverloadFrame,
            31 => Self::Other,
            other => Self::Unknown(other),
        }
    }
}

/// Details of the last bus error the core captured.
///
/// The registers behind this are not cleared, so the value is only meaningful
/// after the core has actually reported a bus error. Out of reset it reads as
/// [`BusErrorKind::Bit`] at [`ErrorPosition::Other`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ErrorCapture {
    /// What went wrong.
    pub kind: BusErrorKind,
    /// Where in the frame it happened.
    pub position: ErrorPosition,
}

/// A frame in the layout the hardware TX and RX buffers use.
///
/// Word 0 is the format word, word 1 the identifier, words 2 and 3 the
/// timestamp, and words 4..20 the payload.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct FrameBuffer {
    pub(super) words: [u32; FRAME_WORDS],
}

impl core::fmt::Debug for FrameBuffer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let used = 4 + self.data_len().div_ceil(4);
        f.debug_struct("FrameBuffer")
            .field("id", &format_args!("{:#x}", self.id()))
            .field("extended", &self.is_extended())
            .field("request", &self.is_request())
            .field("fd", &self.is_fd())
            .field("bit_rate_switch", &self.is_bit_rate_switched())
            .field("dlc", &self.dlc())
            .field("len", &self.data_len())
            .field("timestamp", &self.timestamp())
            .field("words", &&self.words[..used.min(FRAME_WORDS)])
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FrameBuffer {
    fn format(&self, f: defmt::Formatter<'_>) {
        let used = (4 + self.data_len().div_ceil(4)).min(FRAME_WORDS);
        defmt::write!(
            f,
            "FrameBuffer {{ id: {=u32:#x}, extended: {}, request: {}, fd: {}, brs: {}, dlc: {=u8}, len: {=usize}, timestamp: {=u64}, words: {=[?]} }}",
            self.id(),
            self.is_extended(),
            self.is_request(),
            self.is_fd(),
            self.is_bit_rate_switched(),
            self.dlc(),
            self.data_len(),
            self.timestamp(),
            &self.words[..used]
        )
    }
}

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameBuffer {
    const DATA_WORD_OFFSET: usize = 4;

    /// Creates an all-zero frame buffer.
    pub const fn new() -> Self {
        Self {
            words: [0; FRAME_WORDS],
        }
    }

    /// Returns the data length code.
    pub fn dlc(&self) -> u8 {
        (self.words[0] & 0xF) as u8
    }

    /// Returns whether the frame is a request frame.
    ///
    /// CAN FD has no request frames, so this is only meaningful when
    /// [`Self::is_fd`] returns `false`.
    pub fn is_request(&self) -> bool {
        self.words[0] & (1 << 5) != 0
    }

    /// Returns whether the frame uses a 29-bit identifier.
    pub fn is_extended(&self) -> bool {
        self.words[0] & (1 << 6) != 0
    }

    /// Returns whether the frame is in CAN FD format.
    pub fn is_fd(&self) -> bool {
        self.words[0] & (1 << 7) != 0
    }

    /// Returns whether the data phase is sent at the FD bit rate.
    pub fn is_bit_rate_switched(&self) -> bool {
        self.words[0] & (1 << 9) != 0
    }

    /// Returns the error state indicator of the transmitting node.
    pub fn error_state_indicator(&self) -> bool {
        self.words[0] & (1 << 10) != 0
    }

    /// Number of words that follow word 0 for this frame.
    ///
    /// This is CTU CAN FD's RWCNT (read word count) field. ESP-IDF's
    /// `twaifd_struct.h` mislabels it as a re-transmission counter, while its own
    /// RX read loop uses it as a word count, which is what the hardware means.
    pub(super) fn word_count(&self) -> u8 {
        ((self.words[0] >> 11) & 0x1F) as u8
    }

    /// Returns the frame identifier.
    pub fn id(&self) -> u32 {
        if self.is_extended() {
            self.words[1] & EXT_ID_MASK
        } else {
            (self.words[1] >> IDENTIFIER_BASE_SHIFT) & STD_ID_MASK
        }
    }

    /// Returns the timestamp captured for a received frame, or the trigger time
    /// of a frame queued in time-triggered transmission mode.
    pub fn timestamp(&self) -> u64 {
        (self.words[2] as u64) | ((self.words[3] as u64) << 32)
    }

    /// Returns the payload length in bytes.
    ///
    /// Only CAN FD frames use the extended data length codes. A classic frame
    /// with a DLC above 8 still carries 8 bytes, per CAN 2.0, so decoding it
    /// with the FD table would report a length the frame does not have and
    /// hand back bytes that were never received.
    ///
    /// A request frame carries no payload at all; its DLC is the length being
    /// asked for, which [`Self::requested_len`] reports.
    pub fn data_len(&self) -> usize {
        if self.is_request() {
            return 0;
        }
        let len = dlc_to_len(self.dlc()) as usize;
        if self.is_fd() {
            len
        } else {
            len.min(CLASSIC_MAX_DATA_LEN)
        }
    }

    /// Returns the payload length a request frame asks for, in bytes.
    ///
    /// Meaningless unless [`Self::is_request`] returns `true`.
    pub fn requested_len(&self) -> usize {
        (dlc_to_len(self.dlc()) as usize).min(CLASSIC_MAX_DATA_LEN)
    }

    /// Copies the payload into `buf` and returns its length in bytes.
    pub fn data(&self, buf: &mut [u8; MAX_DATA_LEN]) -> usize {
        let len = self.data_len();
        for (i, byte) in buf.iter_mut().enumerate().take(len) {
            let word = self.words[Self::DATA_WORD_OFFSET + i / 4];
            *byte = (word >> (8 * (i % 4))) as u8;
        }
        len
    }

    /// Builds a frame buffer ready to be written to a TX buffer.
    ///
    /// `dlc` must already be encoded; use [`len_to_dlc`] to derive it from a
    /// payload length.
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        id: u32,
        extended: bool,
        rtr: bool,
        fd: bool,
        brs: bool,
        dlc: u8,
        data: &[u8],
        trigger_time: u64,
    ) -> Self {
        debug_assert!(dlc <= MAX_DLC);
        debug_assert!(data.len() <= MAX_DATA_LEN);

        let mut frame = Self::new();

        frame.words[0] = (dlc as u32 & 0xF)
            | ((rtr as u32) << 5)
            | ((extended as u32) << 6)
            | ((fd as u32) << 7)
            | ((brs as u32) << 9);

        frame.words[1] = if extended {
            id & EXT_ID_MASK
        } else {
            (id & STD_ID_MASK) << IDENTIFIER_BASE_SHIFT
        };

        // Writing zero here transmits as soon as the bus is idle (TRM 38.3.8.2).
        frame.words[2] = trigger_time as u32;
        frame.words[3] = (trigger_time >> 32) as u32;

        for (i, &byte) in data.iter().enumerate() {
            frame.words[Self::DATA_WORD_OFFSET + i / 4] |= (byte as u32) << (8 * (i % 4));
        }

        frame
    }
}

/// Converts a CAN FD data length code to a payload length in bytes.
pub const fn dlc_to_len(dlc: u8) -> u8 {
    match dlc {
        0..=8 => dlc,
        9..=12 => (dlc - 8) * 4 + 8,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// Converts a payload length in bytes to the smallest CAN FD data length code
/// that can carry it.
pub const fn len_to_dlc(len: u8) -> u8 {
    match len {
        0..=8 => len,
        9..=24 => (len - 8).div_ceil(4) + 8,
        25..=32 => (len - 24).div_ceil(8) + 12,
        _ => (len - 32).div_ceil(16) + 13,
    }
}

/// Low-level accessor for one CAN FD controller.
pub(super) struct Ll {
    regs: *const RegisterBlock,
}

// SAFETY: the pointer addresses a memory-mapped register block that exists for
// the whole life of the program, so it can be used from any context. Without
// these the drivers holding an `Ll` could not be moved into a task or shared
// with an interrupt handler through a `static`. Whether concurrent access is
// sound is decided one level up: the drivers only read through `&self`, and
// everything that writes takes `&mut self`.
unsafe impl Send for Ll {}
unsafe impl Sync for Ll {}

impl Ll {
    pub(super) fn new(regs: *const RegisterBlock) -> Self {
        Self { regs }
    }

    fn r(&self) -> &RegisterBlock {
        // SAFETY: `regs` points at a live peripheral register block for as long as
        // the driver holds the peripheral.
        unsafe { &*self.regs }
    }

    /// Address of one TX buffer RAM cell.
    ///
    /// The TX buffer RAM is documented in TRM 38.3.8 but absent from the register
    /// summary in TRM 38.6, so it is missing from the SVD and hence from the PAC.
    /// Until esp-pacs models it, address it directly.
    fn txt_buffer(&self, index: u8) -> *mut u32 {
        debug_assert!((index as usize) < self.tx_buffer_count() as usize);
        let base = self.regs as usize;
        (base + TXT_BUFFER_BASE + TXT_BUFFER_STRIDE * index as usize) as *mut u32
    }

    // ---------------------------------------------------------------- identity

    /// Device ID reported by the core. Reads `0xCAFD` on a working core.
    pub(super) fn device_id(&self) -> u16 {
        self.r().device_id_version().read().device_id().bits()
    }

    /// IP core version as `(major, minor)`.
    pub(super) fn version(&self) -> (u8, u8) {
        let r = self.r().device_id_version().read();
        (r.ver_major().bits(), r.ver_minor().bits())
    }

    // ------------------------------------------------------------------- reset

    /// Issues a soft reset. No wait is required afterwards (TRM 38.3.2).
    pub(super) fn reset(&self) {
        self.r().mode_settings().modify(|_, w| w.rst().set_bit());
    }

    /// Enables or disables the controller.
    ///
    /// Most mode bits may only be changed while disabled.
    pub(super) fn enable(&self, enable: bool) {
        self.r().mode_settings().modify(|_, w| w.ena().bit(enable));
    }

    /// Whether the controller is enabled.
    pub(super) fn is_enabled(&self) -> bool {
        self.r().mode_settings().read().ena().bit_is_set()
    }

    // -------------------------------------------------------------------- mode

    /// Applies every mode setting in one write.
    ///
    /// `MODE_SETTINGS` holds all of these bits, so they are written together
    /// rather than through a read-modify-write cycle each.
    ///
    /// Only valid while the controller is disabled.
    pub(super) fn apply_mode_settings(&self, settings: &ModeSettings) {
        debug_assert!(!self.is_enabled(), "mode may only change while disabled");

        self.r().mode_settings().modify(|_, w| {
            // `listen_only` drives three bits together: the ESP32-C5 needs `rom`
            // and `acf` alongside `bmm` to work around errata 0v2 issue 5
            // (esp-idf#17461).
            w.bmm().bit(settings.listen_only);
            w.rom().bit(settings.listen_only);
            w.acf().bit(settings.listen_only);
            w.stm().bit(settings.self_test);
            w.ilbp().bit(settings.loopback);
            // Whether a frame is FD is then decided per frame by its FDF bit.
            w.fde().bit(settings.fd_enabled);
            w.pex().bit(settings.protocol_exception);
            // read_rx_frame depends on the read pointer advancing automatically.
            w.rxbam().bit(settings.rx_auto_increment);
            // Individual filters keep their own enables; this is the group switch.
            w.afm().bit(settings.filters_enabled);
            w.tbfbo().bit(settings.bus_off_tx_fail);
            w.fdrf().bit(settings.drop_request_frames);
            w.tttm().bit(settings.time_triggered_tx);
            match settings.retransmit_limit {
                Some(limit) => {
                    w.rtrle().set_bit();
                    unsafe { w.rtrth().bits(limit) }
                }
                None => w.rtrle().clear_bit(),
            }
        });
    }

    // ------------------------------------------------------------- bit timing

    /// Programs the nominal (arbitration phase) bit timing.
    pub(super) fn set_nominal_timing(&self, timing: &Timing) {
        self.r().btr().write(|w| unsafe {
            w.brp().bits(timing.baud_rate_prescaler);
            w.prop().bits(timing.propagation_segment);
            w.ph1().bits(timing.phase_segment_1);
            w.ph2().bits(timing.phase_segment_2);
            w.sjw().bits(timing.sync_jump_width)
        });
    }

    /// Programs the data phase bit timing used by bit-rate-switched FD frames.
    pub(super) fn set_fd_timing(&self, timing: &Timing) {
        self.r().btr_fd().write(|w| unsafe {
            w.brp_fd().bits(timing.baud_rate_prescaler);
            w.prop_fd().bits(timing.propagation_segment);
            w.ph1_fd().bits(timing.phase_segment_1);
            w.ph2_fd().bits(timing.phase_segment_2);
            w.sjw_fd().bits(timing.sync_jump_width)
        });
    }

    /// Places the secondary sample point.
    ///
    /// `offset` is in function clock cycles, not time quanta, so a caller working
    /// in quanta must multiply by the prescaler first.
    pub(super) fn set_secondary_sample_point(&self, source: SspSource, offset: u8) {
        self.r().trv_delay_ssp_cfg().modify(|_, w| unsafe {
            w.ssp_src().bits(source as u8);
            w.ssp_offset().bits(offset)
        });
    }

    /// Transmitter delay measured by the hardware, in function clock cycles.
    ///
    /// Includes the core's own two cycles of input delay.
    pub(super) fn transmitter_delay(&self) -> u8 {
        self.r().trv_delay_ssp_cfg().read().trv_delay_value().bits()
    }

    // -------------------------------------------------------------- TX buffers

    /// Number of TX buffers the hardware provides.
    pub(super) fn tx_buffer_count(&self) -> u8 {
        self.r()
            .tx_command_txtb_info()
            .read()
            .txt_buffer_count()
            .bits()
    }

    /// State of one TX buffer.
    ///
    /// The caller is responsible for the index being one the hardware has: the
    /// states share a single register, four bits each (TRM 38.22), so a larger
    /// index reads whatever the shift lands on.
    pub(super) fn tx_buffer_state(&self, index: u8) -> TxBufferState {
        debug_assert!(index < self.tx_buffer_count());

        let raw = self.r().tx_status().read().bits();
        TxBufferState::from_bits(((raw >> (4 * index as u32)) & 0xF) as u8)
    }

    /// Writes a frame into a TX buffer.
    ///
    /// The buffer must be in a writable state; see [`TxBufferState::is_writable`].
    pub(super) fn write_tx_buffer(&self, index: u8, frame: &FrameBuffer) {
        let ptr = self.txt_buffer(index);
        for (i, &word) in frame.words.iter().enumerate() {
            // SAFETY: `ptr` addresses a TX buffer RAM cell of FRAME_WORDS words.
            unsafe { ptr.add(i).write_volatile(word) };
        }
    }

    /// Arms a TX buffer for transmission.
    pub(super) fn set_tx_ready(&self, index: u8) {
        self.tx_command(index, |w| w.txcr().set_bit());
    }

    /// Requests that a TX buffer be aborted.
    pub(super) fn set_tx_abort(&self, index: u8) {
        self.tx_command(index, |w| w.txca().set_bit());
    }

    /// Returns a TX buffer to the empty state.
    pub(super) fn set_tx_empty(&self, index: u8) {
        self.tx_command(index, |w| w.txce().set_bit());
    }

    fn tx_command<F>(&self, index: u8, cmd: F)
    where
        F: FnOnce(
            &mut crate::pac::twai0::tx_command_txtb_info::W,
        ) -> &mut crate::pac::twai0::tx_command_txtb_info::W,
    {
        self.r().tx_command_txtb_info().write(|w| {
            cmd(w);
            match index {
                0 => w.txb1().set_bit(),
                1 => w.txb2().set_bit(),
                2 => w.txb3().set_bit(),
                3 => w.txb4().set_bit(),
                4 => w.txb5().set_bit(),
                5 => w.txb6().set_bit(),
                6 => w.txb7().set_bit(),
                _ => w.txb8().set_bit(),
            }
        });
    }

    /// Sets a TX buffer's arbitration priority. Higher wins; ties go to the
    /// lower buffer index.
    pub(super) fn set_tx_priority(&self, index: u8, priority: u8) {
        self.r().tx_priority().modify(|_, w| unsafe {
            match index {
                0 => w.txt1p().bits(priority),
                1 => w.txt2p().bits(priority),
                2 => w.txt3p().bits(priority),
                3 => w.txt4p().bits(priority),
                4 => w.txt5p().bits(priority),
                5 => w.txt6p().bits(priority),
                6 => w.txt7p().bits(priority),
                _ => w.txt8p().bits(priority),
            }
        });
    }

    // --------------------------------------------------------------- RX buffer

    /// Size of the RX buffer, in 32-bit words.
    pub(super) fn rx_buffer_size(&self) -> u16 {
        self.r().rx_mem_info().read().rx_buff_size().bits()
    }

    /// Free space in the RX buffer, in 32-bit words.
    pub(super) fn rx_free_words(&self) -> u16 {
        self.r().rx_mem_info().read().rx_free().bits()
    }

    /// Number of complete frames waiting in the RX buffer.
    pub(super) fn rx_frame_count(&self) -> u16 {
        self.r().rx_status_rx_settings().read().rxfrc().bits()
    }

    /// Returns whether the RX buffer has dropped a frame for lack of space.
    ///
    /// This is the sticky overrun flag of TRM 38.3.9.4, cleared by
    /// [`Ll::clear_overrun`]. Not to be confused with `RXMOF`, which TRM
    /// 38.3.9.6 defines as "the read pointer is in the middle of a frame" and
    /// which is about recovering from a failed read, not about overrun.
    pub(super) fn rx_overrun(&self) -> bool {
        self.r().status().read().dor().bit_is_set()
    }

    /// Returns whether an error counter has reached the error warning limit.
    ///
    /// This is a threshold flag, not a fault confinement state: a node at the
    /// warning limit is still error-active (TRM 38.3.10).
    pub(super) fn error_warning(&self) -> bool {
        self.r().status().read().ewl().bit_is_set()
    }

    /// Selects when a received frame is timestamped.
    pub(super) fn set_timestamp_point(&self, point: TimestampPoint) {
        self.r()
            .rx_status_rx_settings()
            .modify(|_, w| w.rtsop().bit(point == TimestampPoint::StartOfFrame));
    }

    /// Reads one frame out of the RX buffer.
    ///
    /// Requires RX auto-increment mode; see [`Ll::enable_rx_auto_increment`].
    /// The first word carries the count of words that follow.
    pub(super) fn read_rx_frame(&self) -> FrameBuffer {
        let mut frame = FrameBuffer::new();
        let rx_data = self.r().rx_data();

        frame.words[0] = rx_data.read().bits();
        let remaining = (frame.word_count() as usize).min(FRAME_WORDS - 1);
        for word in frame.words[1..=remaining].iter_mut() {
            *word = rx_data.read().bits();
        }

        frame
    }

    // ---------------------------------------------------------------- commands

    /// Discards the RX buffer contents and resets its pointers and frame counter.
    pub(super) fn flush_rx(&self) {
        self.r().command().write(|w| w.rrb().set_bit());
    }

    /// Clears the RX buffer overrun flag.
    pub(super) fn clear_overrun(&self) {
        self.r().command().write(|w| w.cdo().set_bit());
    }

    /// Requests error counter reset, which is how a bus-off node rejoins.
    ///
    /// TRM register 38.3 says this has no effect unless the controller is
    /// bus-off for a reason other than being disabled. It does not hold:
    /// measured on an ESP32-C5, a request made while the controller is
    /// error-active is remembered and reintegrates it the next time it goes
    /// bus-off, which is the sticky behavior TRM 38.3.4 describes instead.
    /// Callers must check the state themselves.
    pub(super) fn request_bus_off_recovery(&self) {
        self.r().command().write(|w| w.ercrst().set_bit());
    }

    /// Resets the RX and TX traffic counters.
    pub(super) fn reset_traffic_counters(&self) {
        self.r()
            .command()
            .write(|w| w.rxfcrst().set_bit().txfcrst().set_bit());
    }

    // ------------------------------------------------------------------ errors

    /// Receive error counter.
    pub(super) fn rec(&self) -> u16 {
        self.r().rec_tec().read().rec_val().bits()
    }

    /// Transmit error counter.
    pub(super) fn tec(&self) -> u16 {
        self.r().rec_tec().read().tec_val().bits()
    }

    /// Current fault confinement state.
    pub(super) fn error_state(&self) -> ErrorState {
        let r = self.r().ewl_erp_fault_state().read();
        // Checked most severe first: the hardware can assert more than one of
        // these while a transition settles.
        if r.bof().bit_is_set() {
            ErrorState::BusOff
        } else if r.erp().bit_is_set() {
            ErrorState::Passive
        } else {
            ErrorState::Active
        }
    }

    /// Sets the error warning limit.
    ///
    /// TRM 38.3.10: the limit fields are only writable in test mode, since
    /// moving them takes the controller outside ISO 11898-1. The caller is
    /// responsible for enabling test mode while the controller is disabled;
    /// `enable_test_mode` does that.
    pub(super) fn set_error_warning_limit(&self, limit: u8) {
        debug_assert!(
            self.r().mode_settings().read().tstm().bit_is_set(),
            "the error warning limit is only writable in test mode"
        );
        self.r()
            .ewl_erp_fault_state()
            .modify(|_, w| unsafe { w.ew_limit().bits(limit) });
    }

    /// Enables test mode, which makes the error counters and their limits
    /// writable (TRM 38.3.10). Only valid while the controller is disabled.
    pub(super) fn enable_test_mode(&self, enable: bool) {
        debug_assert!(
            !self.is_enabled(),
            "test mode may only change while disabled"
        );
        self.r().mode_settings().modify(|_, w| w.tstm().bit(enable));
    }

    // -------------------------------------------------------------- interrupts

    /// Enables the interrupt sources set in `mask`.
    pub(super) fn enable_interrupts(&self, mask: u32) {
        self.r().int_ena_set().write(|w| unsafe { w.bits(mask) });
    }

    /// Disables the interrupt sources set in `mask`.
    pub(super) fn disable_interrupts(&self, mask: u32) {
        self.r().int_ena_clr().write(|w| unsafe { w.bits(mask) });
    }

    /// Reads the masked interrupt status.
    pub(super) fn interrupt_status(&self) -> u32 {
        self.r().int_stat().read().bits()
    }

    /// Clears the interrupt sources set in `mask`.
    pub(super) fn clear_interrupts(&self, mask: u32) {
        self.r().int_stat().write(|w| unsafe { w.bits(mask) });
    }

    // ------------------------------------------------------------------ counters

    /// Number of frames received since the counter was last reset.
    pub(super) fn rx_traffic_counter(&self) -> u32 {
        self.r().rx_fr_ctr().read().val().bits()
    }

    /// Number of frames transmitted since the counter was last reset.
    pub(super) fn tx_traffic_counter(&self) -> u32 {
        self.r().tx_fr_ctr().read().tx_ctr_val().bits()
    }

    // ----------------------------------------------------------------- filters

    /// Sets a mask filter's acceptance code and mask.
    ///
    /// Both are compared against the identifier word, which holds a base
    /// identifier in bits 28..18 and an extended identifier in bits 28..0, so a
    /// standard-identifier filter is shifted up (TRM 38.3.9.9).
    pub(super) fn set_mask_filter(&self, filter: MaskFilter, extended: bool, code: u32, mask: u32) {
        let (code, mask) = if extended {
            (code & EXT_ID_MASK, mask & EXT_ID_MASK)
        } else {
            (
                (code & STD_ID_MASK) << IDENTIFIER_BASE_SHIFT,
                (mask & STD_ID_MASK) << IDENTIFIER_BASE_SHIFT,
            )
        };

        match filter {
            MaskFilter::A => {
                self.r()
                    .filter_a_mask()
                    .write(|w| unsafe { w.bit_mask_a_val().bits(mask) });
                self.r()
                    .filter_a_val()
                    .write(|w| unsafe { w.bit_val_a_val().bits(code) });
            }
            MaskFilter::B => {
                self.r()
                    .filter_b_mask()
                    .write(|w| unsafe { w.bit_mask_b_val().bits(mask) });
                self.r()
                    .filter_b_val()
                    .write(|w| unsafe { w.bit_val_b_val().bits(code) });
            }
            MaskFilter::C => {
                self.r()
                    .filter_c_mask()
                    .write(|w| unsafe { w.bit_mask_c_val().bits(mask) });
                self.r()
                    .filter_c_val()
                    .write(|w| unsafe { w.bit_val_c_val().bits(code) });
            }
        }
    }

    /// Sets the identifier range the range filter accepts.
    ///
    /// For a standard-identifier range the low 18 bits of the upper bound are
    /// set so the whole base identifier is covered (TRM 38.3.9.10).
    pub(super) fn set_range_filter(&self, extended: bool, low: u32, high: u32) {
        let (low, high) = if extended {
            (low & EXT_ID_MASK, high & EXT_ID_MASK)
        } else {
            (
                (low & STD_ID_MASK) << IDENTIFIER_BASE_SHIFT,
                ((high & STD_ID_MASK) << IDENTIFIER_BASE_SHIFT) | 0x3FFFF,
            )
        };
        self.r()
            .filter_ran_low()
            .write(|w| unsafe { w.bit_ran_low_val().bits(low) });
        self.r()
            .filter_ran_high()
            .write(|w| unsafe { w.bit_ran_high_val().bits(high) });
    }

    /// Sets which frame kinds each filter accepts.
    ///
    /// A filter given [`FrameKinds::NONE`] is disabled.
    pub(super) fn set_filter_kinds(
        &self,
        a: FrameKinds,
        b: FrameKinds,
        c: FrameKinds,
        range: FrameKinds,
    ) {
        let bits = a.bits() | (b.bits() << 4) | (c.bits() << 8) | (range.bits() << 12);
        // The upper half of this register is read-only "filter supported"
        // status, so writing zeroes there is ignored and the register does not
        // need to be read back first.
        self.r()
            .filter_control_filter_status()
            .write(|w| unsafe { w.bits(bits) });
    }

    /// Which filters this core actually implements, as `(a, b, c, range)`.
    pub(super) fn filters_supported(&self) -> (bool, bool, bool, bool) {
        let r = self.r().filter_control_filter_status().read();
        (
            r.sfa().bit_is_set(),
            r.sfb().bit_is_set(),
            r.sfc().bit_is_set(),
            r.sfr().bit_is_set(),
        )
    }

    // ----------------------------------------------------------- error capture

    /// Details of the last bus error.
    pub(super) fn error_capture(&self) -> ErrorCapture {
        let r = self.r().err_capt_retr_ctr_alc_ts_info().read();
        ErrorCapture {
            kind: BusErrorKind::from_bits(r.err_type().bits()),
            position: ErrorPosition::from_bits(r.err_pos().bits()),
        }
    }

    /// Retransmission attempts made for the current frame.
    pub(super) fn retransmit_count(&self) -> u8 {
        self.r()
            .err_capt_retr_ctr_alc_ts_info()
            .read()
            .retr_ctr_val()
            .bits()
    }

    /// Error counters for the nominal and data phases, as `(nominal, fd)`.
    pub(super) fn special_error_counters(&self) -> (u16, u16) {
        let r = self.r().err_norm_err_fd().read();
        (r.err_norm_val().bits(), r.err_fd_val().bits())
    }

    // ------------------------------------------------------- timestamp counter

    /// Width of the timestamp counter in bits.
    pub(super) fn timer_bit_width(&self) -> u8 {
        self.r()
            .err_capt_retr_ctr_alc_ts_info()
            .read()
            .ts_bits()
            .bits()
            + 1
    }

    /// Force-enables the timer's register configuration clock.
    pub(super) fn timer_enable_config_clock(&self, enable: bool) {
        self.r()
            .timer_clk_en()
            .modify(|_, w| w.clk_en().bit(enable));
    }

    /// Starts or stops the timestamp counter.
    pub(super) fn timer_enable(&self, enable: bool) {
        self.r().timer_cfg().modify(|_, w| w.timer_ce().bit(enable));
    }

    /// Sets whether the counter counts up.
    pub(super) fn timer_count_up(&self, up: bool) {
        self.r().timer_cfg().modify(|_, w| w.timer_up_dn().bit(up));
    }

    /// Sets the timer prescaler.
    ///
    /// `divider` is the number of function clock cycles per tick; the register
    /// holds one less than that.
    ///
    /// TRM 38.13 names this field `TIMER_STEP` and describes it as a count step
    /// of `TIMER_STEP + 1`, which reads like an increment added on every clock
    /// cycle rather than a divider. It is a divider: measured on an ESP32-C5
    /// against an 80 MHz function clock, field values 0, 1 and 3 make the
    /// counter advance at 80, 40 and 20 MHz respectively, and the counter takes
    /// every intermediate value rather than stepping in multiples.
    pub(super) fn timer_set_divider(&self, divider: u16) {
        debug_assert!(divider >= 1);
        self.r()
            .timer_cfg()
            .modify(|_, w| unsafe { w.timer_step().bits(divider - 1) });
    }

    /// Clears the timestamp counter.
    pub(super) fn timer_clear(&self) {
        self.r().timer_cfg().modify(|_, w| w.timer_clr().set_bit());
    }

    /// Makes the counter free-running over its whole width.
    ///
    /// The pre-load and count-to values (TRM 38.14 to 38.17) bound the counter;
    /// programming them explicitly keeps the timestamps from depending on
    /// whatever a previous user of the peripheral left behind.
    pub(super) fn timer_set_free_running(&self) {
        self.r()
            .timer_ld_val_l()
            .write(|w| unsafe { w.timer_ld_val_l().bits(0) });
        self.r()
            .timer_ld_val_h()
            .write(|w| unsafe { w.timer_ld_val_h().bits(0) });
        self.r()
            .timer_ct_val_l()
            .write(|w| unsafe { w.timer_ct_val_l().bits(u32::MAX) });
        self.r()
            .timer_ct_val_h()
            .write(|w| unsafe { w.timer_ct_val_h().bits(u32::MAX) });
    }

    /// Current timestamp counter value.
    pub(super) fn timer_count(&self) -> u64 {
        // The counter keeps running, so re-read the high word if the low word
        // wrapped between the two reads.
        loop {
            let high = self.r().timestamp_high().read().bits();
            let low = self.r().timestamp_low().read().bits();
            if self.r().timestamp_high().read().bits() == high {
                return ((high as u64) << 32) | low as u64;
            }
        }
    }
}
