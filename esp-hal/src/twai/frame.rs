//! TWAI Frame essentials
use core::ops::Deref;

use super::{EspTwaiError, ExtendedId, Id, StandardId};
use crate::pac::twai0::RegisterBlock;

/// A CAN frame.
pub enum Frame {
    /// A CAN data frame (has data payload).
    Data(DataFrame),
    /// A CAN request frame (has no data payload).
    Request(RequestFrame),
}

impl Deref for Frame {
    type Target = TwaiFrame;

    fn deref(&self) -> &<Self as Deref>::Target {
        match self {
            Frame::Data(frame) => &frame.raw,
            Frame::Request(frame) => &frame.raw,
        }
    }
}

impl From<DataFrame> for Frame {
    fn from(frame: DataFrame) -> Self {
        Self::Data(frame)
    }
}

impl From<RequestFrame> for Frame {
    fn from(frame: RequestFrame) -> Self {
        Self::Request(frame)
    }
}

impl From<TwaiFrame> for Frame {
    fn from(raw: TwaiFrame) -> Self {
        match raw.is_data_frame() {
            true => Frame::Data(DataFrame { raw }),
            false => Frame::Request(RequestFrame { raw }),
        }
    }
}

/// A CAN data frame.
pub struct DataFrame {
    raw: TwaiFrame,
}

impl DataFrame {
    /// Create a new Data Frame.
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Result<Self, EspTwaiError> {
        Self::new_custom_dlc(id, data, data.len())
    }

    /// Create a new Data Frame with a custom DLC.
    pub fn new_custom_dlc(
        id: impl Into<Id>,
        data: &[u8],
        dlc: usize,
    ) -> Result<Self, EspTwaiError> {
        TwaiFrame::new_data(id.into(), data, dlc, false).map(|raw| raw.into())
    }

    /// Create a new Data Frame for self reception.
    pub fn new_self_reception(
        id: impl Into<Id>,
        data: &[u8],
        dlc: usize,
    ) -> Result<Self, EspTwaiError> {
        TwaiFrame::new_data(id.into(), data, dlc, true).map(|raw| raw.into())
    }
}

impl Deref for DataFrame {
    type Target = TwaiFrame;

    fn deref(&self) -> &<Self as Deref>::Target {
        &self.raw
    }
}

impl From<TwaiFrame> for DataFrame {
    fn from(raw: TwaiFrame) -> Self {
        Self { raw }
    }
}

/// A CAN request frame.
pub struct RequestFrame {
    raw: TwaiFrame,
}

impl RequestFrame {
    /// Create a new Request Frame.
    pub fn new(id: impl Into<Id>, dlc: usize) -> Result<Self, EspTwaiError> {
        TwaiFrame::new_request(id, dlc, false).map(|raw| raw.into())
    }

    /// Create a new Request Frame for self reception.
    pub fn new_self_reception(id: impl Into<Id>, dlc: usize) -> Result<Self, EspTwaiError> {
        TwaiFrame::new_request(id, dlc, true).map(|raw| raw.into())
    }
}

impl Deref for RequestFrame {
    type Target = TwaiFrame;

    fn deref(&self) -> &<Self as Deref>::Target {
        &self.raw
    }
}

impl From<TwaiFrame> for RequestFrame {
    fn from(raw: TwaiFrame) -> Self {
        Self { raw }
    }
}

/// A raw TWAI frame.
///
/// Mirror image of the 13 TWAI_DATA_x_REG registers.
#[derive(Clone, Copy)]
pub struct TwaiFrame {
    bytes: [u8; 13],
}

impl TwaiFrame {
    /// Frame information.
    ///
    /// | Offset | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
    /// |--------|-------|-------|-------|-------|-------|-------|-------|-------|
    /// |  0x0   |  FF   |  RTR  |  ---  |  SR   | DLC.3 | DLC.2 | DLC.1 | DLC.0 |
    #[inline(always)]
    fn info(&self) -> u8 {
        self.bytes[0]
    }

    /// Frame Format (FF): specifies whether content is Extended Frame Format (EFF) or Standard
    /// Frame Format (SFF).
    pub fn is_extended_format(&self) -> bool {
        (self.info() & (0b1 << 7)) != 0
    }
    /// Is this a Standard Format frame?
    pub fn is_standard_format(&self) -> bool {
        !self.is_extended_format()
    }

    /// Remote Transmission Request (RTR): specifies whether content is a data frame or a remote
    /// request frame (on-demand polling).
    pub fn is_request_frame(&self) -> bool {
        (self.info() & (0b1 << 6)) != 0
    }
    /// Is this a Data Frame?
    pub fn is_data_frame(&self) -> bool {
        !self.is_request_frame()
    }

    /// Self Reception (SR): indicates whether content was sent by us (using the TWAI_SELF_RX_SEQ
    /// command) or received from the bus.
    pub fn is_self_reception(&self) -> bool {
        (self.info() & (0b1 << 4)) != 0
    }
    /// Is this frame coming from the TWAI bus?
    pub fn is_bus_frame(&self) -> bool {
        !self.is_self_reception()
    }

    /// Data Length Code (DLC): specifies the number of data bytes for a data frame, or the number
    /// of data bytes requested by a remote frame.
    ///
    /// Note: although no frame can have a payload longer than 8, the DLC can be greater than 8 in
    /// rare cases (payload length then is still 8).
    pub fn data_length_code(&self) -> usize {
        (self.info() & 0b1111) as usize
    }
    /// Data length.
    pub fn data_length(&self) -> usize {
        match self.is_request_frame() {
            false => core::cmp::min(8, self.data_length_code()),
            true => 0,
        }
    }

    /// Frame Identifier: 11-bit long for a SFF frame, 29-bit long for an EFF frame.
    #[inline]
    pub fn identifier(&self) -> Id {
        let bytes = self.bytes;
        match self.is_extended_format() {
            false => {
                // Standard Format: 11-bit Identifier, 2 bytes long
                //
                // | Offset | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
                // |--------|-------|-------|-------|-------|-------|-------|-------|-------|
                // |  0x1   | ID.10 | ID.9  | ID.8  | ID.7  | ID.6  | ID.5  | ID.4  | ID.3  |
                // |  0x2   | ID.2  | ID.1  | ID.0  |  ---  |  ---  |  ---  |  ---  |  ---  |
                let raw_id: u16 = ((bytes[1] as u16) << 3) | ((bytes[2] as u16) >> 5);
                // SAFETY: safe because raw_id is 11 bits long (it cannot exceed StandardId::MAX).
                unsafe { StandardId::new_unchecked(raw_id).into() }
            }
            true => {
                // Extended Format: 29-bit Identifier, 4 bytes long
                //
                // | Offset | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
                // |--------|-------|-------|-------|-------|-------|-------|-------|-------|
                // |  0x1   | ID.28 | ID.27 | ID.26 | ID.25 | ID.24 | ID.23 | ID.22 | ID.21 |
                // |  0x2   | ID.20 | ID.19 | ID.18 | ID.17 | ID.16 | ID.15 | ID.14 | ID.13 |
                // |  0x3   | ID.12 | ID.11 | ID.10 | ID.9  | ID.8  | ID.7  | ID.6  | ID.5  |
                // |  0x4   | ID.4  | ID.3  | ID.2  | ID.1  | ID.0  |  ---  |  ---  |  ---  |
                let raw_id: u32 = ((bytes[1] as u32) << 21)
                    | ((bytes[2] as u32) << 13)
                    | ((bytes[3] as u32) << 5)
                    | ((bytes[4] as u32) >> 3);
                // SAFETY: safe because raw_id is 29 bits long (it cannot exceed ExtendedId::MAX)
                unsafe { ExtendedId::new_unchecked(raw_id).into() }
            }
        }
    }

    /// Offset at which frame data starts: 0x3 for a SFF, 0x5 for an EFF.
    #[inline(always)]
    fn data_offset(&self) -> usize {
        match self.is_extended_format() {
            false => 0x3,
            true => 0x5,
        }
    }

    /// Frame Data: data payload, 0 to 8 bytes long.
    ///
    /// Returns a reference to a slice:
    /// * empty in case of a Remote Transmission Request
    /// * 8 bytes long in case DLC > 8
    #[inline]
    pub fn data(&self) -> &[u8] {
        let data_start = self.data_offset();
        let data_end = data_start + self.data_length();
        &self.bytes[data_start..data_end]
    }

    /// Returns a slice reference to the relevant frame bytes.
    pub(super) fn as_slice(&self) -> &[u8] {
        let len = self.data_offset() + self.data_length();
        &self.bytes[0..len]
    }

    /// Make a new raw [`TwaiFrame`] from TWAI_DATA_x_REG registers.
    pub(super) fn new_from_registers(register_block: &RegisterBlock) -> Self {
        let mut bytes: [u8; 13] = [0; 13];
        // SAFETY: Safe because it is a constant-size, read-only access to the 13 data registers
        unsafe {
            copy_from_data_register(&mut bytes, register_block.data(0).as_ptr());
        }
        Self { bytes }
    }

    /// Make a new [`RawFrame`] from parameters.
    unsafe fn new_unchecked(
        id: impl Into<Id>,
        remote_request: bool,
        dlc: usize,
        data: &[u8],
        self_reception: bool,
    ) -> Self {
        let mut bytes = [0u8; 13];

        // Id
        let (extended_format, data_start): (bool, usize) = match id.into() {
            Id::Standard(id) => {
                let raw = id.as_raw();
                bytes[1] = (raw >> 3) as u8;
                bytes[2] = (raw << 5) as u8;
                (false, 3)
            }
            Id::Extended(id) => {
                let raw = id.as_raw();
                bytes[1] = (raw >> 21) as u8;
                bytes[2] = (raw >> 13) as u8;
                bytes[3] = (raw >> 5) as u8;
                bytes[4] = (raw << 3) as u8;
                (true, 5)
            }
        };
        // Frame Info
        let ff = (extended_format as u8) << 7;
        let rtr = (remote_request as u8) << 6;
        let sr = (self_reception as u8) << 4;
        let dlc = (dlc as u8) & 0b1111;
        bytes[0] = ff | rtr | sr | dlc;
        // Data
        let data_end = data_start + data.len();
        bytes[data_start..data_end].copy_from_slice(data);

        Self { bytes }
    }

    /// Create a new data `RawFrame` with the specified ID and payload.
    fn new_data(
        id: impl Into<Id>,
        data: &[u8],
        dlc: usize,
        self_reception: bool,
    ) -> Result<Self, EspTwaiError> {
        let len = data.len();
        // Assert that max data length is 8
        if len > 8 {
            return Err(EspTwaiError::InvalidDataLength(len as u8));
        }
        // Assert that:
        // - Max DLC is 15
        // - Data length smaller than 8 must have equal DLC
        // - Data length equal to 8 must have DLC >= 8
        if (dlc > 15) || ((len < 8) & (dlc != len)) || ((len == 8) & (dlc < 8)) {
            return Err(EspTwaiError::NonCompliantDlc(dlc as u8));
        }
        // SAFETY: Safe because we have validated every parameter above.
        unsafe { Ok(Self::new_unchecked(id, false, dlc, data, self_reception)) }
    }

    /// Create a new request [`RawFrame`] with the ID and DLC.
    fn new_request(
        id: impl Into<Id>,
        dlc: usize,
        self_reception: bool,
    ) -> Result<Self, EspTwaiError> {
        // Assert that max DLC is 15
        if dlc > 15 {
            return Err(EspTwaiError::NonCompliantDlc(dlc as u8));
        }
        // SAFETY: Safe because we have validated every parameter above.
        unsafe { Ok(Self::new_unchecked(id, true, dlc, &[], self_reception)) }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TwaiFrame {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "EspTwaiFrame {{ id: {1=u32}, EFF: {0=7..8}, RTR: {0=6..7}, SR: {0=4..5}, DLC: {0=0..4}, data: {2=[u8]:#x}, raw: {3=[u8]:#x} }}",
            self.bytes[0],
            match self.identifier() {
                Id::Standard(id) => id.as_raw() as u32,
                Id::Extended(id) => id.as_raw(),
            },
            self.data(),
            self.as_slice(),
        );
    }
}

impl core::fmt::Debug for TwaiFrame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EspTwaiFrame")
            .field(
                "id",
                &match self.identifier() {
                    Id::Standard(id) => id.as_raw() as u32,
                    Id::Extended(id) => id.as_raw(),
                },
            )
            .field("EFF", &(self.is_extended_format() as u8))
            .field("RTR", &(self.is_request_frame() as u8))
            .field("SR", &(self.is_self_reception() as u8))
            .field("DLC", &self.data_length_code())
            .field("data", &format_args!("{:02x?}", self.data()))
            .field("raw", &format_args!("{:02x?}", self.as_slice()))
            .finish()
    }
}

impl From<DataFrame> for TwaiFrame {
    fn from(frame: DataFrame) -> Self {
        frame.raw
    }
}

impl From<RequestFrame> for TwaiFrame {
    fn from(frame: RequestFrame) -> Self {
        frame.raw
    }
}

impl From<Frame> for TwaiFrame {
    fn from(frame: Frame) -> Self {
        match frame {
            Frame::Data(frame) => frame.raw,
            Frame::Request(frame) => frame.raw,
        }
    }
}

#[instability::unstable]
impl embedded_can::Frame for TwaiFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let frame = DataFrame::new(id.into(), data).ok()?;
        Some(frame.into())
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        let frame = RequestFrame::new(id.into(), dlc).ok()?;
        Some(frame.into())
    }

    fn is_extended(&self) -> bool {
        self.is_extended_format()
    }

    fn is_remote_frame(&self) -> bool {
        self.is_request_frame()
    }

    fn id(&self) -> embedded_can::Id {
        self.identifier().into()
    }

    fn dlc(&self) -> usize {
        self.data_length_code()
    }

    fn data(&self) -> &[u8] {
        TwaiFrame::data(self)
    }
}

/// Copy data from multiple TWAI_DATA_x_REG registers, packing the source into
/// the destination.
///
/// # Safety
/// This function is marked unsafe because it reads arbitrarily from
/// memory-mapped registers. Specifically, this function is used with the
/// TWAI_DATA_x_REG registers which has different results based on the mode of
/// the peripheral.
#[inline(always)]
pub(super) unsafe fn copy_from_data_register(dest: &mut [u8], src: *const u32) {
    for (i, dest) in dest.iter_mut().enumerate() {
        // Perform a volatile read to avoid compiler optimizations.
        unsafe {
            *dest = src.add(i).read_volatile() as u8;
        }
    }
}
