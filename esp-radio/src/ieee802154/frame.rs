use heapless::Vec;
use ieee802154::mac::{FrameContent, Header};

pub(crate) const FRAME_SIZE: usize = 129;
pub(crate) const FRAME_VERSION_1: u8 = 0x10; // IEEE 802.15.4 - 2006 & 2011
pub(crate) const FRAME_VERSION_2: u8 = 0x20; // IEEE 802.15.4 - 2015

const FRAME_AR_OFFSET: usize = 1;
const FRAME_AR_BIT: u8 = 0x20;
const FRAME_VERSION_OFFSET: usize = 2;
const FRAME_VERSION_MASK: u8 = 0x30;

/// IEEE 802.15.4 MAC frame
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Header
    pub header: Header,
    /// Content
    pub content: FrameContent,
    /// Payload
    pub payload: Vec<u8, FRAME_SIZE>,
    /// This is a 2-byte CRC checksum
    pub footer: [u8; 2],
}

/// IEEE 802.15.4 MAC frame which has been received
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedFrame {
    /// Frame
    pub frame: Frame,
    /// Receiver channel
    pub channel: u8,
    /// Received Signal Strength Indicator (RSSI)
    pub rssi: i8,
    /// Link Quality Indication (LQI)
    pub lqi: u8,
}

pub(crate) fn frame_is_ack_required(frame: &[u8]) -> bool {
    if frame.len() <= FRAME_AR_OFFSET {
        return false;
    }
    (frame[FRAME_AR_OFFSET] & FRAME_AR_BIT) != 0
}

pub(crate) fn frame_get_version(frame: &[u8]) -> u8 {
    if frame.len() <= FRAME_VERSION_OFFSET {
        return 0;
    }
    frame[FRAME_VERSION_OFFSET] & FRAME_VERSION_MASK
}
