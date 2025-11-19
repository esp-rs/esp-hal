use bitfielder::bitfield;

use crate::sdio::Error;

bitfield! {
    /// Represents the I/O OCR register values for supported voltage windows.
    pub IoOcr(MSB0 [u8; 3]): u8 {
        /// Represents support for the 3.5-3.6 voltage window.
        v35: 23;
        /// Represents support for the 3.4-3.5 voltage window.
        v34: 22;
        /// Represents support for the 3.3-3.4 voltage window.
        v33: 21;
        /// Represents support for the 3.2-3.3 voltage window.
        v32: 20;
        /// Represents support for the 3.1-3.2 voltage window.
        v31: 19;
        /// Represents support for the 3.0-3.1 voltage window.
        v30: 18;
        /// Represents support for the 2.9-3.0 voltage window.
        v29: 17;
        /// Represents support for the 2.8-2.9 voltage window.
        v28: 16;
        /// Represents support for the 2.7-2.8 voltage window.
        v27: 15;
        /// Represents support for the 2.6-2.7 voltage window.
        v26: 14;
        /// Represents support for the 2.5-2.6 voltage window.
        v25: 13;
        /// Represents support for the 2.4-2.5 voltage window.
        v24: 12;
        /// Represents support for the 2.3-2.4 voltage window.
        v23: 11;
        /// Represents support for the 2.2-2.3 voltage window.
        v22: 10;
        /// Represents support for the 2.1-2.2 voltage window.
        v21: 9;
        /// Represents support for the 2.0-2.1 voltage window.
        v20: 8;
    }
}

impl IoOcr {
    /// Represents the byte length of the I/O OCR register.
    pub const LEN: usize = 3;
    /// Represents the bitmask of the I/O OCR register.
    pub const MASK: u32 = 0x00ff_ff00;
    /// Represents the default byte value.
    pub const DEFAULT: [u8; Self::LEN] = [0u8; Self::LEN];

    /// Creates a new [IoOcr].
    pub const fn new() -> Self {
        Self(Self::DEFAULT)
    }

    /// Converts a [`u32`] into a [IoOcr].
    pub const fn from_u32(val: u32) -> Self {
        let [_, b0, b1, _] = (val & Self::MASK).to_be_bytes();
        Self([b0, b1, 0])
    }

    /// Converts a [IoOcr] into a [`u32`].
    pub const fn into_u32(self) -> u32 {
        let [b0, b1, b2] = self.0;
        u32::from_be_bytes([0, b0, b1, b2])
    }

    /// Attempts to convert a byte slice into an [IoOcr].
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => Ok(Self([val[0], val[1], val[2]])),
        }
    }

    /// Converts the [IoOcr] into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
    }
}

impl Default for IoOcr {
    fn default() -> Self {
        Self::new()
    }
}

impl From<u32> for IoOcr {
    fn from(val: u32) -> Self {
        Self::from_u32(val)
    }
}

impl From<IoOcr> for u32 {
    fn from(val: IoOcr) -> Self {
        val.into_u32()
    }
}

impl TryFrom<&[u8]> for IoOcr {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<IoOcr> for [u8; IoOcr::LEN] {
    fn from(val: IoOcr) -> Self {
        val.into_bytes()
    }
}
