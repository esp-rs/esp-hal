//! Response types for SPI mode.

use bitfielder::bitfield;

use crate::sdio::Error;

bitfield! {
    /// Represents the SPI mode R5 (`IO_RW_DIRECT` + `IO_RW_EXTENDED`) response.
    pub R5(u16): u8,
    mask: 0x5d_ff,
    default: 0,
    {
        /// Represents the start bit (always 0).
        raw_s: 15;
        /// Indicates a parameter error.
        pub param_error: 14;
        /// Indicates a function number error.
        pub fn_number_error: 12;
        /// Indicates a CRC error.
        pub crc_error: 11;
        /// Indicates an illegal command.
        pub illegal_command: 10;
        /// Indicates idle status.
        pub idle: 8;
        /// Represents the R/W response data.
        pub data: 7, 0;
    }
}

impl R5 {
    /// Represents the byte length of the [R5] response.
    pub const LEN: usize = 2;

    /// Attempts to convert a byte slice into a [R5] response.
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self(u16::from_be_bytes([val[0], val[1]])) {
                r5 if r5.raw_s() => Err(Error::General),
                r5 => Ok(r5),
            },
        }
    }

    /// Converts the [R5] response into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0.to_be_bytes()
    }
}

impl TryFrom<&[u8]> for R5 {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<R5> for [u8; R5::LEN] {
    fn from(val: R5) -> Self {
        val.into_bytes()
    }
}
