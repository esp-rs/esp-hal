use bitfielder::bitfield;

use crate::sdio::{Direction, Error, IoOcr};

bitfield! {
    /// Represents the SD modes R4 (`IO_SEND_OP_COND`) response.
    pub R4(MSB0 [u8; 6]): u32 {
        raw_s: 47;
        raw_direction: 46;
        pub ready: 39;
        pub number_of_functions: 38, 36;
        pub memory_present: 35;
        raw_io_ocr: 31, 8;
        raw_e: 0;
    }
}

impl R4 {
    /// Represents the byte length of the [R4] response.
    pub const LEN: usize = 6;

    /// Gets the direction.
    pub const fn direction(&self) -> Direction {
        Direction::from_bool(self.raw_direction())
    }

    /// Gets the I/O OCR register.
    pub const fn io_ocr(&self) -> IoOcr {
        IoOcr::from_u32(self.raw_io_ocr())
    }

    /// Sets the I/O OCR register.
    pub fn set_io_ocr(&mut self, val: IoOcr) {
        self.set_raw_io_ocr(val.into_u32());
    }

    /// Attempts to convert a byte slice into a [R4].
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self([val[0], val[1], val[2], val[3], val[4], val[5]]) {
                cmd if cmd.raw_s() => Err(Error::General),
                cmd if !cmd.raw_e() => Err(Error::General),
                cmd => Ok(cmd),
            },
        }
    }

    /// Converts the [R4] into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
    }
}

impl TryFrom<&[u8]> for R4 {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<R4> for [u8; R4::LEN] {
    fn from(val: R4) -> Self {
        val.into_bytes()
    }
}
