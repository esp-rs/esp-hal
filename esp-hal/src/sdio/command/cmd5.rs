use bitfielder::bitfield;

use super::Crc;
use crate::sdio::{Direction, Error, IoOcr};

bitfield! {
    /// Represents the SDIO CMD5 `IO_SEND_OP_COND` command.
    ///
    /// Can be used to retrieve the contents of the I/O OCR register.
    pub Cmd5(MSB0 [u8; 6]): u32 {
        raw_s: 47;
        raw_d: 46;
        raw_cmd_index: 45, 40;
        raw_io_ocr: 31, 8;
        raw_crc: 7, 1;
        raw_e: 0;
    }
}

impl Cmd5 {
    /// Represents the [Cmd5] command byte length.
    pub const LEN: usize = 6;
    /// Represents the [Cmd5] command default byte value.
    pub const DEFAULT_ARG: [u8; 4] = [0x0; 4];
    /// Represents the [Cmd5] command index.
    pub const COMMAND_INDEX: u8 = 5;

    /// Creates a new [Cmd5].
    pub const fn new() -> Self {
        Self(Self::default_bytes())
    }

    #[inline]
    const fn start() -> u8 {
        0x40 | Self::COMMAND_INDEX
    }

    #[inline]
    const fn default_bytes() -> [u8; Self::LEN] {
        let [a0, a1, a2, a3] = Self::DEFAULT_ARG;
        let raw = [Self::start(), a0, a1, a2, a3];
        let crc = Crc::calculate(&raw);
        let [r0, r1, r2, r3, r4] = raw;

        [r0, r1, r2, r3, r4, (crc.into_u8() << 1) | 1]
    }

    /// Gets the direction.
    pub const fn direction(&self) -> Direction {
        Direction::from_bool(self.raw_d())
    }

    /// Gets the command index.
    pub const fn command_index(&self) -> u8 {
        self.raw_cmd_index() as u8
    }

    /// Gets the I/O OCR argument.
    pub const fn io_ocr(&self) -> IoOcr {
        IoOcr::from_u32(self.raw_io_ocr())
    }

    /// Sets the I/O OCR argument.
    pub fn set_io_ocr(&mut self, io_ocr: IoOcr) {
        self.set_raw_io_ocr(io_ocr.into_u32());
    }

    /// Gets the CRC.
    pub const fn crc(&self) -> Crc {
        Crc::from_u8(self.raw_crc() as u8)
    }

    /// Calculates and sets the CRC.
    pub fn set_crc(&mut self) {
        self.set_raw_crc(self.calculate_crc().into_u8().into());
    }

    /// Calculates the CRC.
    pub const fn calculate_crc(&self) -> Crc {
        let [b0, b1, b2, b3, b4, _] = self.0;
        Crc::calculate(&[b0, b1, b2, b3, b4])
    }

    /// Verifies if the CRC matches the calculated value.
    pub const fn verify_crc(&self) -> Result<(), Error> {
        match self.calculate_crc().into_u8() == self.crc().into_u8() {
            false => Err(Error::Crc),
            true => Ok(()),
        }
    }

    /// Attempts to convert a byte slice into a [Cmd5].
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self([val[0], val[1], val[2], val[3], val[4], val[5]]) {
                cmd if cmd.command_index() != Self::COMMAND_INDEX => Err(Error::General),
                cmd if cmd.raw_s() => Err(Error::General),
                cmd if !cmd.raw_e() => Err(Error::General),
                cmd if cmd.verify_crc().is_err() => Err(Error::Crc),
                cmd => Ok(cmd),
            },
        }
    }

    /// Converts the [Cmd5] into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
    }
}

impl Default for Cmd5 {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<&[u8]> for Cmd5 {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<Cmd5> for [u8; Cmd5::LEN] {
    fn from(val: Cmd5) -> Self {
        val.into_bytes()
    }
}
