use bitfielder::bitfield;

use super::{Crc, FunctionNumber, RawFlag, RwFlag};
use crate::sdio::{Direction, Error};

bitfield! {
    /// Represents the SDIO CMD52 `IO_RW_DIRECT` command.
    ///
    /// Can be used to access register info, and transfer data.
    pub Cmd52(MSB0 [u8; 6]): u32 {
        raw_s: 47;
        raw_d: 46;
        raw_cmd_index: 45, 40;
        raw_rw_flag: 39;
        raw_fn_number: 38, 36;
        raw_raw_flag: 35;
        raw_reg_addr: 33, 17;
        raw_data: 15, 8;
        raw_crc: 7, 1;
        raw_e: 0;
    }
}

impl Cmd52 {
    /// Represents the [Cmd52] command byte length.
    pub const LEN: usize = 6;
    /// Represents the [Cmd52] command default byte value.
    pub const DEFAULT_ARG: [u8; 4] = [0x0; 4];
    /// Represents the [Cmd52] command index.
    pub const COMMAND_INDEX: u8 = 53;

    /// Creates a new [Cmd52].
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

    /// Gets the function number.
    pub const fn function_number(&self) -> Result<FunctionNumber, Error> {
        FunctionNumber::try_from_u8(self.raw_fn_number() as u8)
    }

    /// Sets the function number.
    pub fn set_function_number(&mut self, number: FunctionNumber) {
        self.set_raw_fn_number(number.into_u8().into());
    }

    /// Gets the read-write flag.
    pub const fn read_write_flag(&self) -> RwFlag {
        RwFlag::from_bool(self.raw_rw_flag())
    }

    /// Sets the read-write flag.
    pub fn set_read_write_flag(&mut self, flag: RwFlag) {
        self.set_raw_rw_flag(flag.into_bool());
    }

    /// Gets the read-write flag.
    pub const fn read_after_write_flag(&self) -> RawFlag {
        RawFlag::from_bool(self.raw_raw_flag())
    }

    /// Sets the read-write flag.
    pub fn set_read_after_write_flag(&mut self, flag: RawFlag) {
        self.set_raw_raw_flag(flag.into_bool());
    }

    /// Gets the register address.
    pub const fn register_address(&self) -> u16 {
        self.raw_reg_addr() as u16
    }

    /// Sets the register address.
    pub fn set_register_address(&mut self, addr: u16) {
        self.set_raw_reg_addr(addr.into());
    }

    /// Gets the data.
    pub const fn data(&self) -> u8 {
        self.raw_data() as u8
    }

    /// Sets the data.
    pub fn set_data(&mut self, data: u8) {
        self.set_raw_data(data.into());
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

    /// Attempts to convert a byte slice into a [Cmd52].
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self([val[0], val[1], val[2], val[3], val[4], val[5]]) {
                cmd if cmd.command_index() != Self::COMMAND_INDEX => Err(Error::General),
                cmd if cmd.raw_s() => Err(Error::General),
                cmd if !cmd.raw_e() => Err(Error::General),
                cmd if cmd.function_number().is_err() => Err(Error::General),
                cmd if cmd.verify_crc().is_err() => Err(Error::Crc),
                cmd => Ok(cmd),
            },
        }
    }

    /// Converts the [Cmd52] into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
    }
}

impl Default for Cmd52 {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<&[u8]> for Cmd52 {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<Cmd52> for [u8; Cmd52::LEN] {
    fn from(val: Cmd52) -> Self {
        val.into_bytes()
    }
}
