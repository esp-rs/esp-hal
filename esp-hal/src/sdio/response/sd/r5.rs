use bitfielder::bitfield;

use crate::sdio::{
    Direction,
    Error,
    command::{CommandIndex, Crc},
    response::Flags,
};

bitfield! {
    /// Represents the SD modes R5 (`IO_RW_DIRECT` + `IO_RW_EXTENDED`) response.
    pub R5(MSB0 [u8; 6]): u8 {
        raw_s: 47;
        raw_direction: 46;
        raw_command_index: 45, 40;
        /// Indicates an illegal command.
        pub illegal_command: 10;
        raw_flags: 23, 16;
        /// Represents the R/W response data.
        pub data: 15, 8;
        raw_crc: 7, 1;
        raw_e: 0;
    }
}

impl R5 {
    /// Represents the byte length of the [R5] response.
    pub const LEN: usize = 6;

    /// Gets the direction.
    pub const fn direction(&self) -> Direction {
        Direction::from_bool(self.raw_direction())
    }

    /// Gets the command index.
    pub const fn command_index(&self) -> Result<CommandIndex, Error> {
        CommandIndex::try_from_inner(self.raw_command_index())
    }

    /// Sets the command index.
    pub fn set_command_index(&mut self, val: CommandIndex) {
        self.set_raw_command_index(val.into_inner());
    }

    /// Gets the response flags.
    pub const fn flags(&self) -> Result<Flags, Error> {
        Flags::try_from_inner(self.raw_flags())
    }

    /// Sets the response flags.
    pub fn set_flags(&mut self, val: Flags) {
        self.set_raw_flags(val.into_inner());
    }

    /// Gets the CRC.
    pub const fn crc(&self) -> Crc {
        Crc::from_u8(self.raw_crc())
    }

    /// Calculates and sets the CRC.
    pub fn set_crc(&mut self) {
        self.set_raw_crc(self.calculate_crc().into_u8());
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

    /// Attempts to convert a byte slice into a [R5].
    pub const fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self([val[0], val[1], val[2], val[3], val[4], val[5]]) {
                cmd if cmd.command_index().is_err() => Err(Error::IllegalCommand),
                cmd if cmd.raw_s() => Err(Error::General),
                cmd if !cmd.raw_e() => Err(Error::General),
                cmd if cmd.flags().is_err() => Err(Error::General),
                cmd if cmd.command_index().is_err() && !cmd.illegal_command() => {
                    Err(Error::IllegalCommand)
                }
                cmd if cmd.verify_crc().is_err() => Err(Error::Crc),
                cmd => Ok(cmd),
            },
        }
    }

    /// Converts the [R5] into a byte array.
    pub const fn into_bytes(self) -> [u8; Self::LEN] {
        self.0
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
