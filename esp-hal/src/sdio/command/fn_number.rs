use crate::sdio::Error;

/// Represents the SDIO command function number.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FunctionNumber {
    /// Indicates access to:
    /// - CCCR (Card Common Control Registers)
    /// - FBR (Function Basic Registers)
    /// - CIS (Card Information Structure)
    Registers = 0,
    /// Indicates use for I/O transfers.
    ///
    /// Can be used in parallel with the other I/O function.
    Io1       = 1,
    /// Indicates use for I/O transfers.
    ///
    /// Can be used in parallel with the other I/O function.
    Io2       = 2,
}

impl FunctionNumber {
    /// Byte value for registers access.
    pub const REGISTERS: u8 = 0;
    /// Byte value for I/O 1 access.
    pub const IO1: u8 = 1;
    /// Byte value for I/O 2 access.
    pub const IO2: u8 = 2;

    /// Creates a new [FunctionNumber].
    pub const fn new() -> Self {
        Self::Registers
    }

    /// Attempts to convert a [`u8`] into a [FunctionNumber].
    pub const fn try_from_u8(val: u8) -> Result<Self, Error> {
        match val {
            Self::REGISTERS => Ok(Self::Registers),
            Self::IO1 => Ok(Self::Io1),
            Self::IO2 => Ok(Self::Io2),
            _ => Err(Error::General),
        }
    }

    /// Converts a [FunctionNumber] into a [`u8`].
    pub const fn into_u8(self) -> u8 {
        match self {
            Self::Registers => Self::REGISTERS,
            Self::Io1 => Self::IO1,
            Self::Io2 => Self::IO2,
        }
    }
}

impl Default for FunctionNumber {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<u8> for FunctionNumber {
    type Error = Error;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        Self::try_from_u8(val)
    }
}

impl From<FunctionNumber> for u8 {
    fn from(val: FunctionNumber) -> Self {
        val.into_u8()
    }
}
