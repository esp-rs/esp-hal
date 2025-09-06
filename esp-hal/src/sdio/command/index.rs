use crate::sdio::Error;

/// Represents the command indices supported by ESP32 SDIO controllers.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandIndex {
    /// Represents the I/O Read/Write Direct command (CMD52) index.
    IoRwDirect   = 52,
    /// Represents the I/O Read/Write Extended command (CMD53) index.
    IoRwExtended = 53,
}

impl CommandIndex {
    /// Represents the raw index value of the CMD52 index.
    pub const IO_RW_DIRECT: u8 = 52;
    /// Represents the raw index value of the CMD53 index.
    pub const IO_RW_EXTENDED: u8 = 53;

    /// Creates a new [CommandIndex].
    pub const fn new() -> Self {
        Self::IoRwDirect
    }

    /// Attempts to convert an inner value into a [CommandIndex].
    pub const fn try_from_inner(val: u8) -> Result<Self, Error> {
        match val {
            Self::IO_RW_DIRECT => Ok(Self::IoRwDirect),
            Self::IO_RW_EXTENDED => Ok(Self::IoRwExtended),
            _ => Err(Error::IllegalCommand),
        }
    }

    /// Converts the [CommandIndex] into an inner value.
    pub const fn into_inner(self) -> u8 {
        self as u8
    }
}

impl Default for CommandIndex {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<u8> for CommandIndex {
    type Error = Error;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        Self::try_from_inner(val)
    }
}

impl From<CommandIndex> for u8 {
    fn from(val: CommandIndex) -> Self {
        val.into_inner()
    }
}
