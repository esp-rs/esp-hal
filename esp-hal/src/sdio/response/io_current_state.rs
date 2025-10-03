use crate::sdio::Error;

/// Represents the current card state.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IoCurrentState {
    /// Indicates the card is disabled.
    Disabled = 0,
    /// Indicates the card's DAT lines are free.
    Command  = 1,
    /// Indicates an active data transfer.
    Transfer = 2,
}

impl IoCurrentState {
    /// Represents the raw disabled value.
    pub const DISABLED: u8 = 0;
    /// Represents the raw command value.
    pub const COMMAND: u8 = 1;
    /// Represents the raw transfer value.
    pub const TRANSFER: u8 = 2;

    /// Creates a new [IoCurrentState].
    pub const fn new() -> Self {
        Self::Disabled
    }

    /// Attempts to convert an inner value into a [IoCurrentState].
    pub const fn try_from_inner(val: u8) -> Result<Self, Error> {
        match val {
            Self::DISABLED => Ok(Self::Disabled),
            Self::COMMAND => Ok(Self::Command),
            Self::TRANSFER => Ok(Self::Transfer),
            _ => Err(Error::General),
        }
    }

    /// Attempts to convert an inner value into a [IoCurrentState].
    pub const fn into_inner(self) -> u8 {
        self as u8
    }
}

impl Default for IoCurrentState {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<u8> for IoCurrentState {
    type Error = Error;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        Self::try_from_inner(val)
    }
}

impl From<IoCurrentState> for u8 {
    fn from(val: IoCurrentState) -> Self {
        val.into_inner()
    }
}
