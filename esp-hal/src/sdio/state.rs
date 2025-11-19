use super::Error;

/// Represents valid states for the SDIO peripheral.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum State {
    /// Represents the initial state.
    ///
    /// The peripheral is preparing for operation, e.g. configuring bus width,
    /// mode, speed etc.
    Init,
    /// Represents the standby state.
    ///
    /// The peripheral is in standby waiting for a command.
    Standby,
    /// Represents the command state.
    ///
    /// The peripheral is processing a command from the host.
    Command,
    /// Represents the transfer state.
    ///
    /// The peripheral is sending/receiving a data transfer.
    Transfer,
}

impl State {
    /// Creates a new [State].
    pub const fn new() -> Self {
        Self::Init
    }

    /// Checks if the [State] transition is valid.
    pub const fn valid_transition(self, state: Self) -> Result<(), Error> {
        match (self, state) {
            (Self::Init, Self::Standby) => Ok(()),
            (Self::Standby, Self::Command) => Ok(()),
            (Self::Command, Self::Init | Self::Standby | Self::Transfer) => Ok(()),
            (Self::Transfer, Self::Init | Self::Command) => Ok(()),
            _ => Err(Error::invalid_transition(self, state)),
        }
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Display for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Init => write!(f, "init"),
            Self::Standby => write!(f, "standby"),
            Self::Command => write!(f, "command"),
            Self::Transfer => write!(f, "transfer"),
        }
    }
}
