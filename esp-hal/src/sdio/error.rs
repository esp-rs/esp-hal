//! Error and result types for SDIO peripherals.

/// Represents the error variants for SDIO peripherals.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// The function and/or type is unimplemented.
    Unimplemented,
}

impl Error {
    /// Creates a new [Error].
    pub const fn new() -> Self {
        Self::Unimplemented
    }

    /// Creates an unimplemented [Error].
    pub const fn unimplemented() -> Self {
        Self::Unimplemented
    }
}

impl Default for Error {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Unimplemented => write!(f, "unimplemented"),
        }
    }
}

impl core::error::Error for Error {}
