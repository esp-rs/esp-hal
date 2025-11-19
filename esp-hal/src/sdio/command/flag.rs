/// Represents the read-write SDIO command flag.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RwFlag {
    /// Indicates a read-only command.
    ReadOnly  = 0,
    /// Indicates a read-write command.
    ReadWrite = 1,
}

impl RwFlag {
    /// Creates a new [RwFlag].
    pub const fn new() -> Self {
        Self::ReadOnly
    }

    /// Converts the [RwFlag] into a bool.
    pub const fn into_bool(self) -> bool {
        matches!(self, Self::ReadWrite)
    }

    /// Converts a bool into a [RwFlag].
    pub const fn from_bool(val: bool) -> Self {
        match val {
            false => Self::ReadOnly,
            true => Self::ReadWrite,
        }
    }

    /// Converts the [RwFlag] into a [`u8`].
    pub const fn into_u8(self) -> u8 {
        self.into_bool() as u8
    }

    /// Converts a [`u8`] into a [RwFlag].
    pub const fn from_u8(val: u8) -> Self {
        Self::from_bool(val != 0)
    }
}

impl Default for RwFlag {
    fn default() -> Self {
        Self::new()
    }
}

impl From<bool> for RwFlag {
    fn from(val: bool) -> Self {
        Self::from_bool(val)
    }
}

impl From<RwFlag> for bool {
    fn from(val: RwFlag) -> Self {
        val.into_bool()
    }
}

impl From<u8> for RwFlag {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<RwFlag> for u8 {
    fn from(val: RwFlag) -> Self {
        val.into_u8()
    }
}

/// Represents the read-after-write SDIO command flag.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RawFlag {
    /// Indicates the command will only write to the device register/address.
    WriteOnly      = 0,
    /// Indicates that the command expects device data from the same register/address in the
    /// response.
    ReadAfterWrite = 1,
}

impl RawFlag {
    /// Creates a new [RawFlag].
    pub const fn new() -> Self {
        Self::WriteOnly
    }

    /// Converts the [RawFlag] into a bool.
    pub const fn into_bool(self) -> bool {
        matches!(self, Self::ReadAfterWrite)
    }

    /// Converts a bool into a [RawFlag].
    pub const fn from_bool(val: bool) -> Self {
        match val {
            false => Self::WriteOnly,
            true => Self::ReadAfterWrite,
        }
    }

    /// Converts the [RawFlag] into a [`u8`].
    pub const fn into_u8(self) -> u8 {
        self.into_bool() as u8
    }

    /// Converts a [`u8`] into a [RawFlag].
    pub const fn from_u8(val: u8) -> Self {
        Self::from_bool(val != 0)
    }
}

impl Default for RawFlag {
    fn default() -> Self {
        Self::new()
    }
}

impl From<bool> for RawFlag {
    fn from(val: bool) -> Self {
        Self::from_bool(val)
    }
}

impl From<RawFlag> for bool {
    fn from(val: RawFlag) -> Self {
        val.into_bool()
    }
}

impl From<u8> for RawFlag {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<RawFlag> for u8 {
    fn from(val: RawFlag) -> Self {
        val.into_u8()
    }
}
