/// Represents the `OP Code` field of CMD53.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OpCode {
    /// Multi-byte read-write to a fixed address.
    FixedAddress        = 0,
    /// Multi-byte read-write to an incrementing address.
    IncrementingAddress = 1,
}

impl OpCode {
    /// Creates a new [OpCode].
    pub const fn new() -> Self {
        Self::FixedAddress
    }

    /// Converts a bool into a [OpCode].
    #[inline]
    pub const fn from_bool(val: bool) -> Self {
        match val {
            false => Self::FixedAddress,
            true => Self::IncrementingAddress,
        }
    }

    /// Converts an [OpCode] into a bool.
    #[inline]
    pub const fn into_bool(self) -> bool {
        matches!(self, Self::IncrementingAddress)
    }

    /// Converts a [`u8`] into a [OpCode].
    #[inline]
    pub const fn from_u8(val: u8) -> Self {
        Self::from_bool(val != 0)
    }

    /// Converts an [OpCode] into a [`u8`].
    #[inline]
    pub const fn into_u8(self) -> u8 {
        self.into_bool() as u8
    }
}

impl Default for OpCode {
    fn default() -> Self {
        Self::new()
    }
}

impl From<bool> for OpCode {
    fn from(val: bool) -> Self {
        Self::from_bool(val)
    }
}

impl From<OpCode> for bool {
    fn from(val: OpCode) -> Self {
        val.into_bool()
    }
}

impl From<u8> for OpCode {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<OpCode> for u8 {
    fn from(val: OpCode) -> Self {
        val.into_u8()
    }
}
