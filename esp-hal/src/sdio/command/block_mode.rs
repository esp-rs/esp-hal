/// Represents the block mode setting in a CMD53 command.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockMode {
    /// Indicates a default bytes-mode transfer.
    Bytes = 0,
    /// Indicates a block-mode transfer.
    Block = 1,
}

impl BlockMode {
    /// Creates a new [BlockMode].
    pub const fn new() -> Self {
        Self::Bytes
    }

    /// Converts a bool into a [BlockMode].
    #[inline]
    pub const fn from_bool(val: bool) -> Self {
        match val {
            false => Self::Bytes,
            true => Self::Block,
        }
    }

    /// Converts a [BlockMode] into a bool.
    #[inline]
    pub const fn into_bool(self) -> bool {
        match self {
            Self::Bytes => false,
            Self::Block => true,
        }
    }

    /// Converts a [`u8`] into a [BlockMode].
    #[inline]
    pub const fn from_u8(val: u8) -> Self {
        Self::from_bool(val != 0)
    }

    /// Converts a [BlockMode] into a [`u8`].
    #[inline]
    pub const fn into_u8(self) -> u8 {
        self.into_bool() as u8
    }
}

impl Default for BlockMode {
    fn default() -> Self {
        Self::new()
    }
}

impl From<bool> for BlockMode {
    fn from(val: bool) -> Self {
        Self::from_bool(val)
    }
}

impl From<u8> for BlockMode {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<BlockMode> for bool {
    fn from(val: BlockMode) -> Self {
        val.into_bool()
    }
}

impl From<BlockMode> for u8 {
    fn from(val: BlockMode) -> Self {
        val.into_u8()
    }
}
