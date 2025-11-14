/// Represents the command-response direction.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    /// Represents a device-to-host transfer, usually a response.
    DeviceToHost = 0,
    /// Represents a host-to-device transfer, usually a command.
    HostToDevice = 1,
}

impl Direction {
    /// Creates a new [Direction].
    pub const fn new() -> Self {
        Self::DeviceToHost
    }

    /// Converts the [Direction] into a bool.
    pub const fn into_bool(self) -> bool {
        matches!(self, Self::HostToDevice)
    }

    /// Converts a bool into a [Direction].
    pub const fn from_bool(val: bool) -> Self {
        match val {
            false => Self::DeviceToHost,
            true => Self::HostToDevice,
        }
    }

    /// Converts the [Direction] into a [`u8`].
    pub const fn into_u8(self) -> u8 {
        self.into_bool() as u8
    }

    /// Converts a [`u8`] into a [Direction].
    pub const fn from_u8(val: u8) -> Self {
        Self::from_bool(val != 0)
    }
}

impl Default for Direction {
    fn default() -> Self {
        Self::new()
    }
}

impl From<bool> for Direction {
    fn from(val: bool) -> Self {
        Self::from_bool(val)
    }
}

impl From<Direction> for bool {
    fn from(val: Direction) -> Self {
        val.into_bool()
    }
}

impl From<u8> for Direction {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl From<Direction> for u8 {
    fn from(val: Direction) -> Self {
        val.into_u8()
    }
}
