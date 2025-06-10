use super::Error;

/// Represents an interrupt to send to the host.
///
/// # Note
///
/// Values derived from [esp-idf](https://github.com/espressif/esp-idf/blob/v5.4.1/components/hal/include/hal/sdio_slave_types.h) SDIO driver.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HostInterrupt {
    /// General purpose host interrupt: 0.
    General0      = 1 << 0,
    /// General purpose host interrupt: 1.
    General1      = 1 << 1,
    /// General purpose host interrupt: 2.
    General2      = 1 << 2,
    /// General purpose host interrupt: 3.
    General3      = 1 << 3,
    /// General purpose host interrupt: 4.
    General4      = 1 << 4,
    /// General purpose host interrupt: 5.
    General5      = 1 << 5,
    /// General purpose host interrupt: 6.
    General6      = 1 << 6,
    /// General purpose host interrupt: 7.
    General7      = 1 << 7,
    /// New packet available to send to host.
    SendNewPacket = 1 << 23,
}

impl HostInterrupt {
    /// Bit value for general purpose host interrupt: 0.
    pub const GENERAL_0: u32 = 1 << 0;
    /// Bit value for general purpose host interrupt: 1.
    pub const GENERAL_1: u32 = 1 << 1;
    /// Bit value for general purpose host interrupt: 2.
    pub const GENERAL_2: u32 = 1 << 2;
    /// Bit value for general purpose host interrupt: 3.
    pub const GENERAL_3: u32 = 1 << 3;
    /// Bit value for general purpose host interrupt: 4.
    pub const GENERAL_4: u32 = 1 << 4;
    /// Bit value for general purpose host interrupt: 5.
    pub const GENERAL_5: u32 = 1 << 5;
    /// Bit value for general purpose host interrupt: 6.
    pub const GENERAL_6: u32 = 1 << 6;
    /// Bit value for general purpose host interrupt: 7.
    pub const GENERAL_7: u32 = 1 << 7;
    /// Bit value for new packet available to send to host.
    pub const SEND_NEW_PACKET: u32 = 1 << 23;

    /// Creates a new [HostInterrupt].
    pub const fn new() -> Self {
        Self::General0
    }

    /// Converts the [HostInterrupt] into a [`u32`].
    #[inline]
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    /// Attempts to convert a [`u32`] into a [HostInterrupt].
    #[inline]
    pub const fn try_from_u32(val: u32) -> Result<Self, Error> {
        match val {
            Self::GENERAL_0 => Ok(Self::General0),
            Self::GENERAL_1 => Ok(Self::General1),
            Self::GENERAL_2 => Ok(Self::General2),
            Self::GENERAL_3 => Ok(Self::General3),
            Self::GENERAL_4 => Ok(Self::General4),
            Self::GENERAL_5 => Ok(Self::General5),
            Self::GENERAL_6 => Ok(Self::General6),
            Self::GENERAL_7 => Ok(Self::General7),
            Self::SEND_NEW_PACKET => Ok(Self::SendNewPacket),
            _ => Err(Error::General),
        }
    }
}

impl From<HostInterrupt> for u32 {
    fn from(val: HostInterrupt) -> Self {
        val.to_u32()
    }
}

impl TryFrom<u32> for HostInterrupt {
    type Error = Error;

    fn try_from(val: u32) -> Result<Self, Self::Error> {
        Self::try_from_u32(val)
    }
}

impl Default for HostInterrupt {
    fn default() -> Self {
        Self::new()
    }
}
