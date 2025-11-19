use enumset::EnumSetType;

/// Represents an interrupt to send to the host.
///
/// # Note
///
/// Values derived from [esp-idf](https://github.com/espressif/esp-idf/blob/v5.4.1/components/hal/include/hal/sdio_slave_types.h) SDIO driver.
#[derive(Debug, EnumSetType)]
pub enum HostInterrupt {
    /// General purpose host interrupt: 0.
    #[enumset(repr = "u8")]
    General0 = 1,
    /// General purpose host interrupt: 1.
    #[enumset(repr = "u8")]
    General1 = 2,
    /// General purpose host interrupt: 2.
    #[enumset(repr = "u8")]
    General2 = 4,
    /// General purpose host interrupt: 3.
    #[enumset(repr = "u8")]
    General3 = 8,
    /// General purpose host interrupt: 4.
    #[enumset(repr = "u8")]
    General4 = 16,
    /// General purpose host interrupt: 5.
    #[enumset(repr = "u8")]
    General5 = 32,
    /// General purpose host interrupt: 6.
    #[enumset(repr = "u8")]
    General6 = 64,
    /// General purpose host interrupt: 7.
    #[enumset(repr = "u8")]
    General7 = 128,
}

impl HostInterrupt {
    /// Creates a new [HostInterrupt].
    pub const fn new() -> Self {
        Self::General0
    }

    /// Gets the raw bit value of the [HostInterrupt].
    pub const fn bits(&self) -> u8 {
        *self as u8
    }
}

impl From<HostInterrupt> for u8 {
    fn from(val: HostInterrupt) -> Self {
        val.bits()
    }
}

impl TryFrom<u8> for HostInterrupt {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            1 => Ok(Self::General0),
            2 => Ok(Self::General1),
            4 => Ok(Self::General2),
            8 => Ok(Self::General3),
            16 => Ok(Self::General4),
            32 => Ok(Self::General5),
            64 => Ok(Self::General6),
            128 => Ok(Self::General7),
            _ => Err(val),
        }
    }
}

impl Default for HostInterrupt {
    fn default() -> Self {
        Self::new()
    }
}

/// Represents an interrupt to sent from the host.
///
/// # Note
///
/// Values derived from [esp-idf](https://github.com/espressif/esp-idf/blob/v5.4.1/components/hal/esp32/include/hal/sdio_slave_ll.h) SDIO driver.
#[derive(Debug, EnumSetType)]
pub enum DeviceInterrupt {
    /// General purpose host interrupt: 0.
    #[enumset(repr = "u8")]
    General0 = 1,
    /// General purpose host interrupt: 1.
    #[enumset(repr = "u8")]
    General1 = 2,
    /// General purpose host interrupt: 2.
    #[enumset(repr = "u8")]
    General2 = 4,
    /// General purpose host interrupt: 3.
    #[enumset(repr = "u8")]
    General3 = 8,
    /// General purpose host interrupt: 4.
    #[enumset(repr = "u8")]
    General4 = 16,
    /// General purpose host interrupt: 5.
    #[enumset(repr = "u8")]
    General5 = 32,
    /// General purpose host interrupt: 6.
    #[enumset(repr = "u8")]
    General6 = 64,
    /// General purpose host interrupt: 7.
    #[enumset(repr = "u8")]
    General7 = 128,
}

impl DeviceInterrupt {
    /// Creates a new [DeviceInterrupt].
    pub const fn new() -> Self {
        Self::General0
    }

    /// Gets the raw bit value of the [DeviceInterrupt].
    pub const fn bits(&self) -> u8 {
        *self as u8
    }
}

impl From<DeviceInterrupt> for u8 {
    fn from(val: DeviceInterrupt) -> Self {
        val.bits()
    }
}

impl TryFrom<u8> for DeviceInterrupt {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            1 => Ok(Self::General0),
            2 => Ok(Self::General1),
            4 => Ok(Self::General2),
            8 => Ok(Self::General3),
            16 => Ok(Self::General4),
            32 => Ok(Self::General5),
            64 => Ok(Self::General6),
            128 => Ok(Self::General7),
            _ => Err(val),
        }
    }
}

impl Default for DeviceInterrupt {
    fn default() -> Self {
        Self::new()
    }
}
