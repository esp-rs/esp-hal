use super::Error;

const OTA_SUBTYPE_OFFSET: u8 = 0x10;

/// A partition type including the sub-type.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PartitionType {
    /// Application.
    App(AppPartitionSubType),
    /// Data.
    Data(DataPartitionSubType),
    /// Bootloader.
    Bootloader(BootloaderPartitionSubType),
    /// Partition table.
    PartitionTable(PartitionTablePartitionSubType),
}

/// A partition type
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RawPartitionType {
    /// Application.
    App = 0,
    /// Data.
    Data,
    /// Bootloader.
    Bootloader,
    /// Partition table.
    PartitionTable,
}

/// Sub-types of an application partition.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AppPartitionSubType {
    /// Factory image
    Factory = 0,
    /// OTA slot 0
    Ota0    = OTA_SUBTYPE_OFFSET,
    /// OTA slot 1
    Ota1,
    /// OTA slot 2
    Ota2,
    /// OTA slot 3
    Ota3,
    /// OTA slot 4
    Ota4,
    /// OTA slot 5
    Ota5,
    /// OTA slot 6
    Ota6,
    /// OTA slot 7
    Ota7,
    /// OTA slot 8
    Ota8,
    /// OTA slot 9
    Ota9,
    /// OTA slot 10
    Ota10,
    /// OTA slot 11
    Ota11,
    /// OTA slot 12
    Ota12,
    /// OTA slot 13
    Ota13,
    /// OTA slot 14
    Ota14,
    /// OTA slot 15
    Ota15,
    /// Test image
    Test,
}

impl AppPartitionSubType {
    pub(crate) fn ota_app_number(&self) -> u8 {
        *self as u8 - OTA_SUBTYPE_OFFSET
    }

    pub(crate) fn from_ota_app_number(number: u8) -> Result<Self, Error> {
        if number > 16 {
            return Err(Error::InvalidArgument);
        }
        Self::try_from(number + OTA_SUBTYPE_OFFSET)
    }
}

impl TryFrom<u8> for AppPartitionSubType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        AppPartitionSubType::from_repr(value).ok_or(Error::Invalid)
    }
}

/// Sub-types of the data partition type.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DataPartitionSubType {
    /// Data partition which stores information about the currently selected OTA
    /// app slot. This partition should be 0x2000 bytes in size. Refer to
    /// the OTA documentation for more details.
    Ota      = 0,
    /// Phy is for storing PHY initialization data. This allows PHY to be
    /// configured per-device, instead of in firmware.
    Phy,
    /// Used for Non-Volatile Storage (NVS).
    Nvs,
    /// Used for storing core dumps while using a custom partition table
    Coredump,
    /// NvsKeys is used for the NVS key partition. (NVS).
    NvsKeys,
    /// Used for emulating eFuse bits using Virtual eFuses.
    EfuseEm,
    /// Implicitly used for data partitions with unspecified (empty) subtype,
    /// but it is possible to explicitly mark them as undefined as well.
    Undefined,
    /// FAT Filesystem Support.
    Fat      = 0x81,
    /// SPIFFS Filesystem.
    Spiffs   = 0x82,
    ///  LittleFS filesystem.
    LittleFs = 0x83,
}

impl TryFrom<u8> for DataPartitionSubType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        DataPartitionSubType::from_repr(value).ok_or(Error::Invalid)
    }
}

/// Sub-type of the bootloader partition type.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BootloaderPartitionSubType {
    /// It is the so-called 2nd stage bootloader.
    Primary = 0,
    /// It is a temporary bootloader partition used by the bootloader OTA update
    /// functionality for downloading a new image.
    Ota     = 1,
}

impl TryFrom<u8> for BootloaderPartitionSubType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        BootloaderPartitionSubType::from_repr(value).ok_or(Error::Invalid)
    }
}

/// Sub-type of the partition table type.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PartitionTablePartitionSubType {
    /// It is the primary partition table.
    Primary = 0,
    /// It is a temporary partition table partition used by the partition table
    /// OTA update functionality for downloading a new image.
    Ota     = 1,
}

impl TryFrom<u8> for PartitionTablePartitionSubType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        PartitionTablePartitionSubType::from_repr(value).ok_or(Error::Invalid)
    }
}
