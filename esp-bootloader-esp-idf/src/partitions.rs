//! # Partition Table Support
//!
//! ## Overview
//!
//! This module allows reading the partition table and conveniently
//! writing/reading partition contents.
//!
//! For more information see <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#built-in-partition-tables>

use embedded_storage::Region;

/// Maximum length of a partition table.
pub const PARTITION_TABLE_MAX_LEN: usize = 0xC00;

const PARTITION_TABLE_OFFSET: u32 =
    esp_config::esp_config_int!(u32, "ESP_BOOTLOADER_ESP_IDF_CONFIG_PARTITION_TABLE_OFFSET");

const RAW_ENTRY_LEN: usize = 32;
const ENTRY_MAGIC: u16 = 0x50aa;
const MD5_MAGIC: u16 = 0xebeb;

/// Represents a single partition entry.
#[repr(C)]
pub struct PartitionEntry<'a> {
    binary: &'a [u8],
}

impl<'a> PartitionEntry<'a> {
    fn new(binary: &'a [u8]) -> Self {
        Self { binary }
    }

    /// The magic value of the entry.
    pub fn magic(&self) -> u16 {
        u16::from_le_bytes(self.binary[..2].try_into().unwrap())
    }

    /// The partition type in raw representation.
    pub fn raw_type(&self) -> u8 {
        self.binary[2]
    }

    /// The partition sub-type in raw representation.
    pub fn raw_subtype(&self) -> u8 {
        self.binary[3]
    }

    /// Offset of the partition on flash.
    pub fn offset(&self) -> u32 {
        u32::from_le_bytes(self.binary[4..][..4].try_into().unwrap())
    }

    /// Length of the partition in bytes.
    pub fn len(&self) -> u32 {
        u32::from_le_bytes(self.binary[8..][..4].try_into().unwrap())
    }

    /// Checks for a zero-length partition.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The label of the partition.
    pub fn label(&self) -> &'a [u8] {
        &self.binary[12..][..16]
    }

    /// The label of the partition as `&str`.
    pub fn label_as_str(&self) -> &'a str {
        let array = self.label();
        let len = array
            .iter()
            .position(|b| *b == 0 || *b == 0xff)
            .unwrap_or(array.len());
        unsafe {
            core::str::from_utf8_unchecked(core::slice::from_raw_parts(array.as_ptr().cast(), len))
        }
    }

    /// Raw flags of this partition. You probably want to use
    /// [Self::is_read_only] and [Self::is_encrypted] instead.
    pub fn flags(&self) -> u32 {
        u32::from_le_bytes(self.binary[28..][..4].try_into().unwrap())
    }

    /// If the partition is read only.
    pub fn is_read_only(&self) -> bool {
        self.flags() & 0b01 != 0
    }

    /// If the partition is encrypted.
    pub fn is_encrypted(&self) -> bool {
        self.flags() & 0b10 != 0
    }

    /// The partition type (type and sub-type).
    pub fn partition_type(&self) -> PartitionType {
        match self.raw_type() {
            0 => PartitionType::App(self.raw_subtype().try_into().unwrap()),
            1 => PartitionType::Data(self.raw_subtype().try_into().unwrap()),
            2 => PartitionType::Bootloader(self.raw_subtype().try_into().unwrap()),
            3 => PartitionType::PartitionTable(self.raw_subtype().try_into().unwrap()),
            _ => unreachable!(),
        }
    }

    /// Provides a "view" into the partition allowing to read/write the
    /// partition contents by using the given [embedded_storage::Storage] and/or
    /// [embedded_storage::ReadStorage] implementation.
    pub fn as_embedded_storage<F>(&'a self, flash: &'a mut F) -> FlashRegion<'a, F>
    where
        F: embedded_storage::ReadStorage,
    {
        FlashRegion { raw: self, flash }
    }
}

impl core::fmt::Debug for PartitionEntry<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PartitionEntry")
            .field("magic", &self.magic())
            .field("raw_type", &self.raw_type())
            .field("raw_subtype", &self.raw_subtype())
            .field("offset", &self.offset())
            .field("len", &self.len())
            .field("label", &self.label_as_str())
            .field("flags", &self.flags())
            .field("is_read_only", &self.is_read_only())
            .field("is_encrypted", &self.is_encrypted())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PartitionEntry<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "PartitionEntry (\
            magic = {}, \
            raw_type = {}, \
            raw_subtype = {}, \
            offset = {}, \
            len = {}, \
            label = {}, \
            flags = {}, \
            is_read_only = {}, \
            is_encrypted = {}\
            )",
            self.magic(),
            self.raw_type(),
            self.raw_subtype(),
            self.offset(),
            self.len(),
            self.label_as_str(),
            self.flags(),
            self.is_read_only(),
            self.is_encrypted()
        )
    }
}

/// Errors which can be returned.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::Display)]
pub enum Error {
    /// The partition table is invalid.
    Invalid,
    /// An operation tries to access data that is out of bounds.
    OutOfBounds,
    /// An error which originates from the embedded-storage implementation.
    StorageError,
    /// The partition is write protected.
    WriteProtected,
}

impl core::error::Error for Error {}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self)
    }
}

/// A partition table.
pub struct PartitionTable<'a> {
    binary: &'a [u8],
    entries: usize,
}

impl<'a> PartitionTable<'a> {
    fn new(binary: &'a [u8]) -> Result<Self, Error> {
        if binary.len() > PARTITION_TABLE_MAX_LEN {
            return Err(Error::Invalid);
        }

        if binary.len() % RAW_ENTRY_LEN != 0 {
            return Err(Error::Invalid);
        }

        let mut raw = Self {
            binary,
            entries: binary.len() / RAW_ENTRY_LEN,
        };

        #[cfg(feature = "validation")]
        {
            let hash = {
                let mut i = 0;
                loop {
                    let entry = raw.get_partition(i).unwrap();
                    if entry.magic() == MD5_MAGIC {
                        break (&entry.binary[16..][..16], i);
                    }

                    i += 1;
                    if i >= raw.entries {
                        return Err(Error::Invalid);
                    }
                }
            };

            use md5::Digest;
            let mut hasher = md5::Md5::new();
            hasher.update(&raw.binary[..hash.1 * RAW_ENTRY_LEN]);
            let calculated_hash = hasher.finalize();

            if *calculated_hash != *hash.0 {
                return Err(Error::Invalid);
            }
        }

        let entries = {
            let mut i = 0;
            loop {
                if raw.get_partition(i).unwrap().magic() != ENTRY_MAGIC {
                    break;
                }

                i += 1;

                if i == raw.entries {
                    break;
                }

                if i > raw.entries {
                    return Err(Error::Invalid);
                }
            }
            i
        };

        raw.entries = entries;

        Ok(raw)
    }

    /// Number of partitions contained in the partition table.
    pub fn len(&self) -> usize {
        self.entries
    }

    /// Checks if there are no recognized partitions.
    pub fn is_empty(&self) -> bool {
        self.entries == 0
    }

    /// Get a partition entry.
    pub fn get_partition(&self, index: usize) -> Result<PartitionEntry<'a>, Error> {
        if index >= self.entries {
            return Err(Error::OutOfBounds);
        }
        Ok(PartitionEntry::new(
            &self.binary[(index * RAW_ENTRY_LEN)..][..RAW_ENTRY_LEN],
        ))
    }

    /// Get the first partition matching the given partition type.
    pub fn find_partition(&self, pt: PartitionType) -> Result<Option<PartitionEntry<'a>>, Error> {
        for i in 0..self.entries {
            let entry = self.get_partition(i)?;
            if entry.partition_type() == pt {
                return Ok(Some(entry));
            }
        }
        Ok(None)
    }
}

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
    Ota0    = 0x10,
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

/// Read the partition table.
///
/// Pass an implementation of [embedded_storage::Storage] which can read from
/// the whole flash and provide storage to read the partition table into.
pub fn read_partition_table<'a>(
    flash: &mut impl embedded_storage::Storage,
    storage: &'a mut [u8],
) -> Result<PartitionTable<'a>, Error> {
    flash
        .read(PARTITION_TABLE_OFFSET, storage)
        .map_err(|_e| Error::StorageError)?;

    PartitionTable::new(storage)
}

/// A flash region is a "view" into the partition.
///
/// It allows to read and write to the partition without the need to account for
/// the partition offset.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashRegion<'a, F> {
    raw: &'a PartitionEntry<'a>,
    flash: &'a mut F,
}

impl<F> embedded_storage::Region for FlashRegion<'_, F>
where
    F: embedded_storage::ReadStorage,
{
    fn contains(&self, address: u32) -> bool {
        address >= self.raw.offset() && address < self.raw.offset() + self.raw.len()
    }
}

impl<F> embedded_storage::ReadStorage for FlashRegion<'_, F>
where
    F: embedded_storage::ReadStorage,
{
    type Error = Error;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let address = offset + self.raw.offset();

        if !self.contains(address) {
            return Err(Error::OutOfBounds);
        }

        if !self.contains(address + bytes.len() as u32) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .read(address, bytes)
            .map_err(|_e| Error::StorageError)
    }

    fn capacity(&self) -> usize {
        self.raw.len() as _
    }
}

impl<F> embedded_storage::Storage for FlashRegion<'_, F>
where
    F: embedded_storage::Storage,
{
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let address = offset + self.raw.offset();

        if self.raw.is_read_only() {
            return Err(Error::WriteProtected);
        }

        if !self.contains(address) {
            return Err(Error::OutOfBounds);
        }

        if !self.contains(address + bytes.len() as u32) {
            return Err(Error::OutOfBounds);
        }

        self.flash
            .write(address, bytes)
            .map_err(|_e| Error::StorageError)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static SIMPLE: &[u8] = include_bytes!("../testdata/single_factory_no_ota.bin");
    static OTA: &[u8] = include_bytes!("../testdata/factory_app_two_ota.bin");

    #[test]
    fn read_simple() {
        let pt = PartitionTable::new(SIMPLE).unwrap();

        assert_eq!(3, pt.len());

        assert_eq!(1, pt.get_partition(0).unwrap().raw_type());
        assert_eq!(1, pt.get_partition(1).unwrap().raw_type());
        assert_eq!(0, pt.get_partition(2).unwrap().raw_type());

        assert_eq!(2, pt.get_partition(0).unwrap().raw_subtype());
        assert_eq!(1, pt.get_partition(1).unwrap().raw_subtype());
        assert_eq!(0, pt.get_partition(2).unwrap().raw_subtype());

        assert_eq!(
            PartitionType::Data(DataPartitionSubType::Nvs),
            pt.get_partition(0).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::Data(DataPartitionSubType::Phy),
            pt.get_partition(1).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::App(AppPartitionSubType::Factory),
            pt.get_partition(2).unwrap().partition_type()
        );

        assert_eq!(0x9000, pt.get_partition(0).unwrap().offset());
        assert_eq!(0xf000, pt.get_partition(1).unwrap().offset());
        assert_eq!(0x10000, pt.get_partition(2).unwrap().offset());

        assert_eq!(0x6000, pt.get_partition(0).unwrap().len());
        assert_eq!(0x1000, pt.get_partition(1).unwrap().len());
        assert_eq!(0x100000, pt.get_partition(2).unwrap().len());

        assert_eq!("nvs", pt.get_partition(0).unwrap().label_as_str());
        assert_eq!("phy_init", pt.get_partition(1).unwrap().label_as_str());
        assert_eq!("factory", pt.get_partition(2).unwrap().label_as_str());

        assert_eq!(false, pt.get_partition(0).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(1).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(2).unwrap().is_read_only());

        assert_eq!(false, pt.get_partition(0).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(1).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(2).unwrap().is_encrypted());
    }

    #[test]
    fn read_ota() {
        let pt = PartitionTable::new(OTA).unwrap();

        assert_eq!(6, pt.len());

        assert_eq!(1, pt.get_partition(0).unwrap().raw_type());
        assert_eq!(1, pt.get_partition(1).unwrap().raw_type());
        assert_eq!(1, pt.get_partition(2).unwrap().raw_type());
        assert_eq!(0, pt.get_partition(3).unwrap().raw_type());
        assert_eq!(0, pt.get_partition(4).unwrap().raw_type());
        assert_eq!(0, pt.get_partition(5).unwrap().raw_type());

        assert_eq!(2, pt.get_partition(0).unwrap().raw_subtype());
        assert_eq!(0, pt.get_partition(1).unwrap().raw_subtype());
        assert_eq!(1, pt.get_partition(2).unwrap().raw_subtype());
        assert_eq!(0, pt.get_partition(3).unwrap().raw_subtype());
        assert_eq!(0x10, pt.get_partition(4).unwrap().raw_subtype());
        assert_eq!(0x11, pt.get_partition(5).unwrap().raw_subtype());

        assert_eq!(
            PartitionType::Data(DataPartitionSubType::Nvs),
            pt.get_partition(0).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::Data(DataPartitionSubType::Ota),
            pt.get_partition(1).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::Data(DataPartitionSubType::Phy),
            pt.get_partition(2).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::App(AppPartitionSubType::Factory),
            pt.get_partition(3).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::App(AppPartitionSubType::Ota0),
            pt.get_partition(4).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::App(AppPartitionSubType::Ota1),
            pt.get_partition(5).unwrap().partition_type()
        );

        assert_eq!(0x9000, pt.get_partition(0).unwrap().offset());
        assert_eq!(0xd000, pt.get_partition(1).unwrap().offset());
        assert_eq!(0xf000, pt.get_partition(2).unwrap().offset());
        assert_eq!(0x10000, pt.get_partition(3).unwrap().offset());
        assert_eq!(0x110000, pt.get_partition(4).unwrap().offset());
        assert_eq!(0x210000, pt.get_partition(5).unwrap().offset());

        assert_eq!(0x4000, pt.get_partition(0).unwrap().len());
        assert_eq!(0x2000, pt.get_partition(1).unwrap().len());
        assert_eq!(0x1000, pt.get_partition(2).unwrap().len());
        assert_eq!(0x100000, pt.get_partition(3).unwrap().len());
        assert_eq!(0x100000, pt.get_partition(4).unwrap().len());
        assert_eq!(0x100000, pt.get_partition(5).unwrap().len());

        assert_eq!("nvs", pt.get_partition(0).unwrap().label_as_str());
        assert_eq!("otadata", pt.get_partition(1).unwrap().label_as_str());
        assert_eq!("phy_init", pt.get_partition(2).unwrap().label_as_str());
        assert_eq!("factory", pt.get_partition(3).unwrap().label_as_str());
        assert_eq!("ota_0", pt.get_partition(4).unwrap().label_as_str());
        assert_eq!("ota_1", pt.get_partition(5).unwrap().label_as_str());

        assert_eq!(false, pt.get_partition(0).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(1).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(2).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(3).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(4).unwrap().is_read_only());
        assert_eq!(false, pt.get_partition(5).unwrap().is_read_only());

        assert_eq!(false, pt.get_partition(0).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(1).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(2).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(3).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(4).unwrap().is_encrypted());
        assert_eq!(false, pt.get_partition(5).unwrap().is_encrypted());
    }
}
