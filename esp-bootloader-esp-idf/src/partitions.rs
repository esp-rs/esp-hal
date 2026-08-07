//! # Partition Table Support
//!
//! ## Overview
//!
//! This module allows reading the partition table and conveniently
//! writing/reading partition contents.
//!
//! For more information see <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#built-in-partition-tables>

/// Maximum length of a partition table.
pub const PARTITION_TABLE_MAX_LEN: usize = 0xC00;

const PARTITION_TABLE_OFFSET: u32 =
    esp_config::esp_config_int!(u32, "ESP_BOOTLOADER_ESP_IDF_CONFIG_PARTITION_TABLE_OFFSET");

const RAW_ENTRY_LEN: usize = 32;
const ENTRY_MAGIC: u16 = 0x50aa;
#[cfg(feature = "validation")]
const MD5_MAGIC: u16 = 0xebeb;

const OTA_SUBTYPE_OFFSET: u8 = 0x10;

use crate::flash::FlashAccess;
pub use crate::flash::FlashStorage;

/// Represents a single partition entry.
#[derive(Clone, Copy)]
pub struct PartitionEntry {
    pub(crate) binary: [u8; RAW_ENTRY_LEN],
}

impl PartitionEntry {
    fn new(binary: &[u8; RAW_ENTRY_LEN]) -> Self {
        Self { binary: *binary }
    }

    /// The magic value of the entry.
    pub fn magic(&self) -> u16 {
        u16::from_le_bytes(unwrap!(self.binary[..2].try_into()))
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
        u32::from_le_bytes(unwrap!(self.binary[4..][..4].try_into()))
    }

    /// Length of the partition in bytes.
    pub fn len(&self) -> u32 {
        u32::from_le_bytes(unwrap!(self.binary[8..][..4].try_into()))
    }

    /// Checks for a zero-length partition.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The label of the partition.
    pub fn label(&self) -> &[u8] {
        &self.binary[12..][..16]
    }

    /// The label of the partition as `&str`.
    pub fn label_as_str(&self) -> &str {
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
        u32::from_le_bytes(unwrap!(self.binary[28..][..4].try_into()))
    }

    /// If the partition is read only.
    pub fn is_read_only(&self) -> bool {
        self.flags() & 0b01 != 0
    }

    /// If the partition is encrypted.
    ///
    /// This is the flag from the partition table.
    /// If flash encryption is enabled certain partition types are encrypted
    /// regardless of this.
    pub fn is_encrypted(&self) -> bool {
        self.flags() & 0b10 != 0
    }

    /// Like [PartitionEntry::is_encrypted] but also takes into account:
    /// - is flash encryption enabled, otherwise this will always return false
    /// - certain partition types are always encrypted, no matter what the partition table says
    pub(crate) fn is_effectively_encrypted(&self) -> bool {
        #[cfg(feature = "std")]
        let enabled = false;

        #[cfg(not(feature = "std"))]
        let enabled = esp_storage::flash_encryption();

        enabled
            && (self.is_encrypted()
                || matches!(self.partition_type(), PartitionType::App(_))
                || matches!(self.partition_type(), PartitionType::PartitionTable(_))
                || matches!(
                    self.partition_type(),
                    PartitionType::Data(DataPartitionSubType::NvsKeys)
                )
                || matches!(
                    self.partition_type(),
                    PartitionType::Data(DataPartitionSubType::Ota)
                ))
    }

    /// The partition type (type and sub-type).
    pub fn partition_type(&self) -> PartitionType {
        match self.raw_type() {
            0 => PartitionType::App(unwrap!(self.raw_subtype().try_into())),
            1 => PartitionType::Data(unwrap!(self.raw_subtype().try_into())),
            2 => PartitionType::Bootloader(unwrap!(self.raw_subtype().try_into())),
            3 => PartitionType::PartitionTable(unwrap!(self.raw_subtype().try_into())),
            _ => unreachable!(),
        }
    }

    /// Provides a "view" into the partition allowing to read/write the
    /// partition contents using the given [`FlashStorage`].
    pub fn as_flash_region<'a, 'd>(self, flash: &'a mut FlashStorage<'d>) -> FlashRegion<'a, 'd> {
        FlashRegion {
            offset: self.offset(),
            len: self.len(),
            partition_type: self.partition_type(),
            read_only: self.is_read_only(),
            encrypted: self.is_effectively_encrypted(),
            flash,
        }
    }

    /// Calculate the SHA-256 digest of this partition.
    ///
    /// - App / bootloader with appended hash: return that digest after verifying it
    /// - App / bootloader without appended hash: hash the image (not the whole partition)
    /// - Other types: hash the entire partition
    ///
    /// For app images this is the **validation hash** (shown by
    /// `esptool.py image-info`), not the ELF file SHA-256 stored in
    /// [`crate::EspAppDesc`].
    pub fn sha256(&self, flash: &mut FlashStorage<'_>) -> Result<[u8; 32], Error> {
        if self.is_empty() {
            return Err(Error::InvalidArgument);
        }

        let address = self.offset();
        let encrypted = self.is_effectively_encrypted();
        let mut size = self.len();

        if matches!(
            self.partition_type(),
            PartitionType::App(_) | PartitionType::Bootloader(_)
        ) {
            let data = get_image_metadata(flash, address, size, encrypted)?;
            if data.hash_appended {
                let calc = sha256_flash_contents(
                    flash,
                    address,
                    data.image_len - PARTITION_HASH_LEN as u32,
                    encrypted,
                )?;
                if calc != data.image_digest {
                    return Err(Error::InvalidImage);
                }
                return Ok(data.image_digest);
            }
            size = data.image_len;
        }

        sha256_flash_contents(flash, address, size, encrypted)
    }
}

impl core::fmt::Debug for PartitionEntry {
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
impl defmt::Format for PartitionEntry {
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The partition table is invalid or doesn't contain a needed partition.
    Invalid,
    /// An operation tries to access data that is out of bounds.
    OutOfBounds,
    /// An error which originates from the embedded-storage implementation.
    StorageError,
    /// The partition is write protected.
    WriteProtected,
    /// The partition is invalid.
    InvalidPartition {
        expected_size: usize,
        expected_type: PartitionType,
    },
    /// Invalid state
    InvalidState,
    /// The given argument is invalid.
    InvalidArgument,
    /// The operation is not supported for this partition (e.g. `as_nor_flash` on an encrypted
    /// partition).
    NotSupported,
    /// The partition does not contain a valid application or bootloader image.
    InvalidImage,
}

impl core::error::Error for Error {}

/// A partition table.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PartitionTable<'a> {
    binary: &'a [[u8; RAW_ENTRY_LEN]],
    entries: usize,
}

impl<'a> PartitionTable<'a> {
    fn new(binary: &'a [u8]) -> Result<Self, Error> {
        if binary.len() > PARTITION_TABLE_MAX_LEN {
            return Err(Error::Invalid);
        }

        let (binary, rem) = binary.as_chunks::<RAW_ENTRY_LEN>();
        if !rem.is_empty() {
            return Err(Error::Invalid);
        }

        if binary.is_empty() {
            return Ok(Self {
                binary: &[],
                entries: 0,
            });
        }

        let mut raw_table = Self {
            binary,
            entries: binary.len(),
        };

        #[cfg(feature = "validation")]
        {
            let index = raw_table
                .binary
                .iter()
                .position(|entry| u16::from_le_bytes([entry[0], entry[1]]) == MD5_MAGIC)
                .ok_or(Error::Invalid)?;
            let hash = &raw_table.binary[index][16..][..16];

            let mut hasher = crate::crypto::Md5::new();

            for entry in &raw_table.binary[..index] {
                hasher.update(entry);
            }
            let calculated_hash = hasher.finalize();

            if calculated_hash != hash {
                return Err(Error::Invalid);
            }
        }

        let entries = {
            let mut i = 0;
            loop {
                if let Ok(entry) = raw_table.get_partition(i) {
                    if entry.magic() != ENTRY_MAGIC {
                        break;
                    }

                    i += 1;

                    if i == raw_table.entries {
                        break;
                    }
                } else {
                    return Err(Error::Invalid);
                }
            }
            i
        };

        raw_table.entries = entries;

        Ok(raw_table)
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
    pub fn get_partition(&self, index: usize) -> Result<PartitionEntry, Error> {
        if index >= self.entries {
            return Err(Error::OutOfBounds);
        }
        Ok(PartitionEntry::new(&self.binary[index]))
    }

    /// Get the first partition matching the given partition type.
    pub fn find_partition(&self, pt: PartitionType) -> Result<Option<PartitionEntry>, Error> {
        for i in 0..self.entries {
            let entry = self.get_partition(i)?;
            if entry.partition_type() == pt {
                return Ok(Some(entry));
            }
        }
        Ok(None)
    }

    /// Returns an iterator over the partitions.
    pub fn iter(&self) -> impl Iterator<Item = PartitionEntry> {
        (0..self.entries).filter_map(|i| self.get_partition(i).ok())
    }

    #[cfg(feature = "std")]
    /// Get the currently booted partition.
    pub fn booted_partition(&self) -> Result<Option<PartitionEntry>, Error> {
        Err(Error::Invalid)
    }

    #[cfg(not(feature = "std"))]
    /// Get the currently booted partition.
    pub fn booted_partition(&self) -> Result<Option<PartitionEntry>, Error> {
        // Read entry 0 from MMU to know which partition is mapped
        //
        // See <https://github.com/espressif/esp-idf/blob/758939caecb16e5542b3adfba0bc85025517db45/components/hal/mmu_hal.c#L124>
        cfg_select! {
            feature = "esp32" => {
                let paddr = unsafe { ((0x3FF10000 as *const u32).read_volatile() & 0xff) << 16 };
            }
            feature = "esp32s2" => {
                let paddr = unsafe {
                    (((0x61801000 + 128 * 4) as *const u32).read_volatile() & 0xff) << 16
                };
            }
            feature = "esp32s3" => {
                // Revisit this once we support XiP from PSRAM for ESP32-S3
                let paddr = unsafe { ((0x600C5000 as *const u32).read_volatile() & 0xff) << 16 };
            }
            any(feature = "esp32c2", feature = "esp32c3") => {
                let paddr = unsafe { ((0x600c5000 as *const u32).read_volatile() & 0xff) << 16 };
            }
            feature = "esp32p4" => {
                // DR_REG_FLASH_SPI0_BASE : 0x5008C000 = DR_REG_HPPERIPH0_BASE + 0x8C000
                // TODO: verify MSPI register for partition physical address read
                let paddr = unsafe {
                    ((0x5008C000 + 0x380) as *mut u32).write_volatile(0); // SPI_MEM_C_MMU_ITEM_INDEX_REG
                    (((0x5008C000 + 0x37c) as *const u32).read_volatile() & 0xff) << 16 // SPI_MEM_C_MMU_ITEM_CONTENT_REG
                };
            }
            feature = "esp32s31" => {
                // Read MMU entry 0, which maps the beginning of the flash
                // virtual-address range.
                let paddr = unsafe {
                    ((0x20500000 + 0x380) as *mut u32).write_volatile(0); // SPI_MEM_C_MMU_ITEM_INDEX_REG
                    (((0x20500000 + 0x37c) as *const u32).read_volatile() & 0x7ff) << 16 // SPI_MEM_C_MMU_ITEM_CONTENT_REG
                };
            }
            any(
                feature = "esp32c5",
                feature = "esp32c6",
                feature = "esp32c61",
                feature = "esp32h2"
            ) => {
                let paddr = unsafe {
                    ((0x60002000 + 0x380) as *mut u32).write_volatile(0);
                    (((0x60002000 + 0x37c) as *const u32).read_volatile() & 0xff) << 16
                };
            }
            _ => {}
        }

        for id in 0..self.len() {
            let entry = self.get_partition(id)?;
            if entry.offset() == paddr {
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

/// Read the partition table.
///
/// Pass [`FlashStorage`] and a buffer to read the partition table into.
pub fn read_partition_table<'a, 'd>(
    flash: &mut FlashStorage<'d>,
    storage: &'a mut [u8],
) -> Result<PartitionTable<'a>, Error> {
    read_partition_table_impl(flash, storage)
}

fn read_partition_table_impl<'a, F: FlashAccess>(
    flash: &mut F,
    storage: &'a mut [u8],
) -> Result<PartitionTable<'a>, Error> {
    #[cfg(feature = "std")]
    let enabled = false;

    #[cfg(not(feature = "std"))]
    let enabled = esp_storage::flash_encryption();

    if enabled {
        flash.flash_read_encrypted(PARTITION_TABLE_OFFSET, storage)?;
    } else {
        flash.flash_read(PARTITION_TABLE_OFFSET, storage)?;
    }

    PartitionTable::new(storage)
}

const PARTITION_HASH_LEN: usize = 32;
const IMAGE_HEADER_MAGIC: u8 = 0xE9;
const IMAGE_HEADER_LEN: u32 = 24;
const IMAGE_MAX_SEGMENTS: u8 = 16;
const IMAGE_MAX_FLASH_ADDR_SIZE: u32 = 16 * 1024 * 1024;

/// Subset of ESP-IDF `esp_image_metadata_t` for partition SHA-256 convenience.
struct ImageMetadata {
    image_len: u32,
    image_digest: [u8; PARTITION_HASH_LEN],
    hash_appended: bool,
}

/// Parse an app/bootloader image on flash and return its length and optional
/// appended SHA-256 digest.
///
/// Walks the image header and segment table, accounts for the checksum
/// padding, and — if the image has a simple hash appended — reads that digest.
/// Does not verify the checksum or load any segments.
fn get_image_metadata<F: FlashAccess>(
    flash: &mut F,
    address: u32,
    part_size: u32,
    encrypted: bool,
) -> Result<ImageMetadata, Error> {
    if part_size == 0 || part_size > IMAGE_MAX_FLASH_ADDR_SIZE {
        return Err(Error::InvalidArgument);
    }

    // process_image_header()
    let mut hdr = [0u8; IMAGE_HEADER_LEN as usize];
    flash_read(flash, address, &mut hdr, encrypted)?;
    // `esp_image_get_metadata` skips header verify, but refuse obvious garbage.
    if hdr[0] != IMAGE_HEADER_MAGIC || hdr[1] > IMAGE_MAX_SEGMENTS {
        return Err(Error::InvalidImage);
    }

    let mut image_len = IMAGE_HEADER_LEN;

    // process_segments()
    for _ in 0..hdr[1] {
        let mut seg = [0u8; 8];
        flash_read(flash, address + image_len, &mut seg, encrypted)?;
        // seg[0..4] - load address
        let data_len = u32::from_le_bytes(unwrap!(seg[4..8].try_into()));
        if data_len % 4 != 0 || data_len >= IMAGE_MAX_FLASH_ADDR_SIZE {
            return Err(Error::InvalidImage);
        }
        image_len = image_len
            .checked_add(8 + data_len)
            .ok_or(Error::InvalidImage)?;
    }

    // process_checksum()
    // add a byte for the checksum, pad to next full 16 byte block
    image_len = (image_len + 1 + 15) & !15;

    // process_appended_hash_and_sig()
    let hash_appended = hdr[23] != 0;
    let mut image_digest = [0u8; PARTITION_HASH_LEN];
    if hash_appended {
        flash_read(flash, address + image_len, &mut image_digest, encrypted)?;
        image_len += PARTITION_HASH_LEN as u32;
    }

    if image_len > part_size {
        return Err(Error::InvalidImage);
    }

    Ok(ImageMetadata {
        image_len,
        image_digest,
        hash_appended,
    })
}

fn flash_read<F: FlashAccess>(
    flash: &mut F,
    address: u32,
    bytes: &mut [u8],
    encrypted: bool,
) -> Result<(), Error> {
    if encrypted {
        flash.flash_read_encrypted(address, bytes)
    } else {
        flash.flash_read(address, bytes)
    }
}

/// Hash `len` bytes of flash starting at `flash_offset`.
///
/// Reads the region in fixed-size chunks so large partitions do not need to be
/// loaded into memory at once.
fn sha256_flash_contents<F: FlashAccess>(
    flash: &mut F,
    mut flash_offset: u32,
    mut len: u32,
    encrypted: bool,
) -> Result<[u8; PARTITION_HASH_LEN], Error> {
    use sha2::{Digest, Sha256};

    let mut hasher = Sha256::new();
    let mut chunk = [0u8; 4096];

    while len > 0 {
        let n = len.min(chunk.len() as u32) as usize;
        flash_read(flash, flash_offset, &mut chunk[..n], encrypted)?;
        hasher.update(&chunk[..n]);
        flash_offset += n as u32;
        len -= n as u32;
    }

    Ok(hasher.finalize().into())
}

/// A flash region is a "view" into the partition.
///
/// It allows to read and write to the partition without the need to account for
/// the partition offset.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashRegion<'a, 'd> {
    pub(crate) offset: u32,
    pub(crate) len: u32,
    pub(crate) partition_type: PartitionType,
    pub(crate) read_only: bool,
    /// Whether the partition is effectively encrypted (see
    /// `PartitionEntry::is_effectively_encrypted`).
    pub(crate) encrypted: bool,
    pub(crate) flash: &'a mut FlashStorage<'d>,
}

impl<'a, 'd> FlashRegion<'a, 'd> {
    /// Returns the size of the partition in bytes.
    pub fn partition_size(&self) -> usize {
        self.len as _
    }

    fn range(&self) -> core::ops::Range<u32> {
        self.offset..self.offset + self.len
    }

    fn in_range(&self, start: u32, len: usize) -> bool {
        self.range().contains(&start) && (start + len as u32 <= self.range().end)
    }

    /// Read bytes from the partition.
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        let address = offset + self.offset;

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        if self.encrypted {
            self.flash.flash_read_encrypted(address, bytes)
        } else {
            self.flash.flash_read(address, bytes)
        }
    }

    /// Write bytes to the partition.
    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        let address = offset + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        if self.encrypted {
            self.flash.flash_write_encrypted(address, bytes)
        } else {
            self.flash.flash_write(address, bytes)
        }
    }

    /// Returns the size of the partition in bytes.
    pub fn capacity(&self) -> usize {
        self.partition_size()
    }

    /// Erase flash in the partition from `from` up to but not including `to`.
    ///
    /// Addresses are relative to the partition start.
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let address_from = from + self.offset;
        let address_to = to + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if from > to {
            return Err(Error::OutOfBounds);
        }

        if !self.in_range(address_from, (address_to - address_from) as usize) {
            return Err(Error::OutOfBounds);
        }

        self.flash.flash_erase(address_from, address_to)
    }
}

#[cfg(feature = "embedded-storage")]
/// [`NorFlash`] and [`MultiwriteNorFlash`] view of a non-encrypted [`FlashRegion`].
pub struct NorFlashRegion<'r, 'a, 'd> {
    region: &'r mut FlashRegion<'a, 'd>,
}

#[cfg(feature = "embedded-storage")]
/// [`NorFlash`] view of an encrypted [`FlashRegion`].
///
/// Write size is one flash sector ([`esp_storage::FlashStorage::SECTOR_SIZE`]): the ROM encrypts
/// whole sectors.
pub struct EncryptedNorFlashRegion<'r, 'a, 'd> {
    region: &'r mut FlashRegion<'a, 'd>,
}

#[cfg(feature = "embedded-storage")]
mod embedded_storage_traits {
    use ::embedded_storage::{
        ReadStorage,
        Region,
        Storage,
        nor_flash::{
            ErrorType,
            MultiwriteNorFlash,
            NorFlash,
            NorFlashError,
            NorFlashErrorKind,
            ReadNorFlash,
        },
    };

    use super::*;

    const NOR_READ_SIZE: usize = <FlashStorage<'static> as FlashAccess>::READ_SIZE;
    const NOR_WRITE_SIZE: usize = <FlashStorage<'static> as FlashAccess>::WRITE_SIZE;
    const NOR_ERASE_SIZE: usize = <FlashStorage<'static> as FlashAccess>::ERASE_SIZE;
    const ENCRYPTED_WRITE_SIZE: usize =
        <FlashStorage<'static> as FlashAccess>::SECTOR_SIZE as usize;

    impl<'a, 'd> FlashRegion<'a, 'd> {
        /// Returns a [`NorFlashRegion`] for [`NorFlash`] access.
        ///
        /// # Errors
        ///
        /// Returns [`Error::NotSupported`] if this partition is treated as encrypted (e.g. app
        /// partitions when flash encryption is enabled).
        pub fn as_nor_flash<'r>(&'r mut self) -> Result<NorFlashRegion<'r, 'a, 'd>, Error> {
            if self.encrypted {
                return Err(Error::NotSupported);
            }

            Ok(NorFlashRegion { region: self })
        }

        /// Returns a [`EncryptedNorFlashRegion`] for [`NorFlash`] access.
        ///
        /// # Errors
        ///
        /// Returns [`Error::NotSupported`] if this partition is not treated as encrypted.
        pub fn as_nor_flash_encrypted<'r>(
            &'r mut self,
        ) -> Result<EncryptedNorFlashRegion<'r, 'a, 'd>, Error> {
            if !self.encrypted {
                return Err(Error::NotSupported);
            }

            Ok(EncryptedNorFlashRegion { region: self })
        }
    }

    impl Region for FlashRegion<'_, '_> {
        fn contains(&self, address: u32) -> bool {
            self.range().contains(&address)
        }
    }

    impl ReadStorage for FlashRegion<'_, '_> {
        type Error = Error;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            FlashRegion::read(self, offset, bytes)
        }

        fn capacity(&self) -> usize {
            FlashRegion::capacity(self)
        }
    }

    impl Storage for FlashRegion<'_, '_> {
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            FlashRegion::write(self, offset, bytes)
        }
    }

    impl NorFlashError for Error {
        fn kind(&self) -> NorFlashErrorKind {
            match self {
                Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
                _ => NorFlashErrorKind::Other,
            }
        }
    }

    impl ErrorType for NorFlashRegion<'_, '_, '_> {
        type Error = Error;
    }

    impl ReadNorFlash for NorFlashRegion<'_, '_, '_> {
        const READ_SIZE: usize = NOR_READ_SIZE;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let address = offset + self.region.offset;

            if !self.region.in_range(address, bytes.len()) {
                return Err(Error::OutOfBounds);
            }

            self.region.flash.flash_read_nor(address, bytes)
        }

        fn capacity(&self) -> usize {
            self.region.capacity()
        }
    }

    impl NorFlash for NorFlashRegion<'_, '_, '_> {
        const WRITE_SIZE: usize = NOR_WRITE_SIZE;
        const ERASE_SIZE: usize = NOR_ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.region.erase(from, to)
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let address = offset + self.region.offset;

            if self.region.read_only {
                return Err(Error::WriteProtected);
            }

            if !self.region.in_range(address, bytes.len()) {
                return Err(Error::OutOfBounds);
            }

            self.region.flash.flash_write_nor(address, bytes)
        }
    }

    impl MultiwriteNorFlash for NorFlashRegion<'_, '_, '_> {}

    impl ErrorType for EncryptedNorFlashRegion<'_, '_, '_> {
        type Error = Error;
    }

    impl ReadNorFlash for EncryptedNorFlashRegion<'_, '_, '_> {
        const READ_SIZE: usize = NOR_READ_SIZE;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let address = offset + self.region.offset;

            if !self.region.in_range(address, bytes.len()) {
                return Err(Error::OutOfBounds);
            }

            self.region.flash.flash_read_encrypted(address, bytes)
        }

        fn capacity(&self) -> usize {
            self.region.capacity()
        }
    }

    impl NorFlash for EncryptedNorFlashRegion<'_, '_, '_> {
        const WRITE_SIZE: usize = ENCRYPTED_WRITE_SIZE;
        const ERASE_SIZE: usize = NOR_ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.region.erase(from, to)
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let address = offset + self.region.offset;

            if self.region.read_only {
                return Err(Error::WriteProtected);
            }

            if !self.region.in_range(address, bytes.len()) {
                return Err(Error::OutOfBounds);
            }

            self.region.flash.flash_write_encrypted(address, bytes)
        }
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

    #[test]
    fn empty_byte_array() {
        let pt = PartitionTable::new(&[]).unwrap();

        assert_eq!(0, pt.len());
        assert!(matches!(pt.get_partition(0), Err(Error::OutOfBounds)));
    }

    #[test]
    fn validation_fails_wo_hash() {
        assert!(matches!(
            PartitionTable::new(&SIMPLE[..RAW_ENTRY_LEN * 3]),
            Err(Error::Invalid)
        ));
    }

    #[test]
    fn validation_fails_wo_hash_max_entries() {
        let mut data = [0u8; PARTITION_TABLE_MAX_LEN];
        for i in 0..96 {
            data[(i * RAW_ENTRY_LEN)..][..RAW_ENTRY_LEN].copy_from_slice(&SIMPLE[..32]);
        }

        assert!(matches!(PartitionTable::new(&data), Err(Error::Invalid)));
    }

    #[test]
    fn validation_succeeds_with_enough_entries() {
        assert_eq!(
            3,
            PartitionTable::new(&SIMPLE[..RAW_ENTRY_LEN * 4])
                .unwrap()
                .len()
        );
    }
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    fn test_flash() -> FlashStorage<'static> {
        let mut flash = FlashStorage::new();
        let mut data = [23u8; 0x10000];
        data[PARTITION_TABLE_OFFSET as usize..][..PARTITION_TABLE_MAX_LEN]
            .copy_from_slice(include_bytes!("../testdata/single_factory_no_ota.bin"));
        flash.write(0, &data).unwrap();
        flash
    }

    #[test]
    fn can_read_write_all_of_nvs() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);
        assert_eq!(nvs_partition.offset, 36864);

        assert_eq!(nvs_partition.capacity(), 24576);

        let mut buffer = [0u8; 24576];
        nvs_partition.read(0, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 23));
        buffer.fill(42);
        nvs_partition.write(0, &buffer).unwrap();
        let mut buffer = [0u8; 24576];
        nvs_partition.read(0, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 42));
    }

    #[test]
    fn cannot_read_write_more_than_partition_size() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);
        assert_eq!(nvs_partition.offset, 36864);

        assert_eq!(nvs_partition.capacity(), 24576);

        let mut buffer = [0u8; 24577];
        assert!(nvs_partition.read(0, &mut buffer) == Err(Error::OutOfBounds));
    }

    #[test]
    fn can_erase_up_to_partition_end() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);

        let capacity = nvs_partition.capacity() as u32;
        assert_eq!(capacity, 24576);

        nvs_partition.write(0, &[42u8; 24576]).unwrap();

        nvs_partition.erase(capacity - 4096, capacity).unwrap();
        let mut buffer = [0u8; 4096];
        nvs_partition.read(capacity - 4096, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 0xff));

        nvs_partition.erase(0, capacity).unwrap();
        let mut buffer = [0u8; 24576];
        nvs_partition.read(0, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 0xff));
    }

    #[test]
    fn cannot_erase_out_of_bounds() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);

        let capacity = nvs_partition.capacity() as u32;

        assert!(nvs_partition.erase(0, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(nvs_partition.erase(capacity, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(nvs_partition.erase(4096, 0) == Err(Error::OutOfBounds));
    }
}

#[cfg(test)]
mod sha256_tests {
    use super::*;

    /// SHA-256 of `0x6000` bytes filled with `0xA5`.
    const NVS_DIGEST: [u8; 32] = [
        0xb0, 0x5b, 0x4f, 0x2c, 0xc2, 0xa7, 0x54, 0x25, 0x54, 0xfa, 0x32, 0x8b, 0xd0, 0x5d, 0x86,
        0x7f, 0x0c, 0x1d, 0xae, 0xed, 0x46, 0x48, 0x8e, 0x31, 0xb0, 0x0c, 0xb0, 0xaa, 0xe5, 0xb5,
        0x49, 0x81,
    ];

    /// SHA-256 of the minimal test image body (32 bytes) with `hash_appended = 1`.
    const IMAGE_DIGEST_WITH_HASH_FLAG: [u8; 32] = [
        0xb2, 0xb7, 0x64, 0x4a, 0x57, 0x62, 0x46, 0x05, 0xf7, 0xe4, 0xb1, 0xc3, 0xbf, 0x96, 0x5a,
        0x20, 0x87, 0x37, 0x3d, 0x7a, 0xc6, 0x2d, 0xf8, 0x6a, 0xcf, 0x2b, 0x1a, 0xcf, 0xe4, 0x8e,
        0xe8, 0xa0,
    ];

    /// SHA-256 of the same minimal image body with `hash_appended = 0`.
    const IMAGE_DIGEST_WITHOUT_HASH_FLAG: [u8; 32] = [
        0x02, 0x50, 0xbb, 0x56, 0xe1, 0x91, 0xf6, 0x6d, 0xde, 0xf1, 0x5e, 0x2d, 0x7c, 0xb4, 0x48,
        0x23, 0x75, 0x36, 0x52, 0x54, 0x7f, 0xc3, 0xd9, 0xd5, 0x83, 0xaa, 0xca, 0x2e, 0xec, 0xfe,
        0x99, 0x00,
    ];

    fn test_flash() -> FlashStorage<'static> {
        let mut flash = FlashStorage::new();
        let mut data = [0xffu8; 0x10000];
        data[PARTITION_TABLE_OFFSET as usize..][..PARTITION_TABLE_MAX_LEN]
            .copy_from_slice(include_bytes!("../testdata/single_factory_no_ota.bin"));
        flash.write(0, &data).unwrap();
        flash
    }

    /// Header-only ESP image (0 segments); body pads to 32 bytes for the checksum.
    fn write_minimal_app_image(
        flash: &mut FlashStorage<'static>,
        offset: u32,
        hash_appended: bool,
    ) {
        let mut image = [0u8; 64];
        image[0] = IMAGE_HEADER_MAGIC;
        image[23] = u8::from(hash_appended);
        // image[1] = 0 segments; bytes 24..32 are checksum padding
        if hash_appended {
            image[32..64].copy_from_slice(&IMAGE_DIGEST_WITH_HASH_FLAG);
            flash.write(offset, &image).unwrap();
        } else {
            flash.write(offset, &image[..32]).unwrap();
        }
    }

    #[test]
    fn sha256_of_data_partition_matches_known_digest() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();

        nvs.as_flash_region(&mut flash)
            .write(0, &[0xa5u8; 0x6000])
            .unwrap();

        assert_eq!(nvs.sha256(&mut flash).unwrap(), NVS_DIGEST);
    }

    #[test]
    fn sha256_of_app_with_appended_hash_returns_validation_digest() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), true);

        assert_eq!(
            factory.sha256(&mut flash).unwrap(),
            IMAGE_DIGEST_WITH_HASH_FLAG
        );
    }

    #[test]
    fn sha256_of_app_without_appended_hash_hashes_image() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), false);

        assert_eq!(
            factory.sha256(&mut flash).unwrap(),
            IMAGE_DIGEST_WITHOUT_HASH_FLAG
        );
    }

    #[test]
    fn sha256_rejects_corrupt_appended_hash() {
        let mut flash = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut flash, &mut buffer).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        write_minimal_app_image(&mut flash, factory.offset(), true);

        // Corrupt the appended digest
        flash.write(factory.offset() + 32, &[0u8; 32]).unwrap();

        assert_eq!(factory.sha256(&mut flash), Err(Error::InvalidImage));
    }
}

#[cfg(all(test, feature = "embedded-storage"))]
mod nor_flash_tests {
    use embedded_storage::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};

    use super::*;

    fn test_flash() -> FlashStorage<'static> {
        let mut flash = FlashStorage::new();
        let mut data = [23u8; 0x10000];
        data[PARTITION_TABLE_OFFSET as usize..][..PARTITION_TABLE_MAX_LEN]
            .copy_from_slice(include_bytes!("../testdata/single_factory_no_ota.bin"));
        flash.write(0, &data).unwrap();
        flash
    }

    #[test]
    fn plain_nor_flash_implements_multi_write() {
        fn assert_multi_write<N: MultiwriteNorFlash>() {}
        assert_multi_write::<NorFlashRegion<'static, 'static, 'static>>();
    }

    #[test]
    fn as_nor_flash_succeeds_on_plain_partition() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);

        assert!(nvs_partition.as_nor_flash().is_ok());
        assert!(matches!(
            nvs_partition.as_nor_flash_encrypted(),
            Err(Error::NotSupported)
        ));
    }

    #[test]
    fn nor_flash_write_sizes() {
        assert_eq!(
            <NorFlashRegion<'static, 'static, 'static> as NorFlash>::WRITE_SIZE,
            <FlashStorage<'static> as FlashAccess>::WRITE_SIZE
        );
        assert_eq!(
            <EncryptedNorFlashRegion<'static, 'static, 'static> as NorFlash>::WRITE_SIZE,
            <FlashStorage<'static> as FlashAccess>::SECTOR_SIZE as usize
        );
        assert_eq!(
            <NorFlashRegion<'static, 'static, 'static> as ReadNorFlash>::READ_SIZE,
            <FlashStorage<'static> as FlashAccess>::READ_SIZE
        );
        assert_eq!(
            <EncryptedNorFlashRegion<'static, 'static, 'static> as ReadNorFlash>::READ_SIZE,
            <FlashStorage<'static> as FlashAccess>::READ_SIZE
        );
    }

    #[test]
    fn nor_flash_erase_bounds() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage);
        let capacity = nvs_partition.capacity() as u32;
        let mut nor_flash = nvs_partition.as_nor_flash().unwrap();

        nor_flash.erase(0, capacity).unwrap();
        let mut buffer = [0u8; 4096];
        nor_flash.read(capacity - 4096, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 0xff));

        assert!(nor_flash.erase(0, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(nor_flash.erase(4096, 0) == Err(Error::OutOfBounds));
    }
}
