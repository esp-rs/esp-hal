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

mod entry;
mod image;
#[cfg(feature = "embedded-storage")]
mod nor_flash;
mod region;
mod table;
mod types;

pub use self::{
    entry::PartitionEntry,
    region::{EncryptedFlashRegion, FlashRegion, PartitionRegion},
    table::{PartitionTable, read_partition_table},
    types::{
        AppPartitionSubType,
        BootloaderPartitionSubType,
        DataPartitionSubType,
        PartitionTablePartitionSubType,
        PartitionType,
        RawPartitionType,
    },
};
pub use crate::flash::FlashStorage;

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
    /// An address or length is not aligned as the operation requires.
    NotAligned,
    /// The partition is write protected.
    WriteProtected,
    /// The partition is invalid.
    InvalidPartition {
        expected_size: usize,
        expected_type: PartitionType,
    },
    /// Invalid state.
    InvalidState,
    /// The given argument is invalid.
    InvalidArgument,
    /// The operation is not supported for this partition (e.g. `as_flash_region` on an encrypted
    /// partition).
    NotSupported,
    /// The partition does not contain a valid application or bootloader image.
    InvalidImage,
}

impl core::error::Error for Error {}
