mod flash_access;
#[cfg(feature = "std")]
mod mock_flash;

pub use flash_access::FlashAccess;

/// Alias for [`esp_storage::FlashStorage`].
///
/// Pass this to [`crate::partitions::read_partition_table`],
/// [`crate::partitions::PartitionEntry::as_flash_region`], [`crate::ota_updater::OtaUpdater`], and
/// related partition/OTA APIs.
#[cfg(not(feature = "std"))]
pub type FlashStorage<'d> = esp_storage::FlashStorage<'d>;

#[cfg(feature = "std")]
pub type FlashStorage<'d> = mock_flash::MockFlash<'d>;
