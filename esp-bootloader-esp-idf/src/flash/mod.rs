mod flash_access;
#[cfg(feature = "std")]
mod mock_flash;

pub use flash_access::FlashAccess;

/// Alignment of plaintext writes, the flash driver programs whole words.
pub(crate) const WORD_SIZE: u32 = 4;
/// Alignment of encrypted writes, one flash encryption block.
pub(crate) const ENCRYPTED_WRITE_SIZE: u32 = 16;
/// Alignment of erases.
pub(crate) const SECTOR_SIZE: u32 = 4096;

/// Alias for [`esp_hal::flash::Flash`].
///
/// Pass this to [`crate::partitions::read_partition_table`],
/// [`crate::partitions::PartitionEntry::as_flash_region`], [`crate::ota_updater::OtaUpdater`], and
/// related partition/OTA APIs.
#[cfg(not(feature = "std"))]
pub type FlashStorage<'d> = esp_hal::flash::Flash<'d, esp_hal::Blocking>;

#[cfg(feature = "std")]
pub type FlashStorage<'d> = mock_flash::MockFlash<'d>;
