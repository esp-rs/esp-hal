use super::{Error, FlashStorage, PartitionEntry, PartitionType};
use crate::flash::{ENCRYPTED_WRITE_SIZE, FlashAccess, SECTOR_SIZE, WORD_SIZE};

impl PartitionEntry {
    /// Provides a plaintext "view" into the partition allowing to read/write the
    /// partition contents using the given [`FlashStorage`].
    ///
    /// The partition containing the running application is always read-only.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] if the partition is effectively encrypted. Use
    /// [`PartitionEntry::as_encrypted_flash_region`] instead.
    pub fn as_flash_region<'a, 'd>(
        self,
        flash: &'a mut FlashStorage<'d>,
    ) -> Result<FlashRegion<'a, 'd>, Error> {
        if self.is_effectively_encrypted() {
            return Err(Error::NotSupported);
        }
        Ok(FlashRegion {
            region: self.region(flash),
        })
    }

    /// Provides a "view" into an encrypted partition, which reads decrypted
    /// data and writes encrypted data.
    ///
    /// The partition containing the running application is always read-only.
    ///
    /// # Errors
    ///
    /// [`Error::NotSupported`] if the partition is not effectively encrypted, i.e. flash
    /// encryption is disabled or the partition type is not encrypted.
    pub fn as_encrypted_flash_region<'a, 'd>(
        self,
        flash: &'a mut FlashStorage<'d>,
    ) -> Result<EncryptedFlashRegion<'a, 'd>, Error> {
        if !self.is_effectively_encrypted() {
            return Err(Error::NotSupported);
        }
        Ok(EncryptedFlashRegion {
            region: self.region(flash),
        })
    }

    /// Provides a "view" into the partition that is plaintext or encrypted,
    /// depending on whether the partition is effectively encrypted.
    ///
    /// The partition containing the running application is always read-only.
    pub fn as_auto_flash_region<'a, 'd>(
        self,
        flash: &'a mut FlashStorage<'d>,
    ) -> AutoFlashRegion<'a, 'd> {
        let region = self.region(flash);
        if self.is_effectively_encrypted() {
            AutoFlashRegion::Encrypted(EncryptedFlashRegion { region })
        } else {
            AutoFlashRegion::Plain(FlashRegion { region })
        }
    }

    fn region<'a, 'd>(self, flash: &'a mut FlashStorage<'d>) -> Region<'a, 'd> {
        Region {
            offset: self.offset(),
            len: self.len(),
            partition_type: self.partition_type(),
            // Modifying the running application would corrupt code or immutable data.
            read_only: self.is_read_only() || self.contains_running_app(),
            flash,
        }
    }

    #[cfg(feature = "std")]
    fn contains_running_app(&self) -> bool {
        false
    }

    #[cfg(not(feature = "std"))]
    fn contains_running_app(&self) -> bool {
        super::table::booted_app_offset()
            .is_some_and(|app| (self.offset()..self.offset() + self.len()).contains(&app))
    }
}

/// The flash range of a partition.
///
/// Holds the bounds, write-protection and alignment checks that the public
/// region types share.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Region<'a, 'd> {
    offset: u32,
    len: u32,
    partition_type: PartitionType,
    read_only: bool,
    flash: &'a mut FlashStorage<'d>,
}

impl Region<'_, '_> {
    fn capacity(&self) -> usize {
        self.len as _
    }

    fn range(&self) -> core::ops::Range<u32> {
        self.offset..self.offset + self.len
    }

    fn in_range(&self, start: u32, len: usize) -> bool {
        self.range().contains(&start) && (start + len as u32 <= self.range().end)
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8], encrypted: bool) -> Result<(), Error> {
        let address = offset + self.offset;

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        if encrypted {
            self.flash.flash_read_encrypted(address, bytes)
        } else {
            self.flash.flash_read(address, bytes)
        }
    }

    fn write(&mut self, offset: u32, bytes: &[u8], encrypted: bool) -> Result<(), Error> {
        let address = offset + self.offset;

        if self.read_only {
            return Err(Error::WriteProtected);
        }

        if !self.in_range(address, bytes.len()) {
            return Err(Error::OutOfBounds);
        }

        let align = if encrypted {
            ENCRYPTED_WRITE_SIZE
        } else {
            WORD_SIZE
        };
        if !offset.is_multiple_of(align) || !bytes.len().is_multiple_of(align as usize) {
            return Err(Error::NotAligned);
        }

        if encrypted {
            self.flash.flash_write_encrypted(address, bytes)
        } else {
            self.flash.flash_write(address, bytes)
        }
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
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

        if !from.is_multiple_of(SECTOR_SIZE) || !to.is_multiple_of(SECTOR_SIZE) {
            return Err(Error::NotAligned);
        }

        self.flash.flash_erase(address_from, address_to)
    }
}

/// A plaintext "view" into a partition.
///
/// It allows to read and write to the partition without the need to account for
/// the partition offset.
///
/// Created by [`PartitionEntry::as_flash_region`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashRegion<'a, 'd> {
    region: Region<'a, 'd>,
}

impl FlashRegion<'_, '_> {
    /// Returns the size of the partition in bytes.
    pub fn partition_size(&self) -> usize {
        self.region.capacity()
    }

    /// Reads bytes from the partition.
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.region.read(offset, bytes, false)
    }

    /// Writes bytes to the partition.
    ///
    /// The target range must be erased first: flash programming can only clear bits.
    ///
    /// # Errors
    ///
    /// - [`Error::WriteProtected`] if the partition is read-only or contains the running
    ///   application.
    /// - [`Error::OutOfBounds`] if the range exceeds the partition.
    /// - [`Error::NotAligned`] if `offset` or the length is not a multiple of 4.
    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.region.write(offset, bytes, false)
    }

    /// Returns the size of the partition in bytes.
    pub fn capacity(&self) -> usize {
        self.region.capacity()
    }

    /// Erases flash in the partition from `from` up to but not including `to`.
    ///
    /// Addresses are relative to the partition start.
    ///
    /// # Errors
    ///
    /// - [`Error::WriteProtected`] if the partition is read-only or contains the running
    ///   application.
    /// - [`Error::OutOfBounds`] if `from > to` or the range exceeds the partition.
    /// - [`Error::NotAligned`] if `from` or `to` is not a multiple of 4096.
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.region.erase(from, to)
    }
}

/// A "view" into an encrypted partition.
///
/// Reads decrypt data, writes encrypt it. Offsets are relative to the partition start.
///
/// This type does not implement the `embedded-storage` NOR flash traits, erased
/// flash does not read back as `0xFF` after decryption.
///
/// Created by [`PartitionEntry::as_encrypted_flash_region`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EncryptedFlashRegion<'a, 'd> {
    region: Region<'a, 'd>,
}

impl EncryptedFlashRegion<'_, '_> {
    /// Returns the size of the partition in bytes.
    pub fn partition_size(&self) -> usize {
        self.region.capacity()
    }

    /// Returns the size of the partition in bytes.
    pub fn capacity(&self) -> usize {
        self.region.capacity()
    }

    /// Reads and decrypts bytes from the partition.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfBounds`] if the range exceeds the partition.
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.region.read(offset, bytes, true)
    }

    /// Encrypts and writes bytes to the partition.
    ///
    /// The target range must be erased first: flash programming can only clear bits.
    ///
    /// # Errors
    ///
    /// - [`Error::WriteProtected`] if the partition is read-only or contains the running
    ///   application.
    /// - [`Error::OutOfBounds`] if the range exceeds the partition.
    /// - [`Error::NotAligned`] if `offset` or the length is not a multiple of 16.
    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.region.write(offset, bytes, true)
    }

    /// Erases flash in the partition from `from` up to but not including `to`.
    ///
    /// # Errors
    ///
    /// - [`Error::WriteProtected`] if the partition is read-only or contains the running
    ///   application.
    /// - [`Error::OutOfBounds`] if `from > to` or the range exceeds the partition.
    /// - [`Error::NotAligned`] if `from` or `to` is not a multiple of 4096.
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.region.erase(from, to)
    }
}

/// A "view" into a partition that is either plaintext or encrypted.
///
/// Used for partitions that are encrypted only when flash encryption is
/// enabled, such as app and OTA data partitions. All methods forward to the
/// wrapped region.
///
/// Created by [`PartitionEntry::as_auto_flash_region`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AutoFlashRegion<'a, 'd> {
    /// The partition is not encrypted.
    Plain(FlashRegion<'a, 'd>),
    /// The partition is encrypted.
    Encrypted(EncryptedFlashRegion<'a, 'd>),
}

impl<'a, 'd> AutoFlashRegion<'a, 'd> {
    fn region(&self) -> &Region<'a, 'd> {
        match self {
            Self::Plain(region) => &region.region,
            Self::Encrypted(region) => &region.region,
        }
    }

    pub(crate) fn partition_type(&self) -> PartitionType {
        self.region().partition_type
    }

    /// Returns the size of the partition in bytes.
    pub fn partition_size(&self) -> usize {
        self.region().capacity()
    }

    /// Returns the size of the partition in bytes.
    pub fn capacity(&self) -> usize {
        self.region().capacity()
    }

    /// Returns whether the partition is encrypted.
    pub fn is_encrypted(&self) -> bool {
        matches!(self, Self::Encrypted(_))
    }

    /// Reads bytes from the partition, see [`FlashRegion::read`] and
    /// [`EncryptedFlashRegion::read`].
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        match self {
            Self::Plain(region) => region.read(offset, bytes),
            Self::Encrypted(region) => region.read(offset, bytes),
        }
    }

    /// Writes bytes to the partition, see [`FlashRegion::write`] and
    /// [`EncryptedFlashRegion::write`].
    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        match self {
            Self::Plain(region) => region.write(offset, bytes),
            Self::Encrypted(region) => region.write(offset, bytes),
        }
    }

    /// Erases the partition from `from` up to but not including `to`, see
    /// [`FlashRegion::erase`].
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        match self {
            Self::Plain(region) => region.erase(from, to),
            Self::Encrypted(region) => region.erase(from, to),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partitions::{
        DataPartitionSubType,
        PARTITION_TABLE_MAX_LEN,
        PARTITION_TABLE_OFFSET,
        read_partition_table,
    };

    fn test_flash() -> FlashStorage<'static> {
        let mut flash = FlashStorage::new();
        let mut data = [23u8; 0x10000];
        data[PARTITION_TABLE_OFFSET as usize..][..PARTITION_TABLE_MAX_LEN]
            .copy_from_slice(include_bytes!("../../testdata/single_factory_no_ota.bin"));
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
        let mut nvs_partition = nvs.as_flash_region(&mut storage).unwrap();
        assert_eq!(nvs_partition.region.offset, 36864);

        assert_eq!(nvs_partition.capacity(), 24576);

        let mut buffer = [0u8; 24576];
        nvs_partition.read(0, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 23));
        buffer.fill(42);
        nvs_partition.erase(0, 24576).unwrap();
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
        let mut nvs_partition = nvs.as_flash_region(&mut storage).unwrap();
        assert_eq!(nvs_partition.region.offset, 36864);

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
        let mut nvs_partition = nvs.as_flash_region(&mut storage).unwrap();

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
        let mut nvs_partition = nvs.as_flash_region(&mut storage).unwrap();

        let capacity = nvs_partition.capacity() as u32;

        assert!(nvs_partition.erase(0, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(nvs_partition.erase(capacity, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(nvs_partition.erase(4096, 0) == Err(Error::OutOfBounds));
    }

    #[test]
    fn rejects_unaligned_write_and_erase() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nvs_partition = nvs.as_flash_region(&mut storage).unwrap();

        assert_eq!(nvs_partition.write(2, &[0; 4]), Err(Error::NotAligned));
        assert_eq!(nvs_partition.write(4, &[0; 6]), Err(Error::NotAligned));
        assert_eq!(nvs_partition.erase(512, 4096), Err(Error::NotAligned));
        assert_eq!(nvs_partition.erase(0, 512), Err(Error::NotAligned));
    }

    #[test]
    fn encrypted_write_requires_16_byte_alignment() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        // Host tests run without flash encryption, so build the region directly.
        let mut region = EncryptedFlashRegion {
            region: nvs.region(&mut storage),
        };

        region.erase(0, 4096).unwrap();
        assert_eq!(region.write(4, &[0; 16]), Err(Error::NotAligned));
        assert_eq!(region.write(16, &[0; 20]), Err(Error::NotAligned));
        region.write(16, &[0x5a; 32]).unwrap();

        let mut buffer = [0u8; 34];
        region.read(15, &mut buffer).unwrap();
        assert_eq!(buffer[0], 0xff);
        assert_eq!(buffer[1..33], [0x5a; 32]);
        assert_eq!(buffer[33], 0xff);
    }

    #[test]
    fn encrypted_region_requires_flash_encryption() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();

        assert!(!nvs.is_effectively_encrypted());
        assert!(nvs.as_flash_region(&mut storage).is_ok());
        assert!(!nvs.as_auto_flash_region(&mut storage).is_encrypted());
        assert!(matches!(
            nvs.as_encrypted_flash_region(&mut storage),
            Err(Error::NotSupported)
        ));
    }
}
