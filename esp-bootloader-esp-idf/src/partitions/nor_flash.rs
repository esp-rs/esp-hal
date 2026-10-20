use embedded_storage::nor_flash::{
    ErrorType,
    MultiwriteNorFlash,
    NorFlash,
    NorFlashError,
    NorFlashErrorKind,
    ReadNorFlash,
};

use super::{Error, FlashRegion};
use crate::flash::{SECTOR_SIZE, WORD_SIZE};

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

impl ErrorType for FlashRegion<'_, '_> {
    type Error = Error;
}

impl ReadNorFlash for FlashRegion<'_, '_> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        FlashRegion::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        FlashRegion::capacity(self)
    }
}

impl NorFlash for FlashRegion<'_, '_> {
    const WRITE_SIZE: usize = WORD_SIZE as usize;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        FlashRegion::erase(self, from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        FlashRegion::write(self, offset, bytes)
    }
}

impl MultiwriteNorFlash for FlashRegion<'_, '_> {}

#[cfg(test)]
mod tests {
    use embedded_storage::nor_flash::{
        MultiwriteNorFlash,
        NorFlash,
        NorFlashError,
        NorFlashErrorKind,
        ReadNorFlash,
    };

    use super::*;
    use crate::partitions::{
        DataPartitionSubType,
        FlashStorage,
        PARTITION_TABLE_MAX_LEN,
        PARTITION_TABLE_OFFSET,
        PartitionType,
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
    fn flash_region_implements_multi_write() {
        fn assert_multi_write<N: MultiwriteNorFlash>() {}
        assert_multi_write::<FlashRegion<'static, 'static>>();
    }

    #[test]
    fn nor_flash_sizes() {
        assert_eq!(
            <FlashRegion<'static, 'static> as ReadNorFlash>::READ_SIZE,
            1
        );
        assert_eq!(<FlashRegion<'static, 'static> as NorFlash>::WRITE_SIZE, 4);
        assert_eq!(
            <FlashRegion<'static, 'static> as NorFlash>::ERASE_SIZE,
            4096
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
        let mut nor_flash = nvs.as_flash_region(&mut storage).unwrap();
        let capacity = ReadNorFlash::capacity(&nor_flash) as u32;

        NorFlash::erase(&mut nor_flash, 0, capacity).unwrap();
        let mut buffer = [0u8; 4096];
        ReadNorFlash::read(&mut nor_flash, capacity - 4096, &mut buffer).unwrap();
        assert!(buffer.iter().all(|v| *v == 0xff));

        assert!(NorFlash::erase(&mut nor_flash, 0, capacity + 4096) == Err(Error::OutOfBounds));
        assert!(NorFlash::erase(&mut nor_flash, 4096, 0) == Err(Error::OutOfBounds));
    }

    #[test]
    fn nor_flash_rejects_unaligned_write() {
        let mut storage = test_flash();

        let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let pt = read_partition_table(&mut storage, &mut buffer).unwrap();

        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();
        let mut nor_flash = nvs.as_flash_region(&mut storage).unwrap();

        let error = NorFlash::write(&mut nor_flash, 1, &[0; 4]).unwrap_err();
        assert_eq!(error.kind(), NorFlashErrorKind::NotAligned);
    }
}
