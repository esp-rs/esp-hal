use super::{
    Error,
    FlashStorage,
    PARTITION_TABLE_MAX_LEN,
    PARTITION_TABLE_OFFSET,
    PartitionEntry,
    PartitionType,
    RAW_ENTRY_LEN,
};
use crate::flash::FlashAccess;

const ENTRY_MAGIC: u16 = 0x50aa;
#[cfg(feature = "validation")]
const MD5_MAGIC: u16 = 0xebeb;

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

    /// Returns the number of partitions contained in the partition table.
    pub fn len(&self) -> usize {
        self.entries
    }

    /// Returns whether there are no recognized partitions.
    pub fn is_empty(&self) -> bool {
        self.entries == 0
    }

    /// Returns a partition entry.
    pub fn get_partition(&self, index: usize) -> Result<PartitionEntry, Error> {
        if index >= self.entries {
            return Err(Error::OutOfBounds);
        }
        Ok(PartitionEntry::new(&self.binary[index]))
    }

    /// Returns the first partition matching the given partition type.
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
    /// Returns the currently booted partition.
    pub fn booted_partition(&self) -> Result<Option<PartitionEntry>, Error> {
        Err(Error::Invalid)
    }

    #[cfg(not(feature = "std"))]
    /// Returns the currently booted partition.
    pub fn booted_partition(&self) -> Result<Option<PartitionEntry>, Error> {
        let Some(paddr) = booted_app_offset() else {
            return Ok(None);
        };

        for id in 0..self.len() {
            let entry = self.get_partition(id)?;
            if entry.offset() == paddr {
                return Ok(Some(entry));
            }
        }

        Ok(None)
    }
}

/// Returns the flash offset of the running application image, or `None` when no MMU entry maps
/// it.
///
/// The bootloader maps the app's first page at the last page of the DROM window, so the app can
/// find its partition. The ESP32's bootloader uses the window's first page.
/// See <https://github.com/espressif/esp-idf/blob/4d59230ddff16327812782151ef0afef202dc6d7/components/bootloader_support/src/bootloader_utility.c#L1080-L1082>.
#[cfg(not(feature = "std"))]
pub(super) fn booted_app_offset() -> Option<u32> {
    use esp_hal::flash::{flash_mmu_drom_entries, flash_mmu_entry_offset};

    let drom = flash_mmu_drom_entries();
    let entry = if cfg!(feature = "esp32") {
        drom.start
    } else {
        drom.end - 1
    };
    let offset = flash_mmu_entry_offset(entry);
    // The ESP32-S31 bootloader that espflash 4.6 bundles is built for 32 KB pages, and maps the
    // app at entry 511 instead of 1023.
    #[cfg(feature = "esp32s31")]
    let offset = offset.or_else(|| flash_mmu_entry_offset(511));
    offset
}

/// Reads the partition table.
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
    let enabled = esp_hal::efuse::flash_encryption();

    if enabled {
        flash.flash_read_encrypted(PARTITION_TABLE_OFFSET, storage)?;
    } else {
        flash.flash_read(PARTITION_TABLE_OFFSET, storage)?;
    }

    PartitionTable::new(storage)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partitions::{AppPartitionSubType, DataPartitionSubType};

    static SIMPLE: &[u8] = include_bytes!("../../testdata/single_factory_no_ota.bin");
    static OTA: &[u8] = include_bytes!("../../testdata/factory_app_two_ota.bin");
    static CUSTOM: &[u8] = include_bytes!("../../testdata/custom_types.bin");

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
    fn read_custom_types() {
        let pt = PartitionTable::new(CUSTOM).unwrap();

        assert_eq!(7, pt.len());

        assert_eq!(
            PartitionType::Unknown {
                raw_type: 0x40,
                raw_subtype: 0x30
            },
            pt.get_partition(2).unwrap().partition_type()
        );
        assert_eq!(
            PartitionType::Unknown {
                raw_type: 1,
                raw_subtype: 0x40
            },
            pt.get_partition(3).unwrap().partition_type()
        );

        let custom = pt
            .find_partition(PartitionType::Unknown {
                raw_type: 0x40,
                raw_subtype: 0x30,
            })
            .unwrap()
            .unwrap();
        assert_eq!("custom", custom.label_as_str());

        let ota_1 = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Ota1))
            .unwrap()
            .unwrap();
        assert_eq!(0x130000, ota_1.offset());
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
