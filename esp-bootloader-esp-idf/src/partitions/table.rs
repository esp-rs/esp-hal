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
        let paddr = booted_app_offset();

        for id in 0..self.len() {
            let entry = self.get_partition(id)?;
            if entry.offset() == paddr {
                return Ok(Some(entry));
            }
        }

        Ok(None)
    }
}

/// Returns the flash offset of the running application image.
#[cfg(not(feature = "std"))]
pub(super) fn booted_app_offset() -> u32 {
    // Read entry 0 from MMU to know which partition is mapped
    //
    // See <https://github.com/espressif/esp-idf/blob/758939caecb16e5542b3adfba0bc85025517db45/components/hal/mmu_hal.c#L124>
    cfg_select! {
        feature = "esp32" => {
            let paddr = unsafe { ((0x3FF10000 as *const u32).read_volatile() & 0xff) << 16 };
        }
        feature = "esp32s2" => {
            let paddr =
                unsafe { (((0x61801000 + 128 * 4) as *const u32).read_volatile() & 0xff) << 16 };
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
        feature = "esp32h4" => {
            let paddr = unsafe {
                ((0x60098000 + 0x380) as *mut u32).write_volatile(0); // SPI_MEM_MMU_ITEM_INDEX_REG
                (((0x60098000 + 0x37c) as *const u32).read_volatile() & 0x1ff) << 16 // SPI_MEM_MMU_ITEM_CONTENT_REG
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

    paddr
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
