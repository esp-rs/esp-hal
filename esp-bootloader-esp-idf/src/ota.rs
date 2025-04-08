use crc::{Algorithm, Crc};
use embedded_storage::{ReadStorage, Storage};

use crate::partitions::FlashRegion;

static ALGO: Algorithm<u32> = Algorithm {
    width: 32,
    poly: 0x04c11db7,
    init: 0,
    refin: true,
    refout: true,
    xorout: 0xffffffff,
    check: 0,
    residue: 0,
};

const SLOT0_DATA_OFFSET: u32 = 0x0000;
const SLOT1_DATA_OFFSET: u32 = 0x1000;

/// This is used to manipulate the OTA-data partition.
///
/// This will ever only deal with OTA-0 and OTA-1 since these two slots are
/// supported by the ESP-IDF bootloader.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    flash: &'a mut FlashRegion<'a, F>,
}

/// Representation of the current OTA slot.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Slot {
    /// If there is a `firmware` app-partition it's used. Otherwise OTA-0
    None,
    /// OTA-0
    Slot0,
    /// OTA-1
    Slot1,
}

impl Slot {
    /// The slot represented as `usize`
    pub fn number(&self) -> usize {
        match self {
            Slot::None => 0,
            Slot::Slot0 => 0,
            Slot::Slot1 => 1,
        }
    }

    /// The next logical OTA slot
    pub fn next(&self) -> Slot {
        match self {
            Slot::None => Slot::Slot0,
            Slot::Slot0 => Slot::Slot1,
            Slot::Slot1 => Slot::Slot0,
        }
    }
}

impl<'a, F> Ota<'a, F>
where
    F: embedded_storage::Storage,
{
    /// Create a [Ota] instance from the given [FlashRegion]
    pub fn new(flash: &'a mut FlashRegion<'a, F>) -> Ota<'a, F> {
        assert!(
            flash.capacity() == 0x2000,
            "OTA-data partition must be 0x2000 bytes in size"
        );
        Ota { flash }
    }

    /// Returns the currently active OTA-slot.
    pub fn current_slot(&mut self) -> Result<Slot, crate::partitions::Error> {
        let (seq0, seq1) = self.get_slot_seq()?;

        let slot = if seq0 == 0xffffffff && seq1 == 0xffffffff {
            Slot::None
        } else if seq0 == 0xffffffff {
            Slot::Slot1
        } else if seq1 == 0xffffffff {
            Slot::Slot0
        } else if seq0 > seq1 {
            Slot::Slot0
        } else {
            Slot::Slot1
        };

        Ok(slot)
    }

    fn get_slot_seq(&mut self) -> Result<(u32, u32), crate::partitions::Error> {
        let mut buffer1 = [0u8; 0x20];
        let mut buffer2 = [0u8; 0x20];
        self.flash.read(SLOT0_DATA_OFFSET, &mut buffer1)?;
        self.flash.read(SLOT1_DATA_OFFSET, &mut buffer2)?;
        let mut seq0bytes = [0u8; 4];
        let mut seq1bytes = [0u8; 4];
        seq0bytes[..].copy_from_slice(&buffer1[..4]);
        seq1bytes[..].copy_from_slice(&buffer2[..4]);
        let seq0 = u32::from_le_bytes(seq0bytes);
        let seq1 = u32::from_le_bytes(seq1bytes);
        Ok((seq0, seq1))
    }

    /// Sets the currently active OTA-slot.
    ///
    /// Passing [Slot::None] will reset the OTA-data
    pub fn set_current_slot(&mut self, slot: Slot) -> Result<(), crate::partitions::Error> {
        if slot == Slot::None {
            self.flash.write(SLOT0_DATA_OFFSET, &[0xffu8; 0x20])?;
            self.flash.write(SLOT1_DATA_OFFSET, &[0xffu8; 0x20])?;
            return Ok(());
        }

        let (seq0, seq1) = self.get_slot_seq()?;

        let new_seq = {
            if seq0 == 0xffffffff && seq1 == 0xffffffff {
                1
            } else if seq0 == 0xffffffff {
                seq1 + 1
            } else if seq1 == 0xffffffff {
                seq0 + 1
            } else {
                u32::max(seq0, seq1) + 1
            }
        };
        let new_seq_le = new_seq.to_le_bytes();

        let crc = Crc::<u32>::new(&ALGO);
        let mut digest = crc.digest();
        digest.update(&new_seq_le);
        let checksum = digest.finalize();
        let checksum_le = checksum.to_le_bytes();

        let mut buffer1 = [0xffu8; 0x20];
        let mut buffer2 = [0xffu8; 0x20];

        self.flash.read(SLOT0_DATA_OFFSET, &mut buffer1)?;
        self.flash.read(SLOT1_DATA_OFFSET, &mut buffer2)?;

        if slot == Slot::Slot0 {
            buffer1[..4].copy_from_slice(&new_seq_le);
            buffer1[28..].copy_from_slice(&checksum_le);
            self.flash.write(SLOT0_DATA_OFFSET, &buffer1)?;
        } else {
            buffer2[..4].copy_from_slice(&new_seq_le);
            buffer2[28..].copy_from_slice(&checksum_le);
            self.flash.write(SLOT1_DATA_OFFSET, &buffer2)?;
        }

        Ok(())
    }
}
