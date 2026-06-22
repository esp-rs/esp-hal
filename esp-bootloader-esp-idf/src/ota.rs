//! # Over The Air Updates (OTA)
//!
//! ## Overview
//! The OTA update mechanism allows a device to update itself based on data
//! received while the normal firmware is running (for example, over Wi-Fi,
//! Bluetooth or Ethernet).
//!
//! OTA requires configuring the Partition Tables of the device with at least
//! two OTA app slot partitions (i.e., ota_0 and ota_1) and an OTA Data
//! Partition.
//!
//! The OTA operation functions write a new app firmware image to whichever OTA
//! app slot that is currently not selected for booting. Once the image is
//! verified, the OTA Data partition is updated to specify that this image
//! should be used for the next boot.
//!
//! Note: The prebuilt bootloaders provided by `espflash` _might not_ include
//! OTA support. In that case you need to build the bootloader yourself.
//!
//! The general procedure to change the active slot
//! - read the partition table [crate::partitions::read_partition_table]
//! - find the Data/Ota partition
//! - initialize [Ota]
//! - read the current slot, change the current slot
//!
//! For more details see <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/ota.html>
use crate::partitions::{
    AppPartitionSubType,
    DataPartitionSubType,
    Error,
    FlashRegion,
    PartitionType,
};

const SLOT0_DATA_OFFSET: u32 = 0x0000;
const SLOT1_DATA_OFFSET: u32 = 0x1000;

const UNINITIALIZED_SEQUENCE: u32 = 0xffffffff;

/// Representation of the current OTA-data slot.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum OtaDataSlot {
    /// If there is a `firmware` app-partition it's used. Otherwise OTA-0
    None,
    /// OTA-0
    Slot0,
    /// OTA-1
    Slot1,
}

impl OtaDataSlot {
    /// The next logical OTA-data slot
    fn next(&self) -> OtaDataSlot {
        match self {
            OtaDataSlot::None => OtaDataSlot::Slot0,
            OtaDataSlot::Slot0 => OtaDataSlot::Slot1,
            OtaDataSlot::Slot1 => OtaDataSlot::Slot0,
        }
    }

    fn offset(&self) -> u32 {
        match self {
            OtaDataSlot::None => SLOT0_DATA_OFFSET,
            OtaDataSlot::Slot0 => SLOT0_DATA_OFFSET,
            OtaDataSlot::Slot1 => SLOT1_DATA_OFFSET,
        }
    }
}

/// OTA image states for checking operability of the app.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Default, Hash, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
pub enum OtaImageState {
    /// Monitor the first boot. The bootloader will change this to
    /// `PendingVerify` if auto-rollback is enabled.
    ///
    /// You want to set this state after activating a newly installed update.
    New           = 0x0,

    /// Bootloader changes [OtaImageState::New] to
    /// [OtaImageState::PendingVerify] to indicate the app should confirm the
    /// image as working.
    PendingVerify = 0x1,

    /// Set by the firmware once it's found to be working. The bootloader will
    /// consider this Slot as working and continue to use it.
    Valid         = 0x2,

    /// Set by the firmware once it's found to be non-working.
    ///
    /// The bootloader will consider this Slot as non-working and not try to
    /// boot it further.
    Invalid       = 0x3,

    /// The bootloader will change the state to [OtaImageState::Aborted] if the
    /// application didn't change [OtaImageState::PendingVerify]
    /// to either [OtaImageState::Valid] or [OtaImageState::Invalid].
    Aborted       = 0x4,

    /// Undefined. The bootloader won't make any assumptions about the working
    /// state of this slot.
    #[default]
    Undefined     = 0xFFFFFFFF,
}

impl TryFrom<u32> for OtaImageState {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        OtaImageState::from_repr(value).ok_or(Error::Invalid)
    }
}

/// OTA selection entry structure (two copies in the OTA data partition).
/// Size of 32 bytes is friendly to flash encryption.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
struct OtaSelectEntry {
    /// OTA sequence number.
    pub ota_seq: u32,
    /// Sequence label (unused in the bootloader).
    pub seq_label: [u8; 20],
    /// OTA image state.
    pub ota_state: OtaImageState,
    /// CRC32 of the `ota_seq` field only.
    pub crc: u32,
}

impl OtaSelectEntry {
    fn read<'a, 'd>(region: &mut FlashRegion<'a, 'd>, offset: u32) -> Result<Self, Error> {
        fn is_valid(buffer: &[u8]) -> bool {
            let ota_seq = u32::from_le_bytes(unwrap!(buffer[0..4].try_into()));
            let ota_state = u32::from_le_bytes(unwrap!(buffer[24..28].try_into()));
            let crc = u32::from_le_bytes(unwrap!(buffer[28..32].try_into()));

            if ota_seq == u32::MAX && ota_state == OtaImageState::Undefined as u32 {
                return true;
            }

            if OtaImageState::try_from(ota_state).is_err() {
                return false;
            }

            let calculated_crc = crate::crypto::Crc32::new().crc(&ota_seq.to_le_bytes());
            if crc != calculated_crc {
                return false;
            }

            return true;
        }

        let mut buffer = [0u8; 32];
        region.read(offset, &mut buffer)?;

        if !is_valid(&buffer) {
            return Err(Error::Invalid);
        }

        let entry: *mut OtaSelectEntry = &mut buffer as *mut _ as *mut OtaSelectEntry;
        let entry = unsafe { *entry };
        Ok(entry)
    }

    fn write<'a, 'd>(
        &mut self,
        region: &mut FlashRegion<'a, 'd>,
        offset: u32,
    ) -> Result<(), Error> {
        let bytes: &mut [u8; 32] = unwrap!(
            unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, 0x20) }.try_into()
        );
        region.write(offset, bytes)?;

        Ok(())
    }
}

/// This is used to manipulate the OTA-data partition.
///
/// If you are looking for a more high-level way to do this, see [crate::ota_updater::OtaUpdater]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ota<'a, 'd> {
    flash: FlashRegion<'a, 'd>,
    ota_partition_count: usize,
}

impl<'a, 'd> Ota<'a, 'd> {
    /// Create a [Ota] instance from the given [FlashRegion] and the count of OTA app partitions
    /// (not including "firmware" and "test" partitions)
    ///
    /// # Errors
    /// A [Error::InvalidPartition] if the given flash region
    /// doesn't represent a Data/Ota partition or the size is unexpected.
    ///
    /// [Error::InvalidArgument] if the `ota_partition_count` exceeds the maximum or if it's 0.
    pub fn new(
        flash: FlashRegion<'a, 'd>,
        ota_partition_count: usize,
    ) -> Result<Ota<'a, 'd>, Error> {
        if ota_partition_count == 0 || ota_partition_count > 16 {
            return Err(Error::InvalidArgument);
        }

        if flash.capacity() != 0x2000
            || flash.raw.partition_type() != PartitionType::Data(DataPartitionSubType::Ota)
        {
            return Err(Error::InvalidPartition {
                expected_size: 0x2000,
                expected_type: PartitionType::Data(DataPartitionSubType::Ota),
            });
        }

        Ok(Ota {
            flash,
            ota_partition_count,
        })
    }

    /// Returns the currently selected app partition.
    ///
    /// This might not be the booted partition if the bootloader failed to boot
    /// the partition and felt back to the last known working app partition.
    ///
    /// See [crate::partitions::PartitionTable::booted_partition] to get the booted
    /// partition.
    pub fn current_app_partition(&mut self) -> Result<AppPartitionSubType, Error> {
        let (seq0, seq1) = self.get_slot_seq()?;

        let slot = if seq0 == UNINITIALIZED_SEQUENCE && seq1 == UNINITIALIZED_SEQUENCE {
            AppPartitionSubType::Factory
        } else if seq0 == UNINITIALIZED_SEQUENCE {
            AppPartitionSubType::from_ota_app_number(
                ((seq1 - 1) % self.ota_partition_count as u32) as u8,
            )?
        } else if seq1 == UNINITIALIZED_SEQUENCE || seq0 > seq1 {
            AppPartitionSubType::from_ota_app_number(
                ((seq0 - 1) % self.ota_partition_count as u32) as u8,
            )?
        } else {
            let counter = u32::max(seq0, seq1) - 1;
            AppPartitionSubType::from_ota_app_number(
                (counter % self.ota_partition_count as u32) as u8,
            )?
        };

        Ok(slot)
    }

    fn get_slot_seq(&mut self) -> Result<(u32, u32), Error> {
        let buffer1 = OtaSelectEntry::read(&mut self.flash, SLOT0_DATA_OFFSET)?;
        let buffer2 = OtaSelectEntry::read(&mut self.flash, SLOT1_DATA_OFFSET)?;
        let seq0 = buffer1.ota_seq;
        let seq1 = buffer2.ota_seq;
        Ok((seq0, seq1))
    }

    /// Sets the currently active OTA-slot.
    ///
    /// Passing [AppPartitionSubType::Factory] will reset the OTA-data.
    ///
    /// # Errors
    ///
    /// [Error::InvalidArgument] if [AppPartitionSubType::Test] is given or if the OTA app partition
    /// number exceeds the value given to the constructor.
    pub fn set_current_app_partition(&mut self, app: AppPartitionSubType) -> Result<(), Error> {
        if app == AppPartitionSubType::Factory {
            self.flash.write(SLOT0_DATA_OFFSET, &[0xffu8; 0x20])?;
            self.flash.write(SLOT1_DATA_OFFSET, &[0xffu8; 0x20])?;
            return Ok(());
        }

        if app == AppPartitionSubType::Test {
            // cannot switch to the test partition - it's a partition
            // which special built bootloaders will boot depending on the state of a pin
            return Err(Error::InvalidArgument);
        }

        let ota_app_index = app.ota_app_number();
        if ota_app_index >= self.ota_partition_count as u8 {
            return Err(Error::InvalidArgument);
        }

        let current = self.current_app_partition()?;

        // no need to update any sequence if the partition isn't changed
        if current != app {
            // the bootloader will look at the two slots in ota-data and get the highest sequence
            // number
            //
            // the booted ota-app-partition is the sequence-nr modulo the number of
            // ota-app-partitions

            // calculate the needed increment of the sequence-number to select the requested OTA-app
            // partition
            let inc = if current == AppPartitionSubType::Factory {
                (((app.ota_app_number()) as i32 + 1) + (self.ota_partition_count as i32)) as u32
                    % self.ota_partition_count as u32
            } else {
                ((((app.ota_app_number()) as i32) - ((current.ota_app_number()) as i32))
                    + (self.ota_partition_count as i32)) as u32
                    % self.ota_partition_count as u32
            };

            // the slot we need to write the new sequence number to
            let slot = self.current_slot()?.next();

            let (seq0, seq1) = self.get_slot_seq()?;
            let new_seq = {
                if seq0 == UNINITIALIZED_SEQUENCE && seq1 == UNINITIALIZED_SEQUENCE {
                    // no ota-app partition is selected
                    inc
                } else if seq0 == UNINITIALIZED_SEQUENCE {
                    // seq1 is the sequence number to increment
                    seq1 + inc
                } else if seq1 == UNINITIALIZED_SEQUENCE {
                    // seq0 is the sequence number to increment
                    seq0 + inc
                } else {
                    u32::max(seq0, seq1) + inc
                }
            };

            let crc = crate::crypto::Crc32::new();
            let checksum = crc.crc(&new_seq.to_le_bytes());

            let mut buffer = OtaSelectEntry::read(&mut self.flash, slot.offset())?;
            buffer.ota_seq = new_seq;
            buffer.crc = checksum;
            buffer.write(&mut self.flash, slot.offset())?;
        }

        Ok(())
    }

    // determine the current ota-data slot by checking the sequence numbers
    fn current_slot(&mut self) -> Result<OtaDataSlot, Error> {
        let (seq0, seq1) = self.get_slot_seq()?;

        let slot = if seq0 == UNINITIALIZED_SEQUENCE && seq1 == UNINITIALIZED_SEQUENCE {
            OtaDataSlot::None
        } else if seq0 == UNINITIALIZED_SEQUENCE {
            OtaDataSlot::Slot1
        } else if seq1 == UNINITIALIZED_SEQUENCE || seq0 > seq1 {
            OtaDataSlot::Slot0
        } else {
            OtaDataSlot::Slot1
        };
        Ok(slot)
    }

    /// Set the [OtaImageState] of the currently selected slot.
    ///
    /// # Errors
    /// A [Error::InvalidState] if no partition is currently selected.
    pub fn set_current_ota_state(&mut self, state: OtaImageState) -> Result<(), Error> {
        if let (UNINITIALIZED_SEQUENCE, UNINITIALIZED_SEQUENCE) = self.get_slot_seq()? {
            Err(Error::InvalidState)
        } else {
            let offset = self.current_slot()?.offset();
            let mut buffer = OtaSelectEntry::read(&mut self.flash, offset)?;
            buffer.ota_state = state;
            buffer.write(&mut self.flash, offset)?;
            Ok(())
        }
    }

    /// Get the [OtaImageState] of the currently selected slot.
    ///
    /// # Errors
    /// A [Error::InvalidState] if no partition is currently selected.
    pub fn current_ota_state(&mut self) -> Result<OtaImageState, Error> {
        if let (UNINITIALIZED_SEQUENCE, UNINITIALIZED_SEQUENCE) = self.get_slot_seq()? {
            Err(Error::InvalidState)
        } else {
            let offset = self.current_slot()?.offset();
            let buffer = OtaSelectEntry::read(&mut self.flash, offset)?;
            Ok(buffer.ota_state)
        }
    }
}

#[cfg(test)]
mod tests {
    use esp_storage::{Flash, FlashStorage};

    use super::*;
    use crate::partitions::PartitionEntry;

    const PARTITION_RAW: [u8; 32] = [
        0xaa, 0x50, // MAGIC
        1,    // TYPE = DATA
        0,    // SUBTYPE = OTA
        0, 0, 0, 0, // OFFSET
        0, 0x20, 0, 0, // LEN (0x2000)
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // LABEL
        0, 0, 0, 0, // FLAGS
    ];

    const SLOT_INITIAL: &[u8] = &[
        255u8, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    ];

    const SLOT_COUNT_1_UNDEFINED: &[u8] = &[
        1u8, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 154, 152, 67, 71,
    ];

    const SLOT_COUNT_1_VALID: &[u8] = &[
        1u8, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 2, 0, 0, 0, 154, 152, 67, 71,
    ];

    const SLOT_COUNT_2_NEW: &[u8] = &[
        2, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 0, 0, 0, 0, 116, 55, 246, 85,
    ];

    const SLOT_COUNT_3_PENDING: &[u8] = &[
        3, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 1, 0, 0, 0, 17, 80, 74, 237,
    ];

    fn ota_region<'a>(
        flash: &'a mut FlashStorage<'static>,
        binary: &'a mut [u8; 32],
    ) -> FlashRegion<'a, 'static> {
        FlashRegion {
            raw: PartitionEntry { binary },
            flash,
        }
    }

    fn init_ota_flash(flash: &mut FlashStorage<'static>) {
        flash.erase(0, 0x2000).unwrap();
    }

    fn read_slot(flash: &mut FlashStorage<'static>, offset: u32) -> [u8; 0x20] {
        let mut buf = [0u8; 0x20];
        flash.read(offset, &mut buf).unwrap();
        buf
    }

    #[test]
    fn test_initial_state_and_next_slot() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Factory
        );
        assert_eq!(
            sut.current_ota_state(),
            Err(crate::partitions::Error::InvalidState)
        );
        assert_eq!(
            sut.set_current_ota_state(OtaImageState::New),
            Err(crate::partitions::Error::InvalidState)
        );
        assert_eq!(
            sut.current_ota_state(),
            Err(crate::partitions::Error::InvalidState)
        );

        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Undefined));

        assert_eq!(&read_slot(&mut flash, 0x0000)[..], SLOT_COUNT_1_UNDEFINED);
        assert_eq!(&read_slot(&mut flash, 0x1000)[..], SLOT_INITIAL);
    }

    #[test]
    fn test_slot0_valid_next_slot() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);
        flash.write(0x0000, SLOT_COUNT_1_VALID).unwrap();
        flash.write(0x1000, SLOT_INITIAL).unwrap();

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Valid));

        sut.set_current_app_partition(AppPartitionSubType::Ota1)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::New).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota1
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::New));

        assert_eq!(&read_slot(&mut flash, 0x0000)[..], SLOT_COUNT_1_VALID);
        assert_eq!(&read_slot(&mut flash, 0x1000)[..], SLOT_COUNT_2_NEW);
    }

    #[test]
    fn test_slot1_new_next_slot() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);
        flash.write(0x0000, SLOT_COUNT_1_VALID).unwrap();
        flash.write(0x1000, SLOT_COUNT_2_NEW).unwrap();

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota1
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::New));

        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::PendingVerify)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::PendingVerify));

        assert_eq!(&read_slot(&mut flash, 0x0000)[..], SLOT_COUNT_3_PENDING);
        assert_eq!(&read_slot(&mut flash, 0x1000)[..], SLOT_COUNT_2_NEW);
    }

    #[test]
    fn test_multi_updates() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Factory
        );
        assert_eq!(
            sut.current_ota_state(),
            Err(crate::partitions::Error::InvalidState)
        );

        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::PendingVerify)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::PendingVerify));

        sut.set_current_app_partition(AppPartitionSubType::Ota1)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::New).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota1
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::New));

        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::Aborted).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Aborted));

        // setting same app partition again
        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::Valid).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Valid));
    }

    #[test]
    fn test_multi_updates_4_apps() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 4).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Factory
        );
        assert_eq!(
            sut.current_ota_state(),
            Err(crate::partitions::Error::InvalidState)
        );

        sut.set_current_app_partition(AppPartitionSubType::Ota0)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::PendingVerify)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota0
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::PendingVerify));

        sut.set_current_app_partition(AppPartitionSubType::Ota1)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::New).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota1
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::New));

        sut.set_current_app_partition(AppPartitionSubType::Ota2)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::Aborted).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota2
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Aborted));

        sut.set_current_app_partition(AppPartitionSubType::Ota3)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::Valid).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota3
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Valid));

        // going back to a previous app image
        sut.set_current_app_partition(AppPartitionSubType::Ota2)
            .unwrap();
        sut.set_current_ota_state(OtaImageState::Invalid).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota2
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Invalid));

        assert_eq!(
            sut.set_current_app_partition(AppPartitionSubType::Ota5),
            Err(crate::partitions::Error::InvalidArgument)
        );

        assert_eq!(
            sut.set_current_app_partition(AppPartitionSubType::Test),
            Err(crate::partitions::Error::InvalidArgument)
        );
    }

    #[test]
    fn test_multi_updates_skip_parts() {
        let mut binary = PARTITION_RAW;

        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mock_region = ota_region(&mut flash, &mut binary);

        let mut sut = Ota::new(mock_region, 16).unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Factory
        );
        assert_eq!(
            sut.current_ota_state(),
            Err(crate::partitions::Error::InvalidState)
        );

        sut.set_current_app_partition(AppPartitionSubType::Ota10)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota10
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Undefined));

        sut.set_current_app_partition(AppPartitionSubType::Ota14)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota14
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Undefined));

        sut.set_current_app_partition(AppPartitionSubType::Ota5)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Ota5
        );
        assert_eq!(sut.current_ota_state(), Ok(OtaImageState::Undefined));
    }

    #[test]
    fn test_ota_slot_next() {
        assert_eq!(OtaDataSlot::None.next(), OtaDataSlot::Slot0);
        assert_eq!(OtaDataSlot::Slot0.next(), OtaDataSlot::Slot1);
        assert_eq!(OtaDataSlot::Slot1.next(), OtaDataSlot::Slot0);
    }

    #[test]
    fn test_read_erased_slot() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mut region = ota_region(&mut flash, &mut binary);
        let entry = OtaSelectEntry::read(&mut region, SLOT0_DATA_OFFSET).unwrap();
        assert_eq!(entry.ota_seq, UNINITIALIZED_SEQUENCE);
        assert_eq!(entry.seq_label, [0xff; 20]);
        assert_eq!(entry.ota_state, OtaImageState::Undefined);
        assert_eq!(entry.crc, UNINITIALIZED_SEQUENCE);
    }

    #[test]
    fn test_read_valid_slot() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);
        flash.write(0x0000, SLOT_COUNT_1_VALID).unwrap();

        let mut region = ota_region(&mut flash, &mut binary);
        let entry = OtaSelectEntry::read(&mut region, SLOT0_DATA_OFFSET).unwrap();
        assert_eq!(entry.ota_seq, 1);
        assert_eq!(entry.ota_state, OtaImageState::Valid);
    }

    #[test]
    fn test_read_rejects_bad_crc() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mut slot = [0u8; 32];
        slot.copy_from_slice(SLOT_COUNT_1_UNDEFINED);
        slot[31] ^= 0xff;
        flash.write(0x0000, &slot).unwrap();

        let mut region = ota_region(&mut flash, &mut binary);
        assert!(matches!(
            OtaSelectEntry::read(&mut region, SLOT0_DATA_OFFSET),
            Err(crate::partitions::Error::Invalid)
        ));
    }

    #[test]
    fn test_read_rejects_unknown_ota_state() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let ota_seq = 1u32;
        let crc = crate::crypto::Crc32::new().crc(&ota_seq.to_le_bytes());
        let mut slot = [0xffu8; 32];
        slot[0..4].copy_from_slice(&ota_seq.to_le_bytes());
        slot[24..28].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        slot[28..32].copy_from_slice(&crc.to_le_bytes());
        flash.write(0x0000, &slot).unwrap();

        let mut region = ota_region(&mut flash, &mut binary);
        assert!(matches!(
            OtaSelectEntry::read(&mut region, SLOT0_DATA_OFFSET),
            Err(crate::partitions::Error::Invalid)
        ));
    }

    #[test]
    fn test_read_rejects_erased_seq_with_non_erased_state() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mut slot = [0xffu8; 32];
        slot[24..28].copy_from_slice(&(OtaImageState::Valid as u32).to_le_bytes());
        flash.write(0x0000, &slot).unwrap();

        let mut region = ota_region(&mut flash, &mut binary);
        assert!(matches!(
            OtaSelectEntry::read(&mut region, SLOT0_DATA_OFFSET),
            Err(crate::partitions::Error::Invalid)
        ));
    }

    #[test]
    fn test_one_corrupt_slot_fails_current_app_partition() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mut corrupt = [0u8; 32];
        corrupt.copy_from_slice(SLOT_COUNT_1_UNDEFINED);
        corrupt[31] ^= 0xff;
        flash.write(0x0000, &corrupt).unwrap();
        flash.write(0x1000, SLOT_COUNT_2_NEW).unwrap();

        let region = ota_region(&mut flash, &mut binary);
        let mut sut = Ota::new(region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition(),
            Err(crate::partitions::Error::Invalid)
        );
    }

    #[test]
    fn test_reset_to_factory_after_corrupt_ota_data() {
        let mut binary = PARTITION_RAW;
        let mut flash = FlashStorage::new(Flash::new());
        init_ota_flash(&mut flash);

        let mut corrupt = [0u8; 32];
        corrupt.copy_from_slice(SLOT_COUNT_1_UNDEFINED);
        corrupt[31] ^= 0xff;
        flash.write(0x0000, &corrupt).unwrap();
        flash.write(0x1000, SLOT_COUNT_2_NEW).unwrap();

        let region = ota_region(&mut flash, &mut binary);
        let mut sut = Ota::new(region, 2).unwrap();
        assert_eq!(
            sut.current_app_partition(),
            Err(crate::partitions::Error::Invalid)
        );

        sut.set_current_app_partition(AppPartitionSubType::Factory)
            .unwrap();
        assert_eq!(
            sut.current_app_partition().unwrap(),
            AppPartitionSubType::Factory
        );
        assert_eq!(&read_slot(&mut flash, 0x0000)[..], SLOT_INITIAL);
        assert_eq!(&read_slot(&mut flash, 0x1000)[..], SLOT_INITIAL);
    }
}
