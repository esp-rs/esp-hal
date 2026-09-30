use super::{DataPartitionSubType, PartitionType, RAW_ENTRY_LEN};

/// Represents a single partition entry.
#[derive(Clone, Copy)]
pub struct PartitionEntry {
    pub(crate) binary: [u8; RAW_ENTRY_LEN],
}

impl PartitionEntry {
    pub(super) fn new(binary: &[u8; RAW_ENTRY_LEN]) -> Self {
        Self { binary: *binary }
    }

    /// Returns the magic value of the entry.
    pub fn magic(&self) -> u16 {
        u16::from_le_bytes(unwrap!(self.binary[..2].try_into()))
    }

    /// Returns the partition type in raw representation.
    pub fn raw_type(&self) -> u8 {
        self.binary[2]
    }

    /// Returns the partition sub-type in raw representation.
    pub fn raw_subtype(&self) -> u8 {
        self.binary[3]
    }

    /// Returns the offset of the partition on flash.
    pub fn offset(&self) -> u32 {
        u32::from_le_bytes(unwrap!(self.binary[4..][..4].try_into()))
    }

    /// Returns the length of the partition in bytes.
    pub fn len(&self) -> u32 {
        u32::from_le_bytes(unwrap!(self.binary[8..][..4].try_into()))
    }

    /// Returns whether the partition has zero length.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the label of the partition.
    pub fn label(&self) -> &[u8] {
        &self.binary[12..][..16]
    }

    /// Returns the label of the partition as `&str`.
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

    /// Returns the raw flags of this partition. You probably want to use
    /// [Self::is_read_only] and [Self::is_encrypted] instead.
    pub fn flags(&self) -> u32 {
        u32::from_le_bytes(unwrap!(self.binary[28..][..4].try_into()))
    }

    /// Returns whether the partition is read-only.
    pub fn is_read_only(&self) -> bool {
        self.flags() & 0b01 != 0
    }

    /// Returns whether the partition is encrypted.
    ///
    /// This is the flag from the partition table.
    /// If flash encryption is enabled certain partition types are encrypted
    /// regardless of this.
    pub fn is_encrypted(&self) -> bool {
        self.flags() & 0b10 != 0
    }

    /// Returns whether the partition is effectively encrypted.
    ///
    /// Unlike [PartitionEntry::is_encrypted], this also takes into account:
    /// - is flash encryption enabled, otherwise this will always return false
    /// - certain partition types are always encrypted, no matter what the partition table says
    pub(crate) fn is_effectively_encrypted(&self) -> bool {
        #[cfg(feature = "std")]
        let enabled = false;

        #[cfg(not(feature = "std"))]
        let enabled = esp_hal::efuse::flash_encryption();

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

    /// Returns the partition type (type and sub-type).
    pub fn partition_type(&self) -> PartitionType {
        match self.raw_type() {
            0 => PartitionType::App(unwrap!(self.raw_subtype().try_into())),
            1 => PartitionType::Data(unwrap!(self.raw_subtype().try_into())),
            2 => PartitionType::Bootloader(unwrap!(self.raw_subtype().try_into())),
            3 => PartitionType::PartitionTable(unwrap!(self.raw_subtype().try_into())),
            _ => unreachable!(),
        }
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
