//! SDIO DMA types for SoCs with a dedicated engine.

use core::fmt::Debug;

use crate::dma::{
    DescriptorFlagFields,
    DescriptorSetGeneric,
    DmaDescriptorGeneric,
    DmaLoopBufGeneric,
    DmaRxBufGeneric,
    DmaRxTxBufGeneric,
    DmaTxBufGeneric,
    Owner,
    PreparationGeneric,
};

/// Convenience alias for the DMA descriptor used with the SDIO dedicated DMA controller.
pub type DmaDescriptor = DmaDescriptorGeneric<DmaDescriptorFlags>;

/// Convenience alias for SDIO dedicated DMA preparation.
pub type Preparation = PreparationGeneric<DmaDescriptorFlags>;

/// Convenience alias for SDIO dedicated DMA loop buffer.
pub type DmaLoopBuf = DmaLoopBufGeneric<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA descriptor set.
#[allow(unused)]
pub(crate) type DescriptorSet<'a> = DescriptorSetGeneric<'a, DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX buffer.
pub type DmaTxBuf = DmaTxBufGeneric<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX buffer.
pub type DmaRxBuf = DmaRxBufGeneric<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX/TX buffer.
pub type DmaRxTxBuf = DmaRxTxBufGeneric<DmaDescriptorFlags>;

bitfield::bitfield! {
    /// DMA descriptor flags for the dedicated SDIO DMA engine.
    ///
    /// Based on the general [DMA](crate::dma::DmaDescriptorFlags) implementation,
    /// with corrections for bitfield layout.
    ///
    /// All of the fields have the same semantics. See [DescriptorFlagFields].
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct DmaDescriptorFlags(u32);

    u16;

    _size, _set_size: 13, 0;
    _length, _set_length: 27, 14;
    _suc_eof, _set_suc_eof: 30;
    _owner, _set_owner: 31;
}

impl DmaDescriptorFlags {
    /// Creates a new [DmaDescriptorFlags].
    pub const fn new() -> Self {
        Self(0)
    }
}

impl Default for DmaDescriptorFlags {
    fn default() -> Self {
        Self::new()
    }
}

impl Debug for DmaDescriptorFlags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaDescriptorFlags")
            .field("size", &self.size())
            .field("length", &self.len())
            .field("suc_eof", &self.suc_eof())
            .field("owner", &self.owner().to_str())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmaDescriptorFlags {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "DmaDescriptorFlags {{ size: {}, length: {}, suc_eof: {}, owner: {} }}",
            self.size(),
            self.len(),
            self.suc_eof(),
            self.owner().to_str()
        );
    }
}

impl DescriptorFlagFields for DmaDescriptorFlags {
    fn empty() -> Self {
        Self(0)
    }

    fn size(&self) -> usize {
        self._size() as usize
    }

    fn set_size(&mut self, size: usize) {
        self._set_size(size as u16);
    }

    fn len(&self) -> usize {
        self._length() as usize
    }

    fn set_len(&mut self, length: usize) {
        self._set_length(length as u16);
    }

    fn suc_eof(&self) -> bool {
        self._suc_eof()
    }

    fn set_suc_eof(&mut self, suc_eof: bool) {
        self._set_suc_eof(suc_eof);
    }

    fn owner(&self) -> Owner {
        self._owner().into()
    }

    fn set_owner(&mut self, owner: Owner) {
        self._set_owner(owner.into());
    }
}
