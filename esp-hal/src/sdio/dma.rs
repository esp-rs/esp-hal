//! SDIO DMA types for SoCs with a dedicated engine.

use core::fmt::Debug;

use crate::dma::{
    ChannelRx as ChannelRxType,
    ChannelTx as ChannelTxType,
    DescriptorFlagFields,
    DescriptorSet as DescriptorSetType,
    DmaDescriptor as DmaDescriptorType,
    DmaLoopBuf as DmaLoopBufType,
    DmaRxBuf as DmaRxBufType,
    DmaRxTxBuf as DmaRxTxBufType,
    DmaTxBuf as DmaTxBufType,
    Owner,
    Preparation as PreparationType,
};

mod buffer;
mod descriptor;

pub use buffer::AtomicBuffer;
pub use descriptor::{AtomicDmaDescriptor, AtomicDmaDescriptors};

/// Convenience alias for the SDIO dedicated DMA descriptor.
pub type DmaDescriptor = DmaDescriptorType<DmaDescriptorFlags>;

/// Convenience alias for SDIO dedicated DMA preparation.
pub type Preparation = PreparationType<DmaDescriptorFlags>;

/// Convenience alias for SDIO dedicated DMA loop buffer.
pub type DmaLoopBuf = DmaLoopBufType<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA descriptor set.
#[allow(unused)]
pub(crate) type DescriptorSet<'a> = DescriptorSetType<'a, DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX buffer.
pub type DmaTxBuf = DmaTxBufType<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX buffer.
pub type DmaRxBuf = DmaRxBufType<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA RX/TX buffer.
pub type DmaRxTxBuf = DmaRxTxBufType<DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA transmit channel.
pub type ChannelTx<Dm, CH> = ChannelTxType<Dm, CH, DmaDescriptorFlags>;

/// Convenience alias for the SDIO dedicated DMA receive channel.
pub type ChannelRx<Dm, CH> = ChannelRxType<Dm, CH, DmaDescriptorFlags>;

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

    /// Converts a [`u32`] into a [DmaDescriptorFlags].
    pub const fn from_u32(val: u32) -> Self {
        Self(val)
    }

    /// Converts a [DmaDescriptorFlags] into a [`u32`].
    pub const fn into_u32(self) -> u32 {
        self.0
    }
}

impl Default for DmaDescriptorFlags {
    fn default() -> Self {
        Self::new()
    }
}

impl From<u32> for DmaDescriptorFlags {
    fn from(val: u32) -> Self {
        Self::from_u32(val)
    }
}

impl From<DmaDescriptorFlags> for u32 {
    fn from(val: DmaDescriptorFlags) -> Self {
        val.into_u32()
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

impl DmaDescriptor {
    /// Creates a new [DmaDescriptor].
    pub const fn new() -> Self {
        Self {
            flags: DmaDescriptorFlags::new(),
            buffer: core::ptr::null_mut(),
            next: core::ptr::null_mut(),
        }
    }
}

impl Default for DmaDescriptor {
    fn default() -> Self {
        Self::new()
    }
}
