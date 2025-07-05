//! SDIO DMA types for SoCs with a dedicated engine.

use core::fmt::Debug;

use crate::dma::Owner;

bitfield::bitfield! {
    /// DMA descriptor flags for the dedicated SDIO DMA engine.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct DmaDescriptorFlags(u32);

    u16;

    /// Specifies the size of the buffer that this descriptor points to.
    pub size, set_size: 13, 0;

    /// Specifies the number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    pub length, set_length: 27, 14;

    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to 1 after receiving
    /// data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in the data sent to
    /// the corresponding peripheral, indicating to the peripheral that this data segment marks the end of one
    /// transfer phase.
    pub suc_eof, set_suc_eof: 30;

    /// Specifies who is allowed to access the buffer that this descriptor points to.
    /// - 0: CPU can access the buffer;
    /// - 1: The SDIO DMA controller can access the buffer.
    pub owner, set_owner: 31;
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
            .field("length", &self.length())
            .field("suc_eof", &self.suc_eof())
            .field("owner", &(if self.owner() { "DMA" } else { "CPU" }))
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
            self.length(),
            self.suc_eof(),
            if self.owner() { "DMA" } else { "CPU" }
        );
    }
}

/// A DMA transfer descriptor.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DmaDescriptor {
    /// Descriptor flags.
    pub flags: DmaDescriptorFlags,

    /// Address of the buffer.
    pub buffer: *mut u8,

    /// Address of the next descriptor.
    /// If the current descriptor is the last one, this value is 0.
    /// This field can only point to internal RAM.
    pub next: *mut DmaDescriptor,
}

impl DmaDescriptor {
    /// Creates an empty DMA descriptor used to initialize the descriptor list.
    pub const fn new() -> Self {
        Self {
            flags: DmaDescriptorFlags::new(),
            buffer: core::ptr::null_mut(),
            next: core::ptr::null_mut(),
        }
    }

    /// Resets the descriptor for a new receive transfer.
    pub fn reset_for_rx(&mut self) {
        // Give ownership to the DMA
        self.set_owner(Owner::Dma);

        // Clear this to allow hardware to set it when the peripheral returns an EOF
        // bit.
        self.set_suc_eof(false);

        // Clear this to allow hardware to set it when it's
        // done receiving data for this descriptor.
        self.set_length(0);
    }

    /// Resets the descriptor for a new transmit transfer. See
    /// [DmaDescriptorFlags::suc_eof] for more details on the `set_eof`
    /// parameter.
    pub fn reset_for_tx(&mut self, set_eof: bool) {
        // Give ownership to the DMA
        self.set_owner(Owner::Dma);

        // The `suc_eof` bit doesn't affect the transfer itself, but signals when the
        // hardware should trigger an interrupt request.
        self.set_suc_eof(set_eof);
    }

    /// Returns the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn size(&self) -> usize {
        self.flags.size() as usize
    }

    /// Set the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn set_size(&mut self, len: usize) {
        self.flags.set_size(len as u16)
    }

    /// Returns the length of the descriptor. See [DmaDescriptorFlags::length].
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.flags.length() as usize
    }

    /// Set the length of the descriptor. See [DmaDescriptorFlags::length].
    pub fn set_length(&mut self, len: usize) {
        self.flags.set_length(len as u16)
    }

    /// Returns the suc_eof bit. See [DmaDescriptorFlags::suc_eof].
    pub fn suc_eof(&self) -> bool {
        self.flags.suc_eof()
    }

    /// Set the suc_eof bit. See [DmaDescriptorFlags::suc_eof].
    pub fn set_suc_eof(&mut self, suc_eof: bool) {
        self.flags.set_suc_eof(suc_eof)
    }

    /// Returns the owner. See [DmaDescriptorFlags::owner].
    pub fn owner(&self) -> Owner {
        self.flags.owner().into()
    }

    /// Set the owner. See [DmaDescriptorFlags::owner].
    pub fn set_owner(&mut self, owner: Owner) {
        self.flags.set_owner(owner.into())
    }
}

impl Default for DmaDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

// The pointers in the descriptor can be Sent.
// Marking this Send also allows DmaBuffer implementations to automatically be
// Send (where the compiler sees fit).
unsafe impl Send for DmaDescriptor {}

#[cfg(feature = "defmt")]
impl defmt::Format for DmaDescriptor {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "flags: {}, buffer: {}, next: {}",
            self.flags,
            self.buffer,
            self.next,
        );
    }
}
