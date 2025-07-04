//! SDIO DMA types for SoCs with a dedicated engine.

use core::{
    fmt::Debug,
    sync::atomic::{AtomicPtr, AtomicU32, Ordering},
};

use super::Owner;

bitfield::bitfield! {
    /// DMA descriptor flags for the ESP32-C6 dedicated SDIO DMA engine.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct DmaDescriptorFlags(u32);

    u16;

    /// Specifies the size of the buffer that this descriptor points to.
    pub size, set_size: 12, 0;

    /// Specifies the number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    pub length, set_length: 27, 13;

    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to 1 after receiving
    /// data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in the data sent to
    /// the corresponding peripheral, indicating to the peripheral that this data segment marks the end of one
    /// transfer phase.
    pub suc_eof, set_suc_eof: 30;

    /// Specifies who is allowed to access the buffer that this descriptor points to.
    /// - 0: CPU can access the buffer;
    /// - 1: The GDMA controller can access the buffer.
    pub owner, set_owner: 31;
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

/// Represents a [DmaDescriptorFlags] in an atomic wrapper.
///
/// ## Note
///
/// Used to store DMA descriptors in an immutable global static.
#[repr(C)]
pub struct AtomicDmaDescriptorFlags {
    flags: AtomicU32,
}

impl AtomicDmaDescriptorFlags {
    /// Creates a new [AtomicDmaDescriptorFlags].
    pub const fn new() -> Self {
        Self {
            flags: AtomicU32::new(0),
        }
    }

    /// Gets the [DmaDescriptorFlags] for the [AtomicDmaDescriptorFlags].
    pub fn descriptor_flags(&self) -> DmaDescriptorFlags {
        DmaDescriptorFlags(self.flags.load(Ordering::Acquire))
    }

    /// Sets the [DmaDescriptorFlags] for the [AtomicDmaDescriptorFlags].
    pub fn set_descriptor_flags(&self, flags: DmaDescriptorFlags) {
        self.flags.store(flags.0, Ordering::Release)
    }

    /// Specifies the size of the buffer that this descriptor points to.
    pub fn size(&self) -> u16 {
        self.descriptor_flags().size()
    }

    /// Sets the specified the size of the buffer that this descriptor points to.
    pub fn set_size(&self, size: u16) {
        let mut flags = self.descriptor_flags();
        flags.set_size(size);
        self.set_descriptor_flags(flags);
    }

    /// Specifies the number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    pub fn length(&self) -> u16 {
        self.descriptor_flags().length()
    }

    /// Sets the specifies the number of valid bytes in the buffer that this descriptor points to.
    pub fn set_length(&self, length: u16) {
        let mut flags = self.descriptor_flags();
        flags.set_length(length);
        self.set_descriptor_flags(flags);
    }

    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to
    /// 1 after receiving data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in
    /// the data sent to the corresponding peripheral, indicating to the peripheral that this
    /// data segment marks the end of one transfer phase.
    pub fn suc_eof(&self) -> bool {
        self.descriptor_flags().suc_eof()
    }

    /// Sets the descriptor EOF bit.
    pub fn set_suc_eof(&self, eof: bool) {
        let mut flags = self.descriptor_flags();
        flags.set_suc_eof(eof);
        self.set_descriptor_flags(flags);
    }

    /// Specifies who is allowed to access the buffer that this descriptor points to.
    /// - 0: CPU can access the buffer;
    /// - 1: The GDMA controller can access the buffer.
    pub fn owner(&self) -> Owner {
        self.descriptor_flags().owner().into()
    }

    /// Sets who is allowed to access the buffer that this descriptor points to.
    /// - 0: CPU can access the buffer;
    /// - 1: The GDMA controller can access the buffer.
    pub fn set_owner(&self, owner: Owner) {
        let mut flags = self.descriptor_flags();
        flags.set_owner(owner.into());
        self.set_descriptor_flags(flags);
    }
}

impl Default for AtomicDmaDescriptorFlags {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for AtomicDmaDescriptorFlags {
    fn clone(&self) -> Self {
        Self {
            flags: AtomicU32::new(self.flags.load(Ordering::Acquire)),
        }
    }
}

impl PartialEq for AtomicDmaDescriptorFlags {
    fn eq(&self, rhs: &Self) -> bool {
        self.descriptor_flags() == rhs.descriptor_flags()
    }
}

impl Debug for AtomicDmaDescriptorFlags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let flags = self.descriptor_flags();

        f.debug_struct("DmaDescriptorFlags")
            .field("size", &flags.size())
            .field("length", &flags.length())
            .field("suc_eof", &flags.suc_eof())
            .field("owner", &(if flags.owner() { "DMA" } else { "CPU" }))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AtomicDmaDescriptorFlags {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        let flags = self.descriptor_flags();

        defmt::write!(
            fmt,
            "DmaDescriptorFlags {{ size: {}, length: {}, suc_eof: {}, owner: {} }}",
            flags.size(),
            flags.length(),
            flags.suc_eof(),
            if flags.owner() { "DMA" } else { "CPU" }
        );
    }
}

/// A DMA transfer descriptor.
#[repr(C)]
#[derive(Debug)]
pub struct DmaDescriptor {
    /// Descriptor flags.
    pub flags: AtomicDmaDescriptorFlags,

    /// Address of the buffer.
    pub buffer: AtomicPtr<u8>,

    /// Address of the next descriptor.
    /// If the current descriptor is the last one, this value is 0.
    /// This field can only point to internal RAM.
    pub next: AtomicPtr<DmaDescriptor>,
}

impl Clone for DmaDescriptor {
    fn clone(&self) -> Self {
        Self {
            flags: self.flags.clone(),
            buffer: AtomicPtr::new(self.buffer.load(Ordering::Acquire)),
            next: AtomicPtr::new(self.next.load(Ordering::Acquire)),
        }
    }
}

impl DmaDescriptor {
    /// An empty DMA descriptor used to initialize the descriptor list.
    pub const fn empty() -> Self {
        Self {
            flags: AtomicDmaDescriptorFlags::new(),
            buffer: AtomicPtr::new(core::ptr::null_mut()),
            next: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    /// Resets the descriptor for a new receive transfer.
    pub fn reset_for_rx(&self) {
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
    pub fn reset_for_tx(&self, set_eof: bool) {
        // Give ownership to the DMA
        self.set_owner(Owner::Dma);

        // The `suc_eof` bit doesn't affect the transfer itself, but signals when the
        // hardware should trigger an interrupt request.
        self.set_suc_eof(set_eof);
    }

    /// Set the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn set_size(&self, len: usize) {
        self.flags.set_size(len as u16)
    }

    /// Set the length of the descriptor. See [DmaDescriptorFlags::length].
    pub fn set_length(&self, len: usize) {
        self.flags.set_length(len as u16)
    }

    /// Returns the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn size(&self) -> usize {
        self.flags.size() as usize
    }

    /// Returns the length of the descriptor. See [DmaDescriptorFlags::length].
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.flags.length() as usize
    }

    /// Set the suc_eof bit. See [DmaDescriptorFlags::suc_eof].
    pub fn set_suc_eof(&self, suc_eof: bool) {
        self.flags.set_suc_eof(suc_eof)
    }

    /// Returns the suc_eof bit. See [DmaDescriptorFlags::suc_eof].
    pub fn suc_eof(&self) -> bool {
        self.flags.suc_eof()
    }

    /// Set the owner. See [DmaDescriptorFlags::owner].
    pub fn set_owner(&self, owner: Owner) {
        self.flags.set_owner(owner)
    }

    /// Returns the owner. See [DmaDescriptorFlags::owner].
    pub fn owner(&self) -> Owner {
        self.flags.owner()
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
            self.flags.descriptor_flags(),
            self.buffer.load(Ordering::Acquire),
            self.next.load(Ordering::Acquire),
        );
    }
}

/// Turns the potentially oversized static `u32`` array reference into a
/// correctly sized `u8` one
#[doc(hidden)]
#[macro_export]
macro_rules! sdio_as_mut_byte_array {
    ($name:expr, $size:expr) => {
        const _: () = assert!($size < $name.len(), "invalid SDIO DMA buffer size");
        unsafe { &mut $name.as_slice_mut()[..$size] }
    };
}
pub use sdio_as_mut_byte_array as as_mut_byte_array;

/// Declares a SDIO DMA buffer with a specific size, aligned to 4 bytes
#[doc(hidden)]
#[macro_export]
macro_rules! declare_aligned_sdio_dma_buffer {
    ($name:ident, $size:expr) => {
        // ESP32 requires word alignment for DMA buffers.
        // ESP32-S2 technically supports byte-aligned DMA buffers, but the
        // transfer ends up writing out of bounds.
        // if the buffer's length is 2 or 3 (mod 4).
        const SIZE: usize = $size + ($size % 4);
        static mut $name: [u8; SIZE] = [0; SIZE];
    };
}
#[doc(hidden)]
pub use declare_aligned_sdio_dma_buffer as declare_aligned_dma_buffer;

#[doc(hidden)]
#[macro_export]
macro_rules! sdio_dma_descriptors_impl {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        let rx = $crate::dma::sdio::dma_descriptors_impl!(
            $rx_size,
            $chunk_size,
            is_circular = $circular
        );
        let tx = $crate::dma::sdio::dma_descriptors_impl!(
            $tx_size,
            $chunk_size,
            is_circular = $circular
        );
        (rx, tx)
    }};

    ($size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        const COUNT: usize =
            $crate::dma::dma_descriptor_count!($size, $chunk_size, is_circular = $circular);
        const EMPTY: $crate::dma::sdio::DmaDescriptor = $crate::dma::sdio::DmaDescriptor::empty();

        static SDIO_DESCRIPTORS: [$crate::dma::sdio::DmaDescriptor; COUNT] = [EMPTY; COUNT];

        &SDIO_DESCRIPTORS
    }};
}
#[doc(hidden)]
pub use sdio_dma_descriptors_impl as dma_descriptors_impl;

#[doc(hidden)]
#[macro_export]
macro_rules! sdio_dma_buffers_impl {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        let rx =
            $crate::dma::sdio::dma_buffers_impl!($rx_size, $chunk_size, is_circular = $circular);
        let tx =
            $crate::dma::sdio::dma_buffers_impl!($tx_size, $chunk_size, is_circular = $circular);
        (rx.0, rx.1, tx.0, tx.1)
    }};

    ($size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        $crate::dma::sdio::declare_aligned_dma_buffer!(BUFFER, $size);

        (
            $crate::dma::sdio::as_mut_byte_array!(BUFFER, $size),
            $crate::dma::sdio::dma_descriptors_impl!($size, $chunk_size, is_circular = $circular),
        )
    }};

    ($size:expr, is_circular = $circular:tt) => {
        $crate::dma::sdio::dma_buffers_impl!(
            $size,
            $crate::dma::BurstConfig::DEFAULT.max_compatible_chunk_size(),
            is_circular = $circular
        );
    };
}
#[doc(hidden)]
pub use sdio_dma_buffers_impl as dma_buffers_impl;

/// Convenience macro to create DMA buffers and descriptors with specific chunk
/// size.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_buffers_chunk_size;
///
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX
/// // and RX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_buffers_chunk_size!(32000, 32000, 4032);
/// # Ok(())
/// # }
/// ```
#[macro_export]
macro_rules! sdio_dma_buffers_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{ $crate::dma_buffers_impl!($rx_size, $tx_size, $chunk_size, is_circular = false) }};

    ($size:expr, $chunk_size:expr) => {
        $crate::dma_buffers_chunk_size!($size, $size, $chunk_size)
    };
}
#[doc(hidden)]
pub use sdio_dma_buffers_chunk_size as dma_buffers_chunk_size;

/// Convenience macro to create circular DMA buffers and descriptors with
/// specific chunk size.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_circular_buffers_chunk_size;
///
/// // RX and TX buffers are 32000 bytes - passing only one parameter makes RX
/// // and TX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_circular_buffers_chunk_size!(32000, 32000, 4032);
/// # Ok(())
/// # }
/// ```
#[macro_export]
macro_rules! sdio_dma_circular_buffers_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{ $crate::dma::sdio::dma_buffers_impl!($rx_size, $tx_size, $chunk_size, is_circular = true) }};

    ($size:expr, $chunk_size:expr) => {{ $crate::dma::sdio::dma_circular_buffers_chunk_size!($size, $size, $chunk_size) }};
}
#[doc(hidden)]
pub use sdio_dma_circular_buffers_chunk_size as dma_circular_buffers_chunk_size;
