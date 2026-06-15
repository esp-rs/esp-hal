//! Helper types for DMA buffers.

use core::ops::{Deref, DerefMut};

use procmacros::doc_replace;

#[cfg(dma_can_access_psram)]
use crate::soc::is_valid_psram_address;
use crate::{
    dma::{DmaAlignmentError, DmaBufError},
    soc::is_valid_ram_address,
};

/// DMA appropriate wrapper type for internal memory values.
///
/// The value wrapped in this type is guaranteed to be safely useable
/// as a DMA buffer or descriptor array, meaning DMA or cache management
/// operations will not corrupt surrounding data.
///
/// [`DmaAlignedMut`] is a reference type that carries this guarantee.
// ESP32-P4 internal memory is cached, enforce alignment to avoid memory
// corruption. Technically only needed for IN buffers and descriptor lists.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(soc_internal_memory_cached, repr(C, align(64)))] // dcache cache line
#[cfg_attr(not(soc_internal_memory_cached), repr(C, align(4)))] // Worst-case word alignment
#[instability::unstable]
pub struct InternalMemory<T>(T);

impl<T> InternalMemory<T> {
    /// Creates a new value aligned for DMA operations in internal memory.
    #[instability::unstable]
    pub const fn new(init: T) -> Self {
        Self(init)
    }

    /// Returns a [`DmaAlignedMut`] to the underlying value.
    ///
    /// # Panics
    ///
    /// Panics if the value is not located at a valid internal RAM address.
    #[instability::unstable]
    pub fn get_typed_mut(&mut self) -> DmaAlignedMut<'_, T> {
        assert!(is_valid_ram_address(&raw const self.0 as usize));
        DmaAlignedMut(&mut self.0)
    }

    /// Returns a mutable reference to the underlying value.
    #[instability::unstable]
    pub fn get(&self) -> &T {
        &self.0
    }

    /// Returns a mutable reference to the underlying value.
    #[instability::unstable]
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.0
    }
}

/// A mutable reference to an [`InternalMemory`] object.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct DmaAlignedMut<'a, T>(&'a mut T)
where
    T: ?Sized;

impl<'a, T> DmaAlignedMut<'a, [T]> {
    #[doc_replace("align_req" => {
        cfg(soc_internal_memory_cached) => "64",
        _ => "4"
    })]
    /// Creates a new [`DmaAlignedMut`] from a mutable slice, if it's
    /// provably compatible.
    ///
    /// In internal memory, the address and length of the slice
    /// must be __align_req__ byte aligned.
    #[instability::unstable]
    pub fn new_slice(ptr: &'a mut [T]) -> Result<Self, DmaBufError> {
        if ptr.is_empty() {
            return Ok(Self(ptr));
        }

        let addr = ptr.as_ptr() as usize;

        if is_valid_ram_address(addr) {
            let alignment = core::mem::align_of::<InternalMemory<()>>();

            if !core::mem::size_of_val(ptr).is_multiple_of(alignment) {
                return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Size));
            }
            if !addr.is_multiple_of(alignment) {
                return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Address));
            }

            return Ok(Self(ptr));
        }

        #[cfg(dma_can_access_psram)]
        if is_valid_psram_address(addr) {
            let alignment = cfg_select! {
                // TODO(esp32p4): PSRAM is cached through the L2 cache, whose line size is
                // configurable (64 or 128 bytes) and is not encoded anywhere yet. Assume the
                // larger, always-safe value until the L2 line size is available.
                soc_internal_memory_cached => 128,
                any(esp32, esp32c5, esp32c61) => 32, // TODO: fixed 32-bytes, metadata-ify
                _ => crate::soc::CONFIG_DATA_CACHE_LINE_SIZE,
            };

            if !core::mem::size_of_val(ptr).is_multiple_of(alignment) {
                return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Size));
            }
            if !addr.is_multiple_of(alignment) {
                return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Address));
            }

            return Ok(Self(ptr));
        }

        Err(DmaBufError::UnsupportedMemoryRegion)
    }
}

impl<'a, T: ?Sized> DmaAlignedMut<'a, T> {
    /// Creates a new [`DmaAlignedMut`] from *any* mutable slice.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the reference is properly aligned for [`InternalMemory`] and
    /// the cachelines occupied by the reference do not overlap with any other data that can be
    /// corrupted by a cacheline invalidation operation.
    #[instability::unstable]
    pub const unsafe fn new_unchecked(ptr: &'a mut T) -> Self {
        Self(ptr)
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    fn do_cache_op(&self) -> bool {
        // Do not do cache operations on zero-sized values.
        let mut do_cache_op = core::mem::size_of_val(self.0) > 0;

        let address = self.0 as *const T as *const () as usize;

        #[cfg(soc_internal_memory_cached)]
        {
            do_cache_op |= is_valid_ram_address(address);
        }
        #[cfg(dma_can_access_psram)]
        {
            do_cache_op |= is_valid_psram_address(address);
        }

        do_cache_op
    }

    /// Writes back the data from the cache to memory.
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    #[instability::unstable]
    pub fn writeback(&mut self) {
        if !self.do_cache_op() {
            return;
        }

        // SAFETY: we own the cachelines and can't trash anything else
        unsafe {
            crate::soc::cache_writeback_addr(
                self.0 as *const T as *const () as u32,
                core::mem::size_of_val(self.0) as u32,
            );
        }
    }

    /// Invalidates the cache lines for this data.
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    #[instability::unstable]
    pub fn invalidate(&mut self) {
        if !self.do_cache_op() {
            return;
        }

        // SAFETY: we own the cachelines and can't trash anything else
        unsafe {
            crate::soc::cache_invalidate_addr(
                self.0 as *const T as *const () as u32,
                core::mem::size_of_val(self.0) as u32,
            );
        }
    }
}

impl<'a, T: ?Sized> DmaAlignedMut<'a, T> {
    /// Converts this object into a mutable reference.
    pub fn into_mut(self) -> &'a mut T {
        self.0
    }
}

impl<'a, T, const N: usize> DmaAlignedMut<'a, [T; N]> {
    /// Converts this object into a slice reference.
    pub fn unsize(self) -> DmaAlignedMut<'a, [T]> {
        DmaAlignedMut(self.0)
    }
}

impl<'a, T: ?Sized> Deref for DmaAlignedMut<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl<'a, T: ?Sized> DerefMut for DmaAlignedMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0
    }
}
