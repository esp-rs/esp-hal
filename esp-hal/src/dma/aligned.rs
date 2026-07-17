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

    /// Returns a const pointer to the wrapped value.
    ///
    /// The returned pointer is aligned appropriately for DMA and points at the
    /// same address that the DMA engine will access.
    #[instability::unstable]
    pub const fn as_ptr(&self) -> *const T {
        &raw const self.0
    }

    /// Returns a [`DmaAlignedMut`] to the underlying value.
    ///
    /// # Panics
    ///
    /// Panics if the value is not located at a valid internal RAM address.
    #[instability::unstable]
    pub fn get_mut(&mut self) -> DmaAlignedMut<'_, T> {
        assert!(is_valid_ram_address(&raw const self.0 as usize));
        DmaAlignedMut(&mut self.0)
    }

    /// Returns a [`DmaAlignedRef`] to the underlying value.
    ///
    /// The reference can be used to read the value and to invalidate its cache
    /// lines, but not to mutate or write it back. This allows polling
    /// DMA-written state (e.g. a descriptor ownership bit) through a shared
    /// borrow.
    ///
    /// # Panics
    ///
    /// Panics if the value is not located at a valid internal RAM address.
    #[instability::unstable]
    pub fn get_ref(&self) -> DmaAlignedRef<'_, T> {
        assert!(is_valid_ram_address(&raw const self.0 as usize));
        DmaAlignedRef(&self.0)
    }
}

/// Returns the DMA alignment (applying to both the start address and the
/// length) required by the memory region containing `addr`, or `None` if
/// `addr` is not in a DMA-capable region.
///
/// Both the address and size of a DMA buffer must be a multiple of this value
/// so that cache maintenance on the buffer cannot corrupt neighbouring data.
pub(crate) fn region_dma_alignment(addr: usize) -> Option<usize> {
    if is_valid_ram_address(addr) {
        return Some(core::mem::align_of::<InternalMemory<()>>());
    }

    #[cfg(dma_can_access_psram)]
    if is_valid_psram_address(addr) {
        return Some(cfg_select! {
            // TODO(esp32p4): PSRAM is cached through the L2 cache, whose line size is
            // configurable (64 or 128 bytes) and is not encoded anywhere yet. Assume the
            // larger, always-safe value until the L2 line size is available.
            soc_internal_memory_cached => 128,
            any(esp32, esp32c5, esp32c61) => 32, // TODO: fixed 32-bytes, metadata-ify
            _ => crate::soc::CONFIG_DATA_CACHE_LINE_SIZE,
        });
    }

    None
}

/// Validates that the value at `addr` spanning `size` bytes lives in a
/// DMA-capable memory region and is aligned correctly for that region.
fn validate_dma_alignment(addr: usize, size: usize) -> Result<(), DmaBufError> {
    // Zero-sized values never alias a cache line, so any address is fine.
    if size == 0 {
        return Ok(());
    }

    let Some(alignment) = region_dma_alignment(addr) else {
        return Err(DmaBufError::UnsupportedMemoryRegion);
    };

    if !size.is_multiple_of(alignment) {
        return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Size));
    }
    if !addr.is_multiple_of(alignment) {
        return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Address));
    }

    Ok(())
}

/// Returns `true` if the value at `addr` spanning `size` bytes occupies a
/// cached memory region and therefore needs explicit cache maintenance.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
fn region_needs_cache_op(addr: usize, size: usize) -> bool {
    // Do not do cache operations on zero-sized values.
    if size == 0 {
        return false;
    }

    let mut in_cached_region = false;

    #[cfg(soc_internal_memory_cached)]
    {
        in_cached_region |= is_valid_ram_address(addr);
    }
    #[cfg(dma_can_access_psram)]
    {
        in_cached_region |= is_valid_psram_address(addr);
    }

    in_cached_region
}

/// A mutable reference to an [`InternalMemory`] object.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct DmaAlignedMut<'a, T>(&'a mut T)
where
    T: ?Sized;

impl<'a, T: ?Sized> DmaAlignedMut<'a, T> {
    #[doc_replace("align_req" => {
        cfg(soc_internal_memory_cached) => "64",
        _ => "4"
    })]
    /// Creates a new [`DmaAlignedMut`] from a mutable variable, if it's
    /// provably compatible.
    ///
    /// In internal memory, the address and size of the variable
    /// must be at least __align_req__ byte aligned.
    #[instability::unstable]
    pub fn new(ptr: &'a mut T) -> Result<Self, DmaBufError> {
        let addr = ptr as *mut T as *mut () as usize;
        validate_dma_alignment(addr, core::mem::size_of_val(ptr))?;
        Ok(Self(ptr))
    }

    /// Creates a new [`DmaAlignedMut`] from *any* mutable slice.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the reference is properly aligned for the memory region it
    /// occupies and the cachelines occupied by the reference do not overlap with any other data
    /// that can be corrupted by a cacheline invalidation operation.
    #[instability::unstable]
    pub const unsafe fn new_unchecked(ptr: &'a mut T) -> Self {
        Self(ptr)
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    fn do_cache_op(&self) -> bool {
        region_needs_cache_op(
            self.0 as *const T as *const () as usize,
            core::mem::size_of_val(self.0),
        )
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
    pub fn invalidate(&self) {
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

    /// Converts this object into a mutable reference.
    pub fn into_inner(self) -> &'a mut T {
        self.0
    }

    /// Reborrows this object with a different lifetime.
    pub fn reborrow<'b>(&'b mut self) -> DmaAlignedMut<'b, T> {
        DmaAlignedMut(self.0)
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

/// A shared reference to an [`InternalMemory`] object.
///
/// Unlike [`DmaAlignedMut`], a [`DmaAlignedRef`] only allows *reading* the
/// value and *invalidating* its cache lines (discarding the CPU-cached copy so
/// a subsequent read observes data written by DMA). It can neither mutate the
/// value nor write it back, so it can be obtained from a shared borrow and used
/// to poll DMA-written, interior-mutable state (e.g. a descriptor's ownership
/// bit) without exclusive access.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct DmaAlignedRef<'a, T>(&'a T)
where
    T: ?Sized;

impl<'a, T: ?Sized> DmaAlignedRef<'a, T> {
    #[doc_replace("align_req" => {
        cfg(soc_internal_memory_cached) => "64",
        _ => "4"
    })]
    /// Creates a new [`DmaAlignedRef`] from a shared reference, if it's
    /// provably compatible.
    ///
    /// In internal memory, the address and size of the value
    /// must be at least __align_req__ byte aligned.
    #[instability::unstable]
    pub fn new(ptr: &'a T) -> Result<Self, DmaBufError> {
        let addr = ptr as *const T as *const () as usize;
        validate_dma_alignment(addr, core::mem::size_of_val(ptr))?;
        Ok(Self(ptr))
    }

    /// Creates a new [`DmaAlignedRef`] from *any* shared reference.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the reference is properly aligned for the memory region it
    /// occupies and the cachelines occupied by the reference do not overlap with any other data
    /// that can be corrupted by a cacheline invalidation operation.
    #[instability::unstable]
    pub const unsafe fn new_unchecked(ptr: &'a T) -> Self {
        Self(ptr)
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    fn do_cache_op(&self) -> bool {
        region_needs_cache_op(
            self.0 as *const T as *const () as usize,
            core::mem::size_of_val(self.0),
        )
    }

    /// Invalidates the cache lines for this data.
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    #[instability::unstable]
    pub fn invalidate(&self) {
        if !self.do_cache_op() {
            return;
        }

        // SAFETY: the wrapped value owns the cachelines it occupies, so
        // invalidating them cannot discard CPU writes to neighbouring data.
        unsafe {
            crate::soc::cache_invalidate_addr(
                self.0 as *const T as *const () as u32,
                core::mem::size_of_val(self.0) as u32,
            );
        }
    }

    /// Writes back the cached copy of this data to memory.
    ///
    /// This only flushes the CPU cache to memory (so a DMA engine reading from
    /// memory observes the latest CPU writes); it does not mutate the value, so
    /// it is sound to perform through a shared reference.
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    #[instability::unstable]
    pub fn writeback(&self) {
        if !self.do_cache_op() {
            return;
        }

        // SAFETY: the wrapped value owns the cachelines it occupies, so writing
        // them back cannot affect neighbouring data.
        unsafe {
            crate::soc::cache_writeback_addr(
                self.0 as *const T as *const () as u32,
                core::mem::size_of_val(self.0) as u32,
            );
        }
    }

    /// Converts this object into a shared reference.
    pub fn into_inner(self) -> &'a T {
        self.0
    }
}

impl<'a, T: ?Sized> Deref for DmaAlignedRef<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.0
    }
}
