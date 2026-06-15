//! Helper types for cacheline-aligned DMA buffers.

use core::ops::{Deref, DerefMut};

use crate::{
    dma::{DmaAlignmentError, DmaBufError},
    soc::is_valid_ram_address,
};

/// A cacheline-aligned type wrapper.
///
/// This type is guaranteed to be safely useable as a DMA buffer or
/// descriptor array.
///
/// [`InternalMemoryMut`] is a reference type that carries this guarantee.
// ESP32-P4 internal memory is cached, enforce alignment to avoid memory
// corruption. Technically only needed for IN buffers and descriptor lists.
#[cfg_attr(soc_internal_memory_cached, repr(C, align(64)))] // dcache cache line
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub struct InternalMemory<T>(T);

impl<T> InternalMemory<T> {
    pub const fn new(init: T) -> Self {
        Self(init)
    }

    pub const fn get_mut(&mut self) -> InternalMemoryMut<'_, T> {
        InternalMemoryMut(&mut self.0)
    }
}

/// A cacheline-aligned byte buffer.
///
/// This type is guaranteed to be safely useable as a DMA buffer.
///
/// [`InternalMemoryMut`] is a reference type that carries this guarantee.
// ESP32 requires word alignment for DMA buffers.
// ESP32-S2 technically supports byte-aligned DMA buffers, but the
// transfer ends up writing out of bounds.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(not(soc_internal_memory_cached), repr(C, align(4)))]
#[doc(hidden)]
pub struct InternalMemoryBuffer<const N: usize>(InternalMemory<[u8; N]>);

impl<const N: usize> Default for InternalMemoryBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> InternalMemoryBuffer<N> {
    pub const fn new() -> Self {
        Self(InternalMemory::new([0u8; N]))
    }

    pub const fn get_mut(&mut self) -> InternalMemoryMut<'_, [u8]> {
        InternalMemoryMut(&mut self.0.0)
    }
}

/// A mutable reference to an [`InternalMemory`] object.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct InternalMemoryMut<'a, T>(&'a mut T)
where
    T: ?Sized;

impl<'a, T: ?Sized> InternalMemoryMut<'a, T> {
    /// Creates a new [`InternalMemoryMut`] from a mutable reference, if it's
    /// provably compatible.
    pub fn new(ptr: &'a mut T) -> Result<Self, DmaBufError> {
        let alignment = if cfg!(soc_internal_memory_cached) {
            64 // FIXME un-magic this
        } else {
            4
        };
        let addr = ptr as *mut T as *mut () as usize;

        if !is_valid_ram_address(addr) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }
        if !core::mem::size_of_val(ptr).is_multiple_of(alignment) {
            return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Size));
        }
        if !addr.is_multiple_of(alignment) {
            return Err(DmaBufError::InvalidAlignment(DmaAlignmentError::Address));
        }

        Ok(Self(ptr))
    }

    /// Creates a new [`InternalMemoryMut`] from *any* mutable reference.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the reference is properly aligned for [`InternalMemory`] and
    /// the cachelines occupied by the reference do not overlap with any other data that can be
    /// corrupted by a cacheline invalidation operation.
    pub const unsafe fn new_unchecked(ptr: &'a mut T) -> Self {
        Self(ptr)
    }
}

impl<'a, T: ?Sized> InternalMemoryMut<'a, T> {
    /// Converts this object into a mutable reference.
    pub fn into_mut(self) -> &'a mut T {
        self.0
    }
}

impl<'a, T, const N: usize> InternalMemoryMut<'a, [T; N]> {
    /// Converts this object into a slice reference.
    pub fn unsize(self) -> InternalMemoryMut<'a, [T]> {
        InternalMemoryMut(self.0)
    }
}

impl<'a, T: ?Sized> Deref for InternalMemoryMut<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl<'a, T: ?Sized> DerefMut for InternalMemoryMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0
    }
}
