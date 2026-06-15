//! Helper types for cacheline-aligned DMA buffers.

// ESP32-P4 internal memory is cached, enforce alignment to avoid memory
// corruption. Technically only needed for IN buffers and descriptor lists.
#[cfg_attr(soc_internal_memory_cached, repr(C, align(64)))] // dcache cache line
#[doc(hidden)]
pub struct InternalMemory<T>(T);

/// A shared reference to an [`InternalMemory`] that is known to be cacheline-aligned.
pub struct InternalMemoryRef<'a, T>(&'a T)
where
    T: ?Sized;

impl<'a, T: ?Sized> InternalMemoryRef<'a, T> {
    /// Converts this object into a reference.
    pub fn into_ref(self) -> &'a T {
        self.0
    }
}

/// A mutable reference to an [`InternalMemory`] that is known to be cacheline-aligned.
pub struct InternalMemoryMut<'a, T>(&'a mut T)
where
    T: ?Sized;

impl<'a, T: ?Sized> InternalMemoryMut<'a, T> {
    /// Converts this object into a mutable reference.
    pub fn into_mut(self) -> &'a mut T {
        self.0
    }
}

impl<T> InternalMemory<T> {
    pub const fn new(init: T) -> Self {
        Self(init)
    }

    pub const fn get(&self) -> InternalMemoryRef<'_, T> {
        InternalMemoryRef(&self.0)
    }

    pub const fn get_mut(&mut self) -> InternalMemoryMut<'_, T> {
        InternalMemoryMut(&mut self.0)
    }
}

// ESP32 requires word alignment for DMA buffers.
// ESP32-S2 technically supports byte-aligned DMA buffers, but the
// transfer ends up writing out of bounds.
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

    pub const fn get(&self) -> InternalMemoryRef<'_, [u8]> {
        InternalMemoryRef(&self.0.0)
    }

    pub const fn get_mut(&mut self) -> InternalMemoryMut<'_, [u8]> {
        InternalMemoryMut(&mut self.0.0)
    }
}
