use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use super::{DmaDescriptor, DmaDescriptorFlags};

/// Represents an atomic DMA decriptor for storage in static variables.
#[repr(C)]
pub struct AtomicDmaDescriptor {
    flags: AtomicU32,
    buffer: AtomicPtr<u8>,
    next: AtomicPtr<Self>,
}

impl AtomicDmaDescriptor {
    const _SIZE_ASSERT: () =
        core::assert!(core::mem::size_of::<Self>() == core::mem::size_of::<DmaDescriptor>());

    /// Creates a new [AtomicDmaDescriptor].
    pub const fn new() -> Self {
        Self {
            flags: AtomicU32::new(0),
            buffer: AtomicPtr::new(core::ptr::null_mut()),
            next: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    /// Gets the DMA descriptor flags.
    pub fn flags(&self) -> DmaDescriptorFlags {
        self.flags.load(Ordering::Acquire).into()
    }

    /// Sets the DMA descriptor flags.
    pub fn set_flags(&self, flags: DmaDescriptorFlags) {
        self.flags.store(flags.into(), Ordering::Release);
    }

    /// Gets the DMA descriptor buffer.
    pub fn buffer(&self) -> *mut u8 {
        self.buffer.load(Ordering::Acquire)
    }

    /// Sets the DMA descriptor buffer.
    pub fn set_buffer(&self, buffer: *mut u8) {
        self.buffer.store(buffer, Ordering::Release);
    }

    /// Gets the next DMA descriptor.
    pub fn next(&self) -> *mut DmaDescriptor {
        self.next.load(Ordering::Acquire) as *mut _
    }

    /// Sets the next DMA descriptor.
    pub fn set_next(&self, next: *mut DmaDescriptor) {
        self.next.store(next as *mut _, Ordering::Release);
    }

    /// Updates the [AtomicDmaDescriptor] from a DMA descriptor.
    pub fn update(&self, desc: DmaDescriptor) {
        self.set_flags(desc.flags);
        self.set_buffer(desc.buffer);
        self.set_next(desc.next);
    }
}

impl From<AtomicDmaDescriptor> for DmaDescriptor {
    fn from(val: AtomicDmaDescriptor) -> Self {
        Self::from(&val)
    }
}

impl From<&AtomicDmaDescriptor> for DmaDescriptor {
    fn from(val: &AtomicDmaDescriptor) -> Self {
        Self {
            flags: val.flags(),
            buffer: val.buffer(),
            next: val.next(),
        }
    }
}

impl From<DmaDescriptor> for AtomicDmaDescriptor {
    fn from(val: DmaDescriptor) -> Self {
        Self {
            flags: AtomicU32::new(val.flags.into()),
            buffer: AtomicPtr::new(val.buffer),
            next: AtomicPtr::new(val.next as *mut _),
        }
    }
}

impl Clone for AtomicDmaDescriptor {
    fn clone(&self) -> Self {
        Self {
            flags: AtomicU32::new(self.flags().into()),
            buffer: AtomicPtr::new(self.buffer()),
            next: AtomicPtr::new(self.next() as *mut _),
        }
    }
}

impl Default for AtomicDmaDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for AtomicDmaDescriptor {
    fn eq(&self, rhs: &Self) -> bool {
        self.flags() == rhs.flags() && self.buffer() == rhs.buffer() && self.next() == rhs.next()
    }
}

impl Eq for AtomicDmaDescriptor {}

impl core::fmt::Debug for AtomicDmaDescriptor {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(f, "{:?}", DmaDescriptor::from(self))
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AtomicDmaDescriptor {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "{}", DmaDescriptor::from(self),);
    }
}

/// Represents a container for [AtomicDmaDescriptors].
///
/// Useful for storing a list of DMA descriptors in static memory.
#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AtomicDmaDescriptors<const N: usize> {
    descriptors: [AtomicDmaDescriptor; N],
}

impl<const N: usize> AtomicDmaDescriptors<N> {
    /// Creates a new [AtomicDmaDescriptors].
    pub const fn new() -> Self {
        // Needed for array initialization because AtomicDmaDescriptor does not impl Copy
        #[allow(clippy::declare_interior_mutable_const)]
        const EMPTY: AtomicDmaDescriptor = AtomicDmaDescriptor::new();

        Self {
            descriptors: [EMPTY; N],
        }
    }

    /// Gets a reference to the list of DMA descriptors.
    pub const fn descriptors(&self) -> &[AtomicDmaDescriptor] {
        &self.descriptors
    }

    /// Gets the address of the inner descriptor list.
    pub fn address(&self) -> u32 {
        self.descriptors.as_ptr() as u32
    }
}

impl<const N: usize> core::ops::Index<usize> for AtomicDmaDescriptors<N> {
    type Output = AtomicDmaDescriptor;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        self.descriptors.index(index)
    }
}

impl<const N: usize> Default for AtomicDmaDescriptors<N> {
    fn default() -> Self {
        Self::new()
    }
}
