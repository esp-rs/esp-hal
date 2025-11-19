use core::sync::atomic::{AtomicU8, Ordering};

/// Represents a byte buffer that can safely be stored in a static variable.
pub struct AtomicBuffer<const N: usize> {
    buffer: [AtomicU8; N],
}

impl<const N: usize> AtomicBuffer<N> {
    /// Creates a new [AtomicBuffer].
    pub const fn new() -> Self {
        // needed for array initialization because AtomicU8 does not impl Copy
        #[allow(clippy::declare_interior_mutable_const)]
        const EMPTY_BYTE: AtomicU8 = AtomicU8::new(0);

        Self {
            buffer: [EMPTY_BYTE; N],
        }
    }

    /// Gets the address of the inner buffer.
    pub fn address(&self) -> u32 {
        self.buffer.as_ptr() as u32
    }

    /// Gets a mutable pointer to the inner buffer.
    ///
    /// This method should be used to provide the buffer to the DMA engine.
    ///
    /// # Safety
    ///
    /// Callers MUST ensure that all other writes to the buffer have completed.
    /// Before reading from the buffer, callers MUST ensure the DMA transfers have completed.
    pub unsafe fn as_ptr_mut(&self) -> *mut u8 {
        self.buffer.as_ptr() as *mut _
    }

    /// Reads bytes from the [AtomicBuffer] into a mutable byte buffer.
    ///
    /// # Notes
    ///
    /// Transfers up to the minimum of length of the two buffers.
    pub fn read(&self, out: &mut [u8]) {
        self.buffer
            .iter()
            .zip(out.iter_mut())
            .for_each(|(lhs, rhs)| {
                *rhs = lhs.load(Ordering::Acquire);
            });
    }

    /// Writes bytes into the [AtomicBuffer] from a byte buffer.
    ///
    /// # Notes
    ///
    /// Transfers up to the minimum of length of the two buffers.
    pub fn write(&self, out: &[u8]) {
        self.buffer.iter().zip(out).for_each(|(lhs, rhs)| {
            lhs.store(*rhs, Ordering::Release);
        });
    }

    /// Transfers bytes from one [AtomicBuffer] into another.
    ///
    /// # Notes
    ///
    /// Transfers up to the minimum of length of the two buffers.
    pub fn transfer<const M: usize>(&self, rhs: &AtomicBuffer<M>) {
        self.buffer
            .iter()
            .zip(rhs.buffer.iter())
            .for_each(|(lhs, rhs)| {
                rhs.store(lhs.load(Ordering::Acquire), Ordering::Release);
            });
    }

    /// Converts the [AtomicBuffer] into a byte array.
    pub fn into_inner(self) -> [u8; N] {
        self.buffer.map(|a| a.load(Ordering::Acquire))
    }

    /// Converts a byte array into an [AtomicBuffer].
    pub fn from_inner(buf: [u8; N]) -> Self {
        Self {
            buffer: buf.map(AtomicU8::new),
        }
    }
}

impl<const N: usize> Default for AtomicBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}
