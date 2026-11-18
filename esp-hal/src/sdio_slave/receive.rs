// Ownership of one receive descriptor and buffer, independent of registers.

use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    ptr::{self, NonNull},
};

use super::state::Error;

/// One word-aligned, three-word SDIO DMA descriptor (TRM 34.5.5.1).
///
/// Storage must remain in internal RAM for the entire transfer. The DMA writes
/// `owner`, `eof`, and `length` in the first word. No CPU reference to that word
/// is exposed while the DMA owns it.
#[repr(C, align(4))]
#[derive(Debug)]
pub struct Descriptor {
    dw0: UnsafeCell<u32>,
    buffer: u32,
    next: u32,
}

impl Descriptor {
    /// Creates an empty descriptor, not owned by the DMA.
    pub const fn new() -> Self {
        Self {
            dw0: UnsafeCell::new(0),
            buffer: 0,
            next: 0,
        }
    }
}

impl Default for Descriptor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Descriptor {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "Descriptor {{ buffer: {=u32:#x}, next: {=u32:#x} }}",
            self.buffer,
            self.next
        );
    }
}

/// A refused receive arm, including the original descriptor and buffer.
///
/// Recover storage with [`Self::into_parts`] before retrying. Debug and Display
/// expose only the error code, never the buffer contents.
pub struct ArmReceiveError {
    error: Error,
    descriptor: &'static mut Descriptor,
    buffer: &'static mut [u32],
}

impl ArmReceiveError {
    pub(super) fn new(
        error: Error,
        descriptor: &'static mut Descriptor,
        buffer: &'static mut [u32],
    ) -> Self {
        Self {
            error,
            descriptor,
            buffer,
        }
    }

    /// Returns the reason why arming was refused.
    pub fn error(&self) -> Error {
        self.error
    }

    /// Returns the reason and the unchanged, exclusively owned storage.
    pub fn into_parts(self) -> (Error, &'static mut Descriptor, &'static mut [u32]) {
        (self.error, self.descriptor, self.buffer)
    }
}

impl core::fmt::Debug for ArmReceiveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ArmReceiveError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

impl core::fmt::Display for ArmReceiveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(&self.error, f)
    }
}

impl core::error::Error for ArmReceiveError {}

#[cfg(feature = "defmt")]
impl defmt::Format for ArmReceiveError {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "ArmReceiveError {{ error: {:?} }}", self.error);
    }
}

/// Owns the static borrows without retaining references to DMA-owned memory.
pub(super) struct Receive {
    descriptor: NonNull<Descriptor>,
    buffer: NonNull<u32>,
    capacity: usize,
    completed: Option<usize>,
    _borrow: PhantomData<(&'static mut Descriptor, &'static mut [u32])>,
}

impl Receive {
    pub(super) fn new(
        descriptor: &'static mut Descriptor,
        buffer: &'static mut [u32],
    ) -> Result<Self, ArmReceiveError> {
        if buffer.is_empty() || buffer.len() > 0x3ffc / 4 {
            return Err(ArmReceiveError::new(
                Error::InvalidBuffer,
                descriptor,
                buffer,
            ));
        }
        let capacity = buffer.len() * 4;
        let buffer = NonNull::new(buffer.as_mut_ptr()).unwrap();
        let descriptor = NonNull::from(descriptor);
        // SAFETY: exclusive static borrows were consumed above. DMA has not
        // started. No references to these allocations are retained or exposed.
        unsafe {
            ptr::write_volatile(
                descriptor.as_ptr(),
                Descriptor {
                    dw0: UnsafeCell::new((1 << 31) | capacity as u32),
                    buffer: buffer.as_ptr() as usize as u32,
                    next: 0,
                },
            );
        }
        Ok(Self {
            descriptor,
            buffer,
            capacity,
            completed: None,
            _borrow: PhantomData,
        })
    }

    pub(super) fn descriptor_address(&self) -> u32 {
        self.descriptor.as_ptr() as usize as u32
    }

    pub(super) fn descriptor_word(&self) -> u32 {
        // SAFETY: storage is owned for 'static. This raw, aligned access does
        // not form a shared reference to the word modified by the DMA.
        unsafe { ptr::read_volatile(ptr::addr_of!((*self.descriptor.as_ptr()).dw0).cast::<u32>()) }
    }

    /// # Safety
    /// The caller must observe successful EOF and a parked DMA, then perform
    /// the CPU/DMA acquire barrier before calling this method.
    pub(super) unsafe fn finish(&mut self) -> Result<usize, Error> {
        let dw0 = self.descriptor_word();
        let length = ((dw0 >> 14) & 0x3fff) as usize;
        if dw0 & (1 << 31) != 0
            || dw0 & (1 << 30) == 0
            || (dw0 & 0x3fff) as usize != self.capacity
            || length > self.capacity
        {
            return Err(Error::InvalidDescriptor);
        }
        self.completed = Some(length);
        Ok(length)
    }

    pub(super) fn received(&self) -> Option<&[u8]> {
        let length = self.completed?;
        // SAFETY: finish() establishes exclusive CPU ownership and validates
        // the length. No method starts DMA again while this allocation is held.
        Some(unsafe { core::slice::from_raw_parts(self.buffer.as_ptr().cast::<u8>(), length) })
    }

    /// # Safety
    /// DMA must be stopped and its memory accesses ordered before restitution.
    pub(super) unsafe fn into_buffers(self) -> (&'static mut Descriptor, &'static mut [u32]) {
        // SAFETY: the caller establishes quiescence; these are the only borrows
        // of the allocations. Consuming self prevents a second restitution.
        unsafe {
            (
                &mut *self.descriptor.as_ptr(),
                core::slice::from_raw_parts_mut(self.buffer.as_ptr(), self.capacity / 4),
            )
        }
    }
}

/// Corrects the bit position in esp-pacs 61f9678. TRM register 34.19 puts
/// TOKEN_NO_REPLACE at bit 0; the PAC accessor addresses bit 1 instead.
pub(super) const fn token_no_replace(bits: u32) -> u32 {
    bits | 1
}

/// A refused transmit arm. The original allocations remain exclusively owned.
pub type ArmTransmitError = ArmReceiveError;

/// Owns a single outbound packet until EOF and DMA quiescence are confirmed.
pub(super) struct Transmit {
    storage: Receive,
    length: usize,
}

impl Transmit {
    pub(super) fn descriptor_word(&self) -> u32 {
        self.storage.descriptor_word()
    }

    pub(super) fn new(
        descriptor: &'static mut Descriptor,
        buffer: &'static mut [u32],
        length: usize,
    ) -> Result<Self, ArmTransmitError> {
        if length == 0 || length > buffer.len().saturating_mul(4) || length > 0x3ffc {
            return Err(ArmReceiveError::new(
                Error::InvalidBuffer,
                descriptor,
                buffer,
            ));
        }
        let storage = Receive::new(descriptor, buffer)?;
        // SAFETY: storage owns both allocations and DMA has not started.
        unsafe {
            ptr::write_volatile(
                ptr::addr_of_mut!((*storage.descriptor.as_ptr()).dw0).cast::<u32>(),
                (1 << 31) | (1 << 30) | ((length as u32) << 14) | storage.capacity as u32,
            );
        }
        Ok(Self { storage, length })
    }

    pub(super) fn descriptor_address(&self) -> u32 {
        self.storage.descriptor_address()
    }

    /// # Safety
    /// Successful RX EOF, parked DMA and the acquire barrier must precede this call.
    pub(super) unsafe fn finish(&mut self) -> Result<usize, Error> {
        if ((self.storage.descriptor_word() >> 14) & 0x3fff) as usize != self.length {
            return Err(Error::InvalidDescriptor);
        }
        // SAFETY: the caller establishes the same quiescence as Receive::finish.
        unsafe { self.storage.finish() }
    }

    /// # Safety
    /// DMA must be stopped and its accesses ordered before restitution.
    pub(super) unsafe fn into_buffers(self) -> (&'static mut Descriptor, &'static mut [u32]) {
        // SAFETY: inherited from the caller.
        unsafe { self.storage.into_buffers() }
    }
}
