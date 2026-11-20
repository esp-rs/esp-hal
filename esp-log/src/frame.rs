//! Framing for the `defmt` global logger, generic over the encoder so it can be tested on the host.

use critical_section::RestoreState;

use crate::{Access, ErasedWrite};

/// Marks the start of a defmt frame. It is not valid UTF-8, so espflash can tell defmt frames
/// apart from other output on the same stream.
pub(crate) const FRAME_START: [u8; 2] = [0xFF, 0x00];

/// Ends a frame that a panic interrupted. rzCOBS frames are zero-terminated.
pub(crate) const FRAME_ABORT: [u8; 1] = [0x00];

pub(crate) trait FrameEncoder {
    fn new() -> Self;
    fn start_frame(&mut self, write: impl FnMut(&[u8]));
    fn write(&mut self, data: &[u8], write: impl FnMut(&[u8]));
    fn end_frame(&mut self, write: impl FnMut(&[u8]));
}

pub(crate) struct Framer<E> {
    encoder: E,
    access: Option<Access>,
    /// Critical-section restore states: `[0]` for a normal frame, `[1]` for a frame that took over
    /// an interrupted one.
    restore: [RestoreState; 2],
}

impl<E: FrameEncoder> Framer<E> {
    pub(crate) const fn new(encoder: E) -> Self {
        Self {
            encoder,
            access: None,
            restore: [RestoreState::invalid(); 2],
        }
    }

    /// Enters a critical section and starts a frame.
    ///
    /// # Safety
    ///
    /// Must be paired with [`Self::release`].
    pub(crate) unsafe fn acquire(&mut self) {
        // Safety: paired with the `release` in `Self::release`.
        let restore = unsafe { critical_section::acquire() };
        // Safety: we are in a critical section.
        let Some(access) = (unsafe { crate::acquire() }) else {
            self.restore[0] = restore;
            return;
        };
        self.restore[access.reentered as usize] = restore;

        let writer = writer_of(&access);
        if access.takeover {
            writer.write_bytes(&FRAME_ABORT);
        }
        if access.reentered {
            // The interrupted frame left the encoder in an unknown state.
            self.encoder = E::new();
        }
        writer.write_bytes(&FRAME_START);
        self.encoder.start_frame(|bytes| writer.write_bytes(bytes));
        self.access = Some(access);
    }

    /// Ends the frame and leaves the critical section.
    ///
    /// # Safety
    ///
    /// Must be paired with [`Self::acquire`].
    pub(crate) unsafe fn release(&mut self) {
        let slot = match self.access.take() {
            Some(access) => {
                let writer = writer_of(&access);
                self.encoder.end_frame(|bytes| writer.write_bytes(bytes));
                writer.flush_bytes();
                // Safety: we are in the critical section of `acquire`.
                unsafe { crate::release(&access) };
                access.reentered as usize
            }
            None => 0,
        };
        // Safety: paired with the `acquire` in `Self::acquire`.
        unsafe { critical_section::release(self.restore[slot]) };
    }

    pub(crate) fn write(&mut self, bytes: &[u8]) {
        if let Some(access) = &self.access {
            let writer = writer_of(access);
            self.encoder.write(bytes, |bytes| writer.write_bytes(bytes));
        }
    }

    pub(crate) fn flush(&mut self) {
        if let Some(access) = &self.access {
            writer_of(access).flush_bytes();
        }
    }
}

fn writer_of(access: &Access) -> &'static mut dyn ErasedWrite {
    // Safety: the writer is `'static`, and only used between `acquire` and `release`.
    unsafe { &mut *access.writer }
}
