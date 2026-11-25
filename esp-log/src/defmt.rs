//! `defmt` global logger, adapted from `defmt-rtt` with a framing prefix.

use core::cell::UnsafeCell;

use crate::frame::{FrameEncoder, Framer};

impl FrameEncoder for defmt::Encoder {
    fn new() -> Self {
        defmt::Encoder::new()
    }

    fn start_frame(&mut self, write: impl FnMut(&[u8])) {
        defmt::Encoder::start_frame(self, write)
    }

    fn write(&mut self, data: &[u8], write: impl FnMut(&[u8])) {
        defmt::Encoder::write(self, data, write)
    }

    fn end_frame(&mut self, write: impl FnMut(&[u8])) {
        defmt::Encoder::end_frame(self, write)
    }
}

struct GlobalFramer(UnsafeCell<Framer<defmt::Encoder>>);

// Safety: only accessed between `acquire` and `release`, inside a critical section.
unsafe impl Sync for GlobalFramer {}

static FRAMER: GlobalFramer = GlobalFramer(UnsafeCell::new(Framer::new(defmt::Encoder::new())));

fn framer() -> &'static mut Framer<defmt::Encoder> {
    // Safety: `defmt` only calls the logger between `acquire` and `release`, and `acquire` enters a
    // critical section.
    unsafe { &mut *FRAMER.0.get() }
}

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // Safety: `defmt` pairs this with `release`.
        unsafe { framer().acquire() }
    }

    unsafe fn release() {
        // Safety: `defmt` pairs this with `acquire`.
        unsafe { framer().release() }
    }

    unsafe fn flush() {
        framer().flush()
    }

    unsafe fn write(bytes: &[u8]) {
        framer().write(bytes)
    }
}

#[cfg(feature = "timestamp")]
defmt::timestamp!("{=u64:ms}", crate::timestamp());
