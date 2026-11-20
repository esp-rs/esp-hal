#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, PulseCode, Tx, TxError};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum WriterState {
    // There's still data left to write
    Active,
    // All data and the final end marker are in channel RAM
    Done,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct RmtWriter {
    // Position of the driver's end marker in channel RAM. The next code goes here.
    offset: u16,

    // Once `Done`: the hardware read position after a stop at the code that ends the data.
    stop_offset: u16,

    state: WriterState,
}

// Copies `src` into the channel RAM ring, starting at `start` and wrapping at `memsize`.
#[inline(always)]
unsafe fn copy_to_ring(ram: *mut PulseCode, memsize: usize, start: usize, src: &[PulseCode]) {
    let first = (memsize - start).min(src.len());
    for (i, code) in src[..first].iter().enumerate() {
        unsafe { ram.add(start + i).write_volatile(*code) };
    }
    for (i, code) in src[first..].iter().enumerate() {
        unsafe { ram.add(i).write_volatile(*code) };
    }
}

impl RmtWriter {
    pub(super) fn new() -> Self {
        Self {
            offset: 0,
            stop_offset: 0,
            state: WriterState::Active,
        }
    }

    #[inline]
    pub(super) fn state(&self) -> WriterState {
        self.state
    }

    // Copies codes from `data` to the channel RAM and advances `data`.
    //
    // An end marker always follows the last written code. If `initial` is set, this fills the
    // RAM except for that end marker. Otherwise, it appends half the RAM. The first new code
    // replaces the old end marker, and it is written last: the hardware reads either the old end
    // marker and stops, or the new code and continues. So a late call ends the transmission
    // cleanly instead of sending old RAM contents.
    //
    // The transmission ends at the first code with a zero length, so nothing after it is copied.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn write(
        &mut self,
        data: &mut &[PulseCode],
        raw: DynChannelAccess<Tx>,
        initial: bool,
    ) {
        if self.state != WriterState::Active {
            return;
        }

        debug_assert!(!initial || self.offset == 0);

        let ram = raw.channel_ram_start();
        let memsize = raw.memsize().codes();

        let max_count = if initial { memsize - 1 } else { memsize / 2 };
        let mut count = data.len().min(max_count);
        let end = data[..count].iter().position(|code| code.is_end_marker());
        if let Some(index) = end {
            count = index + 1;
        }

        let offset = self.offset as usize;
        if count > 0 {
            unsafe {
                copy_to_ring(ram, memsize, (offset + 1) % memsize, &data[1..count]);
                ram.add((offset + count) % memsize)
                    .write_volatile(PulseCode::end_marker());
                ram.add(offset).write_volatile(data[0]);
            }
        }

        let new_offset = (offset + count) % memsize;
        self.offset = new_offset as u16;

        // The panic can never trigger since count <= data.len()!
        data.split_off(..count).unwrap();

        if end.is_some() || data.is_empty() {
            self.state = WriterState::Done;

            // The hardware stops at the last copied code if it ends the data, otherwise at the
            // driver's end marker. After the stop, its read position is one code further.
            let stop = if end.is_some() {
                (new_offset + memsize - 1) % memsize
            } else {
                new_offset
            };
            self.stop_offset = ((stop + 1) % memsize) as u16;
        }
    }

    // Checks, after the end of the transmission, that the hardware stopped at the end of the
    // data and not at an earlier end marker because a refill came too late.
    pub(super) fn finish(&self, raw: DynChannelAccess<Tx>) -> Result<(), TxError> {
        if self.state == WriterState::Done && raw.hw_offset() == self.stop_offset as usize {
            Ok(())
        } else {
            Err(TxError::Underrun)
        }
    }
}
