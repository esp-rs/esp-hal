#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, Error, PulseCode, Tx};

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum WriterState {
    Active,

    Error(Error),

    Done,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct RmtWriter {
    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    pub state: WriterState,
}

impl RmtWriter {
    pub(super) fn new() -> Self {
        Self {
            offset: 0,
            state: WriterState::Active,
        }
    }

    // Copy from `data` to the hardware buffer, advancing the `data` slice accordingly.
    //
    // If `initial` is set, fill the entire buffer. Otherwise, append half the buffer's length from
    // `data`.
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

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();

        let max_count = if initial { memsize } else { memsize / 2 };
        let count = data.len().min(max_count);

        let mut ram_ptr = unsafe { ram_start.add(self.offset as usize) };

        let mut data_ptr = data.as_ptr();
        let data_end = unsafe { data_ptr.add(count) };

        while data_ptr < data_end {
            // SAFETY: The iteration `count` is smaller than both `max_count` and `data.len()` such
            // that incrementing both pointers cannot advance them beyond their allocation's end.
            unsafe {
                ram_ptr.write_volatile(data_ptr.read());
                ram_ptr = ram_ptr.add(1);
                data_ptr = data_ptr.add(1);
            }
        }

        // If the input data was not exhausted, update offset as
        //
        // | initial | offset      | max_count   | new offset  |
        // | ------- + ----------- + ----------- + ----------- |
        // | true    | 0           | memsize     | 0           |
        // | false   | 0           | memsize / 2 | memsize / 2 |
        // | false   | memsize / 2 | memsize / 2 | 0           |
        //
        // Otherwise, the new position is invalid but the new slice is empty and we won't use the
        // offset again. In either case, the unsigned subtraction will not underflow.
        self.offset = memsize as u16 - max_count as u16 - self.offset;

        // The panic can never trigger since count <= data.len()!
        data.split_off(..count).unwrap();
        if data.is_empty() {
            self.state = WriterState::Done;
        }

        debug_assert!(self.offset == 0 || self.offset as usize == memsize / 2);
    }
}
