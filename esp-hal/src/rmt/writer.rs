use super::{DynChannelAccess, PulseCode, Tx};

#[derive(PartialEq)]
pub(crate) enum WriterState {
    Active,

    Done,
}

pub(crate) struct RmtWriter {
    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    pub state: WriterState,
}

impl RmtWriter {
    pub(crate) fn new() -> Self {
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
    pub(crate) fn write<T>(&mut self, data: &mut &[T], raw: DynChannelAccess<Tx>, initial: bool)
    where
        T: Into<PulseCode> + Copy,
    {
        if self.state != WriterState::Active {
            return;
        }

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();

        let max_count = if initial { memsize } else { memsize / 2 };
        let count = data.len().min(max_count);

        debug_assert!(!initial || self.offset == 0);

        let mut ptr = unsafe { ram_start.add(self.offset as usize) };
        for entry in data.iter().take(count) {
            // SAFETY: The iteration `count` is smaller than `max_count` such that incrementing the
            // `ptr` `count` times cannot advance further than `ram_start + memsize`.
            unsafe {
                ptr.write_volatile((*entry).into());
                ptr = ptr.add(1);
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
