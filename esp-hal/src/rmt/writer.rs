use super::{DynChannelAccess, MemSize, PulseCode, Tx};

pub(crate) struct RmtWriter {
    memsize: MemSize,

    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,
}

impl RmtWriter {
    pub(crate) fn new(memsize: MemSize) -> Self {
        Self { memsize, offset: 0 }
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(crate) fn write<T>(&mut self, data: &mut &[T], raw: DynChannelAccess<Tx>, initial: bool)
    where
        T: Into<PulseCode> + Copy,
    {
        let memsize = self.memsize.codes();
        let max_count = if initial { memsize } else { memsize / 2 };
        let count = data.len().min(max_count);

        debug_assert!(!initial || self.offset == 0);

        let ptr = unsafe { raw.channel_ram_start().add(self.offset as usize) };
        for (idx, entry) in data.iter().take(count).enumerate() {
            unsafe {
                ptr.add(idx).write_volatile((*entry).into());
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
        // offset again.
        self.offset = memsize as u16 - max_count as u16 - self.offset;
        data.split_off(..count).unwrap();
        debug_assert!(self.offset == 0 || self.offset as usize == memsize / 2 || data.is_empty());
    }
}
