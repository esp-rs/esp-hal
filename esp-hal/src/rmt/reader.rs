use super::{DynChannelAccess, Error, MemSize, PulseCode, Rx};

#[derive(Debug, PartialEq)]
pub(crate) enum ReaderState {
    Active,

    Error(Error),

    Done,
}

pub(crate) struct RmtReader {
    memsize: MemSize,

    // The position in channel RAM to continue reading from; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    pub state: ReaderState,
}

impl RmtReader {
    pub(crate) fn new(memsize: MemSize) -> Self {
        Self {
            memsize,
            offset: 0,
            state: ReaderState::Active,
        }
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(crate) fn read<T>(&mut self, data: &mut &mut [T], raw: DynChannelAccess<Rx>, final_: bool)
    where
        T: From<PulseCode>,
    {
        if self.state != ReaderState::Active {
            return;
        }

        let memsize = self.memsize.codes();
        let max_count = if final_ { memsize } else { memsize / 2 };
        let count = data.len().min(max_count);
        let mut count0 = count.min(memsize - self.offset as usize);
        let mut count1 = count - count0;

        // Read in up to 2 chunks to allow wrapping around the buffer end
        loop {
            let ptr = unsafe { raw.channel_ram_start().add(self.offset as usize) };
            for (idx, entry) in data.iter_mut().take(count0).enumerate() {
                *entry = unsafe { ptr.add(idx).read_volatile() }.into();
            }

            if count1 == 0 {
                break;
            }

            count0 = count1;
            count1 = 0
        }

        // If count == memsize / 2 codes were read, update offset as
        // - 0 -> memsize / 2
        // - memsize / 2 -> 0
        // Otherwise, for count != memsize / 2, the new position is invalid but the new slice is
        // empty and we won't use the offset again.
        self.offset = (memsize / 2) as u16 - self.offset;
        data.split_off_mut(..count).unwrap();

        if max_count > count {
            self.state = ReaderState::Error(Error::Overflow);
        } else if final_ {
            self.state = ReaderState::Done;
        }

        debug_assert!(self.offset == 0 || self.offset as usize == memsize / 2 || data.is_empty());
    }
}
