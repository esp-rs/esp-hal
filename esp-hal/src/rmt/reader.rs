#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, Error, PulseCode, Rx};

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum ReaderState {
    Active,

    Error(Error),

    Done,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct RmtReader {
    // The position in channel RAM to continue reading from; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    offset: u16,

    pub total: usize,

    pub state: ReaderState,
}

impl RmtReader {
    pub(super) fn new() -> Self {
        Self {
            offset: 0,
            total: 0,
            state: ReaderState::Active,
        }
    }

    // Copy from the hardware buffer to `data`, advancing the `data` slice accordingly.
    //
    // If `final_` is set, read a full buffer length, potentially wrapping around. Otherwise, fetch
    // half the buffer's length.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn read(
        &mut self,
        data: &mut &mut [PulseCode],
        raw: DynChannelAccess<Rx>,
        final_: bool,
    ) {
        if self.state != ReaderState::Active {
            return;
        }

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let offset = self.offset as usize;

        let max_count = if final_ {
            let hw_offset = raw.hw_offset();

            // self.offset -> next code we would read
            // hw_offset -> next code the hardware would write
            // => If both are the same, we're done, max_count = 0
            let max_count = (if offset <= hw_offset { 0 } else { memsize }) + hw_offset - offset;

            debug_assert!(
                max_count == 0 && self.total == 0
                    // We always enable wrapping if it is available. If it's unavailable, rx might
                    // stop when the buffer is full, without an end marker present!
                    // (Checking for two value of hw_offset here, because it's not documented what
                    // happens with the pointer in that case.)
                    || !property!("rmt.has_rx_wrap") && (hw_offset == 0 || hw_offset == memsize)
                    || unsafe {
                        raw.channel_ram_start()
                            .add(hw_offset.checked_sub(1).unwrap_or(memsize - 1))
                            .read_volatile()
                    }
                    .is_end_marker()
            );

            max_count
        } else {
            memsize / 2
        };

        let count = data.len().min(max_count);
        let mut count0 = count.min(memsize - offset);
        let mut count1 = count - count0;

        // Read in up to 2 chunks to allow wrapping around the buffer end. This is more efficient
        // than checking in each iteration of the inner loop whether we reached the buffer end.
        let mut ram_ptr = unsafe { ram_start.add(self.offset as usize) };

        let mut data_ptr = data.as_mut_ptr();

        loop {
            let data_end = unsafe { data_ptr.add(count0) };
            while data_ptr < data_end {
                // SAFETY: The iteration count `count0` is smaller than both `max_count` and
                // `data.len()` such that incrementing both pointers cannot advance them beyond
                // their allocation's end.
                unsafe {
                    data_ptr.write(ram_ptr.read_volatile());
                    ram_ptr = ram_ptr.add(1);
                    data_ptr = data_ptr.add(1);
                }
            }

            if count1 == 0 {
                break;
            }

            count0 = count1;
            count1 = 0;
            ram_ptr = ram_start;
        }

        // Update offset as
        //
        // | offset      | new offset  |
        // | ----------- + ----------- |
        // | 0           | memsize / 2 |
        // | memsize / 2 | 0           |
        //
        // If `count < max_count` or if `final_` is set, the new offset will not correspond to
        // where we stopped reading, but the new offset will not be used again since further calls
        // will immediately return due to `self.state != Active`.
        self.offset = (memsize / 2) as u16 - self.offset;
        self.total += count;

        // The panic can never trigger since count <= data.len()!
        data.split_off_mut(..count).unwrap();

        if count < max_count {
            // `data` exhausted
            self.state = ReaderState::Error(Error::ReceiverError);
        } else if final_ {
            // Caller indicated that we're done
            self.state = ReaderState::Done;
        }

        debug_assert!(self.offset == 0 || self.offset as usize == memsize / 2);
    }
}
