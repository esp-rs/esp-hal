#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, Error, MemSize, PulseCode, Rx};

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
    // The position in channel RAM to continue reading from, i.e. the next code
    // that has not been copied out yet. Always smaller than the channel's RAM
    // size. The position may be invalid if there's no data left.
    offset: u16,

    // The channel's RAM size in codes.
    memsize: u16,

    pub total: usize,

    pub state: ReaderState,
}

impl RmtReader {
    pub(super) fn new(memsize: MemSize) -> Self {
        Self {
            offset: 0,
            memsize: memsize.codes() as u16,
            total: 0,
            state: ReaderState::Active,
        }
    }

    // Copy from the hardware buffer to `data`, advancing the `data` slice accordingly.
    //
    // Copies every code up to the hardware write pointer, potentially wrapping around. The write
    // pointer only advances past codes whose both halves are written.
    //
    // If `final_` is set, the receiver has stopped, and the data must end in an end marker.
    //
    // The return value is only valuable if `final_` is set.
    // In that case `true` indicates that the read completed successfully, while `false` indicates
    // that the read was incomplete and an error occurred.
    // This was only observed on ESP32 (and probably ESP32-S2) - other chips indicated an error in
    // that case.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn read(
        &mut self,
        data: &mut &mut [PulseCode],
        raw: DynChannelAccess<Rx>,
        final_: bool,
    ) -> bool {
        if self.state != ReaderState::Active {
            return true;
        }

        let ram_start = raw.channel_ram_start();
        let memsize = self.memsize as usize;
        let offset = self.offset as usize;

        let hw_offset = raw.hw_offset();

        // self.offset -> next code we would read
        // hw_offset -> next code the hardware would write
        // => If both are the same, there is nothing new, max_count = 0
        let max_count = (if offset <= hw_offset { 0 } else { memsize }) + hw_offset - offset;

        if final_
            && !(max_count == 0 && self.total == 0
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
                .is_end_marker())
        {
            return false;
        }

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

        // If `count < max_count` or if `final_` is set, the new offset will not be used again since
        // further calls will immediately return due to `self.state != Active`.
        self.offset = ((offset + count) % memsize) as u16;
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

        true
    }
}
