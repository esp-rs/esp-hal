#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use super::{DynChannelAccess, PulseCode, Rx, RxError};

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum ReaderState {
    Active,

    Error(RxError),

    Done,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) struct RmtReader {
    // The position in channel RAM to continue reading from, i.e. the next code
    // that has not been copied out yet. The position may be invalid if there's no data left.
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

        if final_ && !self.end_is_valid(raw) {
            return false;
        }

        let (count, max_count) = self.copy(data, raw);

        // The panic can never trigger since count <= data.len()!
        data.split_off_mut(..count).unwrap();

        if count < max_count {
            // `data` exhausted
            self.state = ReaderState::Error(RxError::BufferTooSmall);
        } else if final_ {
            // Caller indicated that we're done
            self.state = ReaderState::Done;
        }

        true
    }

    // self.offset -> next code we would read
    // hw_offset -> next code the hardware would write
    // => If both are the same, there is nothing new.
    #[inline(always)]
    fn available(&self, memsize: usize, hw_offset: usize) -> usize {
        let offset = self.offset as usize;
        (if offset <= hw_offset { 0 } else { memsize }) + hw_offset - offset
    }

    // Whether the stopped receiver ended without an overrun. Call it before copying the codes that
    // are left after the receiver stopped.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn end_is_valid(&self, raw: DynChannelAccess<Rx>) -> bool {
        let memsize = raw.memsize().codes();
        let hw_offset = raw.hw_offset();

        // Nothing was received.
        if self.total == 0 && self.available(memsize, hw_offset) == 0 {
            return true;
        }

        // Without rx wrap, the receiver stops when the channel RAM is full, without an end marker.
        // It is not documented whether the write pointer then reads 0 or `memsize`.
        if !property!("rmt.has_rx_wrap") && (hw_offset == 0 || hw_offset == memsize) {
            return true;
        }

        // Otherwise the last code the hardware wrote must contain the end marker.
        let last = if hw_offset == 0 {
            memsize - 1
        } else {
            hw_offset - 1
        };
        // SAFETY: `last < memsize`, so the read stays in this channel's RAM.
        unsafe { raw.channel_ram_start().add(last).read_volatile() }.is_end_marker()
    }

    // Copies the codes up to the hardware write pointer into `data`, as many as fit.
    //
    // Returns the number of copied codes and the number of available codes.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn copy(
        &mut self,
        data: &mut [PulseCode],
        raw: DynChannelAccess<Rx>,
    ) -> (usize, usize) {
        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let offset = self.offset as usize;

        let max_count = self.available(memsize, raw.hw_offset());

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

        // Without rx wrap, the hardware stops at the end of the channel RAM, so the offset does
        // not wrap either. Wrapping it would read the RAM again.
        self.offset = if property!("rmt.has_rx_wrap") {
            (offset + count) % memsize
        } else {
            offset + count
        } as u16;
        self.total += count;

        (count, max_count)
    }
}
