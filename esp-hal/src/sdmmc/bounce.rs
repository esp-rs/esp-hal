//! Bounce buffer for cache-unaligned DMA transfers.

use super::*;
use crate::dma::aligned::DmaAlignedRef;

// On SoCs with a data cache the IDMAC buffer must be cache-line aligned in
// both base address and length, otherwise a read invalidate would discard
// neighbouring data and a write would miss CPU-cached bytes. Control
// transfers (SCR = 8 B, SSR / SWITCH = 64 B) we cannot align, so they are
// bounced through an aligned scratch buffer. Two L2 cachelines cover every case.
const BOUNCE_LEN: usize = 256;

// Scratch layout for a bounced, cache-unaligned caller buffer: the unaligned
// head occupies `[0, head_len)` and the unaligned tail
// `[BOUNCE_TAIL_OFF, BOUNCE_TAIL_OFF + tail_len)`. Each edge is shorter than
// the region's DMA alignment (<= 128 B), so the two never overlap and both fit
// within `BOUNCE_LEN`.
const BOUNCE_TAIL_OFF: usize = BOUNCE_LEN / 2;

/// Per-transaction scratch in [`EngineSession`]. On cached SoCs it holds the
/// DMA bounce buffer; the engine mutex provides exclusive access.
pub struct Bounce {
    buf: InternalMemory<[u8; BOUNCE_LEN]>,
}

#[derive(Clone, Copy)]
pub struct Split {
    pub head: usize,
    pub tail: usize,
}
impl Split {
    fn middle(self, len: usize) -> usize {
        len - self.head - self.tail
    }
}

impl Bounce {
    pub const INIT: Self = Bounce {
        buf: InternalMemory::new([0; BOUNCE_LEN]),
    };

    /// Splits a caller buffer into cache-unaligned head/tail lengths.
    pub fn split(buf: &[u8]) -> Option<Split> {
        let addr = buf.as_ptr() as usize;
        let len = buf.len();
        let align = crate::dma::aligned::region_dma_alignment(addr)?;
        let head = ((align - addr % align) % align).min(len);
        let tail = (len - head) % align;

        if head != 0 || tail != 0 {
            debug_assert!(head <= BOUNCE_TAIL_OFF && tail <= BOUNCE_LEN - BOUNCE_TAIL_OFF);

            Some(Split { head, tail })
        } else {
            None
        }
    }

    /// Prepares DMA pointers for an unaligned read (head/tail via scratch).
    pub fn read_dma_setup(&mut self, buf: &[u8], split: Split) -> Result<Transfer, Error> {
        let len = buf.len();
        let middle_len = split.middle(len);

        let scratch = self.buf.get_mut();
        // Must invalidate before handing to DMA, otherwise evicting dirty cachelines can
        // corrupt the buffer contents.
        scratch.invalidate();
        let head_ptr = dma_ptr_from_raw(scratch[..].as_ptr())?;
        let tail_ptr = dma_ptr_from_raw(scratch[BOUNCE_TAIL_OFF..].as_ptr())?;
        let middle_ptr = if middle_len != 0 {
            let middle = &buf[split.head..split.head + middle_len];
            unsafe {
                // Safety: middle comes from Bounce::split which uses DMA requirements to split
                // the buffer.
                DmaAlignedRef::new_unchecked(middle)
            }
            .invalidate();
            dma_ptr_from_raw(middle.as_ptr())?
        } else {
            0
        };

        Ok(Transfer::split(
            (head_ptr, split.head),
            (middle_ptr, middle_len),
            (tail_ptr, split.tail),
        ))
    }

    /// Copies bounced head/tail from scratch into `buf` after a DMA read.
    pub fn read_finish(&mut self, buf: &mut [u8], split: Split) {
        let len = buf.len();
        let scratch = self.buf.get_ref();
        buf[..split.head].copy_from_slice(&scratch[..split.head]);
        buf[len - split.tail..]
            .copy_from_slice(&scratch[BOUNCE_TAIL_OFF..BOUNCE_TAIL_OFF + split.tail]);
    }

    /// Prepares DMA pointers for an unaligned write (head/tail via scratch).
    pub fn write_dma_setup(&mut self, buf: &[u8], split: Split) -> Result<Transfer, Error> {
        let len = buf.len();
        let middle_len = split.middle(len);

        let mut scratch = self.buf.get_mut();
        scratch[..split.head].copy_from_slice(&buf[..split.head]);
        scratch[BOUNCE_TAIL_OFF..BOUNCE_TAIL_OFF + split.tail]
            .copy_from_slice(&buf[len - split.tail..]);
        scratch.writeback();
        let head_ptr = dma_ptr_from_raw(scratch[..].as_ptr())?;
        let tail_ptr = dma_ptr_from_raw(scratch[BOUNCE_TAIL_OFF..].as_ptr())?;
        let middle_ptr = if middle_len != 0 {
            let middle = &buf[split.head..split.head + middle_len];
            unsafe {
                // Safety: middle comes from Bounce::split which uses DMA requirements to split
                // the buffer.
                DmaAlignedRef::new_unchecked(middle)
            }
            .writeback();
            dma_ptr_from_raw(middle.as_ptr())?
        } else {
            0
        };
        Ok(Transfer::split(
            (head_ptr, split.head),
            (middle_ptr, middle_len),
            (tail_ptr, split.tail),
        ))
    }
}
