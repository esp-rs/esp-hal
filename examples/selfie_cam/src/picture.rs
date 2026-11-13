//! The 1280x720 RGB565 pictures in PSRAM. Each pixel has its high byte first, as the
//! camera writes it and the panel reads it.
//!
//! A fixed set of frame buffers moves between the camera, the gallery and the panel. A
//! [`Frame`] has one owner that fills it. A finished [`Picture`] can be shown and saved at
//! the same time, and its frame goes back to the pool when the last user drops it.

use alloc::sync::Arc;
use core::{alloc::Layout, ops::Range, slice};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_alloc::{HEAP, MemoryCapability};
use esp_hal::peripherals::CACHE;

pub const WIDTH: usize = 1280;
pub const HEIGHT: usize = 720;
pub const PIXELS: usize = WIDTH * HEIGHT;
pub const BYTES: usize = PIXELS * 2;
/// Room after the picture, so that the camera's end of frame lands in a DMA descriptor
/// that has space left.
const SLACK_BYTES: usize = 4096;
pub const BUFFER_BYTES: usize = BYTES + SLACK_BYTES;
/// The camera fills one and has the next one ready, the newest picture waits for the
/// main loop, and the panel shows one while the one before waits to be dropped.
const FRAMES: usize = 5;
/// The DMA writes PSRAM in whole cache lines.
const ALIGNMENT: usize = 64;

static FREE: Channel<CriticalSectionRawMutex, Frame, FRAMES> = Channel::new();

/// A frame buffer with a single owner.
pub struct Frame(&'static mut [u8]);

impl Frame {
    pub fn pixels(&self) -> &[u16] {
        // SAFETY: the buffer is aligned, and longer than the picture.
        unsafe { slice::from_raw_parts(self.0.as_ptr().cast(), PIXELS) }
    }

    pub fn pixels_mut(&mut self) -> &mut [u16] {
        // SAFETY: as in `pixels`.
        unsafe { slice::from_raw_parts_mut(self.0.as_mut_ptr().cast(), PIXELS) }
    }
}

/// Fills the pool.
pub fn init() {
    let layout = Layout::from_size_align(BUFFER_BYTES, ALIGNMENT).unwrap();
    let (mut start, mut end) = (usize::MAX, 0);
    for _ in 0..FRAMES {
        // SAFETY: the layout is not empty.
        let buffer = unsafe { HEAP.alloc_caps(MemoryCapability::External.into(), layout) };
        assert!(!buffer.is_null(), "no PSRAM for the frame buffers");
        start = start.min(buffer as usize);
        end = end.max(buffer as usize + BUFFER_BYTES);
        // SAFETY: the allocation is never freed, and only this frame refers to it.
        let frame = Frame(unsafe { slice::from_raw_parts_mut(buffer, BUFFER_BYTES) });
        give_back(frame);
    }
    prefetch(start..end);
}

/// Makes the data cache load the next lines of `buffers` while the lines before are
/// read. Both cores share the data cache, so while the other core loads a photo, the
/// scanout interrupt is too slow if each of its cache misses waits for PSRAM.
fn prefetch(buffers: Range<usize>) {
    let cache = CACHE::regs();
    cache
        .l1_dcache_autoload_sct0_addr()
        // SAFETY: any address is valid.
        .write(|w| unsafe { w.l1_dcache_autoload_sct0_addr().bits(buffers.start as u32) });
    cache
        .l1_dcache_autoload_sct0_size()
        // SAFETY: any size is valid.
        .write(|w| unsafe { w.l1_dcache_autoload_sct0_size().bits(buffers.len() as u32) });
    // SAFETY: 2 is a valid trigger mode.
    cache.l1_dcache_autoload_ctrl().modify(|_, w| unsafe {
        // Load ascending addresses, after cache misses and hits.
        w.l1_dcache_autoload_order().clear_bit();
        w.l1_dcache_autoload_trigger_mode().bits(2);
        w.l1_dcache_autoload_sct0_ena().set_bit();
        w.l1_dcache_autoload_ena().set_bit()
    });
}

/// Waits for a free frame buffer.
pub async fn take() -> Frame {
    FREE.receive().await
}

pub fn give_back(frame: Frame) {
    // The pool has room for every frame.
    let _ = FREE.try_send(frame);
}

/// A finished picture.
pub struct Picture(Option<Frame>);

impl Picture {
    pub fn new(frame: Frame) -> Arc<Self> {
        Arc::new(Self(Some(frame)))
    }

    pub fn pixels(&self) -> &[u16] {
        self.0.as_ref().unwrap().pixels()
    }
}

impl Drop for Picture {
    fn drop(&mut self) {
        if let Some(frame) = self.0.take() {
            give_back(frame);
        }
    }
}
