use super::*;

#[cfg(feature = "unstable")]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBounceBufferError {
    /// Framebuffer size is not divisible by bounce buffer size,
    /// or bounce buffers are not equal in size, or bounce buffers are empty.
    InvalidBounceBufferSize,
    /// Bounce buffers or descriptors are not in DRAM.
    UnsupportedMemoryRegion,
}

#[cfg(feature = "unstable")]
/// Bounce buffer state for continuous output from a framebuffer.
///
/// Holds two SRAM bounce buffers and their circular DMA descriptor chain.
#[instability::unstable]
pub struct DmaBounceBuffer {
    descriptors: *mut [DmaDescriptor; 2],
    bounce_bufs: [&'static mut [u8]; 2],
    framebuffer: *const u8,
    fb_len: usize,
    pub(crate) bounce_pos: usize,
    pub(crate) expect_eof_count: usize,
    pub(crate) eof_count: usize,
    pub(crate) ping_pong_idx: usize,
}

#[cfg(feature = "unstable")]
impl DmaBounceBuffer {
    /// Create a new bounce buffer state.
    #[instability::unstable]
    pub fn new(
        descriptors: &'static mut [DmaDescriptor; 2],
        framebuffer: &'static mut [u8],
        bounce_buf0: &'static mut [u8],
        bounce_buf1: &'static mut [u8],
    ) -> Result<Self, DmaBounceBufferError> {
        let bb_size = bounce_buf0.len();
        if bb_size == 0
            || bb_size != bounce_buf1.len()
            || !framebuffer.len().is_multiple_of(bb_size)
        {
            return Err(DmaBounceBufferError::InvalidBounceBufferSize);
        }

        if !is_slice_in_dram(bounce_buf0) || !is_slice_in_dram(bounce_buf1) {
            return Err(DmaBounceBufferError::UnsupportedMemoryRegion);
        }
        if !is_slice_in_dram(core::slice::from_ref(&descriptors[0]))
            || !is_slice_in_dram(core::slice::from_ref(&descriptors[1]))
        {
            return Err(DmaBounceBufferError::UnsupportedMemoryRegion);
        }

        let expect_eof_count = framebuffer.len() / bb_size;
        let fb_len = framebuffer.len();
        let fb_ptr = framebuffer.as_ptr();

        let mut this = Self {
            descriptors: descriptors as *mut [DmaDescriptor; 2],
            bounce_bufs: [bounce_buf0, bounce_buf1],
            framebuffer: fb_ptr,
            fb_len,
            bounce_pos: 0,
            expect_eof_count,
            eof_count: 0,
            ping_pong_idx: 0,
        };

        this.setup_descriptors();

        Ok(this)
    }

    fn setup_descriptors(&mut self) {
        let bb_size = self.bounce_bufs[0].len();
        // SAFETY: `self.descriptors` comes from `&'static mut [DmaDescriptor; 2]`
        // in `new()`, remains valid for program lifetime, and is accessed with
        // exclusive access via `&mut self`.
        let descriptors = unsafe { &mut *self.descriptors };

        descriptors[0] = DmaDescriptor::EMPTY;
        descriptors[0].set_size(bb_size);
        descriptors[0].set_length(bb_size);
        descriptors[0].set_suc_eof(true);
        descriptors[0].set_owner(Owner::Dma);
        descriptors[0].buffer = self.bounce_bufs[0].as_mut_ptr();

        descriptors[1] = DmaDescriptor::EMPTY;
        descriptors[1].set_size(bb_size);
        descriptors[1].set_length(bb_size);
        descriptors[1].set_suc_eof(true);
        descriptors[1].set_owner(Owner::Dma);
        descriptors[1].buffer = self.bounce_bufs[1].as_mut_ptr();

        descriptors[0].next = &mut descriptors[1] as *mut DmaDescriptor;
        descriptors[1].next = &mut descriptors[0] as *mut DmaDescriptor;
    }

    pub(crate) fn first_descriptor_ptr(&mut self) -> *mut DmaDescriptor {
        // SAFETY: Same rationale as in `setup_descriptors()`.
        unsafe { (*self.descriptors).as_mut_ptr() }
    }

    pub(crate) fn initial_fill(&mut self) {
        self.fill_bounce_buf(0);
        self.fill_bounce_buf(1);
    }

    pub(crate) fn refill(&mut self) -> bool {
        let idx = self.ping_pong_idx;
        self.fill_bounce_buf(idx);
        self.ping_pong_idx = 1 - self.ping_pong_idx;

        self.eof_count += 1;
        if self.eof_count >= self.expect_eof_count {
            self.eof_count = 0;
            true
        } else {
            false
        }
    }

    /// Swap the active framebuffer pointer and reset position to start of frame.
    pub(crate) fn set_framebuffer(&mut self, fb: *const u8) {
        self.framebuffer = fb;
        self.bounce_pos = 0;
        self.eof_count = 0;
        self.ping_pong_idx = 0;
    }

    fn fill_bounce_buf(&mut self, bounce_buf_idx: usize) {
        let bb_size = self.bounce_bufs[0].len();

        if self.bounce_pos >= self.fb_len {
            self.bounce_pos = 0;
        }

        let end = self.bounce_pos + bb_size;

        // SAFETY: `self.framebuffer` points to a valid framebuffer of length
        // `self.fb_len`. `bounce_pos` always points to a valid chunk boundary and
        // `bb_size` divides `fb_len`, so this range is in-bounds.
        let src =
            unsafe { core::slice::from_raw_parts(self.framebuffer.add(self.bounce_pos), bb_size) };

        self.bounce_bufs[bounce_buf_idx].copy_from_slice(src);
        self.bounce_pos = end;
    }
}

#[cfg(feature = "unstable")]
/// In-progress view into a [`DmaBounceBuffer`] during an active DMA transfer.
///
/// Accessible via `Deref` on transfer types that use [`DmaBounceBuffer`].
/// Provides methods for double-buffered rendering.
#[instability::unstable]
pub struct DmaBounceBufferView {
    inner: DmaBounceBuffer,
    back_buffer: Option<(*mut u8, usize)>,
}

#[cfg(feature = "unstable")]
impl DmaBounceBufferView {
    /// Register a back buffer for double-buffered rendering.
    ///
    /// The back buffer must be the same size as the framebuffer passed to
    /// [`DmaBounceBuffer::new`].
    #[instability::unstable]
    pub fn set_back_buffer(&mut self, buf: &'static mut [u8]) {
        assert!(
            buf.len() == self.inner.fb_len,
            "back buffer length must match front buffer length"
        );
        self.back_buffer = Some((buf.as_mut_ptr(), buf.len()));
    }

    /// Get a mutable reference to the back buffer for drawing.
    ///
    /// Returns `None` if no back buffer was registered via [`Self::set_back_buffer`].
    #[instability::unstable]
    pub fn back_buffer(&mut self) -> Option<&mut [u8]> {
        self.back_buffer.map(|(ptr, len)| {
            // SAFETY: `ptr` comes from `&'static mut [u8]` set via `set_back_buffer()`.
            // Access is exclusive via `&mut self`.
            unsafe { core::slice::from_raw_parts_mut(ptr, len) }
        })
    }

    /// Returns the length of the framebuffer.
    #[instability::unstable]
    pub fn fb_len(&self) -> usize {
        self.inner.fb_len
    }

    /// Called by poll() to refill a completed bounce buffer from the framebuffer.
    /// Returns true when a frame boundary is reached.
    pub(crate) fn refill(&mut self) -> bool {
        self.inner.refill()
    }

    /// Swap the front and back framebuffer pointers at a frame boundary.
    pub(crate) fn swap_framebuffer(&mut self) {
        if let Some((back_ptr, _)) = self.back_buffer.take() {
            let old_front = self.inner.framebuffer;
            let fb_len = self.inner.fb_len;
            self.inner.set_framebuffer(back_ptr as *const u8);
            self.back_buffer = Some((old_front as *mut u8, fb_len));
        }
    }
}

#[cfg(feature = "unstable")]
// SAFETY: `DmaBounceBuffer` contains a raw pointer to descriptor storage derived
// from a `&'static mut [DmaDescriptor; 2]`. Access to internal mutable state is
// synchronized by `&mut self` methods; no global shared mutable state is used.
unsafe impl Send for DmaBounceBuffer {}
#[cfg(feature = "unstable")]
// SAFETY: Same rationale as for `Send`.
unsafe impl Sync for DmaBounceBuffer {}

#[cfg(feature = "unstable")]
// SAFETY: `DmaBounceBufferView` contains `DmaBounceBuffer` (Send/Sync) and a raw
// pointer to a `&'static mut [u8]` back buffer, accessed exclusively via `&mut self`.
unsafe impl Send for DmaBounceBufferView {}
#[cfg(feature = "unstable")]
// SAFETY: Same rationale as for `Send`.
unsafe impl Sync for DmaBounceBufferView {}

#[cfg(feature = "unstable")]
unsafe impl DmaTxBuffer for DmaBounceBuffer {
    type View = DmaBounceBufferView;
    type Final = DmaBounceBuffer;

    fn prepare(&mut self) -> Preparation {
        self.initial_fill();

        Preparation {
            start: self.first_descriptor_ptr(),
            #[cfg(psram_dma)]
            accesses_psram: false,
            direction: TransferDirection::Out,
            burst_transfer: BurstConfig::default(),
            // Circular chain: owner bit is not guaranteed to be set after wraparound.
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> DmaBounceBufferView {
        DmaBounceBufferView {
            inner: self,
            back_buffer: None,
        }
    }

    fn from_view(view: DmaBounceBufferView) -> DmaBounceBuffer {
        // Back-buffer registration is intentionally dropped here.
        view.inner
    }
}
