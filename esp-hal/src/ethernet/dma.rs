//! EMAC DMA descriptor rings.
//!
//! Implements the chained-ring descriptor layout for the Synopsys DesignWare
//! GMAC as found on ESP32 and ESP32-P4. The driver always uses the **enhanced
//! 32-byte descriptor format** (`ALT_DESC_SIZE = 1` in `EMAC_DMA.dmabusmode`).
//!
//! # Cache coherency (ESP32-P4)
//!
//! ESP32-P4's RISC-V core has a write-back L1/L2 data cache. CPU writes to
//! descriptors and packet buffers are not visible to DMA until the affected
//! cache lines are flushed; similarly, DMA writes to RX descriptors and buffers
//! are not visible to the CPU until the stale cache lines are invalidated.
//!
//! On P4 each descriptor is padded to 64 bytes (`align(64)`) so that no two
//! descriptors share a cache line and a writeback/invalidate on one cannot
//! corrupt CPU-owned state in an adjacent slot.
//!
//! On ESP32 (no write-back cache) a compiler fence alone is sufficient.

use core::{
    cell::UnsafeCell,
    mem::{align_of, offset_of, size_of},
    sync::atomic::{Ordering, fence},
};

// ── Cache helpers ────────────────────────────────────────────────────────────
//
// On ESP32-P4 these call the ROM cache-maintenance functions to make CPU
// writes visible to DMA (writeback) and to discard stale cached data before
// the CPU reads DMA-written memory (invalidate).
//
// On all other targets (no write-back cache) they compile to nothing.

#[inline(always)]
unsafe fn cache_wb<T>(ptr: *const T) {
    unsafe { cache_wb_buf(ptr.cast::<u8>(), size_of::<T>()) }
}

#[inline(always)]
unsafe fn cache_inv<T>(ptr: *const T) {
    unsafe { cache_inv_buf(ptr.cast::<u8>(), size_of::<T>()) }
}

#[inline(always)]
unsafe fn cache_wb_buf(ptr: *const u8, size: usize) {
    #[cfg(esp32p4)]
    unsafe {
        crate::soc::cache_writeback_addr(ptr as u32, size as u32);
    }
    #[cfg(not(esp32p4))]
    let _ = (ptr, size);
}

#[inline(always)]
unsafe fn cache_inv_buf(ptr: *const u8, size: usize) {
    #[cfg(esp32p4)]
    unsafe {
        crate::soc::cache_invalidate_addr(ptr as u32, size as u32);
    }
    #[cfg(not(esp32p4))]
    let _ = (ptr, size);
}

/// Maximum frame size supported per DMA buffer (1518 + FCS + rounding).
const DEFAULT_MAX_FRAME_SIZE: usize = 1524;
pub const MAX_FRAME_SIZE: usize = if cfg!(esp32p4) {
    // Must be cache line aligned
    usize::next_multiple_of(DEFAULT_MAX_FRAME_SIZE, 64)
} else {
    DEFAULT_MAX_FRAME_SIZE
};
/// Minimum accepted RX frame length.
pub const MIN_RX_FRAME_SIZE: usize = 14;

// ── TDES bits ──────────────────────────────────────────────────────────────

/// TX descriptor ownership bit: 1 = owned by DMA.
pub const TDES0_OWN: u32 = 1 << 31;
/// TX interrupt-on-completion.
pub const TDES0_IC: u32 = 1 << 30;
/// TX last-segment flag.
pub const TDES0_LS: u32 = 1 << 29;
/// TX first-segment flag.
pub const TDES0_FS: u32 = 1 << 28;
/// TX second-address-chained mode (next descriptor pointer in TDES3).
pub const TDES0_CHAINED: u32 = 1 << 20;

// ── RDES bits ──────────────────────────────────────────────────────────────

/// RX descriptor ownership bit: 1 = owned by DMA.
pub const RDES0_OWN: u32 = 1 << 31;
/// RX frame length field shift inside RDES0.
pub const RDES0_FL_SHIFT: u32 = 16;
/// RX frame length field mask inside RDES0.
pub const RDES0_FL_MASK: u32 = 0x3fff << RDES0_FL_SHIFT;
/// RX error-summary bit.
pub const RDES0_ES: u32 = 1 << 15;
/// RX first-segment flag.
pub const RDES0_FS: u32 = 1 << 9;
/// RX last-segment flag.
pub const RDES0_LS: u32 = 1 << 8;

/// RX buffer-1 size field mask in RDES1.
pub const RDES1_BUF1_SIZE_MASK: u32 = 0x1fff;
/// RX second-address-chained bit.
pub const RDES1_CHAINED: u32 = 1 << 14;

// ── VolatileCell ────────────────────────────────────────────────────────────

/// An interior-mutable cell that always performs volatile reads and writes.
///
/// This ensures that the compiler never caches descriptor field values in
/// registers, which is necessary because both the CPU and the DMA engine
/// update descriptor words concurrently.
#[repr(transparent)]
pub struct VolatileCell<T: Copy>(UnsafeCell<T>);

impl<T: Copy> VolatileCell<T> {
    /// Creates a new `VolatileCell` with the given initial value.
    pub const fn new(v: T) -> Self {
        Self(UnsafeCell::new(v))
    }

    /// Reads the cell value using a volatile load.
    #[inline]
    pub fn get(&self) -> T {
        unsafe { self.0.get().read_volatile() }
    }

    /// Writes `v` to the cell using a volatile store.
    #[inline]
    pub fn set(&self, v: T) {
        unsafe { self.0.get().write_volatile(v) }
    }
}

// SAFETY: The descriptor rings are only ever accessed from one execution
// context at a time (guarded at the `Ethernet` driver level).
unsafe impl<T: Copy + Send> Send for VolatileCell<T> {}
unsafe impl<T: Copy + Sync> Sync for VolatileCell<T> {}

// ── Descriptor types ────────────────────────────────────────────────────────

/// Current descriptor owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OwnedBy {
    /// The CPU owns the descriptor.
    Cpu,
    /// The DMA engine owns the descriptor.
    Dma,
}

/// TX DMA descriptor (enhanced 32-byte format, `ALT_DESC_SIZE = 1`).
///
/// Layout matches the Synopsys DesignWare GMAC databook for enhanced mode.
/// Words 4–7 are reserved for hardware use (TX timestamp, etc.).
#[cfg_attr(esp32p4, repr(C, align(64)))]
#[cfg_attr(not(esp32p4), repr(C, align(4)))]
pub struct TDes {
    pub(super) tdes0: VolatileCell<u32>,
    pub(super) tdes1: VolatileCell<u32>,
    pub(super) buf_addr: VolatileCell<u32>,
    pub(super) next_desc: VolatileCell<u32>,
    // Enhanced words — set to zero; hardware may write TX timestamp here.
    _tdes4: VolatileCell<u32>,
    _tdes5: VolatileCell<u32>,
    _tdes6: VolatileCell<u32>,
    _tdes7: VolatileCell<u32>,
}

// Verify size and field offsets at compile time.
// On P4 the struct is padded to 64 bytes (one cache line).
#[cfg(not(esp32p4))]
const _: () = ::core::assert!(size_of::<TDes>() == 32);
#[cfg(esp32p4)]
const _: () = ::core::assert!(size_of::<TDes>() == 64);
#[cfg(not(esp32p4))]
const _: () = ::core::assert!(align_of::<TDes>() >= 4);
#[cfg(esp32p4)]
const _: () = ::core::assert!(align_of::<TDes>() >= 64);

const _: () = ::core::assert!(offset_of!(TDes, tdes0) == 0);
const _: () = ::core::assert!(offset_of!(TDes, tdes1) == 4);
const _: () = ::core::assert!(offset_of!(TDes, buf_addr) == 8);
const _: () = ::core::assert!(offset_of!(TDes, next_desc) == 12);

impl TDes {
    /// Zero-initialized descriptor, suitable for `static` initializers.
    pub const fn new_zeroed() -> Self {
        Self {
            tdes0: VolatileCell::new(0),
            tdes1: VolatileCell::new(0),
            buf_addr: VolatileCell::new(0),
            next_desc: VolatileCell::new(0),
            _tdes4: VolatileCell::new(0),
            _tdes5: VolatileCell::new(0),
            _tdes6: VolatileCell::new(0),
            _tdes7: VolatileCell::new(0),
        }
    }

    /// Current ownership of this descriptor.
    pub fn owned_by(&self) -> OwnedBy {
        if self.tdes0.get() & TDES0_OWN != 0 {
            OwnedBy::Dma
        } else {
            OwnedBy::Cpu
        }
    }

    /// Sets the ownership bit.
    pub fn set_owned_by(&self, owner: OwnedBy) {
        let v = self.tdes0.get();
        self.tdes0.set(match owner {
            OwnedBy::Cpu => v & !TDES0_OWN,
            OwnedBy::Dma => v | TDES0_OWN,
        });
    }

    fn set_chained(&self) {
        self.tdes0.set(self.tdes0.get() | TDES0_CHAINED);
    }

    fn set_len_and_flags(&self, len: usize) {
        self.tdes1.set(len as u32 & RDES1_BUF1_SIZE_MASK);
        self.tdes0
            .set((self.tdes0.get() & TDES0_CHAINED) | TDES0_FS | TDES0_LS | TDES0_IC);
    }

    fn set_buffer_addr(&self, addr: *const u8) {
        self.buf_addr.set(addr as u32);
    }

    fn set_next_desc(&self, addr: *const TDes) {
        self.next_desc.set(addr as u32);
    }
}

/// RX DMA descriptor (enhanced 32-byte format, `ALT_DESC_SIZE = 1`).
///
/// Words 4–7 are reserved for hardware use (RX timestamp, VLAN, etc.).
///
/// On ESP32-P4 the alignment is raised to 64 bytes (one cache line).
#[cfg_attr(esp32p4, repr(C, align(64)))]
#[cfg_attr(not(esp32p4), repr(C, align(4)))]
pub struct RDes {
    pub(super) rdes0: VolatileCell<u32>,
    pub(super) rdes1: VolatileCell<u32>,
    pub(super) buf_addr: VolatileCell<u32>,
    pub(super) next_desc: VolatileCell<u32>,
    _rdes4: VolatileCell<u32>,
    _rdes5: VolatileCell<u32>,
    _rdes6: VolatileCell<u32>,
    _rdes7: VolatileCell<u32>,
}

#[cfg(not(esp32p4))]
const _: () = ::core::assert!(size_of::<RDes>() == 32);
#[cfg(esp32p4)]
const _: () = ::core::assert!(size_of::<RDes>() == 64);
#[cfg(not(esp32p4))]
const _: () = ::core::assert!(align_of::<RDes>() >= 4);
#[cfg(esp32p4)]
const _: () = ::core::assert!(align_of::<RDes>() >= 64);

impl RDes {
    /// Zero-initialized descriptor, suitable for `static` initializers.
    pub const fn new_zeroed() -> Self {
        Self {
            rdes0: VolatileCell::new(0),
            rdes1: VolatileCell::new(0),
            buf_addr: VolatileCell::new(0),
            next_desc: VolatileCell::new(0),
            _rdes4: VolatileCell::new(0),
            _rdes5: VolatileCell::new(0),
            _rdes6: VolatileCell::new(0),
            _rdes7: VolatileCell::new(0),
        }
    }

    /// Current ownership of this descriptor.
    pub fn owned_by(&self) -> OwnedBy {
        if self.rdes0.get() & RDES0_OWN != 0 {
            OwnedBy::Dma
        } else {
            OwnedBy::Cpu
        }
    }

    fn set_owned_by(&self, owner: OwnedBy) {
        let v = self.rdes0.get();
        self.rdes0.set(match owner {
            OwnedBy::Cpu => v & !RDES0_OWN,
            OwnedBy::Dma => v | RDES0_OWN,
        });
    }

    fn is_complete_frame(&self) -> bool {
        let s = self.rdes0.get();
        s & RDES0_FS != 0 && s & RDES0_LS != 0
    }

    fn frame_len(&self) -> usize {
        ((self.rdes0.get() & RDES0_FL_MASK) >> RDES0_FL_SHIFT) as usize
    }

    fn configure_buffer(&self, size: usize) {
        self.rdes1.set(
            (self.rdes1.get() & !RDES1_BUF1_SIZE_MASK)
                | (size as u32 & RDES1_BUF1_SIZE_MASK)
                | RDES1_CHAINED,
        );
    }

    fn set_buffer_addr(&self, addr: *const u8) {
        self.buf_addr.set(addr as u32);
    }

    fn set_next_desc(&self, addr: *const RDes) {
        self.next_desc.set(addr as u32);
    }
}

// ── Static DMA storage ──────────────────────────────────────────────────────

/// Static backing storage for all DMA descriptor rings and packet buffers.
///
/// `TX` is the number of transmit slots; `RX` is the number of receive slots.
/// Pass a mutable reference to [`Ethernet::new`][super::Ethernet::new].
pub struct EthernetDmaStorage<const RX: usize, const TX: usize> {
    pub(super) rx_descs: [RDes; RX],
    pub(super) tx_descs: [TDes; TX],
    pub(super) rx_bufs: [[u8; MAX_FRAME_SIZE]; RX],
    pub(super) tx_bufs: [[u8; MAX_FRAME_SIZE]; TX],
}

impl<const RX: usize, const TX: usize> Default for EthernetDmaStorage<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const RX: usize, const TX: usize> EthernetDmaStorage<RX, TX> {
    /// Creates a zero-initialized storage block, suitable for `static` placement.
    pub const fn new() -> Self {
        Self {
            rx_descs: [const { RDes::new_zeroed() }; RX],
            tx_descs: [const { TDes::new_zeroed() }; TX],
            rx_bufs: [[0u8; MAX_FRAME_SIZE]; RX],
            tx_bufs: [[0u8; MAX_FRAME_SIZE]; TX],
        }
    }
}

// SAFETY: `EthernetDmaStorage` is only accessed through the driver, which
// enforces exclusive access via `&mut` borrows.
unsafe impl<const RX: usize, const TX: usize> Send for EthernetDmaStorage<RX, TX> {}
unsafe impl<const RX: usize, const TX: usize> Sync for EthernetDmaStorage<RX, TX> {}

// ── TX ring ─────────────────────────────────────────────────────────────────

/// TX descriptor ring backed by references into `EthernetDmaStorage`.
pub struct TDesRing<'a> {
    descriptors: &'a mut [TDes],
    buffers: &'a mut [[u8; MAX_FRAME_SIZE]],
    index: usize,
}

impl<'a> TDesRing<'a> {
    /// Creates a new TX ring from the given descriptor and buffer slices.
    pub fn new(descriptors: &'a mut [TDes], buffers: &'a mut [[u8; MAX_FRAME_SIZE]]) -> Self {
        assert!(!descriptors.is_empty());
        assert_eq!(descriptors.len(), buffers.len());

        let mut ring = Self {
            descriptors,
            buffers,
            index: 0,
        };
        ring.reset();
        ring
    }

    /// Rebuilds ring links and returns all descriptors to CPU ownership.
    ///
    /// Call once after `EMAC_DMA` soft-reset completes and before starting TX.
    pub fn reset(&mut self) {
        let n = self.descriptors.len();
        for i in 0..n {
            let desc = &self.descriptors[i];
            desc.tdes0.set(0);
            desc.tdes1.set(0);
            desc.set_chained();
            desc.set_buffer_addr(self.buffers[i].as_ptr());
            let next = &raw const self.descriptors[(i + 1) % n];
            desc.set_next_desc(next);
            desc.set_owned_by(OwnedBy::Cpu);
            unsafe { cache_wb(desc) };
        }
        self.index = 0;
        fence(Ordering::Release);
    }

    /// Returns the address of the first descriptor (used to program `EMAC_DMA.dmatxbaseaddr`).
    pub fn base_ptr(&self) -> *const TDes {
        self.descriptors.as_ptr()
    }

    /// Copies `frame` into the next available TX buffer and hands it to DMA.
    ///
    /// Returns `Err(DescriptorError::RingFull)` if no CPU-owned slot is available
    /// and `Err(DescriptorError::FrameTooLarge)` if the frame exceeds [`MAX_FRAME_SIZE`].
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), TxError> {
        if frame.len() > MAX_FRAME_SIZE {
            return Err(TxError::FrameTooLarge);
        }

        let desc = &self.descriptors[self.index];
        unsafe { cache_inv(desc) };
        fence(Ordering::Acquire);
        if desc.owned_by() != OwnedBy::Cpu {
            return Err(TxError::RingFull);
        }

        let buf = &mut self.buffers[self.index];
        buf[..frame.len()].copy_from_slice(frame);
        desc.set_len_and_flags(frame.len());
        desc.set_owned_by(OwnedBy::Dma);
        unsafe {
            cache_wb_buf(buf.as_ptr(), frame.len());
            cache_wb(desc);
        }
        fence(Ordering::Release);

        self.index = (self.index + 1) % self.descriptors.len();
        Ok(())
    }

    /// Returns `true` if the current slot is CPU-owned (ready to accept a frame).
    pub fn has_capacity(&self) -> bool {
        unsafe { cache_inv(&raw const self.descriptors[self.index]) };
        fence(Ordering::Acquire);
        self.descriptors[self.index].owned_by() == OwnedBy::Cpu
    }

    /// Returns a mutable reference to the current TX DMA buffer if the slot is
    /// CPU-owned, enabling zero-copy frame construction.
    ///
    /// After writing the frame, call [`TDesRing::commit`] to hand it to DMA.
    pub fn available_buf(&mut self) -> Option<&mut [u8; MAX_FRAME_SIZE]> {
        unsafe { cache_inv(&raw const self.descriptors[self.index]) };
        fence(Ordering::Acquire);
        if self.descriptors[self.index].owned_by() == OwnedBy::Cpu {
            Some(&mut self.buffers[self.index])
        } else {
            None
        }
    }

    /// Commits the frame written into the buffer returned by [`TDesRing::available_buf`].
    ///
    /// Sets the frame length, hands the descriptor to DMA, and advances the
    /// ring index.  The caller must trigger a TX poll demand after this call
    /// (see `EmacRegs::demand_tx_poll`).
    pub fn commit(&mut self, len: usize) {
        let desc = &self.descriptors[self.index];
        desc.set_len_and_flags(len);
        desc.set_owned_by(OwnedBy::Dma);
        unsafe {
            cache_wb_buf(self.buffers[self.index].as_ptr(), len);
            cache_wb(desc);
        }
        fence(Ordering::Release);
        self.index = (self.index + 1) % self.descriptors.len();
    }
}

/// Error returned by [`TDesRing::transmit`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TxError {
    /// No CPU-owned descriptor is available right now.
    RingFull,
    /// Frame is larger than `MAX_FRAME_SIZE`.
    FrameTooLarge,
}

// ── RX ring ─────────────────────────────────────────────────────────────────

/// RX descriptor ring backed by references into `EthernetDmaStorage`.
pub struct RDesRing<'a> {
    descriptors: &'a mut [RDes],
    buffers: &'a mut [[u8; MAX_FRAME_SIZE]],
    index: usize,
}

impl<'a> RDesRing<'a> {
    /// Creates a new RX ring from the given descriptor and buffer slices.
    pub fn new(descriptors: &'a mut [RDes], buffers: &'a mut [[u8; MAX_FRAME_SIZE]]) -> Self {
        assert!(!descriptors.is_empty());
        assert_eq!(descriptors.len(), buffers.len());

        let mut ring = Self {
            descriptors,
            buffers,
            index: 0,
        };
        ring.reset();
        ring
    }

    /// Rebuilds ring links and returns all descriptors to DMA ownership.
    ///
    /// Call once after `EMAC_DMA` soft-reset completes and before starting RX.
    pub fn reset(&mut self) {
        let n = self.descriptors.len();
        for i in 0..n {
            unsafe { cache_inv_buf(self.buffers[i].as_ptr(), MAX_FRAME_SIZE) };

            let desc = &self.descriptors[i];
            desc.rdes0.set(0);
            desc.configure_buffer(MAX_FRAME_SIZE);
            desc.set_buffer_addr(self.buffers[i].as_mut_ptr());
            let next = &raw const self.descriptors[(i + 1) % n];
            desc.set_next_desc(next);
            desc.set_owned_by(OwnedBy::Dma);
            unsafe { cache_wb(desc) };
        }
        self.index = 0;
        fence(Ordering::Release);
    }

    /// Returns the address of the first descriptor (used to program `EMAC_DMA.dmarxbaseaddr`).
    pub fn base_ptr(&self) -> *const RDes {
        self.descriptors.as_ptr()
    }

    /// Returns a mutable data slice if a frame is ready.
    ///
    /// Loops past error/incomplete/oversized frames, recycling them back to DMA
    /// automatically. Returns `None` only when no CPU-owned descriptor remains.
    /// Call [`RDesRing::pop`] after the returned slice is no longer needed, to
    /// release the descriptor back to DMA.
    pub fn receive(&mut self) -> Option<&mut [u8]> {
        loop {
            unsafe { cache_inv(&raw const self.descriptors[self.index]) };
            fence(Ordering::Acquire);
            let desc = &self.descriptors[self.index];
            if desc.owned_by() != OwnedBy::Cpu {
                return None;
            }

            // Extract all needed values before the desc borrow ends so
            // that recycle_current() can reborrow self.
            let status = desc.rdes0.get();
            let is_complete = desc.is_complete_frame();
            let frame_len = desc.frame_len();

            if status & RDES0_ES != 0 || !is_complete {
                self.recycle_current();
                continue;
            }

            // Strip the 4-byte FCS the GMAC appends to the frame length.
            let len = frame_len.saturating_sub(4);
            if !(MIN_RX_FRAME_SIZE..=MAX_FRAME_SIZE).contains(&len) {
                self.recycle_current();
                continue;
            }

            unsafe { cache_inv_buf(self.buffers[self.index].as_ptr(), len) };
            fence(Ordering::Acquire);
            return Some(&mut self.buffers[self.index][..len]);
        }
    }

    /// Releases the current RX descriptor back to DMA ownership.
    ///
    /// Must be called after every successful [`RDesRing::receive`] call.
    pub fn pop(&mut self) {
        self.recycle_current();
    }

    /// Returns `true` when the current descriptor has been returned by DMA.
    pub fn has_packet(&self) -> bool {
        unsafe { cache_inv(&raw const self.descriptors[self.index]) };
        fence(Ordering::Acquire);
        self.descriptors[self.index].owned_by() == OwnedBy::Cpu
    }

    fn recycle_current(&mut self) {
        let desc = &self.descriptors[self.index];
        desc.rdes0.set(RDES0_OWN);
        unsafe { cache_wb(desc) };
        fence(Ordering::Release);
        self.index = (self.index + 1) % self.descriptors.len();
    }
}
