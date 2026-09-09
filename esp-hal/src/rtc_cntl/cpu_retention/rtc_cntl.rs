//! CPU retention through the RTC_CNTL retention DMA.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr};

use portable_atomic::{AtomicPtr, Ordering};

use crate::rtc_cntl::sleep::LowPower;

const BUFFER_SIZE: usize = property!("sleep.cpu_retention_mem_size");
const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

/// Bytes the DMA descriptor takes at the head of a retention buffer.
pub(crate) const DMA_LINK_SIZE: usize = 16;

/// `repr(align)` takes a literal, so the buffer takes the strictest alignment of every chip. The
/// alignment then needs no check at run time.
const _: () = ::core::assert!(property!("sleep.cpu_retention_mem_align") <= 16);

static INSTALLED: AtomicPtr<CpuRetentionMemory> = AtomicPtr::new(ptr::null_mut());

/// Memory that light sleep uses to retain the CPU domain.
///
/// The retention DMA writes this memory while the CPU is powered down, so the buffer gives no way
/// to read or write its contents.
///
/// Install the buffer with [`LowPower::install_cpu_retention_memory`]. The retention DMA reaches
/// one range of internal RAM only. The [`#[ram(reclaimed)]`][crate::ram] attribute
/// places a static inside that range, in the memory that the bootloader gives back.
#[repr(align(16))]
#[instability::unstable]
pub struct CpuRetentionMemory {
    storage: UnsafeCell<MaybeUninit<[u8; BUFFER_SIZE]>>,
}

// SAFETY: the type is a `MaybeUninit` buffer, and the retention DMA is the only writer.
unsafe impl crate::Uninit for CpuRetentionMemory {}

impl CpuRetentionMemory {
    /// Creates the retention memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            storage: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    fn as_mut_ptr(&self) -> *mut u8 {
        self.storage.get().cast()
    }
}

#[instability::unstable]
impl Default for CpuRetentionMemory {
    fn default() -> Self {
        Self::new()
    }
}

/// The reason why [`LowPower::install_cpu_retention_memory`] refused a buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum CpuRetentionMemoryError {
    /// The buffer is not inside the range that the retention DMA reaches.
    OutOfRange,
    /// The driver holds a retention buffer already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for CpuRetentionMemoryError {}

#[instability::unstable]
impl core::fmt::Display for CpuRetentionMemoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutOfRange => write!(
                f,
                "the buffer is outside the retention memory range (0x{MEM_START:X}..0x{MEM_END:X})"
            ),
            Self::AlreadyInstalled => write!(f, "a retention buffer is installed already"),
        }
    }
}

impl LowPower<'_> {
    #[procmacros::doc_replace]
    /// Gives light sleep the memory that it uses to retain the CPU domain.
    ///
    /// Light sleep powers the CPU domain down only after it has this memory. The memory stays with
    /// the driver for the life of the program, and a second call is an error, because the sleep
    /// path can hold one buffer only.
    ///
    /// # Errors
    ///
    /// Returns [`CpuRetentionMemoryError`] if the driver holds a buffer already, or if the buffer
    /// is not inside the range that the retention DMA reaches. The
    /// [`#[ram(reclaimed)]`][crate::ram] attribute places a static inside that range.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rtc_cntl::{CpuRetentionMemory, sleep::LowPower};
    ///
    /// // A program adds `#[ram(reclaimed)]` here, to place the buffer in the range that the
    /// // retention DMA reaches.
    /// static mut RETENTION: CpuRetentionMemory = CpuRetentionMemory::new();
    ///
    /// let mut lpwr = LowPower::new(peripherals.LPWR);
    /// lpwr.install_cpu_retention_memory(unsafe { &mut *(&raw mut RETENTION) })?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn install_cpu_retention_memory(
        &mut self,
        memory: &'static mut CpuRetentionMemory,
    ) -> Result<(), CpuRetentionMemoryError> {
        let memory = ptr::from_mut(memory);
        let start = memory as usize;

        if start < MEM_START || start + BUFFER_SIZE > MEM_END {
            return Err(CpuRetentionMemoryError::OutOfRange);
        }

        INSTALLED
            .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| CpuRetentionMemoryError::AlreadyInstalled)
    }
}

pub(crate) fn installed_buffer_ptr() -> Option<*mut u8> {
    // SAFETY: `INSTALLED` holds a pointer that `install_cpu_retention_memory` took as a
    // `&'static mut`, so the memory outlives the read.
    let memory = unsafe { INSTALLED.load(Ordering::Acquire).as_ref()? };

    Some(memory.as_mut_ptr())
}

/// Bytes the DMA moves out of a CPU retention buffer, not counting the descriptor.
pub(crate) const fn payload_size() -> usize {
    BUFFER_SIZE - DMA_LINK_SIZE
}

/// RTC_CNTL retention DMA link node. Matches `lldesc_t` in `esp_rom_lldesc.h`.
#[repr(C)]
struct RtcCntlDmaLink {
    word0: u32,
    buf: *mut u8,
    next: u32,
}

/// Writes the descriptor at the head of the buffer and returns the payload that follows it.
///
/// # Safety
///
/// `buffer` must be valid for `DMA_LINK_SIZE + payload_size` bytes, and aligned as the DMA needs.
pub(crate) unsafe fn init_link(buffer: *mut u8, payload_size: usize) -> *mut u8 {
    unsafe {
        let link = buffer.cast::<RtcCntlDmaLink>();
        let payload = buffer.add(DMA_LINK_SIZE);
        let units = (payload_size >> 4) as u32;

        // `lldesc_t` first word, from `rom/lldesc.h`: size in bits 0..12, length in 12..24, `eof`
        // at bit 30 for the only node in the list, `owner` at bit 31 for the DMA. Both counts are
        // in 16-byte units, as `rtc_cntl_hal_dma_link_init` writes them.
        let word0 = units | (units << 12) | (1 << 30) | (1 << 31);

        core::ptr::write_volatile(&raw mut (*link).word0, word0);
        core::ptr::write_volatile(&raw mut (*link).buf, payload);
        core::ptr::write_volatile(&raw mut (*link).next, 0);

        payload
    }
}

/// Writes the descriptor and the four configuration words that the CPU frames begin with.
///
/// The chips agree on every word but the last, which `config_word3` supplies.
///
/// # Safety
///
/// `buffer` must be the installed CPU retention buffer.
pub(crate) unsafe fn init_cpu_dma_link(buffer: *mut u8, config_word3: u32) {
    unsafe {
        let cfg = init_link(buffer, payload_size()).cast::<u32>();

        core::ptr::write_volatile(cfg, 0);
        core::ptr::write_volatile(cfg.add(1), 0);
        core::ptr::write_volatile(cfg.add(2), 0);
        core::ptr::write_volatile(cfg.add(3), config_word3);
    }
}
