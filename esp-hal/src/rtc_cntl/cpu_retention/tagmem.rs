//! Cache tag memory retention through the RTC_CNTL retention DMA.

use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    ptr::{self, NonNull},
};

use portable_atomic::{AtomicPtr, Ordering};
use static_cell::ConstStaticCell;

use crate::{
    rtc_cntl::{cpu_retention::DMA_LINK_SIZE, sleep::LowPower},
    soc::{
        CONFIG_DATA_CACHE_LINE_SIZE,
        CONFIG_DATA_CACHE_SIZE,
        CONFIG_DCACHE_ASSOCIATED_WAYS,
        CONFIG_ICACHE_ASSOCIATED_WAYS,
        CONFIG_INSTRUCTION_CACHE_LINE_SIZE,
        CONFIG_INSTRUCTION_CACHE_SIZE,
    },
};

const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

/// Tag block groups the retention hardware moves for one cache.
///
/// esp-idf reads the geometry at run time with `Cache_Get_Mode`, because a bootloader applies it.
/// esp-hal has it as constants and passes them to the ROM itself, so the count is known when the
/// program is built.
const fn tag_block_groups(size: usize, ways: usize, line_size: usize) -> usize {
    let sets = size / ways / line_size;
    sets * (ways >> 2)
}

const ICACHE_GROUPS: usize = tag_block_groups(
    CONFIG_INSTRUCTION_CACHE_SIZE,
    CONFIG_ICACHE_ASSOCIATED_WAYS,
    CONFIG_INSTRUCTION_CACHE_LINE_SIZE,
);
const DCACHE_GROUPS: usize = tag_block_groups(
    CONFIG_DATA_CACHE_SIZE,
    CONFIG_DCACHE_ASSOCIATED_WAYS,
    CONFIG_DATA_CACHE_LINE_SIZE,
);

/// The group count as `RET_ICACHE_SIZE` and `RET_DCACHE_SIZE` take it.
///
/// The largest cache of this chip needs 256 instruction and 512 data groups, one more than the
/// 8-bit and 9-bit fields hold, so the count wraps: zero means the full count, and every other
/// value means itself. esp-idf masks the same way (`sleep_cpu.c:73,93`).
///
/// No document states this encoding, because the technical reference manual omits these registers
/// and the SVD carries no description of them, so it was measured instead. With the default cache
/// both fields read zero, and a light sleep still overwrote every payload byte of a buffer painted
/// beforehand: 9216 bytes, which is the full 768 groups. Zero therefore means the full count.
const ICACHE_SIZE_FIELD: u8 = (ICACHE_GROUPS & 0xff) as u8;
const DCACHE_SIZE_FIELD: u16 = (DCACHE_GROUPS & 0x1ff) as u16;

/// A tag block is 92 bits for the instruction cache and 88 for the data cache, and the DMA aligns
/// both to 96. Three transfers of the 128-bit bus therefore move four blocks.
///
/// This sizes from the true group counts, not from the wrapped register values, because the DMA
/// moves a full cache when the field reads zero. esp-idf reaches the same size through the
/// fallback in `blk_gs` (`sleep_cpu.c:79,103`).
const PAYLOAD_SIZE: usize =
    ((ICACHE_GROUPS.next_multiple_of(4) + DCACHE_GROUPS.next_multiple_of(4)) << 2) * 3;

const BUFFER_SIZE: usize = PAYLOAD_SIZE + DMA_LINK_SIZE;

static INSTALLED: AtomicPtr<u8> = AtomicPtr::new(ptr::null_mut());

/// Memory that light sleep uses to retain the cache tag memory.
///
/// This storage must be placed to `#[ram(reclaimed, unstable(zeroed))]`, then
/// a static mutable reference to [`CacheTagRetentionMemory`] can be taken out of it.
#[instability::unstable]
pub struct CacheTagRetentionStorage {
    inner: ConstStaticCell<CacheTagRetentionMemory>,
}

impl CacheTagRetentionStorage {
    /// Creates storage for the retention memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            inner: ConstStaticCell::new(CacheTagRetentionMemory {
                storage: UnsafeCell::new(MaybeUninit::uninit()),
            }),
        }
    }

    /// Takes the reference to the retention memory.
    ///
    /// Can only be called once.
    #[instability::unstable]
    pub fn take(&'static self) -> &'static mut CacheTagRetentionMemory {
        self.inner.take()
    }
}

#[instability::unstable]
impl Default for CacheTagRetentionStorage {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: ConstStaticCell is a `MaybeUninit` buffer, and an AtomicBool strapped together,
// both are safe to be zero-initialized.
unsafe impl bytemuck::Zeroable for CacheTagRetentionStorage {}

/// Memory that light sleep uses to retain the cache tag memory.
///
/// Retaining the tags lets the wake path skip the cache invalidate, and the misses that follow it.
/// This trades memory for wake latency, and it is not needed for correctness.
///
/// The size follows the cache geometry that the program is built with, so a program that
/// configures a smaller cache needs less memory.
///
/// The retention DMA writes this memory while the CPU is powered down, so the buffer gives no way
/// to read or write its contents.
///
/// Install the buffer with [`LowPower::install_cache_tag_retention_memory`], in addition to the
/// memory that [`LowPower::install_cpu_retention_memory`][install] takes. Retaining the tags alone
/// does nothing: the tags are lost only when the CPU domain powers down.
///
/// [install]: LowPower::install_cpu_retention_memory
#[repr(align(16))]
#[instability::unstable]
pub struct CacheTagRetentionMemory {
    storage: UnsafeCell<MaybeUninit<[u8; BUFFER_SIZE]>>,
}

impl CacheTagRetentionMemory {
    fn as_mut_ptr(&self) -> *mut u8 {
        self.storage.get().cast()
    }
}

/// The reason why [`LowPower::install_cache_tag_retention_memory`] refused a buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum CacheTagRetentionMemoryError {
    /// The buffer is not inside the range that the retention DMA reaches.
    OutOfRange,
    /// The driver holds a tag memory buffer already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for CacheTagRetentionMemoryError {}

#[instability::unstable]
impl core::fmt::Display for CacheTagRetentionMemoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutOfRange => write!(
                f,
                "the buffer is outside the retention memory range (0x{MEM_START:X}..0x{MEM_END:X})"
            ),
            Self::AlreadyInstalled => write!(f, "a tag memory buffer is installed already"),
        }
    }
}

impl LowPower<'_> {
    #[procmacros::doc_replace]
    /// Gives light sleep the memory that it uses to retain the cache tag memory.
    ///
    /// This is an optimisation, and it is separate from
    /// [`install_cpu_retention_memory`][install] because it costs memory and buys wake latency
    /// rather than correctness. Without it, a retained sleep invalidates both caches on the wake
    /// path, which is correct but slower.
    ///
    /// The memory stays with the driver for the life of the program, and a second call is an
    /// error, because the sleep path can hold one buffer only.
    ///
    /// # Errors
    ///
    /// Returns [`CacheTagRetentionMemoryError`] if the driver holds a buffer already, or if the
    /// buffer is not inside the range that the retention DMA reaches. The
    /// [`#[ram(reclaimed)]`][crate::ram] attribute places a static inside that range.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rtc_cntl::{CacheTagRetentionMemory, CpuRetentionMemory, sleep::LowPower};
    ///
    /// // A program adds `#[ram(reclaimed)]` to both statics, to place them in the range that the
    /// // retention DMA reaches.
    /// static mut RETENTION: CpuRetentionMemory = CpuRetentionMemory::new();
    /// static mut TAG_MEMORY: CacheTagRetentionMemory = CacheTagRetentionMemory::new();
    ///
    /// let mut lpwr = LowPower::new(peripherals.LPWR);
    /// lpwr.install_cpu_retention_memory(unsafe { &mut *(&raw mut RETENTION) })?;
    /// lpwr.install_cache_tag_retention_memory(unsafe { &mut *(&raw mut TAG_MEMORY) })?;
    /// # {after_snippet}
    /// ```
    ///
    /// [install]: LowPower::install_cpu_retention_memory
    #[instability::unstable]
    pub fn install_cache_tag_retention_memory(
        &mut self,
        memory: &'static mut CacheTagRetentionMemory,
    ) -> Result<(), CacheTagRetentionMemoryError> {
        let memory = memory.as_mut_ptr();
        let start = memory as usize;

        if start < MEM_START || start + BUFFER_SIZE > MEM_END {
            return Err(CacheTagRetentionMemoryError::OutOfRange);
        }

        INSTALLED
            .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| CacheTagRetentionMemoryError::AlreadyInstalled)
    }
}

/// The installed buffer, and the payload size the DMA descriptor needs.
pub(crate) fn installed_buffer_ptr() -> Option<NonNull<u8>> {
    let memory = INSTALLED.load(Ordering::Acquire);

    NonNull::new(memory)
}

/// Bytes the DMA moves, not counting the descriptor.
pub(crate) const fn payload_size() -> usize {
    PAYLOAD_SIZE
}

/// The values that `RET_ICACHE_SIZE` and `RET_DCACHE_SIZE` take for this cache.
pub(crate) const fn size_fields() -> (u8, u16) {
    (ICACHE_SIZE_FIELD, DCACHE_SIZE_FIELD)
}
