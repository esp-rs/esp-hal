//! Powers the BLE MAC and baseband down while the controller sleeps.
//!
//! The controller saves its registers to the retention memory before the power-down, and restores
//! them after the power-up.

use core::ptr;

use esp_hal::{
    rtc_cntl::{RetentionMemoryError, RetentionSlot},
    time::{Duration, Instant},
};
use esp_sync::NonReentrantMutex;
use portable_atomic::{AtomicU32, Ordering};

use crate::{
    modem_power::{self, Radio},
    sys::c_types::c_void,
};

unsafe extern "C" {
    fn btdm_deep_sleep_mem_init() -> bool;
    fn btdm_deep_sleep_mem_deinit();
    fn btdm_ble_power_down_dma_copy(copy: bool);
}

// `btdm_deep_sleep_mem_init` allocates three buffers of 1100, 300 and 180 bytes, and
// `btdm_deep_sleep_mem_deinit` frees them. If a blob needs more, `btdm_deep_sleep_mem_init` fails,
// and the MAC and baseband stay powered.
const BUFFER_SIZE: usize = 1100 + 300 + 180;

/// The shortest sleep that gets a power-down.
///
/// A shorter sleep sometimes wakes the chip milliseconds late, and the controller then wakes
/// before the restore.
const MIN_WINDOW: Duration = Duration::from_millis(10);

/// The time that the chip needs to wake and restore the MAC and baseband.
const WAKE_TIME: Duration = Duration::from_micros(1500);

/// The controller can power the MAC and baseband down.
const ENABLED: u32 = 1 << 0;
/// The MAC and baseband have no power.
const POWERED_DOWN: u32 = 1 << 1;
/// The controller fell asleep after the last power-down.
const ARMED: u32 = 1 << 2;

static STATE: AtomicU32 = AtomicU32::new(0);

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type RetentionStorage = esp_hal::rtc_cntl::RetentionStorage<BUFFER_SIZE>;

/// Memory that the BLE controller uses to retain the MAC and baseband while they have no power.
///
/// Install the memory with [`install_retention_memory`].
#[instability::unstable]
pub type RetentionMemory = esp_hal::rtc_cntl::RetentionMemory<BUFFER_SIZE>;

static MEMORY: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();

struct Allocations {
    /// The offset of the first free byte in the retention memory.
    next_free: usize,
    /// The number of buffers in the retention memory that the blob did not free yet.
    live: usize,
}

static ALLOCATIONS: NonReentrantMutex<Allocations> = NonReentrantMutex::new(Allocations {
    next_free: 0,
    live: 0,
});

#[procmacros::doc_replace]
/// Gives the BLE controller the memory that it uses to retain the MAC and baseband.
/// The memory stays with the controller for the life of the program.
///
/// With modem sleep on, the controller then powers the MAC and baseband down while it sleeps.
/// The power-down also takes 768 bytes of internal heap for the PHY state. Install the memory
/// before the controller starts.
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::ble::{RetentionStorage, install_retention_memory};
///
/// #[esp_hal::ram(reclaimed, unstable(zeroed))]
/// static RETENTION: RetentionStorage = RetentionStorage::new();
///
/// install_retention_memory(RETENTION.take())?;
/// # {after_snippet}
/// ```
///
/// # Errors
///
/// Returns [`RetentionMemoryError`] if the controller has retention memory already, or if the
/// memory is outside the range that the backup DMA can reach.
#[instability::unstable]
pub fn install_retention_memory(
    memory: &'static mut RetentionMemory,
) -> Result<(), RetentionMemoryError> {
    MEMORY.install(memory)
}

/// Allocates a buffer from the retention memory.
///
/// This bump allocator starts from the start of the memory again only after the blob frees every
/// buffer, so a new buffer never overlaps a buffer in use. Returns null if the memory has no space
/// left.
pub(super) extern "C" fn malloc_retention(size: u32) -> *mut c_void {
    let Some(memory) = MEMORY.get() else {
        return ptr::null_mut();
    };
    let size = (size as usize).next_multiple_of(4);

    ALLOCATIONS.with(|allocations| {
        if allocations.live == 0 {
            allocations.next_free = 0;
        }
        if allocations.next_free + size > BUFFER_SIZE {
            return ptr::null_mut();
        }

        let buffer = memory.as_ptr().wrapping_add(allocations.next_free);
        allocations.next_free += size;
        allocations.live += 1;
        buffer.cast()
    })
}

/// Frees `ptr` if it is in the retention memory. Returns whether it is.
pub(super) fn try_free(ptr: *mut c_void) -> bool {
    let in_memory = MEMORY.get().is_some_and(|memory| {
        let start = memory.as_ptr() as usize;
        (start..start + BUFFER_SIZE).contains(&(ptr as usize))
    });
    if in_memory {
        ALLOCATIONS.with(|allocations| allocations.live -= 1);
    }
    in_memory
}

/// Records that the controller is up.
///
/// With `power_down`, the controller can power the MAC and baseband down, if retention memory is
/// installed. Call this before `btdm_controller_init`.
pub(crate) fn init(power_down: bool) {
    modem_power::start(Radio::Ble);
    if !power_down || MEMORY.get().is_none() {
        return;
    }

    if !unsafe { btdm_deep_sleep_mem_init() } {
        warn!("The retention memory is too small, the BLE MAC and baseband stay powered in sleep");
        return;
    }

    if !modem_power::enable_power_down(Radio::Ble) {
        unsafe { btdm_deep_sleep_mem_deinit() };
        return;
    }

    STATE.store(ENABLED, Ordering::Release);
}

/// Releases what [`init`] took.
///
/// Call this after `btdm_controller_deinit`.
pub(crate) fn deinit() {
    if STATE.swap(0, Ordering::AcqRel) & ENABLED != 0 {
        unsafe { btdm_deep_sleep_mem_deinit() };
    }
    modem_power::stop(Radio::Ble);
}

/// Lets the next chip sleep power the MAC and baseband down.
///
/// Call this when the controller falls asleep. The controller does not wake after a second
/// power-down in one of its sleeps.
pub(crate) fn controller_sleeps() {
    STATE.fetch_or(ARMED, Ordering::AcqRel);
}

/// Saves the MAC and baseband, then powers them down.
///
/// `remaining` is the time until the controller wakes. Returns how long the chip can sleep, so
/// that the MAC and baseband are restored before the controller wakes.
///
/// Call this only while the controller sleeps.
pub(crate) fn power_down(remaining: Duration) -> Duration {
    if remaining < MIN_WINDOW {
        return remaining;
    }

    let power_down = STATE.fetch_update(Ordering::AcqRel, Ordering::Acquire, |state| {
        (state & (ENABLED | POWERED_DOWN | ARMED) == ENABLED | ARMED)
            .then_some((state | POWERED_DOWN) & !ARMED)
    });
    if power_down.is_err() {
        return remaining;
    }

    let start = Instant::now();
    unsafe { btdm_ble_power_down_dma_copy(true) };

    remaining
        .checked_sub(start.elapsed() + WAKE_TIME)
        .unwrap_or(Duration::ZERO)
}

/// Powers the MAC and baseband up, then restores them.
pub(crate) fn power_up() {
    if STATE.fetch_and(!POWERED_DOWN, Ordering::AcqRel) & POWERED_DOWN != 0 {
        unsafe { btdm_ble_power_down_dma_copy(false) };
    }
}
