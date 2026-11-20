//! The CPU retention memory and its one-shot install API.

use core::ptr::NonNull;

use crate::rtc_cntl::{
    RetentionMemory,
    RetentionMemoryError,
    RetentionSlot,
    RetentionStorage,
    sleep::LowPower,
};

cfg_select! {
    cpu_retention = "software" => {
        pub(crate) use super::software::BUFFER_SIZE;
    }
    _ => {
        pub(crate) const BUFFER_SIZE: usize = property!("sleep.cpu_retention_mem_size");

        /// `repr(align)` takes a literal, so the buffer takes the strictest alignment of every
        /// chip. The alignment then needs no check at run time.
        const _: () = ::core::assert!(property!("sleep.cpu_retention_mem_align") <= 16);
    }
}

static INSTALLED: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();

/// Storage for [`CpuRetentionMemory`].
///
/// Place this storage in [`#[ram(unstable(retention))]`][crate::ram], then a static mutable
/// reference to [`CpuRetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type CpuRetentionStorage = RetentionStorage<BUFFER_SIZE>;

/// Memory that light sleep uses to retain the CPU domain.
///
/// The sleep path writes this memory while the CPU domain has no power, so the buffer gives no way
/// to read or write its contents through safe Rust.
///
/// Install the buffer with [`LowPower::install_cpu_retention_memory`]. The buffer must live in
/// internal RAM that stays powered across light sleep. The
/// [`#[ram(unstable(retention))]`][crate::ram] attribute places a static there, inside the
/// reachable range on chips that use the retention DMA.
#[instability::unstable]
pub type CpuRetentionMemory = RetentionMemory<BUFFER_SIZE>;

/// The reason why [`LowPower::install_cpu_retention_memory`] refused a buffer.
#[instability::unstable]
pub type CpuRetentionMemoryError = RetentionMemoryError;

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
    /// is not inside the range that retention needs. The
    /// [`#[ram(unstable(retention))]`][crate::ram] attribute places a static inside that range.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rtc_cntl::{CpuRetentionStorage, sleep::LowPower};
    ///
    /// #[esp_hal::ram(unstable(retention))]
    /// static RETENTION: CpuRetentionStorage = CpuRetentionStorage::new();
    ///
    /// let mut lpwr = LowPower::new(peripherals.LPWR);
    /// lpwr.install_cpu_retention_memory(RETENTION.take())?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn install_cpu_retention_memory(
        &mut self,
        memory: &'static mut CpuRetentionMemory,
    ) -> Result<(), CpuRetentionMemoryError> {
        INSTALLED.install(memory)
    }
}

pub(crate) fn installed_buffer_ptr() -> Option<NonNull<u8>> {
    INSTALLED.get()
}
