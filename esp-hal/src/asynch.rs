//! Asynchronous utilities.
use core::task::Waker;

use embassy_sync::waitqueue::GenericAtomicWaker;
use esp_sync::RawMutex;
use portable_atomic::{AtomicU8, Ordering};

use crate::system::Cpu;

/// Utility struct to register and wake a waker.
pub struct AtomicWaker {
    waker: GenericAtomicWaker<RawMutex>,
}

impl AtomicWaker {
    /// Creates a new `AtomicWaker`.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            waker: GenericAtomicWaker::new(RawMutex::new()),
        }
    }

    /// Registers a waker. Overwrites the previous waker, if any.
    #[inline]
    pub fn register(&self, w: &Waker) {
        self.waker.register(w);
    }

    /// Wakes the registered waker, if any.
    #[crate::ram]
    pub fn wake(&self) {
        self.waker.wake();
    }
}

/// The core that services the async interrupt of a peripheral.
///
/// esp-hal maps a peripheral interrupt to the core that turns the driver async, and the mapping
/// does not follow the driver. The teardown therefore runs on that core, and leaves the interrupt
/// unmapped: a vectored dispatcher reads the interrupt matrix per source it dispatches, so an
/// unmapped source reaches no handler, which is why the async handler needs no check of its own.
///
/// A driver holds one of these in its `State`, and:
///
/// - calls [`Self::claim`] before it binds its async interrupt handler,
/// - calls [`Self::tear_down`] before it unbinds the handler or drops the driver.
pub struct InterruptAffinity {
    /// The core that services the interrupt, or [`Self::NONE`].
    core: AtomicU8,
}

impl InterruptAffinity {
    /// The value of `core` while no core services the interrupt. Should be 0
    /// so that a new `InterruptAffinity` can be placed into `bss`.
    const NONE: u8 = 0;

    /// Bit indicating a claim.
    const SOME: u8 = 0x80;

    /// Creates an affinity that no core claimed.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            core: AtomicU8::new(Self::NONE),
        }
    }

    /// Records the current core as the core that services the interrupt.
    pub fn claim(&self) {
        self.core
            .store(Cpu::current() as u8 | Self::SOME, Ordering::Release);
    }

    /// Returns whether a core services the interrupt.
    #[inline]
    #[cfg(all(multi_core, feature = "rt"))]
    fn is_active(&self) -> bool {
        self.core.load(Ordering::Acquire) != Self::NONE
    }

    /// Records that no core services the interrupt.
    ///
    /// The teardown of a driver calls this **last**: [`Self::tear_down`] returns as soon as it
    /// observes this store, and its caller then reconfigures the peripheral.
    pub fn release(&self) {
        self.core.store(Self::NONE, Ordering::Release);
    }

    /// Runs the teardown of a driver on the core that services the interrupt, and waits for it
    /// to finish. Returns without running `teardown` if no core services the interrupt.
    ///
    /// `teardown` must quiet the peripheral, unmap the interrupt, then call [`Self::release`],
    /// in that order. It runs in an interrupt handler at [`crate::interrupt::Priority::min`] if
    /// another core services the interrupt.
    ///
    /// The caller must hold no lock that `teardown` takes.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if the core that services the interrupt does not run. Such a
    /// core cannot service the interrupt either, which breaks the contract of
    /// [`crate::Async`].
    pub fn tear_down(&self, teardown: fn()) {
        let core = self.core.load(Ordering::Acquire);
        if core == Self::NONE {
            return;
        }

        #[cfg(all(multi_core, feature = "rt"))]
        {
            // "unwrap"
            let core = core & !Self::SOME;
            let core = if core == Cpu::ProCpu as u8 {
                Cpu::ProCpu
            } else {
                Cpu::AppCpu
            };
            if core != Cpu::current() {
                crate::interrupt::ipc::Ipc::new(unsafe { crate::peripherals::IPC::steal() })
                    .call_function(core, teardown);

                // `call_function` returns once it posted the callback.
                while self.is_active() {
                    core::hint::spin_loop();
                }
                return;
            }
        }

        teardown();
    }
}

/// Tears the async mode of a driver down when the driver goes away.
///
/// A driver whose `into_async` and `into_blocking` move fields out of `self` cannot implement
/// `Drop` itself. Such a driver holds this guard in a field that it moves across those
/// transitions. The guard does nothing if the driver is in blocking mode.
pub struct AsyncModeGuard {
    affinity: &'static InterruptAffinity,
    teardown: fn(),
}

impl AsyncModeGuard {
    /// Creates a guard that tears the async mode down through
    /// [`InterruptAffinity::tear_down`].
    pub fn new(affinity: &'static InterruptAffinity, teardown: fn()) -> Self {
        Self { affinity, teardown }
    }
}

impl Drop for AsyncModeGuard {
    fn drop(&mut self) {
        self.affinity.tear_down(self.teardown);
    }
}

impl core::fmt::Debug for AsyncModeGuard {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("AsyncModeGuard")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AsyncModeGuard {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "AsyncModeGuard")
    }
}
