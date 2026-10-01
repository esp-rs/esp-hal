//! Parking the other core for a flash operation.
//!
//! The other core runs [`park_handler`] as an inter-processor call and spins in RAM, with
//! interrupts disabled, until the operation is done. It only takes the call while its interrupts
//! are enabled, so it is never parked inside a critical section.

use core::sync::atomic::{AtomicU8, Ordering};

use esp_hal::{
    interrupt::ipc::Ipc,
    peripherals::IPC,
    system::Cpu,
    time::{Duration, Instant},
};
use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

use crate::FlashStorageError;

/// How long to wait for the other core to park.
const PARK_TIMEOUT: Duration = Duration::from_millis(50);

const IDLE: u8 = 0;
const REQUESTED: u8 = 1;
const PARKED: u8 = 2;

/// Only the parked core sets `PARKED`, so a late [`park_handler`] can't answer a later request.
static STATE: AtomicU8 = AtomicU8::new(IDLE);

/// Runs on the other core. Must not access flash.
#[procmacros::ram]
fn park_handler() {
    let irq_token = unsafe { SingleCoreInterruptLock.enter() };

    // A request that timed out has nothing to park for.
    if STATE
        .compare_exchange(REQUESTED, PARKED, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        while STATE.load(Ordering::Acquire) == PARKED {
            core::hint::spin_loop();
        }
    }

    unsafe { SingleCoreInterruptLock.exit(irq_token) };
}

/// Parks `core`, spinning until it is parked, so flash is only written once `core` is spinning
/// in RAM.
///
/// Returns [`FlashStorageError::OtherCoreRunning`] if `core` doesn't park within
/// [`PARK_TIMEOUT`]. The request is withdrawn then, and flash must not be written.
pub(crate) fn park(core: Cpu) -> Result<(), FlashStorageError> {
    STATE.store(REQUESTED, Ordering::Release);
    Ipc::new(unsafe { IPC::steal() }).call_function(core, park_handler);

    let start = Instant::now();
    while STATE.load(Ordering::Acquire) != PARKED {
        if start.elapsed() > PARK_TIMEOUT
            && STATE
                .compare_exchange(REQUESTED, IDLE, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
        {
            return Err(FlashStorageError::OtherCoreRunning);
        }
        core::hint::spin_loop();
    }

    Ok(())
}

/// Un-parks the core parked by [`park`].
pub(crate) fn unpark() {
    STATE.store(IDLE, Ordering::Release);
}
