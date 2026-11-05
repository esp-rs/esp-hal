//! Powers the BLE MAC and baseband down while the controller sleeps.
//!
//! The controller saves its registers to the retention memory before the power-down, and restores
//! them after the power-up.

use esp_hal::time::{Duration, Instant};
use esp_sync::NonReentrantMutex;

use super::retention;
use crate::modem_power::{self, Radio};

unsafe extern "C" {
    fn btdm_deep_sleep_mem_init() -> bool;
    fn btdm_deep_sleep_mem_deinit();
    fn btdm_ble_power_down_dma_copy(copy: bool);
}

/// The shortest sleep that gets a power-down.
///
/// A shorter sleep sometimes wakes the chip milliseconds late, and the controller then wakes
/// before the restore.
const MIN_WINDOW: Duration = Duration::from_millis(10);

/// The time that the chip needs to wake and restore the MAC and baseband.
const WAKE_TIME: Duration = Duration::from_micros(1500);

struct State {
    /// Whether the controller can power the MAC and baseband down.
    enabled: bool,
    /// Whether the MAC and baseband have no power.
    powered_down: bool,
    /// Whether the controller fell asleep after the last power-down.
    armed: bool,
}

static STATE: NonReentrantMutex<State> = NonReentrantMutex::new(State {
    enabled: false,
    powered_down: false,
    armed: false,
});

/// Records that the controller is up.
///
/// With `power_down`, the controller can power the MAC and baseband down, if retention memory is
/// installed. Call this before `btdm_controller_init`.
pub(crate) fn init(power_down: bool) {
    modem_power::start(Radio::Ble);
    if !power_down || !retention::is_installed() {
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

    STATE.with(|state| state.enabled = true);
}

/// Releases what [`init`] took.
///
/// Call this after `btdm_controller_deinit`.
pub(crate) fn deinit() {
    let enabled = STATE.with(|state| core::mem::take(&mut state.enabled));
    if enabled {
        unsafe { btdm_deep_sleep_mem_deinit() };
    }
    modem_power::stop(Radio::Ble);
}

/// Lets the next chip sleep power the MAC and baseband down.
///
/// Call this when the controller falls asleep. The controller does not wake after a second
/// power-down in one of its sleeps.
pub(crate) fn controller_sleeps() {
    STATE.with(|state| state.armed = true);
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

    let power_down = STATE.with(|state| {
        if !state.enabled || state.powered_down || !state.armed {
            return false;
        }
        state.powered_down = true;
        state.armed = false;
        true
    });
    if !power_down {
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
    let powered_down = STATE.with(|state| core::mem::take(&mut state.powered_down));

    if powered_down {
        unsafe { btdm_ble_power_down_dma_copy(false) };
    }
}
