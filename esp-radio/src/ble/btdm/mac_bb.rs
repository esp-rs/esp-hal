//! Powers the BLE MAC and baseband down while the controller sleeps.
//!
//! The controller saves its registers to the retention memory before the power-down, and restores
//! them after the power-up. The PHY keeps its frequency state in its own memory, because the modem
//! domain also loses power when no other radio uses it.

use allocator_api2::boxed::Box;
use esp_hal::time::{Duration, Instant};
use esp_sync::NonReentrantMutex;

use super::retention;
use crate::compat::malloc::InternalMemory;

unsafe extern "C" {
    fn btdm_deep_sleep_mem_init() -> bool;
    fn btdm_deep_sleep_mem_deinit();
    fn btdm_ble_power_down_dma_copy(copy: bool);
    fn phy_freq_mem_backup(backup: bool, mem: *mut u32);
}

// The PHY saves 192 words of frequency state.
const PHY_MEMORY_WORDS: usize = 192;

/// The shortest sleep that gets a power-down.
///
/// A shorter sleep sometimes wakes the chip milliseconds late, and the controller then wakes
/// before the restore.
const MIN_WINDOW: Duration = Duration::from_millis(10);

/// The time that the chip needs to wake and restore the MAC and baseband.
const WAKE_TIME: Duration = Duration::from_micros(1500);

struct State {
    /// The PHY memory, present while the controller can power the MAC and baseband down.
    phy_memory: Option<Box<[u32; PHY_MEMORY_WORDS], InternalMemory>>,
    /// Whether the MAC and baseband have no power.
    powered_down: bool,
    /// Whether the controller fell asleep after the last power-down.
    armed: bool,
    /// Whether the modem domain has no power.
    modem_off: bool,
}

static STATE: NonReentrantMutex<State> = NonReentrantMutex::new(State {
    phy_memory: None,
    powered_down: false,
    armed: false,
    modem_off: false,
});

/// Lets the controller power the MAC and baseband down, if retention memory is installed.
///
/// Call this before `btdm_controller_init`.
pub(crate) fn init() {
    if !retention::is_installed() {
        return;
    }

    let Ok(phy_memory) = Box::try_new_in([0; PHY_MEMORY_WORDS], InternalMemory) else {
        warn!("No memory for the PHY state, the BLE MAC and baseband stay powered in sleep");
        return;
    };

    super::chip_specific::init_backup_dma();

    if !unsafe { btdm_deep_sleep_mem_init() } {
        warn!("The retention memory is too small, the BLE MAC and baseband stay powered in sleep");
        return;
    }

    STATE.with(|state| state.phy_memory = Some(phy_memory));
}

/// Releases the memory that [`init`] took.
///
/// Call this after `btdm_controller_deinit`.
pub(crate) fn deinit() {
    let phy_memory = STATE.with(|state| state.phy_memory.take());
    if phy_memory.is_some() {
        unsafe { btdm_deep_sleep_mem_deinit() };
    }
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
        if state.phy_memory.is_none() || state.powered_down || !state.armed {
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
    // The controller calls `modem_power_down` from here, which takes the lock again.
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

/// Powers the modem domain down, if no other radio uses it.
pub(crate) fn modem_power_down() {
    STATE.with(|state| {
        let Some(phy_memory) = state.phy_memory.as_mut() else {
            return;
        };
        // Each radio holds a guard, so a single guard is ours.
        if !crate::RADIO_REFCOUNT.is_single() {
            return;
        }

        {
            let _clock = esp_phy::enable_phy_clock();
            unsafe { phy_freq_mem_backup(true, phy_memory.as_mut_ptr()) };
        }
        crate::common_adapter::disable_wifi_power_domain();
        state.modem_off = true;
    })
}

/// Powers the modem domain up, if [`modem_power_down`] powered it down.
pub(crate) fn modem_power_up() {
    STATE.with(|state| {
        let Some(phy_memory) = state.phy_memory.as_mut() else {
            return;
        };
        if !core::mem::take(&mut state.modem_off) {
            return;
        }

        // The power-up pulses a reset over the modem, which needs the modem clocks.
        let _clock = esp_phy::enable_phy_clock();
        crate::common_adapter::enable_wifi_power_domain();
        unsafe { phy_freq_mem_backup(false, phy_memory.as_mut_ptr()) };
    })
}
