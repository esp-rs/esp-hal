//! Powers the BLE MAC and baseband down while the controller sleeps.
//!
//! The controller saves its registers to the retention memory before the power-down, and restores
//! them after the power-up. The PHY keeps its frequency state in its own memory, because the modem
//! domain also loses power when no other radio uses it.

use allocator_api2::boxed::Box;
use esp_sync::NonReentrantMutex;

use super::retention;
use crate::compat::malloc::InternalMemory;

unsafe extern "C" {
    fn btdm_deep_sleep_mem_init() -> bool;
    fn btdm_deep_sleep_mem_deinit();
}

// The PHY saves 192 words of frequency state.
const PHY_MEMORY_WORDS: usize = 192;

struct State {
    /// The PHY memory, present while the controller can power the MAC and baseband down.
    phy_memory: Option<Box<[u32; PHY_MEMORY_WORDS], InternalMemory>>,
}

static STATE: NonReentrantMutex<State> = NonReentrantMutex::new(State { phy_memory: None });

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
