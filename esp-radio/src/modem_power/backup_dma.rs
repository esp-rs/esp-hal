//! Retention with the backup DMA.
//!
//! A radio saves its MAC and baseband with the backup DMA, then reports its sleep. The domain loses
//! power when every radio that is up sleeps. The PHY keeps its frequency state in memory of this
//! module.

use allocator_api2::boxed::Box;
use enumset::EnumSet;

use super::{Radio, STATE};
use crate::{compat::malloc::InternalMemory, counting_lock::CountingLock};

unsafe extern "C" {
    fn phy_freq_mem_backup(backup: bool, mem: *mut u32);
}

// The PHY saves 192 words of frequency state.
const PHY_MEMORY_WORDS: usize = 192;

pub(super) struct State {
    /// The PHY memory, present while a radio can power the modem domain down.
    phy_memory: Option<Box<[u32; PHY_MEMORY_WORDS], InternalMemory>>,
    /// The radios that sleep.
    asleep: EnumSet<Radio>,
    /// Whether the modem domain has no power.
    modem_off: bool,
}

impl State {
    pub(super) const fn new() -> Self {
        Self {
            phy_memory: None,
            asleep: EnumSet::empty(),
            modem_off: false,
        }
    }

    /// Prepares the PHY memory and the backup DMA. Returns whether they are ready.
    pub(super) fn enable(&mut self) -> bool {
        if self.phy_memory.is_none() {
            let Ok(phy_memory) = Box::try_new_in([0; PHY_MEMORY_WORDS], InternalMemory) else {
                return false;
            };
            self.phy_memory = Some(phy_memory);
            init_backup_dma();
        }
        true
    }

    /// [`sleep`] checks whether the modem domain can lose power, so this does nothing.
    pub(super) fn allow_power_down(&mut self, _allowed: bool) {}

    /// Releases what [`Self::enable`] prepared.
    pub(super) fn release(&mut self) {
        self.phy_memory = None;
    }
}

/// Records that `radio` sleeps, and powers the modem domain down if every radio that is up sleeps.
///
/// Call this after the radio saved its MAC and baseband.
pub(crate) fn sleep(radio: Radio) {
    STATE.with(|state| {
        let allowed = state.power_down_allowed();
        let method = &mut state.method;
        method.asleep.insert(radio);
        if !allowed || !state.up.is_subset(method.asleep) || method.modem_off {
            return;
        }
        let Some(phy_memory) = method.phy_memory.as_mut() else {
            return;
        };

        {
            let _clock = esp_phy::enable_phy_clock();
            unsafe { phy_freq_mem_backup(true, phy_memory.as_mut_ptr()) };
        }
        crate::common_adapter::disable_wifi_power_domain();
        method.modem_off = true;
    })
}

/// Records that `radio` wakes, and powers the modem domain up if [`sleep`] powered it down.
///
/// Call this before the radio restores its MAC and baseband.
pub(crate) fn wake(radio: Radio) {
    STATE.with(|state| {
        let method = &mut state.method;
        method.asleep.remove(radio);
        if !core::mem::take(&mut method.modem_off) {
            return;
        }
        let Some(phy_memory) = method.phy_memory.as_mut() else {
            return;
        };

        // The power-up pulses a reset over the modem, which needs the modem clocks.
        let _clock = esp_phy::enable_phy_clock();
        crate::common_adapter::enable_wifi_power_domain();
        unsafe { phy_freq_mem_backup(false, phy_memory.as_mut_ptr()) };
    })
}

/// The chip has one backup DMA, so two copies must not run at the same time. The ROM gives the
/// unlock no argument.
static BACKUP_DMA_LOCK: CountingLock = CountingLock::new();

/// The ROM calls this before a backup DMA copy.
extern "C" fn backup_dma_lock() {
    unsafe { BACKUP_DMA_LOCK.acquire() };
}

/// The ROM calls this after a backup DMA copy.
extern "C" fn backup_dma_unlock() {
    unsafe { BACKUP_DMA_LOCK.release() };
}

/// Gives the ROM the lock that it takes around a backup DMA copy.
///
/// Call this before the first copy. The ROM calls the lock without a check, so a copy without it
/// jumps to address zero.
fn init_backup_dma() {
    unsafe extern "C" {
        fn ets_apb_backup_init_lock_func(lock: extern "C" fn(), unlock: extern "C" fn());
    }

    unsafe { ets_apb_backup_init_lock_func(backup_dma_lock, backup_dma_unlock) };
}
