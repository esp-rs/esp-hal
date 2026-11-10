//! Powers the BLE MAC and baseband down while the chip is in light sleep.
//!
//! The REGDMA saves the MAC and baseband on the way into light sleep, and restores them on the way
//! out. Their nodes are in the retention memory. The REGDMA of the ESP32-C6 cannot reach them on
//! the clock of the sleep, so there the sleep hooks of the controller copy them while the PLL runs.

use esp_hal::rtc_cntl::{
    RetentionMemoryError,
    RetentionSlot,
    regdma::{self, LinkConfig, LinkMemory, Module},
};
use portable_atomic::{AtomicBool, Ordering};

use crate::modem_power::{self, Radio};

unsafe extern "C" {
    fn r_esp_ble_mac_retention_link_get(size: *mut u8, extra: u8) -> *const LinkConfig;
    #[cfg(bt_controller = "btdm2")]
    fn r_btdm_mac_retention_link_get(size: *mut u8) -> *const LinkConfig;
}

/// The entry of the MAC and baseband.
const OWNER: u32 = cfg_select! {
    regdma_bt_needs_pll => regdma::EXTRA_ENTRY,
    _ => regdma::SLEEP_ENTRY,
};

const fn bb(id: u16, address: u32, length: u16) -> LinkConfig {
    LinkConfig::continuous(id, address, address, length, OWNER)
}

/// The Bluetooth baseband.
const BB: &[LinkConfig] = cfg_select! {
    any(esp32c6, esp32h2) => &[
        bb(0x1500, 0x600a_2000, 93),
        bb(0x1501, 0x600a_2800, 62),
        bb(0x1502, 0x600a_2c00, 19),
    ],
    esp32c61 => &[
        bb(0x1500, 0x600a_2000, 128),
        bb(0x1501, 0x600a_2800, 68),
    ],
    esp32c5 => &[
        bb(0x1500, 0x600a_2000, 128),
        bb(0x1501, 0x600a_2800, 68),
        bb(0x1502, 0x600a_2c00, 19),
    ],
    esp32h4 => &[
        bb(0x1500, 0x600c_2000, 128),
        bb(0x1501, 0x600c_2800, 67),
        bb(0x1502, 0x600c_2c00, 63),
    ],
    esp32s31 => &[
        bb(0x1500, 0x2010_2000, 128),
        bb(0x1501, 0x2010_2800, 68),
        bb(0x1502, 0x2010_2c00, 19),
        bb(0x1503, 0x2010_2400, 64),
    ],
};

/// The bytes that the nodes of the MAC tables of the controller take.
const MAC_SIZE: usize = cfg_select! {
    esp32c6 => 388,
    any(esp32c5, esp32c61, esp32h2) => 412,
    esp32h4 => 620 + 792,
    esp32s31 => 984 + 792,
};

const BUFFER_SIZE: usize = MAC_SIZE + regdma::links_size(BB);

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
/// Whether the nodes of the MAC and baseband are in the retention memory.
static CREATED: AtomicBool = AtomicBool::new(false);

#[procmacros::doc_replace]
/// Gives the BLE controller the memory that it uses to retain the MAC and baseband.
/// The memory stays with the controller for the life of the program.
///
/// With modem sleep on, light sleep then powers the MAC and baseband down while the controller
/// sleeps. The modem domain loses power only when no other radio uses it. The power-down also
/// takes internal heap for the modem clocks and the PHY front end. Install the memory before the
/// controller starts.
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
/// Returns [`RetentionMemoryError`] if the controller has retention memory already.
#[instability::unstable]
pub fn install_retention_memory(
    memory: &'static mut RetentionMemory,
) -> Result<(), RetentionMemoryError> {
    MEMORY.install(memory)
}

/// Returns the MAC tables of the controller.
fn mac_tables() -> [&'static [LinkConfig]; 2] {
    let mut size = 0;
    let ble = unsafe { r_esp_ble_mac_retention_link_get(&mut size, 0) };
    let ble = unsafe { core::slice::from_raw_parts(ble, size as usize) };
    cfg_select! {
        bt_controller = "btdm2" => {
            let btdm = unsafe { r_btdm_mac_retention_link_get(&mut size) };
            [unsafe { core::slice::from_raw_parts(btdm, size as usize) }, ble]
        }
        _ => [ble, &[]],
    }
}

/// Records that the controller is up.
///
/// With `power_down`, light sleep can power the MAC and baseband down, if retention memory is
/// installed. Call this after the controller is initialized.
pub(crate) fn init(power_down: bool) {
    modem_power::start(Radio::Ble);
    if !power_down {
        return;
    }
    let Some(memory) = MEMORY.get() else {
        return;
    };

    #[cfg(regdma_bt_needs_pll)]
    if esp_hal::clock::ll::ClockTree::with(|clocks| clocks.soc_root_clk())
        != Some(esp_hal::clock::ll::SocRootClkConfig::Pll)
    {
        warn!("The CPU does not run on the PLL, the BLE MAC and baseband stay powered in sleep");
        return;
    }

    let [first, second] = mac_tables();
    let mut links = unsafe { LinkMemory::new(memory, BUFFER_SIZE) };
    // SAFETY: The tables describe registers of the BLE MAC and baseband.
    let created = unsafe {
        regdma::create(Module::BtBb, &[BB], &mut links).and_then(|()| {
            regdma::create(Module::BleMac, &[first, second], &mut links)
                .inspect_err(|_| regdma::destroy(Module::BtBb))
        })
    };
    if created.is_err() {
        warn!(
            "The BLE MAC and baseband do not fit the retention memory, they stay powered in sleep"
        );
        return;
    }
    CREATED.store(true, Ordering::Relaxed);
    regdma::attach(Module::BtBb);
    regdma::attach(Module::BleMac);
    modem_power::enable_power_down(Radio::Ble);
}

/// Records that the controller is down, and frees the retention memory.
///
/// Call this while the controller does not sleep, before it is disabled.
pub(crate) fn deinit() {
    if CREATED.swap(false, Ordering::Relaxed) {
        modem_power::disable_power_down(Radio::Ble);
        regdma::destroy(Module::BleMac);
        regdma::destroy(Module::BtBb);
    }
    modem_power::stop(Radio::Ble);
}

/// Whether [`power_down`] saved the MAC and baseband, and [`power_up`] did not restore them yet.
#[cfg(regdma_bt_needs_pll)]
static SAVED: AtomicBool = AtomicBool::new(false);

/// Saves the MAC and baseband before light sleep.
///
/// Call this only when the chip enters light sleep, while the controller sleeps.
#[cfg(regdma_bt_needs_pll)]
pub(crate) fn power_down() {
    if CREATED.load(Ordering::Relaxed) {
        regdma::run_extra_link(true);
        SAVED.store(true, Ordering::Relaxed);
    }
}

/// Restores the MAC and baseband after light sleep, if [`power_down`] saved them.
#[cfg(regdma_bt_needs_pll)]
pub(crate) fn power_up() {
    if SAVED.swap(false, Ordering::Relaxed) {
        regdma::run_extra_link(false);
    }
}
