//! Powers the Wi-Fi MAC and baseband down while the chip is in light sleep.
//!
//! The REGDMA saves the MAC and baseband on the way into light sleep, and restores them on the way
//! out. Their nodes are in the retention memory. The blob joins the nodes into the list of the
//! REGDMA when Wi-Fi starts, and takes them out when Wi-Fi stops. Around the sleep, `pm_mac_sleep`
//! and `pm_mac_wakeup` stop and start the MAC, and `pm_mac_sleep` updates a node of the MAC.

use core::{ffi::c_int, ptr};

use esp_hal::rtc_cntl::{
    RetentionMemoryError,
    RetentionSlot,
    regdma::{self, LinkConfig, LinkMemory, Module, SLEEP_ENTRY},
};
use portable_atomic::{AtomicBool, Ordering};

use super::WifiError;
use crate::{
    modem_power::{self, Radio},
    sys::{c_types::c_void, include::esp_err_t},
};

unsafe extern "C" {
    fn esp_wifi_internal_set_mac_sleep(enable: bool) -> esp_err_t;
    fn esp_wifi_internal_mac_retention_context_get(config_size: *mut c_int) -> *const LinkConfig;
    fn pm_mac_sleep();
    fn pm_mac_wakeup();
}

const fn bb(id: u16, address: u32, length: u16) -> LinkConfig {
    LinkConfig::continuous(id, address, address, length, SLEEP_ENTRY)
}

/// The baseband: AGC, TX, NRX, BB, and BRX where it exists.
// See `phy_init_data.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_phy>.
const BB: &[LinkConfig] = cfg_select! {
    esp32c6 => &[
        bb(0x0b00, 0x600a_7000, 121),
        bb(0x0b01, 0x600a_7400, 14),
        bb(0x0b02, 0x600a_7800, 136),
        bb(0x0b03, 0x600a_7c00, 53),
    ],
    any(esp32c5, esp32c61) => &[
        bb(0x0b00, 0x600a_7000, 130),
        bb(0x0b01, 0x600a_7400, 30),
        bb(0x0b02, 0x600a_7800, 145),
        bb(0x0b03, 0x600a_7c00, 82),
        bb(0x0b04, 0x600a_8000, 39),
    ],
    esp32s31 => &[
        bb(0x0b00, 0x2010_7000, 130),
        bb(0x0b01, 0x2010_7400, 32),
        bb(0x0b02, 0x2010_7800, 145),
        bb(0x0b03, 0x2010_7c00, 84),
        bb(0x0b04, 0x2010_8000, 39),
    ],
};

/// The bytes that the nodes of the MAC table of the blob take.
const MAC_SIZE: usize = cfg_select! {
    esp32c6 => 4032,
    esp32c61 => 4092,
    esp32c5 => 4076,
    esp32s31 => 4104,
};

const BUFFER_SIZE: usize = MAC_SIZE + regdma::links_size(BB);

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type RetentionStorage = esp_hal::rtc_cntl::RetentionStorage<BUFFER_SIZE>;

/// Memory that the Wi-Fi driver uses to retain the MAC and baseband while they have no power.
///
/// Install the memory with [`install_retention_memory`].
#[instability::unstable]
pub type RetentionMemory = esp_hal::rtc_cntl::RetentionMemory<BUFFER_SIZE>;

static MEMORY: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();
/// Whether the nodes of the MAC and baseband are in the retention memory.
static CREATED: AtomicBool = AtomicBool::new(false);

#[procmacros::doc_replace]
/// Gives the Wi-Fi driver the memory that it uses to retain the MAC and baseband.
/// The memory stays with the driver for the life of the program.
///
/// While the station is in power save, light sleep then powers the MAC and baseband down. The
/// modem domain loses power only when no other radio uses it. The power-down also takes internal
/// heap for the modem clocks and the PHY front end. Install the memory before the Wi-Fi controller
/// starts.
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::wifi::{RetentionStorage, install_retention_memory};
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
/// Returns [`RetentionMemoryError`] if the driver has retention memory already.
#[instability::unstable]
pub fn install_retention_memory(
    memory: &'static mut RetentionMemory,
) -> Result<(), RetentionMemoryError> {
    MEMORY.install(memory)
}

/// Records that Wi-Fi is up, and builds the nodes of the MAC and baseband, if retention memory is
/// installed.
///
/// Call this after `esp_wifi_init_internal`.
pub(super) fn init() -> Result<(), WifiError> {
    modem_power::start(Radio::Wifi);
    let Some(memory) = MEMORY.get() else {
        return Ok(());
    };

    let mut size = 0;
    let mac = unsafe { esp_wifi_internal_mac_retention_context_get(&mut size) };
    let mac = unsafe { core::slice::from_raw_parts(mac, size as usize) };
    let mut links = unsafe { LinkMemory::new(memory, BUFFER_SIZE) };
    // SAFETY: The tables describe registers of the Wi-Fi MAC and baseband.
    let created = unsafe {
        regdma::create(Module::WifiBb, &[BB], &mut links).and_then(|()| {
            regdma::create(Module::WifiMac, &[mac], &mut links)
                .inspect_err(|_| regdma::destroy(Module::WifiBb))
        })
    };
    if created.is_err() {
        warn!(
            "The Wi-Fi MAC and baseband do not fit the retention memory, they stay powered in sleep"
        );
        return Ok(());
    }
    CREATED.store(true, Ordering::Relaxed);
    Ok(())
}

/// Stops the power-down.
///
/// Call this before `esp_wifi_deinit_internal`.
pub(super) fn stop_power_down() -> Result<(), WifiError> {
    if regdma::is_attached(Module::WifiMac) {
        unsafe { mac_sleep_retention_detach() };
    }
    Ok(())
}

/// Records that Wi-Fi is down, and frees the retention memory.
///
/// Call this after `esp_wifi_deinit_internal`, because the deinit still uses the MAC.
pub(super) fn deinit() {
    modem_power::stop(Radio::Wifi);
    if CREATED.swap(false, Ordering::Relaxed) {
        regdma::destroy(Module::WifiMac);
        regdma::destroy(Module::WifiBb);
    }
}

/// Stops the MAC, which the REGDMA then saves with the baseband.
///
/// Call this only when the chip enters light sleep. Without the power-down, this does nothing.
pub(super) fn power_down() {
    unsafe { pm_mac_sleep() };
}

/// Starts the MAC after light sleep.
///
/// Without a power-down before, this does nothing.
pub(super) fn power_up() {
    unsafe { pm_mac_wakeup() };
}

/// The return value of the blob hooks when the nodes do not exist.
const NOT_RETAINED: i32 = 1;

/// The blob calls this when Wi-Fi starts.
pub(super) unsafe extern "C" fn bb_sleep_retention_attach() -> i32 {
    if !CREATED.load(Ordering::Relaxed) {
        return NOT_RETAINED;
    }
    regdma::attach(Module::WifiBb);
    0
}

/// The blob calls this when Wi-Fi stops.
pub(super) unsafe extern "C" fn bb_sleep_retention_detach() -> i32 {
    if !CREATED.load(Ordering::Relaxed) {
        return NOT_RETAINED;
    }
    regdma::detach(Module::WifiBb);
    0
}

/// The blob calls this when Wi-Fi starts, after [`bb_sleep_retention_attach`].
pub(super) unsafe extern "C" fn mac_sleep_retention_attach() -> i32 {
    if !CREATED.load(Ordering::Relaxed) {
        return NOT_RETAINED;
    }
    regdma::attach(Module::WifiMac);
    // The blob looks up the node of the MAC state now, so the node must exist.
    let result = unsafe { esp_wifi_internal_set_mac_sleep(true) };
    if result != 0 {
        regdma::detach(Module::WifiMac);
        return result;
    }
    modem_power::enable_power_down(Radio::Wifi);
    0
}

/// The blob calls this when Wi-Fi stops, before [`bb_sleep_retention_detach`].
pub(super) unsafe extern "C" fn mac_sleep_retention_detach() -> i32 {
    if !CREATED.load(Ordering::Relaxed) {
        return NOT_RETAINED;
    }
    modem_power::disable_power_down(Radio::Wifi);
    let result = unsafe { esp_wifi_internal_set_mac_sleep(false) };
    regdma::detach(Module::WifiMac);
    result
}

/// The blob calls this to look up the node of the MAC state.
pub(super) unsafe extern "C" fn sleep_retention_find_link_by_id(id: c_int) -> *mut c_void {
    regdma::find_link_by_id(id as u16).map_or(ptr::null_mut(), |link| link.as_ptr().cast())
}

/// The blob calls this in `pm_mac_sleep`, with the MAC state that the REGDMA restores.
pub(super) unsafe extern "C" fn regdma_link_set_write_wait_content(
    link: *mut c_void,
    value: u32,
    mask: u32,
) {
    if let Some(link) = ptr::NonNull::new(link.cast()) {
        unsafe { regdma::set_write_wait_content(link, value, mask) };
    }
}
