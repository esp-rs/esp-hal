//! # SOC (System-on-Chip) module (ESP32-C5)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C5` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
}
pub(crate) mod regi2c;

pub(crate) use esp32c5 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // Reset TEE security modes. This allows unrestricted access to TEE masters, including DMA.
    // FIXME: this is a temporary workaround until we have a proper solution for TEE security modes.
    for m in crate::peripherals::TEE::regs().m_mode_ctrl_iter() {
        m.reset();
    }
    // this is hacky, but for some reason we must reset the output enable register manually
    crate::peripherals::GPIO::regs().enable().reset();

    // Clear bit reset_event_bypass to ensure that the system bus is also reset during a core reset
    // (WDT), preventing bus freezing caused by an incorrect MSPI core reset in ROM. Mirrors
    // ESP-IDF's bootloader_hardware_init.
    crate::peripherals::PCR::regs()
        .reset_event_bypass()
        .modify(|_, w| w.reset_event_bypass().clear_bit());

    // Mirrors ESP-IDF's `init_rng`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/esp_hw_support/hw_random.c#L114-L120>
    trng::rng_ll_enable();
}

/// Writes back a specific range of data in the cache.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(addr, size);
    }
}

/// Writes back every dirty line of the cache.
///
/// Mirrors ESP-IDF's patch of the ROM's `Cache_WriteBack_All`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_rom/patches/esp_rom_cache_writeback_esp32c5_esp32c61_esp32h4.c#L35-L45>.
/// Like that patch, this function writes back twice.
#[crate::ram]
pub(crate) fn cache_writeback_all() {
    const CACHE_MAP_FLASH_CACHE: u32 = 1 << 4;

    let cache = crate::peripherals::CACHE::regs();
    cache
        .sync_map()
        .write(|w| unsafe { w.bits(CACHE_MAP_FLASH_CACHE) });
    cache.sync_addr().write(|w| unsafe { w.bits(0) });
    cache.sync_size().write(|w| unsafe { w.bits(0) });
    for _ in 0..2 {
        // The register resets with `INVALIDATE_ENA` set, and an invalidate drops the dirty lines.
        unsafe {
            cache
                .sync_ctrl()
                .write_with_zero(|w| w.writeback_ena().set_bit())
        };
        while cache.sync_ctrl().read().sync_done().bit_is_clear() {}
    }
}

/// Invalidate a specific range of addresses in the cache.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(addr, size);
    }
}
