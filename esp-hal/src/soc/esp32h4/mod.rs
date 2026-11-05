//! # SOC (System-on-Chip) module (ESP32-H4)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-H4` chip.

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32h4 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // Core 1 is gated and held in reset after a chip reset, but a software reset of Core 0 leaves
    // whatever start_core1() set up. Undo it so that Core 1 cannot run during startup.
    unsafe { cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true) };
    cpu_control::disable_core1();
    cpu_control::enable_core1_interrupt_matrix();

    // By default, the APM access-path filters only allow masters that are in TEE mode. Every
    // master except the HP CPU boots in REE mode, so the GDMA can neither fetch descriptors nor
    // access memory until the filters are disabled. Reads return zeroes and writes are dropped,
    // without a bus error. Mirrors ESP-IDF's `bootloader_init_mem`.
    crate::peripherals::HP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0) });

    // The RNG's clocks and its sampling logic are off after reset, and the driver has no way to
    // request them - `Peripheral::KEEP_ENABLED` only prevents the clock from being disabled later.
    // Run the same sequence as ESP-IDF's `rng_ll_enable` so that reads return random data.
    crate::peripherals::LP_PERI::regs().clk_en().modify(|_, w| {
        w.rng_apb_ck_en().set_bit();
        w.rng_ck_en().set_bit()
    });
    crate::peripherals::RNG::regs().cfg().modify(|_, w| unsafe {
        w.sample_enable().set_bit();
        w.timer_en().set_bit();
        w.rtc_timer_en().bits(3)
    });
}

/// Writes back a specific range of data in the cache.
#[doc(hidden)]
#[crate::ram]
#[expect(dead_code, reason = "No enabled driver needs cache maintenance yet.")]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }

    super::CACHE_SYNC_LOCK.lock(|| unsafe {
        Cache_WriteBack_Addr(addr, size);
    });
}

/// Writes back every dirty line of the cache.
///
/// The write-back runs twice, as in the ESP-IDF patch of the ROM function `Cache_WriteBack_All`.
/// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_rom/patches/esp_rom_cache_writeback_esp32c5_esp32c61_esp32h4.c#L35-L45>.
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
#[expect(dead_code, reason = "No enabled driver needs cache maintenance yet.")]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    super::CACHE_SYNC_LOCK.lock(|| unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, addr, size);
    });
}
