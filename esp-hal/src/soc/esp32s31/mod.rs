//! # SOC (System-on-Chip) module (ESP32-S31)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S31` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
}
pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32s31 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    if crate::system::Cpu::current() == crate::system::Cpu::ProCpu {
        // The ROM may enter on either core. Only the designated primary core
        // should park and gate its sibling during HAL startup.
        unsafe { cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true) };
        cpu_control::disable_core1();
    }

    // By default, APM access-path filters only allow TEE-mode masters. All
    // masters except the HP CPU boot in REE mode, so AXI GDMA cannot fetch
    // descriptors or touch SRAM until these filters are disabled. Mirrors
    // ESP-IDF `bootloader_init_mem` / esp32c6 `pre_init`.
    disable_apm_filters();

    // TODO: [ESP32S31] IDF-14620 — temp. bring-up fix from ESP-IDF: open all LP/HP
    // peripheral PMS control registers for every security mode.
    open_peri_pms();

    // Unlike the other supported chips, the S31 ROM leaves both SYSTIMER clock
    // gates disabled. `Peripheral::KEEP_ENABLED` only prevents the clock from
    // being disabled later, so perform the same enable/reset sequence as
    // ESP-IDF's S31 systimer LL before `Instant` starts using the counter.
    let systimer = crate::peripherals::HP_SYS_CLKRST::regs().systimer_ctrl0();
    systimer.modify(|_, w| w.apb_clk_en().set_bit());
    systimer.modify(|_, w| w.rst_en().set_bit());
    systimer.modify(|_, w| w.rst_en().clear_bit());
    systimer.modify(|_, w| w.clk_en().set_bit());

    // Mirrors ESP-IDF's `init_rng`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/esp_hw_support/hw_random.c#L114-L120>
    trng::rng_ll_enable();
}

/// Turns off every APM access-path filter.
///
/// The S31 loses the APM settings in light sleep, so the wake code calls this function again. The
/// wake code runs with the caches suspended, so this function must be inlined.
///
/// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_security/include/hal/apm_hal.h#L499-L506>
/// and <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/port/esp32s31/pmu_sleep.c#L458-L462>.
#[inline(always)]
pub(crate) fn disable_apm_filters() {
    crate::peripherals::LP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0) });
    crate::peripherals::HP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0) });
    crate::peripherals::HP_MEM_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0) });
}

/// Opens LP/HP peripheral PMS regions for all security modes.
///
/// Offsets match `bootloader_init_mem` in ESP-IDF's `bootloader_mem.c`.
fn open_peri_pms() {
    write_pms_ctrl_range(pac::LP_PERI_PMS::PTR as usize, 0x00, 0x70);
    write_pms_ctrl_range(pac::HP_PERI0_PMS::PTR as usize, 0x00, 0x78);
    write_pms_ctrl_range(pac::HP_PERI1_PMS::PTR as usize, 0x00, 0x98);
}

fn write_pms_ctrl_range(base: usize, start: usize, end: usize) {
    const OPEN: u32 = 0xFF;
    let mut offset = start;
    while offset <= end {
        // SAFETY: writing permissive PMS ctrl values at SOC-defined offsets,
        // same as ESP-IDF bootloader bring-up for ESP32-S31.
        unsafe {
            (base as *mut u32).add(offset / 4).write_volatile(OPEN);
        }
        offset += 4;
    }
}

const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

// The ROM cache function table, up to the freeze functions. See `cache_internal_stub_table` in
// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_rom/esp32s31/include/esp32s31/rom/cache.h#L168-L188>.
#[repr(C)]
struct CacheInternalStubTable {
    _unused: [usize; 11],
    freeze_l1_icache0_enable: unsafe extern "C" fn(mode: u32),
    freeze_l1_icache0_disable: unsafe extern "C" fn(),
    freeze_l1_icache1_enable: unsafe extern "C" fn(mode: u32),
    freeze_l1_icache1_disable: unsafe extern "C" fn(),
    freeze_l1_dcache_enable: unsafe extern "C" fn(mode: u32),
    freeze_l1_dcache_disable: unsafe extern "C" fn(),
}

unsafe extern "C" {
    static rom_cache_internal_table_ptr: *const CacheInternalStubTable;
}

/// Freezes the L1 caches, which cache the external memory. An access to flash or PSRAM stalls
/// until [`unfreeze_ext_mem_cache`].
///
/// The caller must run from RAM. The other core stalls at its next cache miss until the unfreeze.
///
/// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_mm/esp_cache_utils.c#L48-L79>
/// and <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_cache/esp32s31/include/hal/cache_ll.h#L564-L587>.
#[inline(always)]
pub(crate) fn freeze_ext_mem_cache() {
    // `CACHE_FREEZE_ACK_BUSY`: a cache miss stalls the requester.
    const ACK_BUSY: u32 = 0;
    unsafe {
        let table = &*rom_cache_internal_table_ptr;
        (table.freeze_l1_icache0_enable)(ACK_BUSY);
        (table.freeze_l1_icache1_enable)(ACK_BUSY);
        (table.freeze_l1_dcache_enable)(ACK_BUSY);
    }
}

/// Releases the freeze of [`freeze_ext_mem_cache`].
///
/// The data cache unfreezes first, then the instruction caches.
/// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_cache/esp32s31/include/hal/cache_ll.h#L625-L676>.
#[inline(always)]
pub(crate) fn unfreeze_ext_mem_cache() {
    unsafe {
        let table = &*rom_cache_internal_table_ptr;
        (table.freeze_l1_dcache_disable)();
        (table.freeze_l1_icache1_disable)();
        (table.freeze_l1_icache0_disable)();
    }
}

/// Writes back a specific range of data in the cache.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE, addr, size);
    }
}

/// Writes back every dirty line of the data cache.
///
/// The write-back runs twice, as in the ESP-IDF patch of the ROM function `Cache_WriteBack_All`.
/// See <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_rom/patches/esp_rom_cache_writeback_esp32p4_esp32s31.c#L56-L77>.
#[crate::ram]
pub(crate) fn cache_writeback_all() {
    let cache = crate::peripherals::CACHE::regs();
    cache
        .sync_map()
        .write(|w| unsafe { w.bits(CACHE_MAP_L1_DCACHE) });
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
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, addr, size);
    }
}
