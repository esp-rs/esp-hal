//! Hardware setup that a 2nd-stage bootloader does and that esp-hal relies on.
//!
//! esp-hal calls `__esp_bootloader_init` right before `main`, so this runs before
//! `esp_hal::init`.

#[esp_hal::ram]
#[unsafe(no_mangle)]
extern "C" fn __esp_bootloader_init() {
    disable_flashboot_watchdog();

    #[cfg(esp32h4)]
    enable_bbpll_clocks();

    unlock_flash();
}

/// The ROM arms the MWDT of TIMG0 in flash-boot mode. `esp_hal::init` only clears `wdt_en`, which
/// does not stop it while `wdt_flashboot_mod_en` is set.
///
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/bootloader_support/src/bootloader_init.c#L104-L111>.
#[inline(always)]
fn disable_flashboot_watchdog() {
    let timg0 = esp_hal::peripherals::TIMG0::regs();

    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1) });
    timg0
        .wdtconfig0()
        .modify(|_, w| w.wdt_flashboot_mod_en().clear_bit());
    // The configuration registers are updated asynchronously.
    #[cfg(timergroup_wdt_has_conf_update)]
    timg0
        .wdtconfig0()
        .modify(|_, w| w.wdt_conf_update_en().set_bit());
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });
}

/// The ROM leaves the BBPLL off and the analog I2C master clock gated. `esp_hal::init` programs
/// the PLL over that I2C master and then waits for the calibration, which never finishes while the
/// clock is gated, so the ROM's watchdog resets the chip.
///
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/bootloader_support/src/esp32h4/bootloader_esp32h4.c#L97-L109>.
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/esp_hal_clock/esp32h4/include/hal/clk_tree_ll.h#L66-L69>.
#[cfg(esp32h4)]
#[inline(always)]
fn enable_bbpll_clocks() {
    use esp_hal::peripherals::{MODEM_LPCON, MODEM_SYSCON, PMU};

    // `PMU_HP_ICG_MODEM_CODE_MODEM` is 1 and `PMU_HP_ICG_MODEM_CODE_ACTIVE` is 2.
    const ICG_BITMAP: u8 = (1 << 1) | (1 << 2);

    let lpcon = MODEM_LPCON::regs();
    lpcon.clk_conf_power_st().modify(|r, w| unsafe {
        w.clk_i2c_mst_st_map()
            .bits(r.clk_i2c_mst_st_map().bits() | ICG_BITMAP);
        w.clk_lp_apb_st_map()
            .bits(r.clk_lp_apb_st_map().bits() | ICG_BITMAP)
    });
    lpcon.clk_conf().modify(|_, w| w.clk_i2c_mst_en().set_bit());
    lpcon
        .clk_conf_force_on()
        .modify(|_, w| w.clk_i2c_mst_fo().set_bit());
    MODEM_SYSCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

    let pmu = PMU::regs();
    pmu.imm_hp_ck_power().write(|w| {
        w.tie_high_xpd_bbpll().set_bit();
        w.tie_high_xpd_bbpll_i2c().set_bit()
    });
    pmu.imm_hp_ck_power()
        .write(|w| w.tie_high_global_bbpll_icg().set_bit());
    pmu.hp_active_hp_ck_power().modify(|_, w| {
        w.hp_active_xpd_bbpll().set_bit();
        w.hp_active_xpd_bbpll_i2c().set_bit()
    });
}

/// esp-hal's flash driver writes without clearing the write protection bits of the flash chip.
///
/// See <https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/components/bootloader_support/bootloader_flash/src/bootloader_flash.c#L716>.
#[inline(always)]
fn unlock_flash() {
    let result = unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_unlock() };
    debug_assert_eq!(
        result,
        esp_rom_sys::rom::spiflash::ESP_ROM_SPIFLASH_RESULT_OK,
        "unlocking the flash chip failed"
    );
}
