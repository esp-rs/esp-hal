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

    #[cfg(esp32p4)]
    switch_to_dcdc();

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

/// The ROM leaves the HP domain on its LDO at the uncalibrated reset voltage. esp-hal leaves the
/// eFuse-calibrated regulator voltages and turning on the DCDC converter to the bootloader, and
/// runs the CPU at up to 400 MHz on top of them.
///
/// See <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hw_support/port/esp32p4/rtc_clk_init.c#L58-L88>.
/// See <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hw_support/port/esp32p4/pmu_param.c#L332-L373>.
#[cfg(esp32p4)]
#[inline(always)]
fn switch_to_dcdc() {
    use esp_hal::{
        efuse::{self, ChipRevision},
        peripherals::{LP_SYS, PMU},
    };

    const HP_CALI_ACTIVE_DBIAS_DEFAULT: u8 = 24;
    const LP_CALI_ACTIVE_DBIAS_DEFAULT: u8 = 29;
    const HP_CALI_ACTIVE_DCM_VSET_DEFAULT: u8 = 27;
    const DCDC_STARTUP_TIME_US: u32 = 1000;

    // ESP-IDF reads the voltage calibration only from block version 0.2 on, and not on 1.0.
    let (blk_major, blk_minor) = efuse::block_version();
    let calibrated = (blk_major, blk_minor) >= (0, 2) && (blk_major, blk_minor) != (1, 0);
    let (hp_cali, lp_cali) = if calibrated {
        (
            efuse::read_field_le::<u8>(efuse::ACTIVE_HP_DBIAS),
            efuse::read_field_le::<u8>(efuse::ACTIVE_LP_DBIAS),
        )
    } else {
        (0, 0)
    };
    let hp_dbias = match hp_cali {
        0 => HP_CALI_ACTIVE_DBIAS_DEFAULT,
        cali => (cali + 16).min(31),
    };
    // The LP calibration is 4 steps lower than the DCDC voltage.
    let lp_dbias = match lp_cali {
        0 => LP_CALI_ACTIVE_DBIAS_DEFAULT,
        cali => (cali + 16 + 4).min(31),
    };
    let fb_res_workaround = efuse::chip_revision() >= ChipRevision::from_combined(301);

    let pmu = PMU::regs();
    pmu.hp_active_hp_regulator0().modify(|_, w| unsafe {
        w.hp_active_hp_regulator_xpd().set_bit();
        w.hp_active_hp_regulator_dbias().bits(hp_dbias)
    });
    // The LP_ACTIVE mode registers are the `hp_sleep_lp_*` group.
    pmu.hp_sleep_lp_regulator0()
        .modify(|_, w| unsafe { w.hp_sleep_lp_regulator_dbias().bits(lp_dbias) });

    let dcm_vset = pmu
        .hp_active_hp_regulator0()
        .read()
        .hp_dbias_vol()
        .bits()
        .max(HP_CALI_ACTIVE_DCM_VSET_DEFAULT);

    if fb_res_workaround {
        pmu.dcm_ctrl()
            .modify(|_, w| w.dcdc_fb_res_force_pd().set_bit());
    }
    pmu.dcm_ctrl().modify(|_, w| {
        w.dcdc_done_force().clear_bit();
        w.dcdc_on_req().set_bit()
    });
    pmu.power_dcdc_switch().modify(|_, w| {
        w.force_dcdc_switch_pu().clear_bit();
        w.force_dcdc_switch_pd().clear_bit()
    });
    pmu.hp_active_bias()
        .modify(|_, w| unsafe { w.hp_active_dcm_vset().bits(dcm_vset) });
    // Hands the regulator voltage over to the PMU.
    pmu.hp_active_hp_regulator0()
        .modify(|_, w| w.dig_regulator0_dbias_sel().set_bit());
    esp_rom_sys::rom::ets_delay_us(DCDC_STARTUP_TIME_US);

    if fb_res_workaround {
        // Bit 4 clear selects the digital feedback resistor instead of the analog one.
        LP_SYS::regs()
            .sys_ctrl()
            .modify(|_, w| unsafe { w.lp_fib_sel().bits(0xEF) });
        pmu.dcm_ctrl()
            .modify(|_, w| w.dcdc_fb_res_force_pd().clear_bit());
        esp_rom_sys::rom::ets_delay_us(10);
    }

    pmu.hp_active_hp_regulator0()
        .modify(|_, w| w.hp_active_hp_regulator_xpd().clear_bit());
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
