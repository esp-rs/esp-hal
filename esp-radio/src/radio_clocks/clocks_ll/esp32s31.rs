use crate::reg_access::pac::modem_syscon::clk_conf1::W as ClkConf1W;

/// IDF `modem_clock_hal_enable_soc_pll_source_cg`.
///
/// Reset default of `HP_SYS_CLKRST.modem_conf` is 0x25 (APB + XTAL). Wi-Fi /
/// PHY / BLE need 0x3d (APB + source + PLL select + PLL + XTAL). Without the
/// PLL the MAC/BB have no working clock and `esp_wifi_init` never returns.
fn enable_soc_pll_source_cg() {
    regs!(HP_SYS_CLKRST).modem_conf().modify(|_, w| {
        w.modem_apb_clk_en().set_bit();
        w.modem_rst_en().clear_bit();
        w.modem_clk_en().set_bit();
        w.modem_clk_source_sel().set_bit();
        w.modem_pll_clk_en().set_bit();
        w.modem_xtal_clk_en().set_bit()
    });
}

/// The Wi-Fi baseband clocks in `MODEM_SYSCON.clk_conf1`.
///
/// IDF `BLE_CLOCK_DEPS` only names `WIFI_BB_80X1`, but TX goes through the FE,
/// which uses the rest of them for the calibration `esp_phy` runs. All three
/// protocols gate them as a group.
fn set_wifibb_clocks(w: &mut ClkConf1W, en: bool) -> &mut ClkConf1W {
    w.clk_wifibb_160x1_en().bit(en);
    w.clk_wifibb_80x1_en().bit(en);
    w.clk_wifibb_40x1_en().bit(en);
    w.clk_wifibb_80x_en().bit(en);
    w.clk_wifibb_40x_en().bit(en);
    w.clk_wifibb_80m_en().bit(en);
    w.clk_wifibb_44m_en().bit(en);
    w.clk_wifibb_40m_en().bit(en);
    w.clk_wifibb_22m_en().bit(en)
}

fn pulse_wifibb_reset() {
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifibb().set_bit());
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifibb().clear_bit());
}

/// The Wi-Fi MAC clock and the APB clock that reaches its registers.
///
/// The driver asks for its clocks to be gated while it is still tearing down, and then writes Wi-Fi
/// MAC registers. With these gated, the access hangs the CPU and debug connection.
fn set_wifi_mac_clocks(en: bool) {
    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifimac_en().bit(en)
    });
}

pub(crate) fn enable_wifi(en: bool) {
    if en {
        enable_soc_pll_source_cg();
        pulse_wifibb_reset();
        set_wifi_mac_clocks(true);
    }

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        set_wifibb_clocks(w, en)
    });

    regs!(MODEM_LPCON).clk_conf().modify(|_, w| {
        w.clk_wifipwr_en().bit(en);
        w.clk_coex_en().bit(en)
    });
}

pub(crate) fn enable_ieee802154(en: bool) {
    if en {
        enable_soc_pll_source_cg();
    }

    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zbmac_en().bit(en)
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_btbb_en().bit(en);
        w.clk_btmac_en().bit(en);
        set_wifibb_clocks(w, en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

fn pulse_bt_mac_reset() {
    regs!(MODEM_SYSCON).modem_rst_conf().modify(|_, w| {
        w.rst_btmac().set_bit();
        w.rst_btmac_apb().set_bit();
        w.rst_ble_timer().set_bit();
        w.rst_modem_sec().set_bit()
    });
    regs!(MODEM_SYSCON).modem_rst_conf().modify(|_, w| {
        w.rst_btmac().clear_bit();
        w.rst_btmac_apb().clear_bit();
        w.rst_ble_timer().clear_bit();
        w.rst_modem_sec().clear_bit()
    });
}

pub(crate) fn enable_bt(en: bool) {
    if en {
        enable_soc_pll_source_cg();
        pulse_bt_mac_reset();
        ble_rtc_clk_init();
        // IDF `trng_ll_enable` / `bootloader_random_enable`. Advertising
        // enable is the first `ble_ll_rand()`; that path busy-waits on
        // HW samples. S31 CONF.sample_enable is off at reset.
        enable_trng();
    }
    enable_bt_clocks(en);
}

/// IDF `hal/esp32s31/include/hal/trng_ll.h` `trng_ll_enable`.
fn enable_trng() {
    regs!(LP_PERICLKRST)
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_clk_en().set_bit());
    regs!(LP_PERICLKRST)
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().set_bit());
    regs!(LP_PERICLKRST)
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().clear_bit());
    regs!(TRNG).date().modify(|_, w| w.clk_en().set_bit());
    regs!(TRNG).conf().modify(|_, w| {
        w.sample_enable().set_bit();
        w.noise_crc_en().set_bit()
    });
}

/// IDF `e_btdm_lp_modem_clock_set`: gate BT clocks without a MAC reset.
pub(crate) fn enable_bt_clocks(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_etm_en().bit(en);
        w.clk_modem_sec_en().bit(en);
        w.clk_modem_sec_ecb_en().bit(en);
        w.clk_modem_sec_ccm_en().bit(en);
        w.clk_modem_sec_bah_en().bit(en);
        w.clk_modem_sec_apb_en().bit(en);
        w.clk_ble_timer_en().bit(en)
    });

    if en {
        // Keep the high-res BLE timer running while the modem ICG would
        // otherwise gate it. Advertising enable polls this counter.
        regs!(MODEM_SYSCON)
            .clk_conf_force_on()
            .modify(|_, w| w.clk_ble_timer_fo().set_bit());
    }

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_dac_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_pwdet_adc_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_btbb_en().bit(en);
        w.clk_btmac_en().bit(en);
        set_wifibb_clocks(w, en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));

    if en {
        // IDF `BLE` domain is `ICG_NOGATING_ACTIVE` only; also keep it
        // ungated in the modem state so BLE-only (no Wi-Fi) still runs.
        regs!(MODEM_SYSCON)
            .clk_conf_power_st()
            .modify(|r, w| unsafe { w.clk_bt_st_map().bits(r.clk_bt_st_map().bits() | 0b0110) });
    }
}

pub(crate) fn reset_wifi_mac() {
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifimac().set_bit());
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifimac().clear_bit());
}

pub(crate) fn init_clocks() {
    // done in esp-hal
}

pub(crate) fn deinit_clocks() {
    // nothing to do, `init_clocks` is a no-op
}

/// IDF `btdm_lp` default: `CONFIG_BT_CTRL_LP_CLK_SRC_MAIN_XTAL` at 100 kHz.
pub(crate) const BT_LPCLK_HZ: u64 = 100_000;

/// Select the BLE RTC / modem LP timer clock.
///
/// IDF `btdm_lp_timer_clk_init` + `modem_clock_select_lp_clock_source(PERIPH_BT_MODULE)`.
/// Without a source and `clk_lp_timer_en`, the controller accepts HCI scan
/// enable but never schedules radio windows.
pub(crate) fn ble_rtc_clk_init() {
    let xtal_hz = esp_hal::clock::xtal_clock().as_hz();
    let divider = (xtal_hz / BT_LPCLK_HZ as u32).saturating_sub(1) as u16;

    regs!(MODEM_LPCON)
        .test_conf()
        .modify(|_, w| w.clk_en().set_bit());

    regs!(MODEM_LPCON).lp_timer_conf().modify(|_, w| {
        w.clk_lp_timer_sel_osc_slow().clear_bit();
        w.clk_lp_timer_sel_osc_fast().clear_bit();
        w.clk_lp_timer_sel_xtal32k().clear_bit();
        w.clk_lp_timer_sel_xtal().set_bit();
        unsafe { w.clk_lp_timer_div_num().bits(divider) }
    });

    // IDF `modem_lpcon_ll_reset_ble_rtc_timer`.
    regs!(MODEM_LPCON)
        .rst_conf()
        .modify(|_, w| w.rst_lp_timer().set_bit());
    regs!(MODEM_LPCON)
        .rst_conf()
        .modify(|_, w| w.rst_lp_timer().clear_bit());

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_lp_timer_en().set_bit());
    regs!(MODEM_LPCON)
        .clk_conf_force_on()
        .modify(|_, w| w.clk_lp_timer_fo().set_bit());
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
