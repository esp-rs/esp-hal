pub(crate) fn enable_wifi(en: bool) {
    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifimac_en().bit(en);
        w.clk_wifibb_22m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en)
    });

    // The radios share the coexistence clock, as in ESP-IDF's `COEXIST_CLOCK_DEPS`, and the other
    // radios can be active while Wi-Fi dozes.
    enable_coex(en);
}

pub(crate) fn enable_ieee802154(en: bool) {
    enable_bt_ieee802154_common(en);
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zbmac_en().bit(en)
    });
    enable_coex(en);
}

fn enable_coex(en: bool) {
    fn enable(en: bool) {
        regs!(MODEM_LPCON)
            .clk_conf()
            .modify(|_, w| w.clk_coex_en().bit(en));
    }

    use crate::radio_clocks::Refcount;

    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, enable);
}

pub(crate) fn enable_bt_ieee802154_common(en: bool) {
    fn enable_common(en: bool) {
        regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
            w.clk_etm_en().bit(en);
            w.clk_modem_sec_en().bit(en);
            w.clk_modem_sec_ecb_en().bit(en);
            w.clk_modem_sec_ccm_en().bit(en);
            w.clk_modem_sec_bah_en().bit(en);
            w.clk_modem_sec_apb_en().bit(en);
            w.clk_ble_timer_en().bit(en)
        });

        regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
            w.clk_bt_apb_en().bit(en);
            w.clk_btbb_en().bit(en);
            w.clk_btmac_en().bit(en)
        });
    }

    use crate::radio_clocks::Refcount;

    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, enable_common);
}

pub(crate) fn enable_bt(en: bool) {
    if en {
        ble_rtc_clk_init();
        // IDF `trng_ll_enable` / `bootloader_random_enable`. Advertising
        // enable is the first `ble_ll_rand()`; that path busy-waits on
        // HW samples. S31 CONF.sample_enable is off at reset.
        enable_trng();
    }

    enable_bt_ieee802154_common(en);
    enable_coex(en);
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

pub(crate) fn reset_wifi_mac() {
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifimac().set_bit());
    regs!(MODEM_SYSCON)
        .modem_rst_conf()
        .modify(|_, w| w.rst_wifimac().clear_bit());
}

pub(crate) fn init_clocks() {
    // Every radio and the PHY need the modem on the PLL clock and `clk_wifibb_80x1_en`, before
    // their first register access. Enable the PLL gate, then select it. The values are from
    // ESP-IDF: https://github.com/espressif/esp-idf/blob/69b860b6b8d51eda8ec68eb820833d569dd796ab/components/hal/esp32s31/modem_clock_hal.c#L120-L136
    let modem_conf = regs!(HP_SYS_CLKRST).modem_conf();
    modem_conf.write(|w| unsafe { w.bits(0x15) });
    modem_conf.write(|w| unsafe { w.bits(0x1d) });

    regs!(MODEM_SYSCON)
        .clk_conf1()
        .modify(|_, w| w.clk_wifibb_80x1_en().set_bit());
}

pub(crate) fn deinit_clocks() {
    // The clocks of `init_clocks` stay on.
}

pub(crate) fn ble_rtc_clk_init() {
    // `BLE_LP_CLK` is programmed by the clock tree.
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
