use core::sync::atomic::{AtomicBool, Ordering};

pub(crate) fn enable_wifi(en: bool) {
    regs!(MODEM_SYSCON)
        .clk_conf1()
        .modify(|_, w| w.clk_wifimac_en().bit(en));
}

pub(crate) fn enable_ieee802154(en: bool) {
    enable_bt_ieee802154_common(en);
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zbmac_en().bit(en)
    });
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
    static INITED: AtomicBool = AtomicBool::new(false);

    if INITED.swap(true, Ordering::SeqCst) {
        return;
    }
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
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
