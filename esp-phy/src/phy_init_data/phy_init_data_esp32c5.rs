use crate::sys::include::esp_phy_init_data_t;

const CONFIG_ESP_PHY_MAX_TX_POWER: u8 = 20;

#[allow(unused)]
pub(crate) fn enable_phy(en: bool) {
    esp_hal::peripherals::MODEM_SYSCON::regs()
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_modem_apb_st_map().bits(6);
            w.clk_modem_peri_st_map().bits(4);
            w.clk_wifi_st_map().bits(6);
            w.clk_bt_st_map().bits(4);
            w.clk_fe_st_map().bits(6);
            w.clk_zb_st_map().bits(4)
        });

    esp_hal::peripherals::MODEM_LPCON::regs()
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_lp_apb_st_map().bits(6);
            w.clk_i2c_mst_st_map().bits(6);
            w.clk_coex_st_map().bits(6);
            w.clk_wifipwr_st_map().bits(6)
        });

    esp_hal::peripherals::MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));

    esp_hal::peripherals::MODEM_SYSCON::regs()
        .clk_conf1()
        .modify(|_, w| {
            w.clk_wifi_apb_en().bit(en);
            w.clk_wifibb_22m_en().bit(en);
            w.clk_fe_40m_en().bit(en);
            w.clk_fe_80m_en().bit(en);
            w.clk_wifibb_44m_en().bit(en);
            w.clk_wifimac_en().bit(en)
        });

    esp_hal::peripherals::MODEM_SYSCON::regs()
        .clk_conf1()
        .modify(|r, w| unsafe { w.bits(r.bits() | 0x1fb) });

    esp_hal::peripherals::MODEM_SYSCON::regs()
        .clk_conf1()
        .modify(|_, w| {
            w.clk_fe_apb_en().bit(en);
            w.clk_fe_80m_en().bit(en);
            w.clk_fe_160m_en().bit(en);
            w.clk_fe_dac_en().bit(en);
            w.clk_fe_pwdet_adc_en().bit(en);
            w.clk_fe_adc_en().bit(en)
        });
}

const fn limit(val: u8, low: u8, high: u8) -> u8 {
    if val < low {
        low
    } else if val > high {
        high
    } else {
        val
    }
}

pub(crate) static PHY_INIT_DATA_DEFAULT: esp_phy_init_data_t = esp_phy_init_data_t {
    params: [
        0x00,
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x50),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x50),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x4C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x4C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x4C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x4C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x4C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x3C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x3C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x3C),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x38),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x40),
        limit(CONFIG_ESP_PHY_MAX_TX_POWER * 4, 0, 0x3C),
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0x0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0xF5,
    ],
};
