use crate::sys::include::esp_phy_init_data_t;

const CONFIG_ESP32_PHY_MAX_TX_POWER: u8 = 20;

const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;

pub(crate) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    esp_hal::peripherals::APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe {
            if enable {
                w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
            } else {
                w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
            }
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
        0x00,
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x50),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x50),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x50),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x4c),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x4c),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x4c),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x48),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x44),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x4a),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x46),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x46),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 0, 0x42),
        0x00,
        0x00,
        0x00,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
        0xff,
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
        0x74,
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
    ],
};
