use crate::system::{RadioClockControl, RadioClockController, RadioPeripherals};

const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x00000406;
const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x00030800;

impl RadioClockController for RadioClockControl {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt => bt_clock_enable(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt => bt_clock_disable(),
            RadioPeripherals::Wifi => wifi_clock_disable(),
        }
    }

    fn reset_mac(&mut self) {
        reset_mac();
    }

    fn init_clocks(&mut self) {
        init_clocks();
    }

    fn ble_rtc_clk_init(&mut self) {
        // nothing for this target
    }

    fn reset_rpa(&mut self) {
        // nothing for this target
    }
}

fn enable_phy() {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn bt_clock_enable() {
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_BT_EN_M) });
}

fn bt_clock_disable() {
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_BT_EN_M) });
}

fn wifi_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn wifi_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u8 = 1 << 2;
    let dport = unsafe { &*esp32::DPORT::PTR };
    dport
        .core_rst_en
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() | SYSTEM_MAC_RST) });
    dport
        .core_rst_en
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    let dport = unsafe { &*esp32::DPORT::PTR };

    const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;

    const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN_M
        | DPORT_WIFI_CLK_BT_EN_M
        | DPORT_WIFI_CLK_UNUSED_BIT5
        | DPORT_WIFI_CLK_UNUSED_BIT12
        | DPORT_WIFI_CLK_SDIOSLAVE_EN
        | DPORT_WIFI_CLK_SDIO_HOST_EN
        | DPORT_WIFI_CLK_EMAC_EN;

    dport
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK) });
}
