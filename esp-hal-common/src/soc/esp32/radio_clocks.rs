use crate::system::{RadioClockControl, RadioClockController, RadioPeripherals};

const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const SYSTEM_WIFI_CLK_EN: u32 = 0xFFFFFFFF;

impl RadioClockController for RadioClockControl {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_enable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_disable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_disable(),
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
    let system = unsafe { &*esp32::DPORT::PTR };
    system
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    let system = unsafe { &*esp32::DPORT::PTR };
    system
        .wifi_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn common_wifi_bt_clock_enable() {
    let system = unsafe { &*esp32::DPORT::PTR };
    system
        .perip_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_EN) });
}

fn common_wifi_bt_clock_disable() {
    let system = unsafe { &*esp32::DPORT::PTR };
    system
        .perip_clk_en
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_EN) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u8 = 1 << 2;
    let syscon = unsafe { &*esp32::DPORT::PTR };
    syscon
        .core_rst_en
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() | SYSTEM_MAC_RST) });
    syscon
        .core_rst_en
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    let syscon = unsafe { &*esp32::DPORT::PTR };
    syscon
        .wifi_clk_en
        .modify(|_, w| unsafe { w.bits(0xffffffff) });
}
