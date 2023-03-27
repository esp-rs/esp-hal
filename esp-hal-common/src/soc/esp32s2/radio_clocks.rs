use crate::system::{RadioClockControl, RadioClockController, RadioPeripherals};

// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10,
// 19, 20, 21, 22, 23
const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;
// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
// from experiments `0x00FB9FCF` is not enough for esp-wifi to work
const SYSTEM_WIFI_CLK_EN: u32 = 0xFFFFFFFF;

impl RadioClockController for RadioClockControl {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
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
    let system = unsafe { &*esp32s2::SYSTEM::PTR };
    system
        .perip_clk_en1
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    let system = unsafe { &*esp32s2::SYSTEM::PTR };
    system
        .perip_clk_en1
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn common_wifi_bt_clock_enable() {
    let system = unsafe { &*esp32s2::SYSTEM::PTR };
    system
        .perip_clk_en1
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_EN) });
}

fn common_wifi_bt_clock_disable() {
    let system = unsafe { &*esp32s2::SYSTEM::PTR };
    system
        .perip_clk_en1
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_EN) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_rst_en
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    syscon
        .wifi_rst_en
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_clk_en
        .modify(|_, w| unsafe { w.bits(0xffffffff) });
}
