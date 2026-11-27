//! Enhanced light sleep: the PMU modem state, in which the CPU sleeps while the Wi-Fi modem
//! receives beacons.
//!
//! In light sleep, the Wi-Fi beacon wakeup source moves the chip from the sleep to the modem state
//! shortly before a beacon. On the way, the REGDMA restores the modem domain, and the PHY link
//! turns the RF on. After the beacon, the chip goes back to sleep, or on to the active state when
//! the access point has frames for the station.
//!
//! When the chip wakes up from the modem state, the RF is on already, and the PHY driver keeps it.
//! Otherwise, the PHY driver turns the RF on and off with the PHY link too, because the modem state
//! can turn the RF on only after the PHY link turned it off.

use core::{ffi::c_int, mem::MaybeUninit, ptr::NonNull};

use allocator_api2::boxed::Box;
#[cfg(any(esp32c5, esp32s31))]
use esp_hal::rtc_cntl::regdma::MODEM_ENTRY;
use esp_hal::rtc_cntl::{
    WakeupSource,
    regdma::{self, LinkConfig, LinkMemory, Module, PHY_ENTRY},
    sleep::WrappedSleepConfig,
};
use esp_phy::PhyClockGuard;
use esp_sync::NonReentrantMutex;
use portable_atomic::{AtomicBool, Ordering};

use crate::{
    compat::malloc::InternalMemory,
    sys::include::{
        esp_wifi_internal_light_sleep_configure,
        esp_wifi_internal_modem_state_configure,
        phy_i2c_master_command_attribute_t,
        phy_i2c_master_command_attribute_t__bindgen_ty_1,
        phy_i2c_master_command_mem_cfg,
    },
};

unsafe extern "C" {
    fn pm_mac_modem_rf_already_enabled() -> bool;
    fn pm_mac_modem_clear_rf_power_state();
    #[cfg(any(esp32c5, esp32s31))]
    fn pm_get_wifimac_regdma_link_selection() -> bool;
}

const fn write_on(id: u16, address: u32, value: u32, mask: u32) -> LinkConfig {
    LinkConfig::write(id, address, value, mask, PHY_ENTRY)
}

const fn wait_on(id: u16, address: u32, value: u32, mask: u32) -> LinkConfig {
    LinkConfig::wait(id, address, value, mask, PHY_ENTRY)
}

const fn write_off(id: u16, address: u32, value: u32, mask: u32) -> LinkConfig {
    write_on(id, address, value, mask).skip(false, true)
}

const fn wait_off(id: u16, address: u32, value: u32, mask: u32) -> LinkConfig {
    wait_on(id, address, value, mask).skip(false, true)
}

const I2C_MST_EN: u32 = 1 << 2;
const BURST_DONE: u32 = 1 << 0;
const TSENS_PU: u32 = 1 << 22;
#[cfg(not(esp32s31))]
const RESETN: u32 = 1 << 30;
const CHAN_DONE: u32 = 1 << 8;
const AGC_DIS: u32 = 1 << 29;
const WIFI_BB_EN: u32 = 1 << 1;
const RX_BLOCK: u32 = 1 << 12;
const RX_BLOCK_DONE: u32 = 0x6000;

const PHY_LINK_LENGTH: usize = cfg_select! {
    esp32c6 => 41,
    esp32c61 => 38,
    esp32c5 => 28,
    esp32s31 => 26,
};

/// Returns the nodes of the PHY link. `rf_on` and `rf_off` are the bursts of the analog I2C master
/// that turn the RF on and off.
///
/// The REGDMA runs the nodes that skip the backup when the RF turns on, and the nodes that skip
/// the restore when the RF turns off. The writes and waits on the PMU date register are delays.
// See `sleep_phy.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_hw_support/lowpower/port>.
const fn phy_link(rf_on: u32, rf_off: u32) -> [LinkConfig; PHY_LINK_LENGTH] {
    cfg_select! {
        esp32c6 => {
            const FE_DATA: u32 = 0x600a_0400;
            const FE_CTRL: u32 = 0x600a_0800;
            const LPCON_CLK_CONF: u32 = 0x600a_f018;
            const I2C_MST_CLK_CONF: u32 = 0x600a_f010;
            const BURST_CONF: u32 = 0x600a_f810;
            const BURST_STATUS: u32 = 0x600a_f814;
            const ANA_CONF0: u32 = 0x600a_f818;
            const TSENS: u32 = 0x6000_e058;
            const RF_PWR: u32 = 0x600b_0154;
            const RF_PWR_ON: u32 = 0xf000_0000;
            const SET_CHAN: u32 = 0x600a_00c0;
            const CHAN_EN: u32 = 1 << 14;
            const SET_CHAN_ST: u32 = 0x600a_00cc;
            const AGC_CONF: u32 = 0x600a_7030;
            const WIFI_BB_CFG: u32 = 0x600a_981c;
            const WDEVTXQ_BLOCK: u32 = 0x600a_4ca8;
            const PMU_DATE: u32 = 0x600b_03fc;
            const WAKEUP_CNTL7: u32 = 0x600b_013c;
            [
                LinkConfig::continuous(0xff00, FE_DATA, FE_DATA, 41, PHY_ENTRY),
                LinkConfig::continuous(0xff01, FE_CTRL, FE_CTRL, 87, PHY_ENTRY),
                write_on(0x00, LPCON_CLK_CONF, I2C_MST_EN, I2C_MST_EN),
                // The I2C master runs on the 160 MHz clock.
                write_on(0x01, I2C_MST_CLK_CONF, 1, 1),
                // Resets the state machine of the frequency setting.
                write_on(0x02, SET_CHAN, 0, RESETN),
                // Turns the RF on. The BBPLL calibration starts first.
                write_on(0x03, ANA_CONF0, 0x8, 0xc),
                write_on(0x04, RF_PWR, RF_PWR_ON, RF_PWR_ON),
                write_on(0x05, TSENS, TSENS_PU, TSENS_PU),
                write_on(0x06, BURST_CONF, rf_on, u32::MAX),
                write_on(0x07, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_on(0x08, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_on(0x09, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_on(0x0a, SET_CHAN, RESETN, RESETN),
                write_on(0x0b, SET_CHAN, CHAN_EN, CHAN_EN),
                write_on(0x0c, SET_CHAN, 0, CHAN_EN),
                write_on(0x0d, PMU_DATE, !CHAN_DONE, CHAN_DONE),
                wait_on(0x0e, PMU_DATE, !CHAN_DONE, CHAN_DONE),
                wait_on(0x0f, SET_CHAN_ST, CHAN_DONE, CHAN_DONE),
                write_on(0x10, WIFI_BB_CFG, WIFI_BB_EN, WIFI_BB_EN),
                write_on(0x11, AGC_CONF, 0, AGC_DIS),
                // Unblocks the receiver.
                write_on(0x12, WDEVTXQ_BLOCK, 0, RX_BLOCK),
                // Turns the RF off.
                write_off(0x13, AGC_CONF, AGC_DIS, AGC_DIS),
                write_off(0x14, WIFI_BB_CFG, 0, WIFI_BB_EN),
                write_off(0x15, SET_CHAN, 0, CHAN_EN),
                write_off(0x16, BURST_CONF, rf_off, u32::MAX),
                write_off(0x17, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_off(0x18, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_off(0x19, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_off(0x1a, TSENS, 0, TSENS_PU),
                write_off(0x1b, RF_PWR, 0, RF_PWR_ON),
                write_off(0x1c, ANA_CONF0, 0x4, 0xc),
                write_off(0x1d, LPCON_CLK_CONF, 0, I2C_MST_EN),
                // Blocks the receiver.
                write_off(0x1e, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x1f, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x20, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                write_off(0x21, WDEVTXQ_BLOCK, RX_BLOCK, RX_BLOCK),
                write_off(0x22, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x23, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x24, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                // The analog wait time of the PMU wakeup.
                write_on(0x25, WAKEUP_CNTL7, 0x0020_0000, 0xffff_0000),
                write_off(0x26, WAKEUP_CNTL7, 0x0973_0000, 0xffff_0000),
            ]
        }
        esp32c61 => {
            const LPCON_CLK_CONF: u32 = 0x600a_f018;
            const BURST_CONF: u32 = 0x600a_f810;
            const BURST_STATUS: u32 = 0x600a_f814;
            const ANA_CONF0: u32 = 0x600a_f818;
            const TSENS: u32 = 0x6000_e058;
            const RF_PWR: u32 = 0x600b_0158;
            const RF_PWR_ON: u32 = 0xf000_0000;
            const SET_CHAN: u32 = 0x600a_001c;
            const CHAN_EN: u32 = 1 << 17;
            const SET_CHAN_ST: u32 = 0x600a_0028;
            const AGC_CONF: u32 = 0x600a_7030;
            const WIFI_BB_CFG: u32 = 0x600a_9c18;
            const WDEVTXQ_BLOCK: u32 = 0x600a_4ca8;
            const PMU_DATE: u32 = 0x600b_01a8;
            const WAKEUP_CNTL7: u32 = 0x600b_0140;
            [
                write_on(0x00, LPCON_CLK_CONF, I2C_MST_EN, I2C_MST_EN),
                // Resets the state machine of the frequency setting.
                write_on(0x01, SET_CHAN, 0, RESETN),
                // Turns the RF on. The BBPLL calibration starts first.
                write_on(0x02, ANA_CONF0, 0x8, 0xc),
                write_on(0x03, RF_PWR, RF_PWR_ON, RF_PWR_ON),
                write_on(0x04, TSENS, TSENS_PU, TSENS_PU),
                write_on(0x05, BURST_CONF, rf_on, u32::MAX),
                write_on(0x06, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_on(0x07, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_on(0x08, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_on(0x09, SET_CHAN, RESETN, RESETN),
                write_on(0x0a, SET_CHAN, CHAN_EN, CHAN_EN),
                write_on(0x0b, SET_CHAN, 0, CHAN_EN),
                write_on(0x0c, PMU_DATE, !CHAN_DONE, CHAN_DONE),
                wait_on(0x0d, PMU_DATE, !CHAN_DONE, CHAN_DONE),
                wait_on(0x0e, SET_CHAN_ST, CHAN_DONE, CHAN_DONE),
                write_on(0x0f, WIFI_BB_CFG, WIFI_BB_EN, WIFI_BB_EN),
                write_on(0x10, AGC_CONF, 0, AGC_DIS),
                // Unblocks the receiver.
                write_on(0x11, WDEVTXQ_BLOCK, 0, RX_BLOCK),
                // Turns the RF off.
                write_off(0x12, AGC_CONF, AGC_DIS, AGC_DIS),
                write_off(0x13, WIFI_BB_CFG, 0, WIFI_BB_EN),
                write_off(0x14, SET_CHAN, 0, CHAN_EN),
                write_off(0x15, BURST_CONF, rf_off, u32::MAX),
                write_off(0x16, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_off(0x17, PMU_DATE, !BURST_DONE, BURST_DONE),
                wait_off(0x18, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_off(0x19, TSENS, 0, TSENS_PU),
                write_off(0x1a, RF_PWR, 0, RF_PWR_ON),
                write_off(0x1b, ANA_CONF0, 0x4, 0xc),
                write_off(0x1c, LPCON_CLK_CONF, 0, I2C_MST_EN),
                // Blocks the receiver.
                write_off(0x1d, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x1e, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x1f, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                write_off(0x20, WDEVTXQ_BLOCK, RX_BLOCK, RX_BLOCK),
                write_off(0x21, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x22, PMU_DATE, !0, RX_BLOCK_DONE),
                wait_off(0x23, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                // The analog wait time of the PMU wakeup.
                write_on(0x24, WAKEUP_CNTL7, 0x0020_0000, 0xffff_0000),
                write_off(0x25, WAKEUP_CNTL7, 0x0973_0000, 0xffff_0000),
            ]
        }
        esp32c5 => {
            const LPCON_CLK_CONF: u32 = 0x600a_f018;
            const BURST_CONF: u32 = 0x600a_f810;
            const BURST_STATUS: u32 = 0x600a_f814;
            const ANA_CONF0: u32 = 0x600a_f818;
            const TSENS: u32 = 0x6000_e058;
            const RF_PWR: u32 = 0x600b_0158;
            const RF_PWR_ON: u32 = 0xf380_0000;
            const SET_CHAN: u32 = 0x600a_001c;
            const CHAN_EN: u32 = 1 << 17;
            const SET_CHAN_ST: u32 = 0x600a_0028;
            const AGC_CONF: u32 = 0x600a_7030;
            const WIFI_BB_CFG: u32 = 0x600a_9c18;
            const WDEVTXQ_BLOCK: u32 = 0x600a_4ca8;
            const WAKEUP_CNTL7: u32 = 0x600b_0140;
            [
                write_on(0x00, LPCON_CLK_CONF, I2C_MST_EN, I2C_MST_EN),
                // Resets the state machine of the frequency setting.
                write_on(0x01, SET_CHAN, 0, RESETN),
                // Turns the RF on. The BBPLL calibration starts first.
                write_on(0x02, ANA_CONF0, 0x8, 0xc),
                write_on(0x03, RF_PWR, RF_PWR_ON, RF_PWR_ON),
                write_on(0x04, TSENS, TSENS_PU, TSENS_PU),
                write_on(0x05, BURST_CONF, rf_on, u32::MAX),
                wait_on(0x06, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_on(0x07, SET_CHAN, RESETN, RESETN),
                write_on(0x08, SET_CHAN, CHAN_EN, CHAN_EN),
                write_on(0x09, SET_CHAN, 0, CHAN_EN),
                wait_on(0x0a, SET_CHAN_ST, CHAN_DONE, CHAN_DONE),
                write_on(0x0b, WIFI_BB_CFG, WIFI_BB_EN, WIFI_BB_EN),
                write_on(0x0c, AGC_CONF, 0, AGC_DIS),
                // Unblocks the receiver.
                write_on(0x0d, WDEVTXQ_BLOCK, 0, RX_BLOCK),
                // Turns the RF off.
                write_off(0x0e, AGC_CONF, AGC_DIS, AGC_DIS),
                write_off(0x0f, WIFI_BB_CFG, 0, WIFI_BB_EN),
                write_off(0x10, SET_CHAN, 0, CHAN_EN),
                write_off(0x11, BURST_CONF, rf_off, u32::MAX),
                wait_off(0x12, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_off(0x13, TSENS, 0, TSENS_PU),
                write_off(0x14, RF_PWR, 0, RF_PWR_ON),
                write_off(0x15, ANA_CONF0, 0x4, 0xc),
                write_off(0x16, LPCON_CLK_CONF, 0, I2C_MST_EN),
                // Blocks the receiver.
                wait_off(0x17, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                write_off(0x18, WDEVTXQ_BLOCK, RX_BLOCK, RX_BLOCK),
                wait_off(0x19, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                // The analog wait time of the PMU wakeup.
                write_on(0x1a, WAKEUP_CNTL7, 0x0020_0000, 0xffff_0000),
                write_off(0x1b, WAKEUP_CNTL7, 0x0973_0000, 0xffff_0000),
            ]
        }
        esp32s31 => {
            const LPCON_CLK_CONF: u32 = 0x2010_f018;
            const BURST_CONF: u32 = 0x2010_f810;
            const BURST_STATUS: u32 = 0x2010_f814;
            const ANA_CONF0: u32 = 0x2010_f818;
            const TSENS: u32 = 0x2081_8000;
            const RF_PWR: u32 = 0x2070_4184;
            const RF_PWR_ON: u32 = 0xffff_0000;
            const SET_CHAN: u32 = 0x2010_001c;
            const CHAN_EN: u32 = 1 << 19;
            const SET_CHAN_ST: u32 = 0x2010_0028;
            const AGC_CONF: u32 = 0x2010_7030;
            const WIFI_BB_CFG: u32 = 0x2010_9c18;
            const WDEVTXQ_BLOCK: u32 = 0x2010_4cac;
            const WAKEUP_CNTL7: u32 = 0x2070_416c;
            [
                write_on(0x00, LPCON_CLK_CONF, I2C_MST_EN, I2C_MST_EN),
                // Turns the RF on. The BBPLL calibration starts first.
                write_on(0x01, ANA_CONF0, 0x8, 0xc),
                write_on(0x02, RF_PWR, RF_PWR_ON, RF_PWR_ON),
                write_on(0x03, TSENS, TSENS_PU, TSENS_PU),
                write_on(0x04, BURST_CONF, rf_on, u32::MAX),
                wait_on(0x05, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_on(0x06, SET_CHAN, CHAN_EN, CHAN_EN),
                write_on(0x07, SET_CHAN, 0, CHAN_EN),
                wait_on(0x08, SET_CHAN_ST, CHAN_DONE, CHAN_DONE),
                write_on(0x09, WIFI_BB_CFG, WIFI_BB_EN, WIFI_BB_EN),
                write_on(0x0a, AGC_CONF, 0, AGC_DIS),
                // Unblocks the receiver.
                write_on(0x0b, WDEVTXQ_BLOCK, 0, RX_BLOCK),
                // Turns the RF off.
                write_off(0x0c, AGC_CONF, AGC_DIS, AGC_DIS),
                write_off(0x0d, WIFI_BB_CFG, 0, WIFI_BB_EN),
                write_off(0x0e, SET_CHAN, 0, CHAN_EN),
                write_off(0x0f, BURST_CONF, rf_off, u32::MAX),
                wait_off(0x10, BURST_STATUS, BURST_DONE, BURST_DONE),
                write_off(0x11, TSENS, 0, TSENS_PU),
                write_off(0x12, RF_PWR, 0, RF_PWR_ON),
                write_off(0x13, ANA_CONF0, 0x4, 0xc),
                write_off(0x14, LPCON_CLK_CONF, 0, I2C_MST_EN),
                // Blocks the receiver.
                wait_off(0x15, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                write_off(0x16, WDEVTXQ_BLOCK, RX_BLOCK, RX_BLOCK),
                wait_off(0x17, WDEVTXQ_BLOCK, 0, RX_BLOCK_DONE),
                // The analog wait time of the PMU wakeup.
                write_on(0x18, WAKEUP_CNTL7, 0x0020_0000, 0xffff_0000),
                write_off(0x19, WAKEUP_CNTL7, 0x0973_0000, 0xffff_0000),
            ]
        }
    }
}

/// The nodes of the PHY link that start and stop the clock of the analog I2C master, with their
/// skip flags for the save and the restore.
///
/// The PHY driver runs this clock itself, so the PHY link skips these nodes when software runs it.
// See `sleep_phy_link_config` in `sleep_phy.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_hw_support/lowpower/port>.
const I2C_MASTER_CLOCK_NODES: &[(u16, bool, bool)] = cfg_select! {
    esp32c6 => &[
        (0x00, true, false),
        (0x01, true, false),
        (0x1d, false, true),
    ],
    esp32c61 => &[(0x00, true, false), (0x1c, false, true)],
    esp32c5 => &[(0x00, true, false), (0x16, false, true)],
    esp32s31 => &[(0x00, true, false), (0x14, false, true)],
};

/// The system clocks of the modem state, before the radios restore.
///
/// AHB_CLK must stay at 40 MHz or below in the modem state. Wi-Fi needs the CPU clock from the PLL,
/// and light sleep keeps the AHB divider of that clock, so no node sets the divider.
// See `sleep_clock.c` of each chip in
// <https://github.com/espressif/esp-idf/tree/4d80c1b29ae/components/esp_hw_support/lowpower/port>.
const MODEM_STATE_CLOCK: &[LinkConfig] = cfg_select! {
    // Light sleep powers the BBPLL down. The nodes power it up and calibrate it.
    esp32s31 => &[
        LinkConfig::write(0x0600, 0x2070_40f4, 0x1100_0000, 0x1100_0000, MODEM_ENTRY),
        LinkConfig::write(0x0601, 0x2070_40f4, 0x0010_0000, 0x0010_0000, MODEM_ENTRY),
        LinkConfig::write(0x0602, 0x2058_9000, 0x4000_0000, 0x4000_0000, MODEM_ENTRY),
        LinkConfig::write(0x0603, 0x2070_401c, 0x1100_0000, 0x1100_0000, MODEM_ENTRY),
        LinkConfig::write(0x0604, 0x2071_0018, 0x4000_0000, 0x4000_0000, MODEM_ENTRY),
        LinkConfig::write(0x0605, 0x2058_7174, 0, 0x80, MODEM_ENTRY),
        LinkConfig::wait(0x0606, 0x2058_7174, 0x40, 0x40, MODEM_ENTRY),
        LinkConfig::write(0x0607, 0x2058_7174, 0x80, 0x80, MODEM_ENTRY),
    ],
    _ => &[],
};

/// Returns the nodes that configure the BBPLL for a 48 MHz XTAL_CLK, with the `burst` of the
/// analog I2C master.
#[cfg(esp32c5)]
fn bbpll_48mhz(burst: u32) -> [LinkConfig; 4] {
    const LPCON_CLK_CONF: u32 = 0x600a_f018;
    [
        LinkConfig::write(0x0106, LPCON_CLK_CONF, I2C_MST_EN, I2C_MST_EN, MODEM_ENTRY),
        LinkConfig::write(0x0107, 0x600a_f810, burst, u32::MAX, MODEM_ENTRY),
        LinkConfig::wait(0x0108, 0x600a_f814, BURST_DONE, BURST_DONE, MODEM_ENTRY),
        LinkConfig::write(0x0109, LPCON_CLK_CONF, 0, I2C_MST_EN, MODEM_ENTRY),
    ]
}

/// Returns the bursts of the analog I2C master for the command types of
/// `phy_i2c_master_command_mem_cfg`: RF off, RF on, and the BBPLL configuration.
// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_phy/src/phy_common.c#L381-L429>.
fn bursts() -> [u32; 3] {
    // Two I2C masters, each with a command of every type.
    let mut commands = [phy_i2c_master_command_attribute_t {
        cmd_type: 0,
        config: phy_i2c_master_command_attribute_t__bindgen_ty_1 {
            start: 0,
            end: 0,
            host_id: 0,
        },
    }; 6];
    let mut size = commands.len() as c_int;
    unsafe { phy_i2c_master_command_mem_cfg(commands.as_mut_ptr(), &mut size) };
    let size = (size.max(0) as usize).min(commands.len());

    let mut bursts = [0; 3];
    for command in &commands[..size] {
        let config = command.config;
        let Some(burst) = bursts.get_mut(command.cmd_type as usize) else {
            continue;
        };
        if config.start == 0xff || config.end == 0xff {
            continue;
        }
        let (start, end) = (config.start as u32, config.end as u32);
        *burst |= if config.host_id != 0 {
            (1 << 31) | (end << 22) | (start << 16)
        } else {
            (1 << 15) | (end << 6) | start
        };
    }
    bursts
}

/// The memory of the REGDMA nodes, while the blob uses the modem state.
static MEMORY: NonReentrantMutex<Option<Box<[MaybeUninit<u32>], InternalMemory>>> =
    NonReentrantMutex::new(None);

/// Keeps the front-end clocks on after the PHY turns off. Light sleep saves the front-end
/// registers when the PHY is already off, and the modem state restores them.
static PHY_CLOCK: NonReentrantMutex<Option<PhyClockGuard<'static>>> = NonReentrantMutex::new(None);

/// Whether the PHY link turned the RF off at least once.
static LINK_DONE: AtomicBool = AtomicBool::new(false);

/// Builds the PHY link and the clock nodes of the modem state, and lets the blob use the modem
/// state.
///
/// Call this after `mac_bb::init`, before Wi-Fi starts.
pub(super) fn init() {
    if !super::mac_bb::is_retained() {
        warn!("Enhanced light sleep needs the Wi-Fi retention memory, so it stays off");
        return;
    }
    #[cfg_attr(not(esp32c5), expect(unused_variables))]
    let [rf_off, rf_on, bbpll_config] = bursts();
    let phy_link = phy_link(rf_on, rf_off);
    let bbpll: &[LinkConfig] = cfg_select! {
        esp32c5 => {
            if esp_hal::clock::xtal_clock() == esp_hal::time::Rate::from_mhz(48) {
                &bbpll_48mhz(bbpll_config)
            } else {
                &[]
            }
        }
        _ => &[],
    };

    let size = regdma::links_size(&phy_link)
        + regdma::links_size(MODEM_STATE_CLOCK)
        + regdma::links_size(bbpll);
    let Ok(mut memory) = Box::try_new_zeroed_slice_in(size / 4, InternalMemory) else {
        warn!("Enhanced light sleep stays off: no memory for the REGDMA nodes");
        return;
    };
    let mut links = unsafe { LinkMemory::new(NonNull::from(&mut *memory).cast(), size) };

    // SAFETY: The tables describe registers that the REGDMA can reach.
    let created = unsafe {
        regdma::create(
            Module::ModemStateClock,
            &[MODEM_STATE_CLOCK, bbpll],
            &mut links,
        )
        .and_then(|()| regdma::create(Module::PhyLink, &[&phy_link], &mut links))
    };
    if let Err(error) = created {
        warn!("Enhanced light sleep stays off: {:?}", error);
        destroy_modules();
        return;
    }
    for module in [Module::ModemStateClock, Module::PhyLink] {
        unwrap!(regdma::attach(module));
    }
    MEMORY.with(|slot| *slot = Some(memory));
    PHY_CLOCK.with(|slot| *slot = Some(esp_phy::enable_phy_clock()));

    esp_phy::set_rf_hooks(Some(esp_phy::RfHooks {
        turn_on: rf_turn_on,
        turn_off: rf_turn_off,
    }));
    unsafe {
        esp_wifi_internal_modem_state_configure(true);
        esp_wifi_internal_light_sleep_configure(true);
    }
    WakeupSource::WifiBeacon.enable_with_hooks(Some(sleep_entry), None);
}

/// Undoes [`init`].
///
/// Call this after `esp_wifi_deinit_internal`, before `mac_bb::deinit`.
pub(super) fn deinit() {
    let Some(memory) = MEMORY.with(Option::take) else {
        return;
    };
    WakeupSource::WifiBeacon.disable();
    unsafe {
        esp_wifi_internal_light_sleep_configure(false);
        esp_wifi_internal_modem_state_configure(false);
    }
    esp_phy::set_rf_hooks(None);
    LINK_DONE.store(false, Ordering::Relaxed);
    destroy_modules();
    drop(memory);
    PHY_CLOCK.with(|slot| *slot = None);
}

fn destroy_modules() {
    regdma::destroy(Module::PhyLink);
    regdma::destroy(Module::ModemStateClock);
}

/// Clears the flag of the blob that the modem state turned the RF on, after `pm_mac_sleep`.
///
/// On the ESP32-C6, the flag can stay set after the modem state, and the PHY would then not wake
/// up the next time.
// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_phy/src/phy_init.c#L270-L280>.
#[cfg(esp32c6)]
pub(super) fn clear_rf_flag() {
    if unsafe { pm_mac_modem_rf_already_enabled() } {
        unsafe { pm_mac_modem_clear_rf_power_state() };
    }
}

fn sleep_entry(config: &mut WrappedSleepConfig<'_>) {
    // Until the PHY link turned the RF off once, the modem state cannot turn the RF on.
    if !config.is_deep_sleep() && !LINK_DONE.load(Ordering::Relaxed) {
        config.reject_sleep();
    }
}

// See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_phy/src/phy_init.c#L302-L334>
// and <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_phy/src/phy_init.c#L391-L407>.

fn rf_turn_on() -> bool {
    if unsafe { pm_mac_modem_rf_already_enabled() } {
        // The modem state turned the RF on. The temperature sensor needs another power-up after
        // the modem state.
        power_up_temperature_sensor();
        return true;
    }
    LINK_DONE.load(Ordering::Relaxed) && run_phy_link(false)
}

fn rf_turn_off() -> bool {
    unsafe { pm_mac_modem_clear_rf_power_state() };
    let done = run_phy_link(true);
    if done {
        LINK_DONE.store(true, Ordering::Relaxed);
    }
    done
}

/// Runs the PHY link from software. Returns whether the REGDMA ran every node.
fn run_phy_link(backup: bool) -> bool {
    for &(id, ..) in I2C_MASTER_CLOCK_NODES {
        regdma::set_skip(Module::PhyLink, id, true, true);
    }
    // The chip can wake up from the modem state before the REGDMA turned the RF off, and the modem
    // sleep flag of the MAC stays set. For that case, the blob asks for the Wi-Fi MAC trigger,
    // which clears the flag.
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29ae/components/esp_phy/src/phy_init.c#L314-L324>.
    let wifi_mac = cfg_select! {
        any(esp32c5, esp32s31) => unsafe { pm_get_wifimac_regdma_link_selection() },
        _ => false,
    };
    let result = regdma::run_phy_link(backup, wifi_mac);
    for &(id, skip_backup, skip_restore) in I2C_MASTER_CLOCK_NODES {
        regdma::set_skip(Module::PhyLink, id, skip_backup, skip_restore);
    }
    if let Err(error) = result {
        warn!("The PHY link failed: {:?}", error);
    }
    result.is_ok()
}

fn power_up_temperature_sensor() {
    cfg_select! {
        temp_sensor_register_block_apb_saradc => {
            regs!(APB_SARADC)
                .tsens_ctrl()
                .modify(|_, w| w.pu().set_bit());
        }
        temp_sensor_register_block_lp_tsens => {
            regs!(LP_TSENS).ctrl().modify(|_, w| w.power_up().set_bit());
        }
    }
}
