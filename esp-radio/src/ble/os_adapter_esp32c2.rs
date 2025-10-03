use procmacros::BuilderLite;

use super::*;
use crate::{
    binary::include::esp_bt_controller_config_t,
    hal::{
        clock::{Clock, RtcClock},
        efuse::Efuse,
        interrupt,
        peripherals::{BT, Interrupt},
    },
};

pub(crate) static mut ISR_INTERRUPT_4: (*mut c_void, *mut c_void) =
    (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut ISR_INTERRUPT_7: (*mut c_void, *mut c_void) =
    (core::ptr::null_mut(), core::ptr::null_mut());

/// Bluetooth controller configuration.
#[derive(BuilderLite, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Config {
    /// The priority of the RTOS task.
    task_priority: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // same priority as the wifi task, when using esp-rtos (I'm assuming it's MAX_PRIO - 2)
            task_priority: 29,
        }
    }
}

pub(crate) fn create_ble_config(config: &Config) -> esp_bt_controller_config_t {
    let main_xtal_freq = RtcClock::xtal_freq().mhz() as u8;

    let rtc_freq = if main_xtal_freq == 26 { 40000 } else { 32000 };

    // keep them aligned with BT_CONTROLLER_INIT_CONFIG_DEFAULT in ESP-IDF
    // ideally _some_ of these values should be configurable
    esp_bt_controller_config_t {
        config_version: 0x20250310,
        ble_ll_resolv_list_size: 4,
        ble_hci_evt_hi_buf_count: 30,
        ble_hci_evt_lo_buf_count: 8,
        ble_ll_sync_list_cnt: 5,
        ble_ll_sync_cnt: 0,
        ble_ll_rsp_dup_list_count: 20,
        ble_ll_adv_dup_list_count: 20,
        ble_ll_tx_pwr_dbm: 9,
        rtc_freq,
        ble_ll_sca: 60,
        ble_ll_scan_phy_number: 1,
        ble_ll_conn_def_auth_pyld_tmo: 3000,
        ble_ll_jitter_usecs: 16,
        ble_ll_sched_max_adv_pdu_usecs: 376,
        ble_ll_sched_direct_adv_max_usecs: 502,
        ble_ll_sched_adv_max_usecs: 852,
        ble_scan_rsp_data_max_len: 31,
        ble_ll_cfg_num_hci_cmd_pkts: 1,
        ble_ll_ctrl_proc_timeout_ms: 40000,
        nimble_max_connections: 2,
        ble_whitelist_size: 12,
        ble_acl_buf_size: 255,
        ble_acl_buf_count: 24,
        ble_hci_evt_buf_size: 70,
        ble_multi_adv_instances: 1,
        ble_ext_adv_max_size: 31,
        controller_task_stack_size: 4096,
        controller_task_prio: config.task_priority,
        controller_run_cpu: 0,
        enable_qa_test: 0,
        enable_bqb_test: 0,
        enable_uart_hci: 0,
        ble_hci_uart_port: 0,
        ble_hci_uart_baud: 0,
        ble_hci_uart_data_bits: 0,
        ble_hci_uart_stop_bits: 0,
        ble_hci_uart_flow_ctrl: 0,
        ble_hci_uart_uart_parity: 0,
        enable_tx_cca: 0,
        cca_rssi_thresh: (256 - 50) as u8,
        sleep_en: 0,
        coex_phy_coded_tx_rx_time_limit: 0,
        dis_scan_backoff: 0,
        ble_scan_classify_filter_enable: 0,
        cca_drop_mode: 0,  //???
        cca_low_tx_pwr: 0, //???
        main_xtal_freq,
        version_num: Efuse::minor_chip_version(),
        ignore_wl_for_direct_adv: 0,
        csa2_select: 0,
        ble_aa_check: 0,
        ble_llcp_disc_flag: 0, /* (BT_CTRL_BLE_LLCP_CONN_UPDATE |
                                * BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE |
                                * BT_CTRL_BLE_LLCP_PHY_UPDATE */
        scan_backoff_upperlimitmax: 256,
        vhci_enabled: 0,
        config_magic: 0x5A5AA5A5,
    }
}

pub(crate) fn bt_periph_module_enable() {
    // nothing
}

pub(crate) fn disable_sleep_mode() {
    // nothing
}

pub(super) unsafe extern "C" fn esp_intr_alloc(
    source: u32,
    flags: u32,
    handler: *mut c_void,
    arg: *mut c_void,
    ret_handle: *mut *mut c_void,
) -> i32 {
    trace!(
        "esp_intr_alloc {} {} {:?} {:?} {:?}",
        source, flags, handler, arg, ret_handle
    );

    unsafe {
        match source {
            4 => {
                ISR_INTERRUPT_4 = (handler, arg);
                unwrap!(interrupt::enable(
                    Interrupt::BT_MAC,
                    interrupt::Priority::Priority1
                ));
            }
            7 => {
                ISR_INTERRUPT_7 = (handler, arg);
                unwrap!(interrupt::enable(
                    Interrupt::LP_TIMER,
                    interrupt::Priority::Priority1
                ));
            }
            _ => panic!("Unexpected interrupt source {}", source),
        }
    }

    0
}

pub(super) fn ble_rtc_clk_init() {
    // stealing BT is safe, since it is passed into the initialization function of the BLE
    // controller.
    let mut bt = unsafe { BT::steal() };
    bt.ble_rtc_clk_init();
}

pub(super) unsafe extern "C" fn esp_reset_rpa_moudle() {
    trace!("esp_reset_rpa_moudle");
    // stealing BT is safe, since it is passed into the initialization function of the BLE
    // controller.
    let mut bt = unsafe { BT::steal() };
    bt.reset_rpa();
}

// Provide the symbol for < eco4 to make the linker happy
#[unsafe(no_mangle)]
unsafe fn g_ble_lll_rfmgmt_env_p() -> *mut esp_wifi_sys::c_types::c_void {
    // prevent "undefined symbol: g_ble_lll_rfmgmt_env_p" for ESP32-C2 < eco4
    unreachable!()
}
