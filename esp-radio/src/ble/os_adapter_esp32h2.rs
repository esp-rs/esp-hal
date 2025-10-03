use procmacros::BuilderLite;

use super::*;
use crate::{
    binary::include::esp_bt_controller_config_t,
    hal::{
        clock::ModemClockController,
        interrupt,
        peripherals::{BT, Interrupt},
    },
};

pub(crate) static mut ISR_INTERRUPT_15: (*mut c_void, *mut c_void) =
    (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut ISR_INTERRUPT_3: (*mut c_void, *mut c_void) =
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
    // keep them aligned with BT_CONTROLLER_INIT_CONFIG_DEFAULT in ESP-IDF
    // ideally _some_ of these values should be configurable
    esp_bt_controller_config_t {
        config_version: 0x20250606,
        ble_ll_resolv_list_size: 4,
        ble_hci_evt_hi_buf_count: 30,
        ble_hci_evt_lo_buf_count: 8,
        ble_ll_sync_list_cnt: 5,
        ble_ll_sync_cnt: 0,
        ble_ll_rsp_dup_list_count: 20,
        ble_ll_adv_dup_list_count: 20,
        ble_ll_tx_pwr_dbm: 9,
        rtc_freq: 32000,
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
        enable_tx_cca: 0,
        cca_rssi_thresh: (256 - 50) as u8,
        sleep_en: 0,
        coex_phy_coded_tx_rx_time_limit: 0,
        dis_scan_backoff: 0,
        ble_scan_classify_filter_enable: 1,
        cca_drop_mode: 0,  //???
        cca_low_tx_pwr: 0, //???
        main_xtal_freq: 32,
        ignore_wl_for_direct_adv: 0,
        cpu_freq_mhz: 96,
        enable_pcl: 0,
        // version_num: 0,
        csa2_select: 1,
        enable_csr: 0,
        ble_aa_check: 0,
        ble_llcp_disc_flag: 0, /* (BT_CTRL_BLE_LLCP_CONN_UPDATE |
                                * BT_CTRL_BLE_LLCP_CHAN_MAP_UPDATE |
                                * BT_CTRL_BLE_LLCP_PHY_UPDATE) */
        scan_backoff_upperlimitmax: 256,
        ble_chan_ass_en: 0,
        ble_data_lenth_zero_aux: 0,
        vhci_enabled: 0,
        ptr_check_enabled: 0,
        ble_adv_tx_options: 0,
        skip_unnecessary_checks_en: 0,
        fast_conn_data_tx_en: 0,
        ch39_txpwr: 9,
        adv_rsv_cnt: 1,
        conn_rsv_cnt: 2,
        config_magic: 0x5A5AA5A5,
    }
}

pub(crate) fn bt_periph_module_enable() {
    // stealing BT is safe, since it is passed into the initialization function of the BLE
    // controller.
    let mut bt = unsafe { BT::steal() };
    bt.enable_modem_clock(true);
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
            3 => {
                ISR_INTERRUPT_3 = (handler, arg);
                unwrap!(interrupt::enable(
                    Interrupt::LP_BLE_TIMER,
                    interrupt::Priority::Priority1
                ));
            }
            15 => {
                ISR_INTERRUPT_15 = (handler, arg);
                unwrap!(interrupt::enable(
                    Interrupt::BT_MAC,
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
