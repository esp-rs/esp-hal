use crate::{
    binary::include::esp_bt_controller_config_t,
    hal::{
        clock::RadioClockController,
        interrupt,
        peripherals::{Interrupt, RADIO_CLK},
    },
};

pub(crate) static mut ISR_INTERRUPT_4: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut ISR_INTERRUPT_7: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static BLE_CONFIG: esp_bt_controller_config_t = esp_bt_controller_config_t {
    config_version: 0x20231124,
    ble_ll_resolv_list_size: 4,
    ble_hci_evt_hi_buf_count: 30,
    ble_hci_evt_lo_buf_count: 8,
    ble_ll_sync_list_cnt: 5,
    ble_ll_sync_cnt: 20,
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
    controller_task_prio: 253,
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
    main_xtal_freq: 40,
    version_num: 0, // chips revision: EFUSE.blk0_rdata5.rd_wafer_version_minor
    ignore_wl_for_direct_adv: 0,
    csa2_select: 0,
    config_magic: 0x5A5AA5A5,
};

pub(crate) fn bt_periph_module_enable() {
    // nothing
}

pub(crate) fn disable_sleep_mode() {
    // nothing
}

pub(super) unsafe extern "C" fn esp_intr_alloc(
    source: u32,
    flags: u32,
    handler: *mut crate::binary::c_types::c_void,
    arg: *mut crate::binary::c_types::c_void,
    ret_handle: *mut *mut crate::binary::c_types::c_void,
) -> i32 {
    trace!(
        "esp_intr_alloc {} {} {:?} {:?} {:?}",
        source,
        flags,
        handler,
        arg,
        ret_handle
    );

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

    0
}

pub(super) fn ble_rtc_clk_init() {
    // stealing RADIO_CLK is safe since it is passed (as mutable reference or by
    // value) into `init`
    let radio_clocks = unsafe { RADIO_CLK::steal() };
    RadioClockController::new(radio_clocks).ble_rtc_clk_init();
}

pub(super) unsafe extern "C" fn esp_reset_rpa_moudle() {
    trace!("esp_reset_rpa_moudle");
    // stealing RADIO_CLK is safe since it is passed (as mutable reference or by
    // value) into `init`
    let radio_clocks = unsafe { RADIO_CLK::steal() };
    RadioClockController::new(radio_clocks).reset_rpa();
}
