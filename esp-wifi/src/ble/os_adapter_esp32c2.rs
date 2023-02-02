use crate::binary::include::esp_bt_controller_config_t;

pub(crate) static mut ISR_INTERRUPT_4: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut ISR_INTERRUPT_7: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static BLE_CONFIG: esp_bt_controller_config_t = esp_bt_controller_config_t {
    config_version: 0x20220824,
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
    log::trace!(
        "esp_intr_alloc {} {} {:p} {:p} {:p}",
        source,
        flags,
        handler,
        arg,
        ret_handle
    );

    match source {
        4 => ISR_INTERRUPT_4 = (handler, arg),
        7 => ISR_INTERRUPT_7 = (handler, arg),
        _ => panic!("Unexpected interrupt source {}", source),
    }

    0
}

pub(super) fn ble_rtc_clk_init() {
    /*
    // modem_clkrst_reg
    // LP_TIMER_SEL_XTAL32K -> 0
    // LP_TIMER_SEL_XTAL -> 1
    // LP_TIMER_SEL_8M -> 0
    // LP_TIMER_SEL_RTC_SLOW -> 0
    SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, 1, 0, MODEM_CLKRST_LP_TIMER_SEL_XTAL32K_S);
    SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, 1, 1, MODEM_CLKRST_LP_TIMER_SEL_XTAL_S);
    SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, 1, 0, MODEM_CLKRST_LP_TIMER_SEL_8M_S);
    SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, 1, 0, MODEM_CLKRST_LP_TIMER_SEL_RTC_SLOW_S);

    #ifdef CONFIG_XTAL_FREQ_26
        // LP_TIMER_CLK_DIV_NUM -> 130
        SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM, 129, MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM_S);
    #else
        // LP_TIMER_CLK_DIV_NUM -> 250
        SET_PERI_REG_BITS(MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG, MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM, 249, MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM_S);
    #endif // CONFIG_XTAL_FREQ_26

    // MODEM_CLKRST_ETM_CLK_ACTIVE -> 1
    // MODEM_CLKRST_ETM_CLK_SEL -> 0
    SET_PERI_REG_BITS(MODEM_CLKRST_ETM_CLK_CONF_REG, 1, 1, MODEM_CLKRST_ETM_CLK_ACTIVE_S);
    SET_PERI_REG_BITS(MODEM_CLKRST_ETM_CLK_CONF_REG, 1, 0, MODEM_CLKRST_ETM_CLK_SEL_S);
    */

    const MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG: u32 = DR_REG_MODEM_CLKRST_BASE + 0x4;
    const DR_REG_MODEM_CLKRST_BASE: u32 = 0x6004d800;
    const MODEM_CLKRST_LP_TIMER_SEL_XTAL32K_S: u32 = 3;
    const MODEM_CLKRST_LP_TIMER_SEL_XTAL_S: u32 = 2;
    const MODEM_CLKRST_LP_TIMER_SEL_8M_S: u32 = 1;
    const MODEM_CLKRST_LP_TIMER_SEL_RTC_SLOW_S: u32 = 0;
    const MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM: u32 = 0x000000FF;
    const MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM_S: u32 = 4;
    const MODEM_CLKRST_ETM_CLK_CONF_REG: u32 = DR_REG_MODEM_CLKRST_BASE + 0x10;
    const MODEM_CLKRST_ETM_CLK_ACTIVE_S: u32 = 1;
    const MODEM_CLKRST_ETM_CLK_SEL_S: u32 = 0;

    #[inline(always)]
    fn set_peri_reg_bits(reg: u32, bit_map: u32, value: u32, shift: u32) {
        let ptr = reg as *mut u32;
        unsafe {
            ptr.write_volatile(
                ptr.read_volatile() & (!((bit_map) << (shift)))
                    | (((value) & (bit_map)) << (shift)),
            );
        }
    }

    set_peri_reg_bits(
        MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG,
        1,
        0,
        MODEM_CLKRST_LP_TIMER_SEL_XTAL32K_S,
    );
    set_peri_reg_bits(
        MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG,
        1,
        1,
        MODEM_CLKRST_LP_TIMER_SEL_XTAL_S,
    );
    set_peri_reg_bits(
        MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG,
        1,
        0,
        MODEM_CLKRST_LP_TIMER_SEL_8M_S,
    );
    set_peri_reg_bits(
        MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG,
        1,
        0,
        MODEM_CLKRST_LP_TIMER_SEL_RTC_SLOW_S,
    );

    // assume 40MHz xtal
    set_peri_reg_bits(
        MODEM_CLKRST_MODEM_LP_TIMER_CONF_REG,
        MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM,
        249,
        MODEM_CLKRST_LP_TIMER_CLK_DIV_NUM_S,
    );

    set_peri_reg_bits(
        MODEM_CLKRST_ETM_CLK_CONF_REG,
        1,
        1,
        MODEM_CLKRST_ETM_CLK_ACTIVE_S,
    );
    set_peri_reg_bits(
        MODEM_CLKRST_ETM_CLK_CONF_REG,
        1,
        0,
        MODEM_CLKRST_ETM_CLK_SEL_S,
    );
}

pub(super) unsafe extern "C" fn esp_reset_rpa_moudle() {
    log::trace!("esp_reset_rpa_moudle");

    /*
        DPORT_SET_PERI_REG_MASK(SYSTEM_CORE_RST_EN_REG, BLE_RPA_REST_BIT);
        DPORT_CLEAR_PERI_REG_MASK(SYSTEM_CORE_RST_EN_REG, BLE_RPA_REST_BIT);
    */
    const DR_REG_SYSCON_BASE: u32 = 0x60026000;
    const SYSCON_WIFI_RST_EN_REG: u32 = DR_REG_SYSCON_BASE + 0x18;
    const SYSTEM_WIFI_RST_EN_REG: u32 = SYSCON_WIFI_RST_EN_REG;
    const SYSTEM_CORE_RST_EN_REG: u32 = SYSTEM_WIFI_RST_EN_REG;
    const BLE_RPA_REST_BIT: u32 = 1 << 27;

    let ptr = SYSTEM_CORE_RST_EN_REG as *mut u32;
    ptr.write_volatile(ptr.read_volatile() | BLE_RPA_REST_BIT);
    ptr.write_volatile(ptr.read_volatile() & !BLE_RPA_REST_BIT);
}
