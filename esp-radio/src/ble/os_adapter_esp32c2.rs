use procmacros::BuilderLite;

use super::*;
use crate::{
    ble::InvalidConfigError,
    hal::{
        clock::Clocks,
        efuse::Efuse,
        interrupt,
        peripherals::{BT, Interrupt},
    },
    sys::include::esp_bt_controller_config_t,
};

pub(crate) static mut ISR_INTERRUPT_4: (*mut c_void, *mut c_void) =
    (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut ISR_INTERRUPT_7: (*mut c_void, *mut c_void) =
    (core::ptr::null_mut(), core::ptr::null_mut());

/// Transmission Power Level
#[derive(Default, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxPower {
    /// -24 dBm
    N24,
    /// -21 dBm
    N21,
    /// -18 dBm
    N18,
    /// -15 dBm
    N15,
    /// -12 dBm
    N12,
    /// -9 dBm
    N9,
    /// -6 dBm
    N6,
    /// -3 dBm
    N3,
    /// 0 dBm
    N0,
    /// 3 dBm
    P3,
    /// 6 dBm
    P6,
    /// 9 dBm
    #[default]
    P9,
    /// 12 dBm
    P12,
    /// 15 dBm
    P15,
    /// 18 dBm
    P18,
    /// 20 dBm
    P20,
}

#[allow(dead_code)]
impl TxPower {
    fn idx(self) -> esp_power_level_t {
        match self {
            Self::N24 => esp_power_level_t_ESP_PWR_LVL_N24,
            Self::N21 => esp_power_level_t_ESP_PWR_LVL_N21,
            Self::N18 => esp_power_level_t_ESP_PWR_LVL_N18,
            Self::N15 => esp_power_level_t_ESP_PWR_LVL_N15,
            Self::N12 => esp_power_level_t_ESP_PWR_LVL_N12,
            Self::N9 => esp_power_level_t_ESP_PWR_LVL_N9,
            Self::N6 => esp_power_level_t_ESP_PWR_LVL_N6,
            Self::N3 => esp_power_level_t_ESP_PWR_LVL_N3,
            Self::N0 => esp_power_level_t_ESP_PWR_LVL_N0,
            Self::P3 => esp_power_level_t_ESP_PWR_LVL_P3,
            Self::P6 => esp_power_level_t_ESP_PWR_LVL_P6,
            Self::P9 => esp_power_level_t_ESP_PWR_LVL_P9,
            Self::P12 => esp_power_level_t_ESP_PWR_LVL_P12,
            Self::P15 => esp_power_level_t_ESP_PWR_LVL_P15,
            Self::P18 => esp_power_level_t_ESP_PWR_LVL_P18,
            Self::P20 => esp_power_level_t_ESP_PWR_LVL_P20,
        }
    }

    fn dbm(self) -> i8 {
        match self {
            Self::N24 => -24,
            Self::N21 => -21,
            Self::N18 => -18,
            Self::N15 => -15,
            Self::N12 => -12,
            Self::N9 => -9,
            Self::N6 => -6,
            Self::N3 => -3,
            Self::N0 => 0,
            Self::P3 => 3,
            Self::P6 => 6,
            Self::P9 => 9,
            Self::P12 => 12,
            Self::P15 => 15,
            Self::P18 => 18,
            Self::P20 => 20,
        }
    }
}

/// Bluetooth controller configuration.
#[derive(BuilderLite, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The priority of the RTOS task.
    task_priority: u8,

    /// The stack size of the RTOS task.
    task_stack_size: u16,

    /// The maximum number of simultaneous connections.
    ///
    /// Range: 1 - 2
    max_connections: u16,

    /// Enable QA test mode.
    qa_test_mode: bool,

    /// Enable BQB test mode.
    bqb_test: bool,

    /// Size of the resolvable private address list.
    ll_resolv_list_size: u16,

    /// Maximum number of periodic advertiser list.
    ///
    /// Range: 1 - 5
    ll_sync_list_cnt: u8,

    /// Maximum number of periodic advertising syncs.
    ///
    /// Range: 0 - 3
    ll_sync_cnt: u8,

    /// Count of duplicated lists for scan response packets
    ///
    /// Range: 1 - 100
    ll_rsp_dup_list_count: u16,

    /// Count of duplicated lists for advertising packets
    ///
    /// Range: 1 - 100
    ll_adv_dup_list_count: u16,

    /// Default TX power.
    default_tx_power: TxPower,

    /// High Priority HCI Event Buffer count
    hci_high_buffer_count: u16,

    /// Low Priority HCI Event Buffer count
    hci_low_buffer_count: u16,

    /// Size of the whitelist.
    whitelist_size: u8,

    /// Buffer size of ACL (Asynchronous Connection-Less) data
    acl_buf_size: u16,

    /// Buffer count of ACL data
    acl_buf_count: u16,

    /// Buffer size for HCI event data
    hci_evt_buf_size: u16,

    /// Maximum number of extended advertising instances.
    multi_adv_instances: u16,

    /// Maximum size of extended advertising data
    ///
    /// Range: 0 - 1650
    ext_adv_max_size: u16,

    /// Disable scan backoff
    dis_scan_backoff: bool,

    /// The value of upperlimitmax during scan backoff procedure
    ///
    /// The value of upperlimitmax needs to be a power of 2.
    ///
    /// Range: 1 - 256
    scan_backoff_max: u16,

    /// Enables verification of the Access Address within the `CONNECT_IND` PDU.
    ///
    /// Enabling this option will add stricter verification of the Access Address in the
    /// `CONNECT_IND` PDU. This improves security by ensuring that only connection requests with
    /// valid Access Addresses are accepted. If disabled, only basic checks are applied,
    /// improving compatibility.
    verify_access_address: bool,

    /// Enable BLE Clear Channel Assessment (CCA).
    cca: bool,

    /// Absolute value of hardware-triggered CCA threshold.
    ///
    /// The CCA threshold is always negative.
    ///
    /// If the channel assessment result exceeds the CCA threshold (e.g. -75 dBm), indicating
    /// the channel is busy, the hardware will not transmit packets on that channel.
    ///
    /// Range: 20 dBm - 100 dBm
    cca_threshold: u8,

    /// Disconnect when Instant Passed (0x28) occurs during ACL connection update.
    disconnect_llcp_conn_update: bool,

    /// Disconnect when Instant Passed (0x28) occurs during ACL channel map update.
    disconnect_llcp_chan_map_update: bool,

    /// Disconnect when Instant Passed (0x28) occurs during ACL PHY update.
    disconnect_llcp_phy_update: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // same priority as the wifi task, when using esp-rtos (I'm assuming it's MAX_PRIO - 2)
            task_priority: 29,
            task_stack_size: CONFIG_BT_LE_CONTROLLER_TASK_STACK_SIZE as _,
            max_connections: CONFIG_BT_LE_MAX_CONNECTIONS as _,
            qa_test_mode: false,
            bqb_test: false,
            ll_resolv_list_size: CONFIG_BT_LE_LL_RESOLV_LIST_SIZE as _,
            ll_sync_list_cnt: CONFIG_BT_LE_MAX_PERIODIC_ADVERTISER_LIST as _,
            ll_sync_cnt: CONFIG_BT_LE_MAX_PERIODIC_SYNCS as _,
            ll_rsp_dup_list_count: CONFIG_BT_LE_LL_DUP_SCAN_LIST_COUNT as _,
            ll_adv_dup_list_count: CONFIG_BT_LE_LL_DUP_SCAN_LIST_COUNT as _,
            default_tx_power: TxPower::default(),
            hci_high_buffer_count: CONFIG_BT_LE_HCI_EVT_HI_BUF_COUNT as _,
            hci_low_buffer_count: CONFIG_BT_LE_HCI_EVT_LO_BUF_COUNT as _,
            whitelist_size: CONFIG_BT_LE_WHITELIST_SIZE as _,
            acl_buf_size: CONFIG_BT_LE_ACL_BUF_SIZE as _,
            acl_buf_count: CONFIG_BT_LE_ACL_BUF_COUNT as _,
            hci_evt_buf_size: CONFIG_BT_LE_HCI_EVT_BUF_SIZE as _,
            multi_adv_instances: CONFIG_BT_LE_MAX_EXT_ADV_INSTANCES as _,
            ext_adv_max_size: CONFIG_BT_LE_EXT_ADV_MAX_SIZE as _,
            dis_scan_backoff: false,
            scan_backoff_max: CONFIG_BT_CTRL_SCAN_BACKOFF_UPPERLIMITMAX as _,
            verify_access_address: false,
            disconnect_llcp_conn_update: false,
            disconnect_llcp_chan_map_update: false,
            disconnect_llcp_phy_update: false,
            cca_threshold: 65,
            cca: false,
        }
    }
}

impl Config {
    pub(crate) fn validate(&self) -> Result<(), InvalidConfigError> {
        crate::ble::validate_range!(self, max_connections, 1, 2);
        crate::ble::validate_range!(self, ll_sync_cnt, 0, 3);
        crate::ble::validate_range!(self, ll_sync_list_cnt, 1, 5);
        crate::ble::validate_range!(self, ll_rsp_dup_list_count, 1, 100);
        crate::ble::validate_range!(self, ll_adv_dup_list_count, 1, 100);
        crate::ble::validate_range!(self, whitelist_size, 1, 31);
        crate::ble::validate_range!(self, multi_adv_instances, 0, 4);
        crate::ble::validate_range!(self, ext_adv_max_size, 0, 1650);
        crate::ble::validate_range!(self, scan_backoff_max, 1, 256);
        crate::ble::validate_range!(self, cca_threshold, 20, 100);
        Ok(())
    }
}

pub(crate) fn create_ble_config(config: &Config) -> esp_bt_controller_config_t {
    let main_xtal_freq = Clocks::get().xtal_clock.as_mhz() as u8;

    let rtc_freq = if main_xtal_freq == 26 { 40000 } else { 32000 };

    // keep them aligned with BT_CONTROLLER_INIT_CONFIG_DEFAULT in ESP-IDF
    // ideally _some_ of these values should be configurable
    esp_bt_controller_config_t {
        config_version: CONFIG_VERSION,
        ble_ll_resolv_list_size: config.ll_resolv_list_size,
        ble_hci_evt_hi_buf_count: config.hci_high_buffer_count,
        ble_hci_evt_lo_buf_count: config.hci_low_buffer_count,
        ble_ll_sync_list_cnt: config.ll_sync_list_cnt,
        ble_ll_sync_cnt: config.ll_sync_cnt,
        ble_ll_rsp_dup_list_count: config.ll_rsp_dup_list_count,
        ble_ll_adv_dup_list_count: config.ll_adv_dup_list_count,
        ble_ll_tx_pwr_dbm: config.default_tx_power.dbm() as u8,
        rtc_freq,
        ble_ll_sca: CONFIG_BT_LE_LL_SCA as _,
        ble_ll_scan_phy_number: if CONFIG_BT_LE_LL_CFG_FEAT_LE_CODED_PHY != 0 {
            2
        } else {
            1
        },
        ble_ll_conn_def_auth_pyld_tmo: 3000,
        ble_ll_jitter_usecs: 16,
        ble_ll_sched_max_adv_pdu_usecs: 376,
        ble_ll_sched_direct_adv_max_usecs: 502,
        ble_ll_sched_adv_max_usecs: 852,
        ble_scan_rsp_data_max_len: 31,
        ble_ll_cfg_num_hci_cmd_pkts: 1,
        ble_ll_ctrl_proc_timeout_ms: 40000,
        nimble_max_connections: config.max_connections,
        ble_whitelist_size: config.whitelist_size,
        ble_acl_buf_size: config.acl_buf_size,
        ble_acl_buf_count: config.acl_buf_count,
        ble_hci_evt_buf_size: config.hci_evt_buf_size,
        ble_multi_adv_instances: config.multi_adv_instances,
        ble_ext_adv_max_size: config.ext_adv_max_size,
        controller_task_stack_size: config.task_stack_size,
        controller_task_prio: config.task_priority,
        controller_run_cpu: 0,
        enable_qa_test: config.qa_test_mode as u8,
        enable_bqb_test: config.bqb_test as u8,
        enable_uart_hci: 0,
        ble_hci_uart_port: 0,
        ble_hci_uart_baud: 0,
        ble_hci_uart_data_bits: 0,
        ble_hci_uart_stop_bits: 0,
        ble_hci_uart_flow_ctrl: 0,
        ble_hci_uart_uart_parity: 0,
        enable_tx_cca: config.cca as u8,
        cca_rssi_thresh: (256 - config.cca_threshold as u32) as u8,
        sleep_en: 0, // CONFIG_BT_LE_SLEEP_ENABLE unset
        coex_phy_coded_tx_rx_time_limit: CONFIG_BT_LE_COEX_PHY_CODED_TX_RX_TLIM_EFF as _,
        dis_scan_backoff: config.dis_scan_backoff as u8,
        ble_scan_classify_filter_enable: 0,
        cca_drop_mode: 0,  //???
        cca_low_tx_pwr: 0, //???
        main_xtal_freq,
        version_num: Efuse::minor_chip_version(),
        ignore_wl_for_direct_adv: 0,
        csa2_select: CONFIG_BT_LE_50_FEATURE_SUPPORT as _,
        ble_aa_check: config.verify_access_address as u8,
        ble_llcp_disc_flag: config.disconnect_llcp_conn_update as u8
            | ((config.disconnect_llcp_chan_map_update as u8) << 1)
            | ((config.disconnect_llcp_phy_update as u8) << 2),
        scan_backoff_upperlimitmax: config.scan_backoff_max,
        vhci_enabled: CONFIG_BT_LE_HCI_INTERFACE_USE_RAM as _,
        config_magic: CONFIG_MAGIC,
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
unsafe fn g_ble_lll_rfmgmt_env_p() -> *mut crate::sys::c_types::c_void {
    // prevent "undefined symbol: g_ble_lll_rfmgmt_env_p" for ESP32-C2 < eco4
    unreachable!()
}
