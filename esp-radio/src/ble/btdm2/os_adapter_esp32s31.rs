use procmacros::BuilderLite;

use super::*;
use crate::{
    ble::InvalidConfigError,
    hal::{
        interrupt::{self, InterruptHandler, Priority},
        peripherals::Interrupt,
        system::Cpu,
    },
    interrupt_dispatch::Handler,
    sys::include::esp_bt_controller_config_t,
};

static ISR_INTERRUPT_MAC: Handler = Handler::new();
static ISR_INTERRUPT_MAC_INT1: Handler = Handler::new();
static ISR_INTERRUPT_BB: Handler = Handler::new();
static ISR_INTERRUPT_BB_NMI: Handler = Handler::new();
static ISR_INTERRUPT_LP_TIMER: Handler = Handler::new();
static ISR_INTERRUPT_BLE_TIMER: Handler = Handler::new();
static ISR_INTERRUPT_BLE_SEC: Handler = Handler::new();

/// The interrupts the controller may ask for, each with the slot holding its
/// handler and the trampoline that dispatches from that slot.
const BLE_INTERRUPTS: &[(Interrupt, &Handler, extern "C" fn())] = &[
    (Interrupt::MODEM_BT_MAC, &ISR_INTERRUPT_MAC, MODEM_BT_MAC),
    (
        Interrupt::MODEM_BT_MAC_INT1,
        &ISR_INTERRUPT_MAC_INT1,
        MODEM_BT_MAC_INT1,
    ),
    (Interrupt::MODEM_BT_BB, &ISR_INTERRUPT_BB, MODEM_BT_BB),
    (
        Interrupt::MODEM_BT_BB_NMI,
        &ISR_INTERRUPT_BB_NMI,
        MODEM_BT_BB_NMI,
    ),
    (
        Interrupt::MODEM_LP_TIMER,
        &ISR_INTERRUPT_LP_TIMER,
        MODEM_LP_TIMER,
    ),
    (
        Interrupt::MODEM_BLE_TIMER,
        &ISR_INTERRUPT_BLE_TIMER,
        MODEM_BLE_TIMER,
    ),
    (
        Interrupt::MODEM_BLE_SEC,
        &ISR_INTERRUPT_BLE_SEC,
        MODEM_BLE_SEC,
    ),
];

/// Antenna Selection
#[derive(Default, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Antenna {
    /// Use Antenna 0
    #[default]
    Antenna0 = 0,
    /// Use Antenna 1
    Antenna1 = 1,
}

/// Transmission Power Level
#[derive(Default, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxPower {
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

impl TxPower {
    fn dbm(self) -> i8 {
        match self {
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
    /// The CPU core on which the BLE controller task should run.
    task_cpu: Cpu,
    /// The maximum number of simultaneous connections.
    max_connections: u8,
    /// Enable QA test mode.
    qa_test_mode: bool,
    /// Default TX antenna.
    default_tx_antenna: Antenna,
    /// Default RX antenna.
    default_rx_antenna: Antenna,
    /// Default TX power.
    default_tx_power: TxPower,
    /// Coexistence: limit on MAX Tx/Rx time for coded-PHY connection.
    limit_time_for_coded_phy_connection: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            task_priority: crate::preempt::max_task_priority()
                .saturating_sub(2)
                .min(255) as u8,
            task_stack_size: CONFIG_BT_CTRL_TASK_STACK_SIZE as u16,
            task_cpu: Cpu::ProCpu,
            max_connections: DEFAULT_BT_LE_MAX_CONNECTIONS as u8,
            qa_test_mode: false,
            default_tx_antenna: Antenna::default(),
            default_rx_antenna: Antenna::default(),
            default_tx_power: TxPower::default(),
            limit_time_for_coded_phy_connection: false,
        }
    }
}

impl Config {
    pub(crate) fn validate(&self) -> Result<(), InvalidConfigError> {
        crate::ble::validate_range!(
            self,
            task_priority,
            0,
            crate::preempt::max_task_priority().min(255) as u8
        );
        crate::ble::validate_range!(self, max_connections, 1, 10);
        Ok(())
    }
}

pub(crate) fn create_ble_config(config: &Config) -> esp_bt_controller_config_t {
    // BLE-only defaults from IDF `BT_CONTROLLER_INIT_CONFIG_DEFAULT`.
    let ble = esp_bt_controller_config_t__bindgen_ty_1 {
        config_version: BLE_CONFIG_VERSION,
        ble_ll_resolv_list_size: CONFIG_BT_LE_LL_RESOLV_LIST_SIZE as u16,
        ble_hci_evt_hi_buf_count: DEFAULT_BT_LE_HCI_EVT_HI_BUF_COUNT as u16,
        ble_hci_evt_lo_buf_count: DEFAULT_BT_LE_HCI_EVT_LO_BUF_COUNT as u16,
        ble_ll_sync_list_cnt: DEFAULT_BT_LE_MAX_PERIODIC_ADVERTISER_LIST as u8,
        ble_ll_sync_cnt: DEFAULT_BT_LE_MAX_PERIODIC_SYNCS as u8,
        ble_ll_rsp_dup_list_count: CONFIG_BT_LE_LL_DUP_SCAN_LIST_COUNT as u16,
        ble_ll_adv_dup_list_count: CONFIG_BT_LE_LL_DUP_SCAN_LIST_COUNT as u16,
        ble_ll_tx_pwr_dbm: config.default_tx_power.dbm() as u8,
        rtc_freq: crate::radio_clocks::clocks_ll::BT_LPCLK_HZ,
        ble_ll_sca: CONFIG_BT_LE_LL_SCA as u16,
        ble_ll_scan_phy_number: BLE_LL_SCAN_PHY_NUMBER_N as u8,
        ble_ll_conn_def_auth_pyld_tmo: BLE_LL_CONN_DEF_AUTH_PYLD_TMO_N as u16,
        ble_ll_jitter_usecs: BLE_LL_JITTER_USECS_N as u8,
        ble_ll_sched_max_adv_pdu_usecs: BLE_LL_SCHED_MAX_ADV_PDU_USECS_N as u16,
        ble_ll_sched_direct_adv_max_usecs: BLE_LL_SCHED_DIRECT_ADV_MAX_USECS_N as u16,
        ble_ll_sched_adv_max_usecs: BLE_LL_SCHED_ADV_MAX_USECS_N as u16,
        ble_scan_rsp_data_max_len: DEFAULT_BT_LE_SCAN_RSP_DATA_MAX_LEN_N as u16,
        ble_ll_cfg_num_hci_cmd_pkts: BLE_LL_CFG_NUM_HCI_CMD_PKTS_N as u8,
        ble_ll_ctrl_proc_timeout_ms: BLE_LL_CTRL_PROC_TIMEOUT_MS_N,
        nimble_max_connections: config.max_connections as u16,
        ble_whitelist_size: DEFAULT_BT_NIMBLE_WHITELIST_SIZE as u8,
        ble_acl_buf_size: DEFAULT_BT_LE_ACL_BUF_SIZE as u16,
        ble_acl_buf_count: DEFAULT_BT_LE_ACL_BUF_COUNT as u16,
        ble_hci_evt_buf_size: DEFAULT_BT_LE_HCI_EVT_BUF_SIZE as u16,
        ble_multi_adv_instances: DEFAULT_BT_LE_MAX_EXT_ADV_INSTANCES as u16,
        ble_ext_adv_max_size: DEFAULT_BT_LE_EXT_ADV_MAX_SIZE as u16,
        controller_task_stack_size: config.task_stack_size,
        controller_task_prio: config.task_priority,
        controller_run_cpu: config.task_cpu as u8,
        enable_qa_test: config.qa_test_mode as u8,
        enable_bqb_test: 0,
        enable_tx_cca: DEFAULT_BT_LE_TX_CCA_ENABLED as u8,
        cca_rssi_thresh: (256 - DEFAULT_BT_LE_CCA_RSSI_THRESH) as u8,
        sleep_en: 0,
        coex_phy_coded_tx_rx_time_limit: if cfg!(feature = "coex") {
            config.limit_time_for_coded_phy_connection as u8
        } else {
            DEFAULT_BT_LE_COEX_PHY_CODED_TX_RX_TLIM_EFF as u8
        },
        dis_scan_backoff: 0,
        ble_scan_classify_filter_enable: 1,
        cca_drop_mode: 0,
        cca_low_tx_pwr: 0,
        main_xtal_freq: CONFIG_XTAL_FREQ as u8,
        ignore_wl_for_direct_adv: 0,
        enable_pcl: 0,
        csa2_select: 1,
        enable_csr: 0,
        backoff_rssi: -100,
        iso_enabled: DEFAULT_BT_LE_ISO_ENABLED != 0,
        iso_bqb_test: false,
        iso_fra_unseg: DEFAULT_BT_LE_ISO_FRA_UNSEG != 0,
        iso_nsfc_en: DEFAULT_BT_LE_ISO_NSFC_EN != 0,
        iso_nsfc_num: DEFAULT_BT_LE_ISO_NSFC_NUM as u8,
        iso_buf_count: DEFAULT_BT_LE_ISO_BUF_COUNT as u8,
        iso_buf_size: DEFAULT_BT_LE_ISO_BUF_SIZE as u16,
        iso_big_count: DEFAULT_BT_LE_ISO_BIG as u8,
        iso_bis_count: DEFAULT_BT_LE_ISO_BIS as u16,
        iso_bis_per_big: DEFAULT_BT_LE_ISO_BIS_PER_BIG as u8,
        iso_cig_count: DEFAULT_BT_LE_ISO_CIG as u8,
        iso_cis_count: DEFAULT_BT_LE_ISO_CIS as u16,
        iso_cis_per_cig: DEFAULT_BT_LE_ISO_CIS_PER_CIG as u8,
        config_magic: BLE_CONFIG_MAGIC,
    };

    let btdm = esp_btdm_controller_config_t {
        version: BTDM_CONFIG_VERSION,
        task_stack_size: config.task_stack_size,
        task_prio: config.task_priority,
        task_run_cpu: config.task_cpu as u8,
        hci_cmd_num: CONFIG_BT_CTRL_HCI_CMD_NUM as u8,
        hci_conn_num: config.max_connections,
        sleep_en: 0,
        version_num: 0,
        bluetooth_mode: esp_bt_mode_t_ESP_BT_MODE_BLE as u8,
        magic: BTDM_CONFIG_MAGIC_VALUE,
    };

    esp_bt_controller_config_t {
        ble,
        bredr: unsafe { core::mem::zeroed() },
        btdm,
    }
}

pub(crate) fn btdm_controller_mem_init() {}

pub(crate) fn bt_periph_module_enable() {
    crate::radio_clocks::clocks_ll::enable_bt(true);
}

pub(crate) fn disable_sleep_mode() {}

pub(super) unsafe fn osal_intr_alloc(
    source: u32,
    func: unsafe extern "C" fn(*mut crate::sys::c_types::c_void),
    arg: *mut crate::sys::c_types::c_void,
) -> i32 {
    trace!(
        "btdm osal_intr_alloc source={} fn={:?} arg={:?}",
        source, func, arg
    );

    // The blob passes IDF interrupt source numbers, which are the PAC's
    // `Interrupt` discriminants.
    let entry = u8::try_from(source)
        .ok()
        .and_then(|source| Interrupt::try_from(source).ok())
        .and_then(|source| BLE_INTERRUPTS.iter().find(|(int, _, _)| *int == source));

    let Some(&(int, slot, trampoline)) = entry else {
        panic!("Unsupported BLE interrupt source {}", source);
    };

    slot.set(func as *const crate::sys::c_types::c_void, arg);
    interrupt::bind_handler(int, InterruptHandler::new(trampoline, Priority::Priority1));
    0
}

pub(crate) fn shutdown_ble_isr() {
    for &(int, _, _) in BLE_INTERRUPTS {
        for core in Cpu::all() {
            interrupt::disable(core, int);
        }
    }
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BT_MAC() {
    ISR_INTERRUPT_MAC.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BT_BB() {
    ISR_INTERRUPT_BB.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BT_BB_NMI() {
    ISR_INTERRUPT_BB_NMI.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_LP_TIMER() {
    ISR_INTERRUPT_LP_TIMER.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BLE_TIMER() {
    ISR_INTERRUPT_BLE_TIMER.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BLE_SEC() {
    ISR_INTERRUPT_BLE_SEC.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn MODEM_BT_MAC_INT1() {
    ISR_INTERRUPT_MAC_INT1.dispatch();
}
