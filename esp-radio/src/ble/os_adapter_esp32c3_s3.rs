use procmacros::BuilderLite;

use super::*;
#[cfg(multi_core)]
use crate::hal::system::Cpu;
use crate::{
    ble::InvalidConfigError,
    common_adapter::*,
    hal::{interrupt::Priority, peripherals::BT},
    interrupt_dispatch::Handler,
};

static ISR_INTERRUPT_5: Handler = Handler::new();
static ISR_INTERRUPT_8: Handler = Handler::new();

#[repr(C)]
pub(super) struct osi_funcs_s {
    magic: u32,
    version: u32,
    interrupt_alloc: Option<
        unsafe extern "C" fn(i32, i32, extern "C" fn(*const ()), *const (), *mut *const ()) -> i32,
    >,
    interrupt_free: Option<unsafe extern "C" fn(*const ()) -> i32>,
    interrupt_handler_set: Option<unsafe extern "C" fn(i32, extern "C" fn(*const ()), *const ())>,
    interrupt_disable: Option<unsafe extern "C" fn()>,
    interrupt_restore: Option<unsafe extern "C" fn()>,
    task_yield: Option<unsafe extern "C" fn()>,
    task_yield_from_isr: Option<unsafe extern "C" fn()>,
    semphr_create: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    semphr_delete: Option<unsafe extern "C" fn(*mut c_void)>,
    semphr_take_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut bool) -> i32>,
    semphr_give_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut bool) -> i32>,
    semphr_take: Option<unsafe extern "C" fn(*mut c_void, u32) -> i32>,
    semphr_give: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    mutex_create: Option<unsafe extern "C" fn() -> *const ()>,
    mutex_delete: Option<unsafe extern "C" fn(*const ())>,
    mutex_lock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    mutex_unlock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    queue_create: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    queue_delete: Option<unsafe extern "C" fn(*mut c_void)>,
    queue_send: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, u32) -> i32>,
    queue_send_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
    queue_recv: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, u32) -> i32>,
    queue_recv_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
    task_create: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *const c_char,
            u32,
            *mut c_void,
            u32,
            *mut c_void,
            u32,
        ) -> i32,
    >,
    task_delete: Option<unsafe extern "C" fn(*mut ())>,
    is_in_isr: Option<unsafe extern "C" fn() -> i32>,
    cause_sw_intr_to_core: Option<unsafe extern "C" fn(i32, i32) -> i32>,
    malloc: Option<unsafe extern "C" fn(u32) -> *mut c_void>,
    malloc_internal: Option<unsafe extern "C" fn(u32) -> *mut c_void>,
    free: Option<unsafe extern "C" fn(*mut c_void)>,
    read_efuse_mac: Option<unsafe extern "C" fn(*const ()) -> i32>,
    srand: Option<unsafe extern "C" fn(u32)>,
    rand: Option<unsafe extern "C" fn() -> i32>,
    btdm_lpcycles_2_hus: Option<unsafe extern "C" fn(u32, u32) -> u32>,
    btdm_hus_2_lpcycles: Option<unsafe extern "C" fn(u32) -> u32>,
    btdm_sleep_check_duration: Option<unsafe extern "C" fn(i32) -> i32>,
    btdm_sleep_enter_phase1: Option<unsafe extern "C" fn(i32)>,
    btdm_sleep_enter_phase2: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase1: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase2: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase3: Option<unsafe extern "C" fn()>,
    coex_wifi_sleep_set: Option<unsafe extern "C" fn(i32)>,
    coex_core_ble_conn_dyn_prio_get: Option<unsafe extern "C" fn(*mut i32, *mut i32) -> i32>,
    coex_schm_register_btdm_callback: Option<unsafe extern "C" fn(*const ()) -> i32>,
    coex_schm_status_bit_set: Option<unsafe extern "C" fn(i32, i32)>,
    coex_schm_status_bit_clear: Option<unsafe extern "C" fn(i32, i32)>,
    coex_schm_interval_get: Option<unsafe extern "C" fn() -> i32>,
    coex_schm_curr_period_get: Option<unsafe extern "C" fn() -> u8>,
    coex_schm_curr_phase_get: Option<unsafe extern "C" fn() -> *const ()>,
    interrupt_on: Option<unsafe extern "C" fn(i32) -> i32>,
    interrupt_off: Option<unsafe extern "C" fn(i32) -> i32>,
    esp_hw_power_down: Option<unsafe extern "C" fn()>,
    esp_hw_power_up: Option<unsafe extern "C" fn()>,
    ets_backup_dma_copy: Option<unsafe extern "C" fn(u32, u32, u32, i32)>,
    ets_delay_us: Option<unsafe extern "C" fn(u32)>,
    btdm_rom_table_ready: Option<unsafe extern "C" fn()>,
    coex_bt_wakeup_request: Option<unsafe extern "C" fn()>,
    coex_bt_wakeup_request_end: Option<unsafe extern "C" fn()>,
    get_time_us: Option<unsafe extern "C" fn() -> u64>,
    assert: Option<unsafe extern "C" fn()>,
}

pub(super) static G_OSI_FUNCS: osi_funcs_s = osi_funcs_s {
    magic: 0xfadebead,
    version: 0x0001000A,
    interrupt_alloc: Some(ble_os_adapter_chip_specific::interrupt_set),
    interrupt_free: Some(ble_os_adapter_chip_specific::interrupt_clear),
    interrupt_handler_set: Some(ble_os_adapter_chip_specific::interrupt_handler_set),
    interrupt_disable: Some(interrupt_disable),
    interrupt_restore: Some(interrupt_enable),
    task_yield: Some(task_yield),
    task_yield_from_isr: Some(task_yield_from_isr),
    semphr_create: Some(semphr_create),
    semphr_delete: Some(semphr_delete),
    semphr_take_from_isr: Some(semphr_take_from_isr),
    semphr_give_from_isr: Some(semphr_give_from_isr),
    semphr_take: Some(semphr_take),
    semphr_give: Some(semphr_give),
    mutex_create: Some(mutex_create),
    mutex_delete: Some(mutex_delete),
    mutex_lock: Some(mutex_lock),
    mutex_unlock: Some(mutex_unlock),
    queue_create: Some(queue_create),
    queue_delete: Some(queue_delete),
    queue_send: Some(queue_send),
    queue_send_from_isr: Some(queue_send_from_isr),
    queue_recv: Some(queue_recv),
    queue_recv_from_isr: Some(queue_recv_from_isr),
    task_create: Some(task_create),
    task_delete: Some(task_delete),
    is_in_isr: Some(is_in_isr),
    cause_sw_intr_to_core: None,
    malloc: Some(crate::ble::malloc),
    malloc_internal: Some(crate::ble::malloc_internal),
    free: Some(crate::ble::free),
    read_efuse_mac: Some(read_efuse_mac),
    srand: Some(crate::ble::btdm::srand),
    rand: Some(crate::ble::btdm::rand),
    btdm_lpcycles_2_hus: Some(btdm_lpcycles_2_hus),
    btdm_hus_2_lpcycles: Some(btdm_hus_2_lpcycles),
    btdm_sleep_check_duration: Some(btdm_sleep_check_duration),
    btdm_sleep_enter_phase1: Some(btdm_sleep_enter_phase1),
    btdm_sleep_enter_phase2: Some(btdm_sleep_enter_phase2),
    btdm_sleep_exit_phase1: Some(btdm_sleep_exit_phase1),
    btdm_sleep_exit_phase2: Some(btdm_sleep_exit_phase2),
    btdm_sleep_exit_phase3: Some(btdm_sleep_exit_phase3),
    coex_wifi_sleep_set: Some(ble_os_adapter_chip_specific::coex_wifi_sleep_set),
    coex_core_ble_conn_dyn_prio_get: Some(
        ble_os_adapter_chip_specific::coex_core_ble_conn_dyn_prio_get,
    ),
    coex_schm_register_btdm_callback: Some(coex_schm_register_btdm_callback),
    coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    coex_schm_interval_get: Some(coex_schm_interval_get),
    coex_schm_curr_period_get: Some(coex_schm_curr_period_get),
    coex_schm_curr_phase_get: Some(coex_schm_curr_phase_get),
    interrupt_on: Some(ble_os_adapter_chip_specific::interrupt_on),
    interrupt_off: Some(ble_os_adapter_chip_specific::interrupt_off),
    esp_hw_power_down: Some(ble_os_adapter_chip_specific::esp_hw_power_down),
    esp_hw_power_up: Some(ble_os_adapter_chip_specific::esp_hw_power_up),
    ets_backup_dma_copy: Some(ble_os_adapter_chip_specific::ets_backup_dma_copy),
    ets_delay_us: Some(ets_delay_us_wrapper),
    btdm_rom_table_ready: Some(btdm_rom_table_ready_wrapper),
    coex_bt_wakeup_request: Some(coex_bt_wakeup_request),
    coex_bt_wakeup_request_end: Some(coex_bt_wakeup_request_end),
    get_time_us: Some(get_time_us_wrapper),
    assert: Some(assert_wrapper),
};

extern "C" fn get_time_us_wrapper() -> u64 {
    // Get time in microseconds since boot
    crate::preempt::now()
}

extern "C" fn assert_wrapper() {
    panic!("assert_wrapper called - inspect the logs");
}

extern "C" fn coex_schm_register_btdm_callback(_callback: *const ()) -> i32 {
    trace!("coex_schm_register_btdm_callback");

    #[cfg(coex)]
    unsafe {
        // COEX_SCHM_CALLBACK_TYPE_BT
        coex_schm_register_callback(1, _callback as *mut crate::sys::c_types::c_void)
    }

    #[cfg(not(coex))]
    0
}

extern "C" fn coex_schm_interval_get() -> i32 {
    trace!("coex_schm_interval_get");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_interval_get() as i32
    }

    #[cfg(not(coex))]
    0
}

extern "C" fn coex_schm_curr_period_get() -> u8 {
    trace!("coex_schm_curr_period_get");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_curr_period_get()
    }

    #[cfg(not(coex))]
    0
}

extern "C" fn coex_schm_curr_phase_get() -> *const () {
    trace!("coex_schm_curr_phase_get");

    #[cfg(coex)]
    unsafe {
        crate::sys::include::coex_schm_curr_phase_get().cast()
    }

    #[cfg(not(coex))]
    core::ptr::null()
}

extern "C" fn coex_bt_wakeup_request() {
    trace!("coex_bt_wakeup_request");

    unsafe extern "C" {
        fn btdm_wakeup_request();
    }

    unsafe {
        btdm_wakeup_request();
    }
}

extern "C" fn coex_bt_wakeup_request_end() {
    trace!("coex_bt_wakeup_request_end");

    unsafe extern "C" {
        fn btdm_in_wakeup_requesting_set(set: bool);
    }

    unsafe {
        btdm_in_wakeup_requesting_set(false);
    }
}

extern "C" fn ets_delay_us_wrapper(us: u32) {
    unsafe extern "C" {
        fn ets_delay_us(us: u32);
    }

    unsafe {
        ets_delay_us(us);
    }
}

extern "C" fn btdm_rom_table_ready_wrapper() {
    trace!("btdm_rom_table_ready_wrapper");

    unsafe extern "C" {
        // fn ble_cca_funcs_reset();
        fn ble_dtm_funcs_reset();
        fn ble_42_adv_funcs_reset();
        fn ble_init_funcs_reset();
        fn ble_con_funcs_reset();
        fn ble_scan_funcs_reset();
        fn ble_ext_adv_funcs_reset();
        fn ble_ext_scan_funcs_reset();
        fn ble_base_funcs_reset();
        fn ble_enc_funcs_reset();
    }

    unsafe {
        ble_base_funcs_reset();
        ble_42_adv_funcs_reset();
        ble_ext_adv_funcs_reset();
        ble_dtm_funcs_reset();
        ble_scan_funcs_reset();
        ble_ext_scan_funcs_reset();
        ble_enc_funcs_reset();
        ble_init_funcs_reset();
        ble_con_funcs_reset();
    }
}

unsafe extern "C" {
    fn btdm_controller_rom_data_init() -> i32;
}

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

#[allow(dead_code)]
impl TxPower {
    fn idx(self) -> esp_power_level_t {
        match self {
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

/// BLE CCA mode.
#[derive(Default, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CcaMode {
    /// Disabled
    #[default]
    Disabled          = 0,
    /// Hardware Triggered
    HardwareTriggered = 1,
    /// Software Triggered (experimental)
    SoftwareTriggered = 2,
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
    #[cfg(multi_core)]
    task_cpu: Cpu,

    /// The maximum number of simultaneous connections.
    ///
    /// Range: 1 - 10
    max_connections: u8,

    /// Enable QA test mode.
    qa_test_mode: bool,

    /// Maximum number of devices in scan duplicate filtering list.
    ///
    /// Range: 10 - 1000
    scan_duplicate_list_count: u16,

    /// Scan duplicate filtering list refresh period in seconds.
    ///
    /// Range: 0 - 1000 seconds
    scan_duplicate_refresh_period: u16,

    /// Enables verification of the Access Address within the `CONNECT_IND` PDU.
    ///
    /// Enabling this option will add stricter verification of the Access Address in the
    /// `CONNECT_IND` PDU. This improves security by ensuring that only connection requests with
    /// valid Access Addresses are accepted. If disabled, only basic checks are applied,
    /// improving compatibility.
    // TODO: this needs additional setup
    verify_access_address: bool,

    /// Enable BLE channel assessment.
    channel_assessment: bool,

    /// Enable BLE ping procedure.
    ping: bool,

    /// Default TX antenna.
    default_tx_antenna: Antenna,

    /// Default RX antenna.
    default_rx_antenna: Antenna,

    /// Default TX power.
    default_tx_power: TxPower,

    /// Coexistence: limit on MAX Tx/Rx time for coded-PHY connection.
    limit_time_for_coded_phy_connection: bool,

    /// Enable / disable uncoded phy / coded phy hardware re-correction.
    hw_recorrect_en: bool,

    /// BLE Clear Channel Assessment (CCA) mode.
    cca_mode: CcaMode,

    /// Absolute value of hardware-triggered CCA threshold.
    ///
    /// The CCA threshold is always negative.
    ///
    /// If the channel assessment result exceeds the CCA threshold (e.g. -75 dBm), indicating
    /// the channel is busy, the hardware will not transmit packets on that channel.
    ///
    /// Range: 20 dBm - 100 dBm
    cca_threshold: u8,

    /// Enable / disable auxiliary packets when the extended ADV data length is zero.
    data_length_zero_aux: bool,

    /// Enable / disable DTM.
    dtm: bool,

    /// Enable / disable encryption.
    encryption: bool,

    /// Enable / disable connection.
    connection: bool,

    /// Enable / disable scanning.
    scan: bool,

    /// Enable / disable ADV.
    adv: bool,

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
            task_priority: crate::preempt::max_task_priority()
                .saturating_sub(2)
                .min(255) as u8,
            task_stack_size: 8192, // 4096?
            #[cfg(multi_core)]
            task_cpu: Cpu::ProCpu,
            max_connections: 6,
            qa_test_mode: false,
            scan_duplicate_list_count: 100,
            scan_duplicate_refresh_period: 0,
            verify_access_address: true,
            channel_assessment: false,
            ping: false,
            default_tx_antenna: Antenna::default(),
            default_rx_antenna: Antenna::default(),
            default_tx_power: TxPower::default(),
            limit_time_for_coded_phy_connection: false,
            hw_recorrect_en: AGC_RECORRECT_EN != 0,
            cca_threshold: 75, // CONFIG_BT_CTRL_HW_CCA_VAL is 0 which is not valid
            cca_mode: CcaMode::default(),
            data_length_zero_aux: false,
            dtm: true,
            encryption: true,
            connection: true,
            scan: true,
            adv: true,
            disconnect_llcp_conn_update: false,
            disconnect_llcp_chan_map_update: false,
            disconnect_llcp_phy_update: false,
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
        crate::ble::validate_range!(self, scan_duplicate_list_count, 10, 1000);
        crate::ble::validate_range!(self, scan_duplicate_refresh_period, 0, 1000);
        crate::ble::validate_range!(self, cca_threshold, 20, 100);

        Ok(())
    }
}

pub(crate) fn create_ble_config(config: &Config) -> esp_bt_controller_config_t {
    // keep them aligned with BT_CONTROLLER_INIT_CONFIG_DEFAULT in ESP-IDF
    // ideally _some_ of these values should be configurable
    esp_bt_controller_config_t {
        version: 0x02505080,
        controller_task_stack_size: config.task_stack_size,
        controller_task_prio: config.task_priority,
        #[cfg(multi_core)]
        controller_task_run_cpu: config.task_cpu as u8,
        #[cfg(single_core)]
        controller_task_run_cpu: 0,

        bluetooth_mode: esp_bt_mode_t_ESP_BT_MODE_BLE as _,

        ble_max_act: config.max_connections,
        sleep_mode: 0,
        sleep_clock: 0,
        ble_st_acl_tx_buf_nb: 0,
        ble_hw_cca_check: 0,
        ble_adv_dup_filt_max: 30,
        coex_param_en: false,
        ce_len_type: 0,
        coex_use_hooks: false,
        hci_tl_type: 1,
        hci_tl_funcs: core::ptr::null_mut(),
        txant_dft: config.default_tx_antenna as u8,
        rxant_dft: config.default_rx_antenna as u8,
        txpwr_dft: config.default_tx_power as u8,
        cfg_mask: CFG_MASK,

        // Bluetooth mesh options, currently not supported
        scan_duplicate_mode: 0, // normal mode
        scan_duplicate_type: 0,
        mesh_adv_size: 0,

        normal_adv_size: config.scan_duplicate_list_count,
        coex_phy_coded_tx_rx_time_limit: if cfg!(feature = "coex") {
            config.limit_time_for_coded_phy_connection as u8
        } else {
            0
        },
        hw_target_code: if cfg!(esp32c3) {
            0x01010000
        } else {
            0x02010000
        },
        // esp-idf: "Please do not modify this value"
        slave_ce_len_min: SLAVE_CE_LEN_MIN_DEFAULT as _,
        hw_recorrect_en: config.hw_recorrect_en as u8,
        cca_thresh: config.cca_threshold,
        dup_list_refresh_period: config.scan_duplicate_refresh_period,
        scan_backoff_upperlimitmax: 0,
        ble_50_feat_supp: BT_CTRL_50_FEATURE_SUPPORT != 0,
        ble_cca_mode: config.cca_mode as u8,
        ble_chan_ass_en: config.channel_assessment as u8,
        ble_data_lenth_zero_aux: config.data_length_zero_aux as u8,
        ble_ping_en: config.ping as u8,
        ble_llcp_disc_flag: config.disconnect_llcp_conn_update as u8
            | ((config.disconnect_llcp_chan_map_update as u8) << 1)
            | ((config.disconnect_llcp_phy_update as u8) << 2),
        run_in_flash: false,
        dtm_en: config.dtm,
        enc_en: config.encryption,
        qa_test: config.qa_test_mode,
        connect_en: config.connection,
        scan_en: config.scan,
        ble_aa_check: config.verify_access_address,
        ble_log_mode_en: if cfg!(feature = "print-logs-from-driver") {
            4095
        } else {
            0
        },
        ble_log_level: if cfg!(feature = "print-logs-from-driver") {
            5
        } else {
            0
        },
        adv_en: config.adv,
        magic: ESP_BT_CTRL_CONFIG_MAGIC_VAL,
    }
}

pub(crate) unsafe extern "C" fn interrupt_on(intr_num: i32) -> i32 {
    trace!("interrupt_on {}", intr_num);

    // NO-OP
    0
}

pub(crate) unsafe extern "C" fn interrupt_off(_intr_num: i32) -> i32 {
    todo!();
}

pub(crate) fn btdm_controller_mem_init() {
    unsafe {
        btdm_controller_rom_data_init();
    }
}

pub(crate) fn bt_periph_module_enable() {
    // nothing
}

pub(crate) fn disable_sleep_mode() {
    // nothing
}

// OSI functions

pub(crate) unsafe extern "C" fn interrupt_set(
    cpu_no: i32,
    intr_source: i32,
    handler: extern "C" fn(*const ()),
    arg: *const (),
    ret_handle: *mut *const (),
) -> i32 {
    trace!(
        "interrupt_set {} {} {} {} {}",
        cpu_no, intr_source, handler as usize, arg as u32, ret_handle as usize,
    );

    unsafe {
        interrupt_handler_set(intr_source, handler, arg);
    }

    0
}

pub(crate) unsafe extern "C" fn interrupt_clear(_handler: *const ()) -> i32 {
    todo!();
}

pub(crate) unsafe extern "C" fn interrupt_handler_set(
    interrupt_no: i32,
    func: extern "C" fn(*const ()),
    arg: *const (),
) {
    trace!(
        "interrupt_handler_set {} {:?} {:?}",
        interrupt_no, func, arg
    );

    match interrupt_no {
        5 => unsafe {
            ISR_INTERRUPT_5.set(func as *const c_void, arg.cast());
            #[cfg(esp32c3)]
            BT::steal().enable_rwbt_interrupt(Priority::Priority1);
            BT::steal().enable_bb_interrupt(Priority::Priority1);
        },
        8 => unsafe {
            ISR_INTERRUPT_8.set(func as *const c_void, arg.cast());
            BT::steal().enable_rwble_interrupt(Priority::Priority1);
        },
        _ => panic!("Unsupported interrupt number {}", interrupt_no),
    }
}

pub(crate) unsafe extern "C" fn coex_wifi_sleep_set(sleep: i32) {
    trace!(
        "ignored coex_wifi_sleep_set {} - because original implementation does the same",
        sleep
    );
}

#[allow(unused_variables, dead_code)]
pub(crate) unsafe extern "C" fn coex_core_ble_conn_dyn_prio_get(
    low: *mut i32,
    high: *mut i32,
) -> i32 {
    unsafe extern "C" {
        fn coex_core_ble_conn_dyn_prio_get(low: *mut i32, high: *mut i32) -> i32;
    }
    trace!("coex_core_ble_conn_dyn_prio_get");

    #[cfg(coex)]
    return unsafe { coex_core_ble_conn_dyn_prio_get(low, high) };

    #[cfg(not(coex))]
    0
}

pub(crate) unsafe extern "C" fn esp_hw_power_down() {
    todo!();
}

pub(crate) unsafe extern "C" fn esp_hw_power_up() {
    todo!();
}

pub(crate) unsafe extern "C" fn ets_backup_dma_copy(
    _reg: u32,
    _mem_addr: u32,
    _num: u32,
    _to_rem: i32,
) {
    todo!();
}

#[cfg(esp32c3)]
#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn RWBT() {
    ISR_INTERRUPT_5.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn RWBLE() {
    ISR_INTERRUPT_8.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn BT_BB() {
    ISR_INTERRUPT_5.dispatch();
}

pub(crate) fn shutdown_ble_isr() {
    unsafe {
        #[cfg(esp32c3)]
        BT::steal().disable_rwbt_interrupt_on_all_cores();
        BT::steal().disable_rwble_interrupt_on_all_cores();
        BT::steal().disable_bb_interrupt_on_all_cores();
    }
}
