#[doc(hidden)]
pub mod os_adapter;

use core::{cell::RefCell, marker::PhantomData};

use crate::common_adapter::*;
use crate::EspWifiInitialization;

use crate::esp_wifi_result;
use critical_section::Mutex;
use embedded_svc::wifi::{AccessPointInfo, AuthMethod, Protocol, SecondaryChannel};
use enumset::EnumSet;
use enumset::EnumSetType;
use esp_hal_common::peripheral::Peripheral;
use esp_hal_common::peripheral::PeripheralRef;
use esp_wifi_sys::include::esp_interface_t_ESP_IF_WIFI_AP;
use esp_wifi_sys::include::esp_wifi_disconnect;
use esp_wifi_sys::include::esp_wifi_get_mode;
use esp_wifi_sys::include::esp_wifi_set_protocol;
use esp_wifi_sys::include::wifi_ap_config_t;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WEP;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK;
use esp_wifi_sys::include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK;
use esp_wifi_sys::include::wifi_cipher_type_t_WIFI_CIPHER_TYPE_TKIP;
use esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP;
use esp_wifi_sys::include::wifi_mode_t_WIFI_MODE_AP;
use esp_wifi_sys::include::wifi_mode_t_WIFI_MODE_APSTA;
use esp_wifi_sys::include::wifi_mode_t_WIFI_MODE_NULL;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

#[doc(hidden)]
pub use os_adapter::*;
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};

#[cfg(feature = "mtu-1514")]
const MTU: usize = 1514;
#[cfg(feature = "mtu-1500")]
const MTU: usize = 1500;
#[cfg(feature = "mtu-1492")]
const MTU: usize = 1492;
#[cfg(feature = "mtu-746")]
const MTU: usize = 746;

#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
#[cfg(feature = "esp32s2")]
use esp32s2_hal as hal;
#[cfg(feature = "esp32s3")]
use esp32s3_hal as hal;

use hal::macros::ram;

#[cfg(feature = "utils")]
pub mod utils;

#[cfg(coex)]
use crate::binary::include::{coex_adapter_funcs_t, coex_pre_init, esp_coex_adapter_register};

use crate::{
    binary::include::{
        __BindgenBitfieldUnit, esp_err_t, esp_interface_t_ESP_IF_WIFI_STA, esp_supplicant_init,
        esp_wifi_connect, esp_wifi_init_internal, esp_wifi_internal_free_rx_buffer,
        esp_wifi_internal_reg_rxcb, esp_wifi_internal_tx, esp_wifi_scan_start, esp_wifi_set_config,
        esp_wifi_set_country, esp_wifi_set_mode, esp_wifi_set_ps, esp_wifi_set_tx_done_cb,
        esp_wifi_start, esp_wifi_stop, g_wifi_default_wpa_crypto_funcs, wifi_active_scan_time_t,
        wifi_auth_mode_t_WIFI_AUTH_OPEN, wifi_config_t,
        wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL, wifi_country_t, wifi_init_config_t,
        wifi_interface_t_WIFI_IF_STA, wifi_mode_t_WIFI_MODE_STA, wifi_osi_funcs_t,
        wifi_pmf_config_t, wifi_scan_config_t, wifi_scan_method_t_WIFI_FAST_SCAN,
        wifi_scan_threshold_t, wifi_scan_time_t, wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL, wifi_sta_config_t, wpa_crypto_funcs_t,
        ESP_WIFI_OS_ADAPTER_MAGIC, ESP_WIFI_OS_ADAPTER_VERSION, WIFI_INIT_CONFIG_MAGIC,
    },
    compat::queue::SimpleQueue,
};
use log::debug;

#[derive(Debug, Clone, Copy)]
pub enum WifiMode {
    Sta,
    Ap,
}

impl WifiMode {
    pub fn is_ap(&self) -> bool {
        match self {
            WifiMode::Sta => false,
            WifiMode::Ap => true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct DataFrame<'a> {
    len: usize,
    data: [u8; 1536],
    _phantom: PhantomData<&'a ()>,
}

impl<'a> DataFrame<'a> {
    pub(crate) fn new() -> DataFrame<'a> {
        DataFrame {
            len: 0,
            data: [0u8; 1536],
            _phantom: Default::default(),
        }
    }

    pub(crate) fn from_bytes(bytes: &[u8]) -> DataFrame {
        let mut data = DataFrame::new();
        data.len = bytes.len();
        data.data[..bytes.len()].copy_from_slice(bytes);
        data
    }

    pub(crate) fn slice(&'a self) -> &'a [u8] {
        &self.data[..self.len]
    }
}

pub(crate) static DATA_QUEUE_RX: Mutex<RefCell<SimpleQueue<DataFrame, 5>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

pub(crate) static DATA_QUEUE_TX: Mutex<RefCell<SimpleQueue<DataFrame, 3>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

#[derive(Debug, Clone, Copy)]
pub enum WifiError {
    InternalError(InternalWifiError),
    WrongClockConfig,
    Disconnected,
    UnknownWifiMode,
}
#[repr(i32)]
#[derive(Debug, FromPrimitive, EnumSetType)]
pub enum WifiEvent {
    WifiReady = 0,
    ScanDone,
    StaStart,
    StaStop,
    StaConnected,
    StaDisconnected,
    StaAuthmodeChange,
    StaWpsErSuccess,
    StaWpsErFailed,
    StaWpsErTimeout,
    StaWpsErPin,
    StaWpsErPbcOverlap,
    ApStart,
    ApStop,
    ApStaconnected,
    ApStadisconnected,
    ApProbereqrecved,
    FtmReport,
    StaBssRssiLow,
    ActionTxStatus,
    RocDone,
    StaBeaconTimeout,
}

#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
pub enum InternalWifiError {
    ///Invalid argument
    EspErrInvalidArg = 0x102,
    ///WiFi driver was not installed by esp_wifi_init */
    EspErrWifiNotInit = 0x3001,
    ///WiFi driver was not started by esp_wifi_start */
    EspErrWifiNotStarted = 0x3002,
    ///WiFi driver was not stopped by esp_wifi_stop */
    EspErrWifiNotStopped = 0x3003,
    ///WiFi interface error */   
    EspErrWifiIf = 0x3004,
    ///WiFi mode error */
    EspErrWifiMode = 0x3005,
    ///WiFi internal state error */  
    EspErrWifiState = 0x3006,
    ///WiFi internal control block of station or soft-AP error */  
    EspErrWifiConn = 0x3007,
    ///WiFi internal NVS module error */  
    EspErrWifiNvs = 0x3008,
    ///MAC address is invalid */  
    EspErrWifiMac = 0x3009,
    ///SSID is invalid */
    EspErrWifiSsid = 0x300A,
    ///Password is invalid */
    EspErrWifiPassword = 0x300B,
    ///Timeout error */
    EspErrWifiTimeout = 0x300C,
    ///WiFi is in sleep state(RF closed) and wakeup fail */
    EspErrWifiWakeFail = 0x300D,
    ///The caller would block */
    EspErrWifiWouldBlock = 0x300E,
    ///Station still in disconnect status */
    EspErrWifiNotConnect = 0x300F,
    ///Failed to post the event to WiFi task */
    EspErrWifiPost = 0x3012,
    ///Invalid WiFi state when init/deinit is called */
    EspErrWifiInitState = 0x3013,
    ///Returned when WiFi is stopping */
    EspErrWifiStopState = 0x3014,
    ///The WiFi connection is not associated */
    EspErrWifiNotAssoc = 0x3015,
    ///The WiFi TX is disallowed */
    EspErrWifiTxDisallow = 0x3016,
}

#[cfg(all(feature = "esp32c3", coex))]
static mut G_COEX_ADAPTER_FUNCS: coex_adapter_funcs_t = coex_adapter_funcs_t {
    _version: crate::binary::include::COEX_ADAPTER_VERSION as i32,
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take_from_isr: Some(semphr_take_from_isr_wrapper),
    _semphr_give_from_isr: Some(semphr_give_from_isr_wrapper),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _is_in_isr: Some(is_in_isr_wrapper),
    _malloc_internal: Some(malloc),
    _free: Some(free),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _env_is_chip: Some(env_is_chip),
    _slowclk_cal_get: Some(slowclk_cal_get),
    _magic: crate::binary::include::COEX_ADAPTER_MAGIC as i32,
};

#[cfg(all(feature = "esp32s3", coex))]
static mut G_COEX_ADAPTER_FUNCS: coex_adapter_funcs_t = coex_adapter_funcs_t {
    _version: crate::binary::include::COEX_ADAPTER_VERSION as i32,
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take_from_isr: Some(semphr_take_from_isr_wrapper),
    _semphr_give_from_isr: Some(semphr_give_from_isr_wrapper),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _is_in_isr: Some(is_in_isr_wrapper),
    _malloc_internal: Some(malloc),
    _free: Some(free),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _env_is_chip: Some(env_is_chip),
    _slowclk_cal_get: Some(slowclk_cal_get),
    _magic: crate::binary::include::COEX_ADAPTER_MAGIC as i32,
};

#[cfg(all(feature = "esp32", coex))]
static mut G_COEX_ADAPTER_FUNCS: coex_adapter_funcs_t = coex_adapter_funcs_t {
    _version: crate::binary::include::COEX_ADAPTER_VERSION as i32,
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take_from_isr: Some(semphr_take_from_isr_wrapper),
    _semphr_give_from_isr: Some(semphr_give_from_isr_wrapper),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _is_in_isr: Some(is_in_isr_wrapper),
    _malloc_internal: Some(malloc),
    _free: Some(free),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _spin_lock_create: Some(spin_lock_create),
    _spin_lock_delete: Some(spin_lock_delete),
    _int_disable: Some(wifi_int_disable),
    _int_enable: Some(wifi_int_restore),
    _timer_disarm: Some(timer_disarm),
    _timer_done: Some(timer_done),
    _timer_setfn: Some(timer_setfn),
    _timer_arm_us: Some(timer_arm_us),
    _env_is_chip: Some(env_is_chip),
    _slowclk_cal_get: Some(slowclk_cal_get),
    _magic: crate::binary::include::COEX_ADAPTER_MAGIC as i32,
};

#[cfg(coex)]
unsafe extern "C" fn semphr_take_from_isr_wrapper(
    semphr: *mut crate::binary::c_types::c_void,
    hptw: *mut crate::binary::c_types::c_void,
) -> i32 {
    crate::common_adapter::semphr_take_from_isr(semphr as *const (), hptw as *const ())
}

#[cfg(coex)]
unsafe extern "C" fn semphr_give_from_isr_wrapper(
    semphr: *mut crate::binary::c_types::c_void,
    hptw: *mut crate::binary::c_types::c_void,
) -> i32 {
    crate::common_adapter::semphr_give_from_isr(semphr as *const (), hptw as *const ())
}

#[cfg(coex)]
unsafe extern "C" fn is_in_isr_wrapper() -> i32 {
    // like original implementation
    0
}

#[cfg(coex)]
pub(crate) fn coex_initialize() -> i32 {
    log::debug!("call coex-initialize");
    unsafe {
        let res = esp_coex_adapter_register(
            &mut G_COEX_ADAPTER_FUNCS as *mut _ as *mut coex_adapter_funcs_t,
        );
        if res != 0 {
            log::error!("Error: esp_coex_adapter_register {}", res);
            return res;
        }
        let res = coex_pre_init();
        if res != 0 {
            log::error!("Error: coex_pre_init {}", res);
            return res;
        }
        0
    }
}

pub unsafe extern "C" fn coex_init() -> i32 {
    log::debug!("coex-init");
    #[cfg(coex)]
    return crate::binary::include::coex_init();

    #[cfg(not(coex))]
    0
}

#[no_mangle]
static g_wifi_osi_funcs: wifi_osi_funcs_t = wifi_osi_funcs_t {
    _version: ESP_WIFI_OS_ADAPTER_VERSION as i32,
    _env_is_chip: Some(env_is_chip),
    _set_intr: Some(set_intr),
    _clear_intr: Some(clear_intr),
    _set_isr: Some(set_isr),
    _ints_on: Some(ints_on),
    _ints_off: Some(ints_off),
    _is_from_isr: Some(is_from_isr),
    _spin_lock_create: Some(spin_lock_create),
    _spin_lock_delete: Some(spin_lock_delete),
    _wifi_int_disable: Some(wifi_int_disable),
    _wifi_int_restore: Some(wifi_int_restore),
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _wifi_thread_semphr_get: Some(wifi_thread_semphr_get),
    _mutex_create: Some(mutex_create),
    _recursive_mutex_create: Some(recursive_mutex_create),
    _mutex_delete: Some(mutex_delete),
    _mutex_lock: Some(mutex_lock),
    _mutex_unlock: Some(mutex_unlock),
    _queue_create: Some(queue_create),
    _queue_delete: Some(queue_delete),
    _queue_send: Some(queue_send),
    _queue_send_from_isr: Some(queue_send_from_isr),
    _queue_send_to_back: Some(queue_send_to_back),
    _queue_send_to_front: Some(queue_send_to_front),
    _queue_recv: Some(queue_recv),
    _queue_msg_waiting: Some(queue_msg_waiting),
    _event_group_create: Some(event_group_create),
    _event_group_delete: Some(event_group_delete),
    _event_group_set_bits: Some(event_group_set_bits),
    _event_group_clear_bits: Some(event_group_clear_bits),
    _event_group_wait_bits: Some(event_group_wait_bits),
    _task_create_pinned_to_core: Some(task_create_pinned_to_core),
    _task_create: Some(task_create),
    _task_delete: Some(task_delete),
    _task_delay: Some(task_delay),
    _task_ms_to_tick: Some(task_ms_to_tick),
    _task_get_current_task: Some(task_get_current_task),
    _task_get_max_priority: Some(task_get_max_priority),
    _malloc: Some(malloc),
    _free: Some(free),
    _event_post: Some(event_post),
    _get_free_heap_size: Some(get_free_heap_size),
    _rand: Some(rand),
    _dport_access_stall_other_cpu_start_wrap: Some(dport_access_stall_other_cpu_start_wrap),
    _dport_access_stall_other_cpu_end_wrap: Some(dport_access_stall_other_cpu_end_wrap),
    _wifi_apb80m_request: Some(wifi_apb80m_request),
    _wifi_apb80m_release: Some(wifi_apb80m_release),
    _phy_disable: Some(phy_disable),
    _phy_enable: Some(phy_enable),
    _phy_update_country_info: Some(phy_update_country_info),
    _read_mac: Some(read_mac),
    _timer_arm: Some(timer_arm),
    _timer_disarm: Some(timer_disarm),
    _timer_done: Some(timer_done),
    _timer_setfn: Some(timer_setfn),
    _timer_arm_us: Some(timer_arm_us),
    _wifi_reset_mac: Some(wifi_reset_mac),
    _wifi_clock_enable: Some(wifi_clock_enable),
    _wifi_clock_disable: Some(wifi_clock_disable),
    _wifi_rtc_enable_iso: Some(wifi_rtc_enable_iso),
    _wifi_rtc_disable_iso: Some(wifi_rtc_disable_iso),
    _esp_timer_get_time: Some(esp_timer_get_time),
    _nvs_set_i8: Some(nvs_set_i8),
    _nvs_get_i8: Some(nvs_get_i8),
    _nvs_set_u8: Some(nvs_set_u8),
    _nvs_get_u8: Some(nvs_get_u8),
    _nvs_set_u16: Some(nvs_set_u16),
    _nvs_get_u16: Some(nvs_get_u16),
    _nvs_open: Some(nvs_open),
    _nvs_close: Some(nvs_close),
    _nvs_commit: Some(nvs_commit),
    _nvs_set_blob: Some(nvs_set_blob),
    _nvs_get_blob: Some(nvs_get_blob),
    _nvs_erase_key: Some(nvs_erase_key),
    _get_random: Some(get_random),
    _get_time: Some(get_time),
    _random: Some(random),
    _log_write: Some(log_write),
    _log_writev: Some(log_writev),
    _log_timestamp: Some(log_timestamp),
    _malloc_internal: Some(malloc_internal),
    _realloc_internal: Some(realloc_internal),
    _calloc_internal: Some(calloc_internal),
    _zalloc_internal: Some(zalloc_internal),
    _wifi_malloc: Some(wifi_malloc),
    _wifi_realloc: Some(wifi_realloc),
    _wifi_calloc: Some(wifi_calloc),
    _wifi_zalloc: Some(wifi_zalloc),
    _wifi_create_queue: Some(wifi_create_queue),
    _wifi_delete_queue: Some(wifi_delete_queue),
    _coex_init: Some(coex_init),
    _coex_deinit: Some(coex_deinit),
    _coex_enable: Some(coex_enable),
    _coex_disable: Some(coex_disable),
    _coex_status_get: Some(coex_status_get),
    _coex_condition_set: Some(coex_condition_set),
    _coex_wifi_request: Some(coex_wifi_request),
    _coex_wifi_release: Some(coex_wifi_release),
    _coex_wifi_channel_set: Some(coex_wifi_channel_set),
    _coex_event_duration_get: Some(coex_event_duration_get),
    _coex_pti_get: Some(coex_pti_get),
    _coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    _coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    _coex_schm_interval_set: Some(coex_schm_interval_set),
    _coex_schm_interval_get: Some(coex_schm_interval_get),
    _coex_schm_curr_period_get: Some(coex_schm_curr_period_get),
    _coex_schm_curr_phase_get: Some(coex_schm_curr_phase_get),
    _coex_schm_curr_phase_idx_set: Some(coex_schm_curr_phase_idx_set),
    _coex_schm_curr_phase_idx_get: Some(coex_schm_curr_phase_idx_get),
    #[cfg(any(
        feature = "esp32c3",
        feature = "esp32c2",
        feature = "esp32c6",
        feature = "esp32s3",
        feature = "esp32s2",
    ))]
    _slowclk_cal_get: Some(slowclk_cal_get),
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    _phy_common_clock_disable: Some(
        crate::wifi::os_adapter::os_adapter_chip_specific::phy_common_clock_disable,
    ),
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    _phy_common_clock_enable: Some(
        crate::wifi::os_adapter::os_adapter_chip_specific::phy_common_clock_enable,
    ),
    _coex_register_start_cb: Some(coex_register_start_cb),

    #[cfg(any(feature = "esp32c6"))]
    _regdma_link_set_write_wait_content: Some(
        os_adapter_chip_specific::regdma_link_set_write_wait_content_dummy,
    ),
    #[cfg(any(feature = "esp32c6"))]
    _sleep_retention_find_link_by_id: Some(
        os_adapter_chip_specific::sleep_retention_find_link_by_id_dummy,
    ),
    #[cfg(any(feature = "esp32c6"))]
    _sleep_retention_entries_create: Some(
        os_adapter_chip_specific::sleep_retention_entries_create_dummy,
    ),
    #[cfg(any(feature = "esp32c6"))]
    _sleep_retention_entries_destroy: Some(
        os_adapter_chip_specific::sleep_retention_entries_destroy_dummy,
    ),

    _magic: ESP_WIFI_OS_ADAPTER_MAGIC as i32,
};

const CONFIG_FEATURE_WPA3_SAE_BIT: u64 = 1 << 0;

#[no_mangle]
static mut g_wifi_feature_caps: u64 = CONFIG_FEATURE_WPA3_SAE_BIT;

static mut G_CONFIG: wifi_init_config_t = wifi_init_config_t {
    osi_funcs: &g_wifi_osi_funcs as *const _ as *mut _,

    // dummy for now - populated in init
    wpa_crypto_funcs: wpa_crypto_funcs_t {
        size: 0,
        version: 1,
        aes_wrap: None,
        aes_unwrap: None,
        hmac_sha256_vector: None,
        sha256_prf: None,
        hmac_md5: None,
        hamc_md5_vector: None,
        hmac_sha1: None,
        hmac_sha1_vector: None,
        sha1_prf: None,
        sha1_vector: None,
        pbkdf2_sha1: None,
        rc4_skip: None,
        md5_vector: None,
        aes_encrypt: None,
        aes_encrypt_init: None,
        aes_encrypt_deinit: None,
        aes_decrypt: None,
        aes_decrypt_init: None,
        aes_decrypt_deinit: None,
        aes_128_encrypt: None,
        aes_128_decrypt: None,
        omac1_aes_128: None,
        ccmp_decrypt: None,
        ccmp_encrypt: None,
        aes_gmac: None,
    },
    static_rx_buf_num: 10,
    dynamic_rx_buf_num: 32,
    tx_buf_type: 1, // offset 0x78
    static_tx_buf_num: 0,
    dynamic_tx_buf_num: 32,
    cache_tx_buf_num: 0,
    csi_enable: 1,
    ampdu_rx_enable: 0,
    ampdu_tx_enable: 0,
    amsdu_tx_enable: 0,
    nvs_enable: 0,
    nano_enable: 0,
    rx_ba_win: 6,
    wifi_task_core_id: 0,
    beacon_max_len: 752,
    mgmt_sbuf_num: 32,
    feature_caps: CONFIG_FEATURE_WPA3_SAE_BIT,
    sta_disconnected_pm: false,
    espnow_max_encrypt_num: 7, // 2 for ESP32-C2
    magic: WIFI_INIT_CONFIG_MAGIC as i32,
};

pub fn get_sta_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 0);
    }
}

pub fn get_ap_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 1);
    }
}

pub fn wifi_init() -> Result<(), WifiError> {
    unsafe {
        G_CONFIG.wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs;
        G_CONFIG.feature_caps = g_wifi_feature_caps;

        crate::wifi_set_log_verbose();

        #[cfg(coex)]
        {
            esp_wifi_result!(coex_init())?;
        }

        esp_wifi_result!(esp_wifi_init_internal(&G_CONFIG))?;
        esp_wifi_result!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;

        crate::wifi_set_log_verbose();
        esp_wifi_result!(esp_supplicant_init())?;

        esp_wifi_result!(esp_wifi_set_tx_done_cb(Some(esp_wifi_tx_done_cb)))?;

        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_STA,
            Some(recv_cb)
        ))?;

        // until we support APSTA we just register the same callback for AP and STA
        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_AP,
            Some(recv_cb)
        ))?;

        #[cfg(any(feature = "esp32", feature = "esp32s3"))]
        {
            static mut NVS_STRUCT: [u32; 12] = [0; 12];
            crate::common_adapter::chip_specific::g_misc_nvs =
                &NVS_STRUCT as *const _ as *const u32 as u32;
        }

        Ok(())
    }
}

unsafe extern "C" fn recv_cb(
    buffer: *mut crate::binary::c_types::c_void,
    len: u16,
    eb: *mut crate::binary::c_types::c_void,
) -> esp_err_t {
    critical_section::with(|cs| {
        let mut queue = DATA_QUEUE_RX.borrow_ref_mut(cs);
        if !queue.is_full() {
            let src = core::slice::from_raw_parts_mut(buffer as *mut u8, len as usize);
            let packet = DataFrame::from_bytes(src);
            queue.enqueue(packet).unwrap();
            esp_wifi_internal_free_rx_buffer(eb);

            #[cfg(feature = "embassy-net")]
            embassy::RECEIVE_WAKER.wake();

            0
        } else {
            esp_wifi_internal_free_rx_buffer(eb);
            1
        }
    })
}

#[ram]
unsafe extern "C" fn esp_wifi_tx_done_cb(
    _ifidx: u8,
    _data: *mut u8,
    _data_len: *mut u16,
    _tx_status: bool,
) {
    debug!("esp_wifi_tx_done_cb");
}

pub fn wifi_start() -> Result<(), WifiError> {
    unsafe {
        esp_wifi_result!(esp_wifi_start())?;

        #[cfg(any(coex, feature = "ps-min-modem"))]
        esp_wifi_result!(esp_wifi_set_ps(
            crate::binary::include::wifi_ps_type_t_WIFI_PS_MIN_MODEM
        ))?;

        #[cfg(not(any(coex, feature = "ps-min-modem")))]
        esp_wifi_result!(esp_wifi_set_ps(
            crate::binary::include::wifi_ps_type_t_WIFI_PS_NONE
        ))?;

        let cntry_code = [b'C', b'N', 0];
        let country = wifi_country_t {
            cc: core::mem::transmute(cntry_code),
            schan: 1,
            nchan: 13,
            max_tx_power: 20,
            policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        };
        esp_wifi_result!(esp_wifi_set_country(&country))?;
    }

    Ok(())
}

unsafe extern "C" fn coex_register_start_cb(
    _cb: ::core::option::Option<unsafe extern "C" fn() -> crate::binary::c_types::c_int>,
) -> crate::binary::c_types::c_int {
    #[cfg(coex)]
    return crate::binary::include::coex_register_start_cb(_cb);

    #[cfg(not(coex))]
    0
}

pub fn wifi_start_scan(block: bool) -> i32 {
    let scan_time = wifi_scan_time_t {
        active: wifi_active_scan_time_t { min: 10, max: 20 },
        passive: 20,
    };

    let scan_config = wifi_scan_config_t {
        ssid: core::ptr::null_mut(),
        bssid: core::ptr::null_mut(),
        channel: 0,
        show_hidden: false,
        scan_type: wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        scan_time: scan_time,
    };

    unsafe { esp_wifi_scan_start(&scan_config, block) }
}

pub fn new_with_config<'d>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = esp_hal_common::radio::Wifi> + 'd,
    config: embedded_svc::wifi::Configuration,
) -> (WifiDevice<'d>, WifiController<'d>) {
    if !inited.is_wifi() {
        panic!("Not initialized for Wifi use");
    }
    
    esp_hal_common::into_ref!(device);
    match config {
        embedded_svc::wifi::Configuration::None => panic!(),
        embedded_svc::wifi::Configuration::Client(_) => (),
        embedded_svc::wifi::Configuration::AccessPoint(_) => (),
        embedded_svc::wifi::Configuration::Mixed(_, _) => panic!(),
    };
    (
        WifiDevice::new(unsafe { device.clone_unchecked() }),
        WifiController::new_with_config(device, config),
    )
}

pub fn new_with_mode<'d>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = esp_hal_common::radio::Wifi> + 'd,
    mode: WifiMode,
) -> (WifiDevice<'d>, WifiController<'d>) {
    new_with_config(
        inited,
        device,
        match mode {
            WifiMode::Sta => embedded_svc::wifi::Configuration::Client(Default::default()),
            WifiMode::Ap => embedded_svc::wifi::Configuration::AccessPoint(Default::default()),
        },
    )
}

pub fn new<'d>(
    inited: &EspWifiInitialization,
    device: impl Peripheral<P = esp_hal_common::radio::Wifi> + 'd,
) -> (WifiDevice<'d>, WifiController<'d>) {
    new_with_config(&inited, device, Default::default())
}

/// A wifi device implementing smoltcp's Device trait.
pub struct WifiDevice<'d> {
    _device: PeripheralRef<'d, esp_hal_common::radio::Wifi>,
}

impl<'d> WifiDevice<'d> {
    pub(crate) fn new(_device: PeripheralRef<'d, esp_hal_common::radio::Wifi>) -> WifiDevice {
        Self { _device }
    }

    pub(crate) fn get_wifi_mode(&self) -> Result<WifiMode, WifiError> {
        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut mode) })?;

        #[allow(non_upper_case_globals)]
        match mode {
            wifi_mode_t_WIFI_MODE_STA => Ok(WifiMode::Sta),
            wifi_mode_t_WIFI_MODE_AP => Ok(WifiMode::Ap),
            _ => Err(WifiError::UnknownWifiMode),
        }
    }
}

/// A wifi controller implementing embedded_svc::Wifi traits
pub struct WifiController<'d> {
    _device: PeripheralRef<'d, esp_hal_common::radio::Wifi>,
    config: embedded_svc::wifi::Configuration,
}

impl<'d> WifiController<'d> {
    pub(crate) fn new_with_config(
        _device: PeripheralRef<'d, esp_hal_common::radio::Wifi>,
        config: embedded_svc::wifi::Configuration,
    ) -> Self {
        Self { _device, config }
    }

    /// Set the wifi mode.
    /// This will set the wifi protocol to the desired protocol, the default for this is:
    /// `WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N`
    /// # Arguments:
    /// * `protocol` - The desired protocol
    /// # Example:
    /// ```
    /// use embedded_svc::wifi::Protocol;
    /// use esp_wifi::wifi::WifiController;
    /// let mut wifi = WifiController::new();
    /// wifi.set_mode(Protocol::P802D11BGNLR);
    /// ```
    pub fn set_mode(&mut self, protocol: Protocol) -> Result<(), WifiError> {
        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        esp_wifi_result!(unsafe { esp_wifi_get_mode(&mut mode) })?;
        esp_wifi_result!(unsafe { esp_wifi_set_protocol(mode, protocol as u8) })?;
        Ok(())
    }

    #[allow(unused)]
    fn is_sta_enabled(&self) -> Result<bool, WifiError> {
        let mut mode: esp_wifi_sys::include::wifi_mode_t = 0;
        esp_wifi_result!(unsafe { esp_wifi_sys::include::esp_wifi_get_mode(&mut mode) })?;

        Ok(mode == wifi_mode_t_WIFI_MODE_STA)
    }

    #[allow(unused)]
    fn is_ap_enabled(&self) -> Result<bool, WifiError> {
        let mut mode: esp_wifi_sys::include::wifi_mode_t = 0;
        esp_wifi_result!(unsafe { esp_wifi_sys::include::esp_wifi_get_mode(&mut mode) })?;

        Ok(mode == wifi_mode_t_WIFI_MODE_AP || mode == wifi_mode_t_WIFI_MODE_APSTA)
    }

    fn scan_results<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
        let scan = || -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
            let mut scanned = heapless::Vec::<AccessPointInfo, N>::new();
            let mut bss_total: u16 = N as u16;

            unsafe {
                esp_wifi_result!(crate::binary::include::esp_wifi_scan_get_ap_num(
                    &mut bss_total
                ))?;
                if bss_total as usize > N {
                    bss_total = N as u16;
                }

                let mut records = [crate::binary::include::wifi_ap_record_t {
                    bssid: [0u8; 6],
                    ssid: [0u8; 33],
                    primary: 0u8,
                    second: 0u32,
                    rssi: 0i8,
                    authmode: 0u32,
                    pairwise_cipher: 0u32,
                    group_cipher: 0u32,
                    ant: 0u32,
                    _bitfield_align_1: [0u32; 0],
                    _bitfield_1: crate::binary::include::__BindgenBitfieldUnit::new([0u8; 4usize]),
                    country: crate::binary::include::wifi_country_t {
                        cc: [0; 3],
                        schan: 0u8,
                        nchan: 0u8,
                        max_tx_power: 0i8,
                        policy: 0u32,
                    },
                    he_ap: crate::binary::include::wifi_he_ap_info_t {
                        _bitfield_align_1: [0u8; 0],
                        _bitfield_1: crate::binary::include::wifi_he_ap_info_t::new_bitfield_1(
                            0, 0, 0,
                        ),
                        bssid_index: 0,
                    },
                }; N];

                esp_wifi_result!(crate::binary::include::esp_wifi_scan_get_ap_records(
                    &mut bss_total,
                    &mut records as *mut crate::binary::include::wifi_ap_record_t,
                ))?;

                for i in 0..bss_total {
                    let record = records[i as usize];
                    let ssid_strbuf =
                        crate::compat::common::StrBuf::from(&record.ssid as *const u8);

                    let auth_method = match record.authmode {
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::WEP,
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => {
                            AuthMethod::WPA
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => {
                            AuthMethod::WPA2Personal
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => {
                            AuthMethod::WPAWPA2Personal
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => {
                            AuthMethod::WPA2Enterprise
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => {
                            AuthMethod::WPA3Personal
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => {
                            AuthMethod::WPA2WPA3Personal
                        }
                        crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => {
                            AuthMethod::WAPIPersonal
                        }
                        _ => panic!(),
                    };

                    let mut ssid = heapless::String::<32>::new();
                    ssid.push_str(ssid_strbuf.as_str_ref()).ok();

                    let ap_info = AccessPointInfo {
                        ssid: ssid,
                        bssid: record.bssid,
                        channel: record.primary,
                        secondary_channel: match record.second {
                            crate::binary::include::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => {
                                SecondaryChannel::None
                            }
                            crate::binary::include::wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => {
                                SecondaryChannel::Above
                            }
                            crate::binary::include::wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => {
                                SecondaryChannel::Below
                            }
                            _ => panic!(),
                        },
                        signal_strength: record.rssi,
                        protocols: EnumSet::empty(), // TODO
                        auth_method: auth_method,
                    };

                    scanned.push(ap_info).ok();
                }
            }

            Ok((scanned, bss_total as usize))
        };

        scan().map_err(|e| {
            // upon scan failure, list should be cleared to avoid memory leakage
            unsafe { crate::binary::include::esp_wifi_clear_ap_list() };
            e
        })
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
impl<'d> Device for WifiDevice<'d> {
    type RxToken<'a> = WifiRxToken where Self: 'a;
    type TxToken<'a> = WifiTxToken where Self: 'a;

    fn receive(
        &mut self,
        _instant: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        critical_section::with(|cs| {
            let rx = DATA_QUEUE_RX.borrow_ref_mut(cs);
            let tx = DATA_QUEUE_TX.borrow_ref_mut(cs);
            if !rx.is_empty() && !tx.is_full() {
                Some((WifiRxToken::default(), WifiTxToken::default()))
            } else {
                None
            }
        })
    }

    fn transmit(&mut self, _instant: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        critical_section::with(|cs| {
            let tx = DATA_QUEUE_TX.borrow_ref_mut(cs);
            if !tx.is_full() {
                Some(WifiTxToken::default())
            } else {
                None
            }
        })
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(1);
        caps
    }
}

#[derive(Debug, Default)]
pub struct WifiRxToken {}

impl RxToken for WifiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        critical_section::with(|cs| {
            let mut queue = DATA_QUEUE_RX.borrow_ref_mut(cs);

            let mut data = queue
                .dequeue()
                .expect("unreachable: transmit()/receive() ensures there is a packet to process");
            let buffer = unsafe { core::slice::from_raw_parts(&data.data as *const u8, data.len) };
            dump_packet_info(&buffer);
            f(&mut data.data[..])
        })
    }
}

#[derive(Debug, Default)]
pub struct WifiTxToken {}

impl TxToken for WifiTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let res = critical_section::with(|cs| {
            let mut queue = DATA_QUEUE_TX.borrow_ref_mut(cs);

            let mut packet = DataFrame::new();
            packet.len = len;
            let res = f(&mut packet.data[..len]);
            queue
                .enqueue(packet)
                .expect("unreachable: transmit()/receive() ensures there is a buffer free");
            res
        });

        send_data_if_needed();
        res
    }
}

pub fn send_data_if_needed() {
    critical_section::with(|cs| {
        let mut queue = DATA_QUEUE_TX.borrow_ref_mut(cs);
        let mut wifi_mode = 0u32;
        unsafe {
            esp_wifi_get_mode(&mut wifi_mode);
        }

        #[allow(non_upper_case_globals)]
        let is_ap = matches!(
            wifi_mode,
            wifi_mode_t_WIFI_MODE_AP | wifi_mode_t_WIFI_MODE_APSTA
        );

        while let Some(packet) = queue.dequeue() {
            log::trace!("sending... {} bytes", packet.len);
            dump_packet_info(packet.slice());

            let interface = if is_ap {
                wifi_interface_t_WIFI_IF_AP
            } else {
                wifi_interface_t_WIFI_IF_STA
            };

            unsafe {
                let _res = esp_wifi_internal_tx(
                    interface,
                    &packet.data as *const _ as *mut crate::binary::c_types::c_void,
                    packet.len as u16,
                );
                if _res != 0 {
                    log::warn!("esp_wifi_internal_tx {}", _res);
                }
                log::trace!("esp_wifi_internal_tx {}", _res);
            }
            #[cfg(feature = "embassy-net")]
            embassy::TRANSMIT_WAKER.wake();
        }
    });
}

impl embedded_svc::wifi::Wifi for WifiController<'_> {
    type Error = WifiError;

    /// This currently only supports the `Client` and `AccessPoint` capability.
    fn get_capabilities(&self) -> Result<EnumSet<embedded_svc::wifi::Capability>, Self::Error> {
        // we only support STA and AP mode
        let mut caps = EnumSet::empty();
        caps.insert(embedded_svc::wifi::Capability::Client);
        caps.insert(embedded_svc::wifi::Capability::AccessPoint);
        Ok(caps)
    }

    /// A blocking wifi network scan.
    fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true))?;
        self.scan_results()
    }

    /// Get the currently used configuration.
    fn get_configuration(&self) -> Result<embedded_svc::wifi::Configuration, Self::Error> {
        Ok(self.config.clone())
    }

    /// Set the configuration, you need to use Wifi::connect() for connecting to an AP
    /// Trying anything but `Configuration::Client` or `Configuration::AccessPoint` will result in a panic!
    fn set_configuration(
        &mut self,
        conf: &embedded_svc::wifi::Configuration,
    ) -> Result<(), Self::Error> {
        self.config = conf.clone();

        match conf {
            embedded_svc::wifi::Configuration::None => panic!(),
            embedded_svc::wifi::Configuration::Client(config) => {
                esp_wifi_result!(unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA) })?;

                debug!("Wifi mode STA set");
                let bssid: [u8; 6] = match &config.bssid {
                    Some(bssid_ref) => *bssid_ref,
                    None => [0; 6],
                };

                let mut cfg = wifi_config_t {
                    sta: wifi_sta_config_t {
                        ssid: [0; 32],
                        password: [0; 64],
                        scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
                        bssid_set: config.bssid.is_some(),
                        bssid,
                        channel: config.channel.unwrap_or(0u8),
                        listen_interval: 3,
                        sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                        threshold: wifi_scan_threshold_t {
                            rssi: -99,
                            authmode: match config.auth_method {
                                AuthMethod::None => wifi_auth_mode_t_WIFI_AUTH_OPEN,
                                AuthMethod::WEP => wifi_auth_mode_t_WIFI_AUTH_WEP,
                                AuthMethod::WPA => wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
                                AuthMethod::WPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
                                AuthMethod::WPAWPA2Personal => {
                                    wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK
                                }
                                AuthMethod::WPA2Enterprise => {
                                    wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE
                                }
                                AuthMethod::WPA3Personal => wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
                                AuthMethod::WPA2WPA3Personal => {
                                    wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK
                                }
                                AuthMethod::WAPIPersonal => wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
                            },
                        },
                        pmf_cfg: wifi_pmf_config_t {
                            capable: true,
                            required: false,
                        },
                        sae_pwe_h2e: 3,
                        _bitfield_align_1: [0u16; 0],
                        _bitfield_1: __BindgenBitfieldUnit::new([0u8; 4usize]),
                        failure_retry_cnt: 1,
                        _bitfield_align_2: [0u8; 0],
                        _bitfield_2: __BindgenBitfieldUnit::new([0u8; 1usize]),
                        __bindgen_padding_0: 0u16,
                    },
                };

                unsafe {
                    cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
                    cfg.sta.password[0..(config.password.len())]
                        .copy_from_slice(config.password.as_bytes());
                }
                esp_wifi_result!(unsafe {
                    esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg)
                })?;
            }
            embedded_svc::wifi::Configuration::AccessPoint(config) => {
                esp_wifi_result!(unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP) })?;

                debug!("Wifi mode AP set");

                let mut cfg = wifi_config_t {
                    ap: wifi_ap_config_t {
                        ssid: [0u8; 32usize],
                        password: [0u8; 64usize],
                        ssid_len: 0,
                        channel: config.channel,
                        authmode: match config.auth_method {
                            AuthMethod::None => wifi_auth_mode_t_WIFI_AUTH_OPEN,
                            AuthMethod::WEP => wifi_auth_mode_t_WIFI_AUTH_WEP,
                            AuthMethod::WPA => wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
                            AuthMethod::WPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
                            AuthMethod::WPAWPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
                            AuthMethod::WPA2Enterprise => {
                                wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE
                            }
                            AuthMethod::WPA3Personal => wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
                            AuthMethod::WPA2WPA3Personal => {
                                wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK
                            }
                            AuthMethod::WAPIPersonal => wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
                        },
                        ssid_hidden: if config.ssid_hidden { 1 } else { 0 },
                        max_connection: config.max_connections as u8,
                        beacon_interval: 100,
                        pairwise_cipher: wifi_cipher_type_t_WIFI_CIPHER_TYPE_TKIP,
                        ftm_responder: false,
                        pmf_cfg: wifi_pmf_config_t {
                            capable: true,
                            required: false,
                        },
                    },
                };

                unsafe {
                    cfg.ap.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
                    cfg.ap.ssid_len = config.ssid.len() as u8;
                    cfg.ap.password[0..(config.password.len())]
                        .copy_from_slice(config.password.as_bytes());
                }
                esp_wifi_result!(unsafe {
                    esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut cfg)
                })?;
            }
            embedded_svc::wifi::Configuration::Mixed(_, _) => panic!(),
        };

        Ok(())
    }

    fn start(&mut self) -> Result<(), Self::Error> {
        crate::wifi::wifi_start()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    fn connect(&mut self) -> Result<(), Self::Error> {
        esp_wifi_result!(unsafe {
            WIFI_STATE = -1;
            esp_wifi_connect()
        })
    }

    fn disconnect(&mut self) -> Result<(), Self::Error> {
        esp_wifi_result!(unsafe { esp_wifi_disconnect() })
    }

    fn is_started(&self) -> Result<bool, Self::Error> {
        match crate::wifi::get_wifi_state() {
            crate::wifi::WifiState::Invalid => Ok(false),
            // We assume that wifi has been started in every other states
            _ => Ok(true),
        }
    }

    fn is_connected(&self) -> Result<bool, Self::Error> {
        match crate::wifi::get_wifi_state() {
            crate::wifi::WifiState::StaConnected => Ok(true),
            crate::wifi::WifiState::StaDisconnected => Err(WifiError::Disconnected),
            //FIXME: Should any other enum value trigger an error instead of returning false?
            _ => Ok(false),
        }
    }
}

#[allow(unreachable_code, unused_variables)]
fn dump_packet_info(buffer: &[u8]) {
    #[cfg(not(feature = "dump-packets"))]
    return;

    log::info!("@WIFIFRAME {:02x?}", buffer);
}

#[macro_export]
macro_rules! esp_wifi_result {
    ($value:expr) => {
        if $value != crate::binary::include::ESP_OK as i32 {
            Err(WifiError::InternalError(
                FromPrimitive::from_i32($value).unwrap(),
            ))
        } else {
            core::result::Result::<(), WifiError>::Ok(())
        }
    };
}

#[cfg(feature = "embassy-net")]
pub(crate) mod embassy {
    use super::*;
    use embassy_net_driver::{Capabilities, Driver, RxToken, TxToken};
    use embassy_sync::waitqueue::AtomicWaker;

    pub(crate) static TRANSMIT_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static LINK_STATE: AtomicWaker = AtomicWaker::new();

    impl RxToken for WifiRxToken {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            critical_section::with(|cs| {
                let mut queue = DATA_QUEUE_RX.borrow_ref_mut(cs);

                let mut data = queue.dequeue().expect(
                    "unreachable: transmit()/receive() ensures there is a packet to process",
                );
                let buffer =
                    unsafe { core::slice::from_raw_parts(&data.data as *const u8, data.len) };
                dump_packet_info(&buffer);
                f(&mut data.data[..])
            })
        }
    }

    impl TxToken for WifiTxToken {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            let res = critical_section::with(|cs| {
                let mut queue = DATA_QUEUE_TX.borrow_ref_mut(cs);

                let mut packet = DataFrame::new();
                packet.len = len;
                let res = f(&mut packet.data[..len]);
                queue
                    .enqueue(packet)
                    .expect("unreachable: transmit()/receive() ensures there is a buffer free");
                res
            });

            send_data_if_needed();
            res
        }
    }

    impl Driver for WifiDevice<'_> {
        type RxToken<'a> = WifiRxToken
    where
        Self: 'a;

        type TxToken<'a> = WifiTxToken
    where
        Self: 'a;

        fn receive(
            &mut self,
            cx: &mut core::task::Context,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            RECEIVE_WAKER.register(cx.waker());
            critical_section::with(|cs| {
                let rx = DATA_QUEUE_RX.borrow_ref_mut(cs);
                let tx = DATA_QUEUE_TX.borrow_ref_mut(cs);
                if !rx.is_empty() && !tx.is_full() {
                    Some((WifiRxToken::default(), WifiTxToken::default()))
                } else {
                    None
                }
            })
        }

        fn transmit(&mut self, cx: &mut core::task::Context) -> Option<Self::TxToken<'_>> {
            TRANSMIT_WAKER.register(cx.waker());
            critical_section::with(|cs| {
                let tx = DATA_QUEUE_TX.borrow_ref_mut(cs);
                if !tx.is_full() {
                    Some(WifiTxToken::default())
                } else {
                    None
                }
            })
        }

        fn link_state(&mut self, cx: &mut core::task::Context) -> embassy_net_driver::LinkState {
            LINK_STATE.register(cx.waker());

            match self.get_wifi_mode() {
                Ok(WifiMode::Sta) => {
                    if matches!(get_wifi_state(), WifiState::StaConnected) {
                        embassy_net_driver::LinkState::Up
                    } else {
                        embassy_net_driver::LinkState::Down
                    }
                }
                Ok(WifiMode::Ap) => {
                    if matches!(
                        get_wifi_state(),
                        WifiState::ApStart
                            | WifiState::ApStaConnected
                            | WifiState::ApStaDisconnected
                    ) {
                        embassy_net_driver::LinkState::Up
                    } else {
                        embassy_net_driver::LinkState::Down
                    }
                }
                _ => {
                    log::warn!("Unknown wifi mode in link_state");
                    embassy_net_driver::LinkState::Down
                }
            }
        }

        fn capabilities(&self) -> Capabilities {
            let mut caps = Capabilities::default();
            caps.max_transmission_unit = MTU;
            caps.max_burst_size = Some(1);
            caps
        }

        fn ethernet_address(&self) -> [u8; 6] {
            let mut mac = [0; 6];
            match self.get_wifi_mode() {
                Ok(WifiMode::Ap) => get_ap_mac(&mut mac),
                Ok(WifiMode::Sta) => get_sta_mac(&mut mac),
                _ => get_sta_mac(&mut mac),
            }
            mac
        }
    }
}

#[cfg(feature = "async")]
mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use num_traits::FromPrimitive;

    use super::*;

    // TODO assumes STA mode only
    impl<'d> WifiController<'d> {
        /// Async version of [`embedded_svc::wifi::Wifi`]'s `scan_n` method
        pub async fn scan_n<const N: usize>(
            &mut self,
        ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), WifiError> {
            critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove(WifiEvent::ScanDone));
            esp_wifi_result!(crate::wifi::wifi_start_scan(false))?;
            WifiEventFuture::new(WifiEvent::ScanDone).await;
            self.scan_results()
        }

        /// Async version of [`embedded_svc::wifi::Wifi`]'s `start` method
        pub async fn start(&mut self) -> Result<(), WifiError> {
            critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove(WifiEvent::StaStart));
            wifi_start()?;

            let is_ap = match self.config {
                embedded_svc::wifi::Configuration::None => panic!(),
                embedded_svc::wifi::Configuration::Client(_) => false,
                embedded_svc::wifi::Configuration::AccessPoint(_) => true,
                embedded_svc::wifi::Configuration::Mixed(_, _) => panic!(),
            };

            if is_ap {
                WifiEventFuture::new(WifiEvent::ApStart).await;
            } else {
                WifiEventFuture::new(WifiEvent::StaStart).await;
            }
            Ok(())
        }

        /// Async version of [`embedded_svc::wifi::Wifi`]'s `stop` method
        pub async fn stop(&mut self) -> Result<(), WifiError> {
            critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove(WifiEvent::StaStop));
            embedded_svc::wifi::Wifi::stop(self)?;

            let is_ap = match self.config {
                embedded_svc::wifi::Configuration::None => panic!(),
                embedded_svc::wifi::Configuration::Client(_) => false,
                embedded_svc::wifi::Configuration::AccessPoint(_) => true,
                embedded_svc::wifi::Configuration::Mixed(_, _) => panic!(),
            };

            if is_ap {
                WifiEventFuture::new(WifiEvent::ApStop).await;
            } else {
                WifiEventFuture::new(WifiEvent::StaStop).await;
            }
            Ok(())
        }

        /// Async version of [`embedded_svc::wifi::Wifi`]'s `connect` method
        pub async fn connect(&mut self) -> Result<(), WifiError> {
            critical_section::with(|cs| {
                WIFI_EVENTS
                    .borrow_ref_mut(cs)
                    .remove_all(WifiEvent::StaConnected | WifiEvent::StaDisconnected)
            });
            let err = embedded_svc::wifi::Wifi::connect(self).err();
            match embassy_futures::select::select(
                WifiEventFuture::new(WifiEvent::StaConnected),
                WifiEventFuture::new(WifiEvent::StaDisconnected),
            )
            .await
            {
                embassy_futures::select::Either::First(_) => Ok(()),
                embassy_futures::select::Either::Second(_) => {
                    Err(err.unwrap_or(WifiError::Disconnected))
                }
            }
        }

        /// Async version of [`embedded_svc::wifi::Wifi`]'s `Disconnect` method
        pub async fn disconnect(&mut self) -> Result<(), WifiError> {
            critical_section::with(|cs| {
                WIFI_EVENTS
                    .borrow_ref_mut(cs)
                    .remove(WifiEvent::StaDisconnected)
            });
            embedded_svc::wifi::Wifi::disconnect(self)?;
            WifiEventFuture::new(WifiEvent::StaDisconnected).await;
            Ok(())
        }

        /// Wait for a [`WifiEvent`].
        pub async fn wait_for_event(&mut self, event: WifiEvent) {
            critical_section::with(|cs| WIFI_EVENTS.borrow_ref_mut(cs).remove(event));
            WifiEventFuture::new(event).await;
        }
    }

    impl WifiEvent {
        pub(crate) fn waker(&self) -> &'static AtomicWaker {
            match self {
                WifiEvent::ScanDone => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaStart => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaConnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaDisconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaStop => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::WifiReady => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaAuthmodeChange => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErSuccess => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErFailed => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErTimeout => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErPin => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaWpsErPbcOverlap => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStart => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStop => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStaconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApStadisconnected => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ApProbereqrecved => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::FtmReport => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaBssRssiLow => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::ActionTxStatus => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::RocDone => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                WifiEvent::StaBeaconTimeout => {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
            }
        }
    }

    pub(crate) struct WifiEventFuture {
        event: WifiEvent,
    }

    impl WifiEventFuture {
        pub fn new(event: WifiEvent) -> Self {
            Self { event }
        }
    }

    impl core::future::Future for WifiEventFuture {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            self.event.waker().register(cx.waker());
            if critical_section::with(|cs| WIFI_EVENTS.borrow_ref(cs).contains(self.event)) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}
