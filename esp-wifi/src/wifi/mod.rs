#[doc(hidden)]
pub mod os_adapter;

use core::{cell::RefCell, marker::PhantomData};

use crate::common_adapter::*;

use critical_section::Mutex;
use embedded_svc::wifi::{AccessPointInfo, AuthMethod, SecondaryChannel};
use enumset::EnumSet;
use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;
use crate::esp_wifi_result;

#[doc(hidden)]
pub use os_adapter::*;
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};

#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
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
use log::{debug, info};

#[cfg(feature = "dump_packets")]
static DUMP_PACKETS: bool = true;
#[cfg(not(feature = "dump_packets"))]
static DUMP_PACKETS: bool = false;

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
}
#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[derive(FromPrimitive)]
pub enum WifiEvent {
    WifiEventWifiReady = 0,           
    WifiEventScanDone,                
    WifiEventStaStart,                
    WifiEventStaStop,                 
    WifiEventStaConnected,            
    WifiEventStaDisconnected,         
    WifiEventStaAuthmodeChange,      
    WifiEventStaWpsErSuccess,       
    WifiEventStaWpsErFailed,        
    WifiEventStaWpsErTimeout,       
    WifiEventStaWpsErPin,           
    WifiEventStaWpsErPbcOverlap,   
    WifiEventApStart,                 
    WifiEventApStop,                  
    WifiEventApStaconnected,          
    WifiEventApStadisconnected,       
    WifiEventApProbereqrecved,        
    WifiEventFtmReport,               
    WifiEventStaBssRssiLow,         
    WifiEventActionTxStatus,         
    WifiEventRocDone,                 
    WifiEventStaBeaconTimeout,       
    WifiEventMax,                      
}

#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[derive(FromPrimitive)]
pub enum InternalWifiError {
    ///WiFi driver was not installed by esp_wifi_init */
    EspErrWifiNotInit    = 0x3001,  
    ///WiFi driver was not started by esp_wifi_start */ 
    EspErrWifiNotStarted = 0x3002,   
    ///WiFi driver was not stopped by esp_wifi_stop */
    EspErrWifiNotStopped = 0x3003,
    ///WiFi interface error */   
    EspErrWifiIf          = 0x3004,   
    ///WiFi mode error */
    EspErrWifiMode        = 0x3005, 
    ///WiFi internal state error */  
    EspErrWifiState       = 0x3006, 
    ///WiFi internal control block of station or soft-AP error */  
    EspErrWifiConn        = 0x3007,
    ///WiFi internal NVS module error */  
    EspErrWifiNvs         = 0x3008, 
    ///MAC address is invalid */  
    EspErrWifiMac         = 0x3009,  
    ///SSID is invalid */ 
    EspErrWifiSsid        = 0x300A,  
    ///Password is invalid */ 
    EspErrWifiPassword    = 0x300B, 
    ///Timeout error */ 
    EspErrWifiTimeout     = 0x300C, 
    ///WiFi is in sleep state(RF closed) and wakeup fail */ 
    EspErrWifiWakeFail   = 0x300D,  
     ///The caller would block */
    EspErrWifiWouldBlock = 0x300E,
    ///Station still in disconnect status */
    EspErrWifiNotConnect = 0x300F, 
     ///Failed to post the event to WiFi task */
    EspErrWifiPost        = 0x3012, 
    ///Invalid WiFi state when init/deinit is called */
    EspErrWifiInitState  = 0x3013,  
    ///Returned when WiFi is stopping */
    EspErrWifiStopState  = 0x3014, 
    ///The WiFi connection is not associated */ 
    EspErrWifiNotAssoc   = 0x3015,  
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
        feature = "esp32s3",
        feature = "esp32s2"
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
    _magic: ESP_WIFI_OS_ADAPTER_MAGIC as i32,
};

const CONFIG_FEATURE_WPA3_SAE_BIT: u64 = 1 << 0;

// unsafe impl Sync for wifi_init_config_t {}
// unsafe impl Sync for wifi_osi_funcs_t {}

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

        crate::wifi_set_log_verbose();
        esp_wifi_result!(esp_supplicant_init())?;

        esp_wifi_result!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;

        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
                bssid_set: false,
                bssid: [0; 6],
                channel: 0,
                listen_interval: 3,
                sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                threshold: wifi_scan_threshold_t {
                    rssi: 20,
                    authmode: wifi_auth_mode_t_WIFI_AUTH_OPEN,
                },
                pmf_cfg: wifi_pmf_config_t {
                    capable: false,
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
        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))?;

        esp_wifi_result!(esp_wifi_set_tx_done_cb(Some(esp_wifi_tx_done_cb)))?;

        esp_wifi_result!(esp_wifi_internal_reg_rxcb(esp_interface_t_ESP_IF_WIFI_STA, Some(recv_cb)))?;

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
            queue.enqueue(packet);
            esp_wifi_internal_free_rx_buffer(eb);
            0
        } else {
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

        #[cfg(any(coex, feature = "ps_min_modem"))]
        esp_wifi_result!(esp_wifi_set_ps(crate::binary::include::wifi_ps_type_t_WIFI_PS_MIN_MODEM))?;

        #[cfg(not(any(coex, feature = "ps_min_modem")))]
        esp_wifi_result!(esp_wifi_set_ps(crate::binary::include::wifi_ps_type_t_WIFI_PS_NONE))?;

        let cntry_code = [b'C', b'N', 0];
        let country = wifi_country_t {
            cc: cntry_code,
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

pub fn wifi_start_scan() -> i32 {
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

    unsafe { esp_wifi_scan_start(&scan_config, true) }
}

pub fn wifi_connect(ssid: &str, password: &str) -> Result<(), WifiError> {
    unsafe {
        let mut cfg = wifi_config_t {
            sta: wifi_sta_config_t {
                ssid: [0; 32],
                password: [0; 64],
                scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
                bssid_set: false,
                bssid: [0; 6],
                channel: 0,
                listen_interval: 3,
                sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                threshold: wifi_scan_threshold_t {
                    rssi: -99,
                    authmode: wifi_auth_mode_t_WIFI_AUTH_OPEN,
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

        cfg.sta.ssid[0..(ssid.len())].copy_from_slice(ssid.as_bytes());
        cfg.sta.password[0..(password.len())].copy_from_slice(password.as_bytes());

        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))?;
    
        esp_wifi_result!(esp_wifi_connect())
    }
}

pub fn wifi_stop() -> Result<(), WifiError> {
    unsafe { esp_wifi_result!(esp_wifi_stop()) }
}

/// A wifi device implementing smoltcp's Device trait.
pub struct WifiDevice {
    config: embedded_svc::wifi::Configuration,
}

impl WifiDevice {
    pub fn new_with_config(config: embedded_svc::wifi::Configuration) -> WifiDevice {
        WifiDevice { config }
    }

    pub fn new() -> WifiDevice {
        Self {
            config: Default::default()
        }
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
impl<'a> Device<'a> for WifiDevice {
    type RxToken = WifiRxToken;

    type TxToken = WifiTxToken;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        critical_section::with(|cs| {
            let queue = DATA_QUEUE_RX.borrow_ref_mut(cs);

            if !queue.is_empty() {
                Some((WifiRxToken::default(), WifiTxToken::default()))
            } else {
                None
            }
        })
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(WifiTxToken::default())
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1514;
        caps.max_burst_size = Some(1);
        caps
    }
}

#[derive(Debug, Default)]
pub struct WifiRxToken {}

impl RxToken for WifiRxToken {
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        critical_section::with(|cs| {
            let mut queue = DATA_QUEUE_RX.borrow_ref_mut(cs);

            if let Some(mut data) = queue.dequeue() {
                let buffer =
                    unsafe { core::slice::from_raw_parts(&data.data as *const u8, data.len) };
                dump_packet_info(&buffer);
                f(&mut data.data[..])
            } else {
                Err(smoltcp::Error::Exhausted)
            }
        })
    }
}

#[derive(Debug, Default)]
pub struct WifiTxToken {}

impl TxToken for WifiTxToken {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        let res = critical_section::with(|cs| {
            let mut queue = DATA_QUEUE_TX.borrow_ref_mut(cs);

            if queue.is_full() {
                Err(smoltcp::Error::Exhausted)
            } else {
                let mut packet = DataFrame::new();
                packet.len = len;
                let res = f(&mut packet.data[..len]);
                queue.enqueue(packet);
                res
            }
        });

        send_data_if_needed();
        res
    }
}

pub fn send_data_if_needed() {
    critical_section::with(|cs| {
        let mut queue = DATA_QUEUE_TX.borrow_ref_mut(cs);

        while let Some(packet) = queue.dequeue() {
            log::trace!("sending... {} bytes", packet.len);
            dump_packet_info(packet.slice());

            unsafe {
                let _res = esp_wifi_internal_tx(
                    wifi_interface_t_WIFI_IF_STA,
                    &packet.data as *const _ as *mut crate::binary::c_types::c_void,
                    packet.len as u16,
                );
                if _res != 0 {
                    log::warn!("esp_wifi_internal_tx {}", _res);
                }
                log::trace!("esp_wifi_internal_tx {}", _res);
            }
        }
    });
}

impl embedded_svc::wifi::Wifi for WifiDevice {
    type Error = WifiError;

    /// This currently only supports the `Client` capability.
    fn get_capabilities(&self) -> Result<EnumSet<embedded_svc::wifi::Capability>, Self::Error> {
        // for now we only support STA mode
        let mut caps = EnumSet::empty();
        caps.insert(embedded_svc::wifi::Capability::Client);
        Ok(caps)
    }

    /// A blocking wifi network scan.
    fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        esp_wifi_result!(crate::wifi::wifi_start_scan())?;

        let mut scanned = heapless::Vec::<AccessPointInfo, N>::new();
        let mut bss_total: u16 = N as u16;

        unsafe {
            esp_wifi_result!(crate::binary::include::esp_wifi_scan_get_ap_num(&mut bss_total))?;
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
                    _bitfield_1: crate::binary::include::wifi_he_ap_info_t::new_bitfield_1(0, 0, 0),
                    bssid_index: 0,
                },
            }; N];

            esp_wifi_result!(crate::binary::include::esp_wifi_scan_get_ap_records(
                &mut bss_total,
                &mut records as *mut crate::binary::include::wifi_ap_record_t,
            ))?;

            for i in 0..bss_total {
                let record = records[i as usize];
                let ssid_strbuf = crate::compat::common::StrBuf::from(&record.ssid as *const u8);

                let auth_method = match record.authmode {
                    crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
                    crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::WEP,
                    crate::binary::include::wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthMethod::WPA,
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
    }

    /// Get the currently used configuration.
    fn get_configuration(&self) -> Result<embedded_svc::wifi::Configuration, Self::Error> {
        Ok(self.config.clone())
    }

    /// Set the configuration, you need to use Wifi::connect() for connecting
    /// Currently only `ssid` and `password` is used. Trying anything but `Configuration::Client` will result in a panic!
    fn set_configuration(
        &mut self,
        conf: &embedded_svc::wifi::Configuration,
    ) -> Result<(), Self::Error> {
        self.config = conf.clone();

        match conf {
            embedded_svc::wifi::Configuration::None => panic!(),
            embedded_svc::wifi::Configuration::Client(_) => {}
            embedded_svc::wifi::Configuration::AccessPoint(_) => panic!(),
            embedded_svc::wifi::Configuration::Mixed(_, _) => panic!(),
        };

        Ok(())
    }

    fn start(&mut self) -> Result<(), Self::Error> {
        crate::wifi::wifi_start()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        crate::wifi::wifi_stop()
    }

    fn connect(&mut self) -> Result<(), Self::Error> {
        if let embedded_svc::wifi::Configuration::Client(config) = &self.config {
            crate::wifi::wifi_connect(&config.ssid, &config.password)?;
        } else {
            panic!();
        }
        Ok(())
    }

    fn disconnect(&mut self) -> Result<(), Self::Error> {
        //FIXME: Is there a way to disconnect from Wifi?
        Ok(())
    }

    fn is_started(&self) -> Result<bool, Self::Error> {
        match crate::wifi::get_wifi_state() {
            crate::wifi::WifiState::StaStart => Ok(true),
            crate::wifi::WifiState::StaConnected => Ok(true),
            //FIXME: Should any of the enum values trigger an error instead of returning false?
            _ => Ok(false),
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

fn dump_packet_info(buffer: &[u8]) {
    if !DUMP_PACKETS {
        return;
    }

    let ef = smoltcp::wire::EthernetFrame::new_unchecked(buffer);
    info!(
        "src={:x?} dst={:x?} type={:x?}",
        ef.src_addr(),
        ef.dst_addr(),
        ef.ethertype()
    );
    match ef.ethertype() {
        smoltcp::wire::EthernetProtocol::Ipv4 => {
            let ip = smoltcp::wire::Ipv4Packet::new_unchecked(ef.payload());
            info!(
                "src={:?} dst={:?} proto={:x?}",
                ip.src_addr(),
                ip.dst_addr(),
                ip.protocol()
            );

            match ip.protocol() {
                smoltcp::wire::IpProtocol::HopByHop => {}
                smoltcp::wire::IpProtocol::Icmp => {}
                smoltcp::wire::IpProtocol::Igmp => {}
                smoltcp::wire::IpProtocol::Tcp => {
                    let tp = smoltcp::wire::TcpPacket::new_unchecked(ip.payload());
                    info!("src={:?} dst={:?}", tp.src_port(), tp.dst_port());
                }
                smoltcp::wire::IpProtocol::Udp => {
                    let up = smoltcp::wire::UdpPacket::new_unchecked(ip.payload());
                    info!("src={:?} dst={:?}", up.src_port(), up.dst_port());
                }
                smoltcp::wire::IpProtocol::Ipv6Route => {}
                smoltcp::wire::IpProtocol::Ipv6Frag => {}
                smoltcp::wire::IpProtocol::Icmpv6 => {}
                smoltcp::wire::IpProtocol::Ipv6NoNxt => {}
                smoltcp::wire::IpProtocol::Ipv6Opts => {}
                smoltcp::wire::IpProtocol::Unknown(_) => {}
            }
        }
        smoltcp::wire::EthernetProtocol::Arp => {
            let ap = smoltcp::wire::ArpPacket::new_unchecked(ef.payload());
            info!(
                "src={:x?} dst={:x?} src proto addr={:x?}",
                ap.source_hardware_addr(),
                ap.target_hardware_addr(),
                ap.source_protocol_addr()
            );
        }
        smoltcp::wire::EthernetProtocol::Ipv6 => {}
        smoltcp::wire::EthernetProtocol::Unknown(_) => {}
    }
}

#[macro_export]
macro_rules! esp_wifi_result {
    ($value:expr) => {
        if $value != crate::binary::include::ESP_OK as i32 {
            Err(WifiError::InternalError(FromPrimitive::from_i32($value).unwrap()))
        } else {
            core::result::Result::<(), WifiError>::Ok(())
        }
    }
}
