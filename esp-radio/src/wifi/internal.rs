use super::os_adapter::{self, *};
use crate::{
    common_adapter::*,
    sys::include::{
        ESP_WIFI_OS_ADAPTER_MAGIC,
        ESP_WIFI_OS_ADAPTER_VERSION,
        wifi_init_config_t,
        wifi_osi_funcs_t,
    },
};
#[cfg(coex)]
use crate::{hal::ram, sys::c_types::c_void};

#[cfg(all(coex, any(esp32, esp32c2, esp32c3, esp32c6, esp32s3)))]
pub(super) static mut G_COEX_ADAPTER_FUNCS: crate::sys::include::coex_adapter_funcs_t =
    crate::sys::include::coex_adapter_funcs_t {
        _version: crate::sys::include::COEX_ADAPTER_VERSION as i32,
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
        _esp_timer_get_time: Some(__esp_radio_esp_timer_get_time),
        _env_is_chip: Some(env_is_chip),
        _magic: crate::sys::include::COEX_ADAPTER_MAGIC as i32,
        _timer_disarm: Some(ets_timer_disarm),
        _timer_done: Some(ets_timer_done),
        _timer_setfn: Some(ets_timer_setfn),
        _timer_arm_us: Some(ets_timer_arm_us),

        #[cfg(esp32)]
        _spin_lock_create: Some(spin_lock_create),
        #[cfg(esp32)]
        _spin_lock_delete: Some(spin_lock_delete),
        #[cfg(esp32)]
        _int_disable: Some(wifi_int_disable),
        #[cfg(esp32)]
        _int_enable: Some(wifi_int_restore),

        #[cfg(esp32c2)]
        _slowclk_cal_get: Some(slowclk_cal_get),

        _debug_matrix_init: Some(esp_coexist_debug_matrix_init_wrapper),
        _xtal_freq_get: Some(xtal_freq_get_wrapper),
    };

#[cfg(coex)]
#[ram]
unsafe extern "C" fn xtal_freq_get_wrapper() -> i32 {
    crate::hal::clock::Clocks::get().xtal_clock.as_mhz() as i32
}

#[cfg(coex)]
unsafe extern "C" fn esp_coexist_debug_matrix_init_wrapper(
    _evt: i32,
    _sig: i32,
    _rev: bool,
) -> i32 {
    // CONFIG_ESP_COEX_GPIO_DEBUG not supported
    crate::sys::include::ESP_ERR_NOT_SUPPORTED as i32
}

#[cfg(coex)]
#[ram]
unsafe extern "C" fn semphr_take_from_isr_wrapper(semphr: *mut c_void, hptw: *mut c_void) -> i32 {
    unsafe { crate::common_adapter::semphr_take_from_isr(semphr, hptw as *mut bool) }
}

#[cfg(coex)]
#[ram]
unsafe extern "C" fn semphr_give_from_isr_wrapper(semphr: *mut c_void, hptw: *mut c_void) -> i32 {
    unsafe { crate::common_adapter::semphr_give_from_isr(semphr, hptw as *mut bool) }
}

#[cfg(coex)]
unsafe extern "C" fn is_in_isr_wrapper() -> i32 {
    crate::is_interrupts_disabled() as i32
}

#[unsafe(no_mangle)]
pub(crate) static __ESP_RADIO_G_WIFI_OSI_FUNCS: wifi_osi_funcs_t = wifi_osi_funcs_t {
    _version: ESP_WIFI_OS_ADAPTER_VERSION as i32,
    _env_is_chip: Some(env_is_chip),
    _set_intr: Some(set_intr),
    _clear_intr: Some(clear_intr),
    _set_isr: Some(os_adapter_chip_specific::set_isr),
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
    _phy_disable: Some(os_adapter::phy_disable),
    _phy_enable: Some(os_adapter::phy_enable),
    _phy_update_country_info: Some(phy_update_country_info),
    _read_mac: Some(read_mac),
    _timer_arm: Some(ets_timer_arm),
    _timer_disarm: Some(ets_timer_disarm),
    _timer_done: Some(ets_timer_done),
    _timer_setfn: Some(ets_timer_setfn),
    _timer_arm_us: Some(ets_timer_arm_us),
    _wifi_reset_mac: Some(wifi_reset_mac),
    _wifi_clock_enable: Some(wifi_clock_enable),
    _wifi_clock_disable: Some(wifi_clock_disable),
    _wifi_rtc_enable_iso: Some(wifi_rtc_enable_iso),
    _wifi_rtc_disable_iso: Some(wifi_rtc_disable_iso),
    _esp_timer_get_time: Some(__esp_radio_esp_timer_get_time),
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
    #[cfg(feature = "sys-logs")]
    _log_write: Some(log_write),
    #[cfg(not(feature = "sys-logs"))]
    _log_write: None,
    #[cfg(feature = "sys-logs")]
    _log_writev: Some(log_writev),
    #[cfg(not(feature = "sys-logs"))]
    _log_writev: None,
    _log_timestamp: Some(log_timestamp),
    _malloc_internal: Some(malloc_internal),
    _realloc_internal: Some(realloc_internal),
    _calloc_internal: Some(calloc_internal_wrapper),
    _zalloc_internal: Some(zalloc_internal),
    _wifi_malloc: Some(wifi_malloc),
    _wifi_realloc: Some(wifi_realloc),
    _wifi_calloc: Some(wifi_calloc),
    _wifi_zalloc: Some(wifi_zalloc),
    _wifi_create_queue: Some(wifi_create_queue),
    _wifi_delete_queue: Some(wifi_delete_queue),
    _coex_init: Some(super::coex_init),
    _coex_deinit: Some(coex_deinit),
    _coex_enable: Some(coex_enable),
    _coex_disable: Some(coex_disable),
    _coex_status_get: Some(coex_status_get),
    _coex_condition_set: None,
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
    #[cfg(any(esp32c3, esp32c2, esp32c6, esp32h2, esp32s3, esp32s2))]
    _slowclk_cal_get: Some(slowclk_cal_get),
    #[cfg(any(esp32, esp32s2))]
    _phy_common_clock_disable: Some(os_adapter_chip_specific::phy_common_clock_disable),
    #[cfg(any(esp32, esp32s2))]
    _phy_common_clock_enable: Some(os_adapter_chip_specific::phy_common_clock_enable),
    _coex_register_start_cb: Some(coex_register_start_cb),

    #[cfg(esp32c6)]
    _regdma_link_set_write_wait_content: Some(
        os_adapter_chip_specific::regdma_link_set_write_wait_content_dummy,
    ),
    #[cfg(esp32c6)]
    _sleep_retention_find_link_by_id: Some(
        os_adapter_chip_specific::sleep_retention_find_link_by_id_dummy,
    ),
    _coex_schm_process_restart: Some(coex_schm_process_restart_wrapper),
    _coex_schm_register_cb: Some(coex_schm_register_cb_wrapper),

    _coex_schm_flexible_period_set: Some(coex_schm_flexible_period_set),
    _coex_schm_flexible_period_get: Some(coex_schm_flexible_period_get),
    _coex_schm_get_phase_by_idx: Some(coex_schm_get_phase_by_idx),

    _magic: ESP_WIFI_OS_ADAPTER_MAGIC as i32,
};

const WIFI_ENABLE_WPA3_SAE: u64 = 1 << 0;
const WIFI_ENABLE_ENTERPRISE: u64 = 1 << 7;
// const WIFI_FTM_INITIATOR: u64 = 1 << 2;
// const WIFI_FTM_RESPONDER: u64 = 1 << 3;
// const WIFI_ENABLE_GCMP: u64 = 1 << 4;
// const WIFI_ENABLE_GMAC: u64 = 1 << 5;
// const WIFI_ENABLE_11R: u64 = 1 << 6;

const WIFI_FEATURE_CAPS: u64 = WIFI_ENABLE_WPA3_SAE | WIFI_ENABLE_ENTERPRISE;

#[unsafe(no_mangle)]
pub(super) static mut __ESP_RADIO_G_WIFI_FEATURE_CAPS: u64 = WIFI_FEATURE_CAPS;

pub(super) static mut G_CONFIG: wifi_init_config_t = unsafe { core::mem::zeroed() };
