//! Strong definitions overriding weak symbols in the Wi-Fi blobs, mirroring the stubs ESP-IDF
//! defines in `components/esp_wifi/src/wifi_init.c` when the corresponding feature is disabled.
//!
//! Each group must only be compiled for chips whose blobs define the overridden symbols:
//! these `no_mangle` functions can't be garbage-collected, so on other chips they only waste flash.

#[cfg(any(
    all(wifi_has_nan, not(wifi_nan_sync_enable)),
    all(wifi_has_ftm, not(wifi_ftm_enable))
))]
use crate::sys::include::{ESP_OK, esp_err_t};

#[cfg(all(wifi_has_nan, not(wifi_nan_sync_enable)))]
mod nan {
    use super::*;
    use crate::sys::{
        c_types::{c_int, c_void},
        include::wifi_nan_sync_config_t,
    };

    #[unsafe(no_mangle)]
    extern "C" fn nan_start() -> esp_err_t {
        ESP_OK as esp_err_t
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_stop() -> esp_err_t {
        ESP_OK as esp_err_t
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_input(_p1: *mut c_void, _p2: c_int, _p3: c_int) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_sm_handle_event(_p1: *mut c_void, _p2: c_int) {}

    #[unsafe(no_mangle)]
    extern "C" fn wifi_create_nan() -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn wifi_nan_set_config_local(_p: *mut wifi_nan_sync_config_t) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_dp_post_tx(_p1: *mut c_void, _p2: *mut c_void) -> esp_err_t {
        ESP_OK as esp_err_t
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_dp_delete_peer(_p: *mut c_void) {}

    #[unsafe(no_mangle)]
    extern "C" fn nan_dp_search_node(_p: *const u8) -> *mut c_void {
        core::ptr::null_mut()
    }

    #[unsafe(no_mangle)]
    extern "C" fn nan_ndp_resp_timeout_process(_p: *mut c_void) {}
}

#[cfg(all(wifi_has_ftm, not(wifi_ftm_enable)))]
mod ftm {
    use super::*;

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_ftm_attach() -> esp_err_t {
        ESP_OK as esp_err_t
    }

    #[unsafe(no_mangle)]
    extern "C" fn ftm_initiator_cleanup() {}
}

#[cfg(not(wifi_softap_support))]
mod softap {
    use crate::sys::c_types::{c_int, c_void};

    #[unsafe(no_mangle)]
    extern "C" fn net80211_softap_funcs_init() {}

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_ap_try_sa_query(_p: *mut c_void) -> bool {
        false
    }

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_ap_sa_query_timeout(_p: *mut c_void) -> bool {
        false
    }

    #[unsafe(no_mangle)]
    extern "C" fn add_mic_ie_bip(_p: *mut c_void) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_free_beacon_eb() {}

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_pwrsave(_p1: *mut c_void, _p2: *mut c_void) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn cnx_node_remove(_p: *mut c_void) {}

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_set_tim(_p: *mut c_void, _arg: c_int) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_is_bufferable_mmpdu(_p: *mut c_void) -> bool {
        false
    }

    #[unsafe(no_mangle)]
    extern "C" fn cnx_node_leave(_p: *mut c_void, _arg: u8) {}

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_beacon_construct(
        _p1: *mut c_void,
        _p2: *mut c_void,
        _p3: *mut c_void,
        _p4: *mut c_void,
    ) {
    }

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_assoc_resp_construct(_p: *mut c_void, _arg: c_int) -> *mut c_void {
        core::ptr::null_mut()
    }

    #[unsafe(no_mangle)]
    extern "C" fn ieee80211_alloc_proberesp(_p: *mut c_void, _arg: c_int) -> *mut c_void {
        core::ptr::null_mut()
    }

    #[unsafe(no_mangle)]
    extern "C" fn hostap_query_mac_in_list(_p: *const u8, _arg: c_int) -> bool {
        false
    }

    #[unsafe(no_mangle)]
    extern "C" fn hostap_add_in_mac_list(_p: *const u8, _arg: c_int) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn hostap_del_mac_info_from_list(_p: *const u8, _arg: c_int) -> c_int {
        0
    }

    #[unsafe(no_mangle)]
    extern "C" fn create_new_bss_for_sa_query_failed_sta(_arg: u8) {}
}

mod beacon_offset {
    unsafe extern "C" {
        fn pm_beacon_offset_funcs_empty_init();
    }

    #[unsafe(no_mangle)]
    extern "C" fn pm_beacon_offset_funcs_init() {
        unsafe { pm_beacon_offset_funcs_empty_init() }
    }
}
