//! IDF `porting_btdm` controller (nested BLE/BR-EDR/BTDM config + `esp_*` VHCI).
//!
//! Distinct from the original BTDM OSI / `API_vhci_host_*` path used on
//! ESP32/C3/S3 (`bt_controller = "btdm"`).
//!
//! The second half of this file is the `wr_btdm_osal_*` OSAL the blob calls
//! back into. Objects are heap-allocated (IDF's `BTDM_MEMPOOL_ALLOC` path is
//! not used) and, as with IDF's `BTDM_OSAL_USE_ESP_TIMER`, one tick is one
//! millisecond.

use core::{
    mem::{size_of, transmute},
    ptr::NonNull,
    sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering},
};

use esp_phy::PhyInitGuard;
use portable_atomic::AtomicBool;

use super::{Config, ReceivedPacket};
use crate::{
    ble::{HCI_OUT_COLLECTOR, HciOutCollector},
    compat::{
        self,
        common::str_from_c,
        mutex,
        queue::{
            queue_create,
            queue_delete,
            queue_messages_waiting,
            queue_receive,
            queue_remove,
            queue_send_to_back,
            queue_send_to_front,
            queue_try_receive_from_isr,
            queue_try_send_to_back_from_isr,
            queue_try_send_to_front_from_isr,
        },
        semaphore,
    },
    hal::{ram, system::Cpu},
    sys::{
        c_types::{c_char, c_void},
        include::*,
    },
    time::{blob_ticks_to_micros, blob_ticks_to_millis, millis_to_blob_ticks},
};

#[cfg_attr(esp32s31, path = "os_adapter_esp32s31.rs")]
pub(crate) mod chip_specific;

const EXT_FUNC_VERSION: u32 = 0x20250819;
const EXT_FUNC_MAGIC: u32 = 0xA5A5A5A5;
const ACL_DATA_MBUF_LEADINGSPACE: usize = 4;

static CONTROLLER_STATUS: AtomicU32 =
    AtomicU32::new(esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_IDLE);
/// IDF `s_bt_active`. While set, `e_btdm_lp_modem_clock_set(false)` is a
/// no-op so the blob cannot gate MAC/BB clocks between HCI commands.
static BT_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Out-parameter of `r_ble_ll_get_npl_element_info`: how many OSAL elements of
/// each kind the controller needs. IDF pre-allocates a pool per kind; we
/// allocate on demand, so only `evt_count` is of any use to us.
#[derive(Default)]
#[repr(C)]
struct OsalElemNum {
    evt_count: u16,
    evtq_count: u16,
    co_count: u16,
    sem_count: u16,
    mutex_count: u16,
}

#[repr(C)]
struct ExtFuncsT {
    ext_version: u32,
    malloc: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    free: Option<unsafe extern "C" fn(*mut c_void)>,
    osi_assert: Option<unsafe extern "C" fn(u32, *const c_void, u32, u32)>,
    os_random: Option<unsafe extern "C" fn() -> u32>,
    ecc_gen_key_pair: Option<unsafe extern "C" fn(*mut u8, *mut u8) -> i32>,
    ecc_gen_dh_key: Option<unsafe extern "C" fn(*const u8, *const u8, *const u8, *mut u8) -> i32>,
    esp_reset_rpa_moudle: Option<unsafe extern "C" fn()>,
    ecc_aes_cmac: Option<unsafe extern "C" fn(*const u8, *const u8, usize, *mut u8) -> i32>,
    magic: u32,
}

static EXT_FUNCS: ExtFuncsT = ExtFuncsT {
    ext_version: EXT_FUNC_VERSION,
    malloc: Some(wr_btdm_osal_malloc),
    free: Some(wr_btdm_osal_free),
    osi_assert: Some(osi_assert),
    os_random: Some(os_random),
    ecc_gen_key_pair: Some(ecc_gen_key_pair),
    ecc_gen_dh_key: Some(ecc_gen_dh_key),
    esp_reset_rpa_moudle: Some(reset_rpa),
    ecc_aes_cmac: Some(ecc_aes_cmac),
    magic: EXT_FUNC_MAGIC,
};

/// Chained memory buffer used by the NimBLE-style controller.
#[repr(C)]
struct OsMbuf {
    om_data: *const u8,
    om_flags: u8,
    om_pkthdr_len: u8,
    om_len: u16,
    om_omp: *const c_void,
    next: *const OsMbuf,
}

unsafe extern "C" {
    fn r_btdm_task_init(cfg: *mut esp_btdm_controller_config_t) -> i32;
    fn r_btdm_task_deinit();
    fn r_btdm_task_enable() -> i32;
    fn r_btdm_task_disable();
    fn r_btdm_hal_rtc_freq_set(freq: u64);
    fn r_esp_ble_change_rtc_freq(freq: u32);

    fn esp_register_ext_funcs(funcs: *const ExtFuncsT) -> i32;
    fn esp_unregister_ext_funcs();
    fn esp_ble_register_bb_funcs() -> i32;
    fn esp_ble_unregister_bb_funcs() -> i32;
    fn r_ble_controller_init(cfg: *mut esp_bt_controller_config_t__bindgen_ty_1) -> i32;
    fn r_ble_controller_deinit() -> i32;
    fn r_esp_ble_msys_init(
        msys_size1: u16,
        msys_size2: u16,
        msys_cnt1: u16,
        msys_cnt2: u16,
        from_heap: u8,
    ) -> i32;
    fn r_esp_ble_msys_deinit();
    fn r_esp_ble_ll_set_public_addr(addr: *const u8);
    fn r_ble_ll_get_npl_element_info(
        cfg: *mut esp_bt_controller_config_t__bindgen_ty_1,
        elem: *mut OsalElemNum,
    ) -> i32;
    fn r_base_stack_initEnv() -> i32;
    fn r_base_stack_deinitEnv() -> i32;
    fn r_base_stack_enable() -> i32;
    fn r_base_stack_disable();
    fn r_conn_stack_initEnv() -> i32;
    fn r_conn_stack_deinitEnv();
    fn r_conn_stack_enable() -> i32;
    fn r_conn_stack_disable();
    fn bt_bb_v2_init_cmplx(value: u8);
    fn coex_pti_v2();
    fn bt_bb_get_tx_pwr_table(length: *mut u8) -> *const i8;

    fn r_ble_hci_trans_cfg_hs(
        evt: Option<unsafe extern "C" fn(cmd: *const u8, arg: *const c_void) -> i32>,
        evt_arg: *const c_void,
        acl_cb: Option<unsafe extern "C" fn(om: *const OsMbuf, arg: *const c_void) -> i32>,
        acl_arg: *const c_void,
    );
    fn r_ble_hci_trans_buf_free(buf: *const u8);
    fn r_ble_hci_trans_hs_acl_tx(om: *mut OsMbuf) -> i32;
    fn r_os_msys_get_pkthdr(dsize: u16, user_hdr_len: u16) -> *mut OsMbuf;
    fn r_os_mbuf_append(om: *mut OsMbuf, src: *const u8, len: u16) -> i32;
    fn r_os_mbuf_free_chain(om: *mut OsMbuf) -> i32;

    fn r_btdm_hci_trans_buf_alloc(typ: u8, conn_handle: u16) -> *mut HciDriverPacket;
    fn r_btdm_hci_trans_buf_free(pkt: *mut HciDriverPacket);
    fn r_btdm_hci_trans_rx(pkt: *mut HciDriverPacket) -> i32;
    fn r_btdm_hci_trans_register_tx(tx_func: *const c_void, async_: bool) -> i32;

    // IDF `UC_BT_CTRL_CONN_FC_ENABLE` is hardcoded on. The BLE stack
    // calls these accessors after a connection; without `env_init` the
    // env pointer stays NULL and `r_sym_bt_E5PSb5NmRdfYVGFUOLdR` load-
    // faults at offset 1.
    fn r_btdm_hci_fc_env_init() -> i32;
    fn r_btdm_hci_fc_env_deinit();
    fn r_btdm_hci_fc_enable() -> i32;
    fn r_btdm_hci_fc_disable();
}

/// IDF `hci_driver_packet_t` (`STAILQ_ENTRY` is one next pointer).
#[repr(C)]
struct HciDriverPacket {
    next: *mut HciDriverPacket,
    data: *mut u8,
    length: u16,
    flags: u8,
    type_: u8,
}

const HCI_DRIVER_TYPE_CMD: u8 = 0x01;

unsafe extern "C" fn osi_assert(ln: u32, fn_name: *const c_void, param1: u32, param2: u32) {
    unsafe {
        let name = crate::compat::common::str_from_c(fn_name.cast());
        panic!("BLE assert {}:{} 0x{:x} 0x{:x}", name, ln, param1, param2);
    }
}

unsafe extern "C" fn os_random() -> u32 {
    unsafe { crate::common_adapter::random() as u32 }
}

unsafe extern "C" fn reset_rpa() {
    crate::radio_clocks::clocks_ll::reset_rpa();
}

unsafe extern "C" fn ecc_gen_key_pair(_pub: *mut u8, _priv: *mut u8) -> i32 {
    -1
}

unsafe extern "C" fn ecc_gen_dh_key(
    _x: *const u8,
    _y: *const u8,
    _priv: *const u8,
    _out: *mut u8,
) -> i32 {
    -1
}

unsafe extern "C" fn ecc_aes_cmac(
    _key: *const u8,
    _input: *const u8,
    _len: usize,
    _out: *mut u8,
) -> i32 {
    -1
}

fn controller_status() -> u32 {
    CONTROLLER_STATUS.load(Ordering::Relaxed)
}

fn set_controller_status(status: u32) {
    CONTROLLER_STATUS.store(status, Ordering::Relaxed);
}

/// Number of entries to size the event queue for, following IDF's
/// `esp_bt_controller_init` seed of `3 + 100` plus the `+10` workaround in
/// `ble_osal_elem_calc`.
fn eventq_depth(cfg: *mut esp_bt_controller_config_t) -> u16 {
    let mut elem = OsalElemNum::default();
    // On failure `elem` stays zeroed, leaving us with IDF's seed.
    unsafe { r_ble_ll_get_npl_element_info(&raw mut (*cfg).ble, &mut elem) };
    elem.evt_count.saturating_add(113)
}

fn ble_stack_init(cfg: *mut esp_bt_controller_config_t) -> i32 {
    unsafe {
        let res = esp_register_ext_funcs(&raw const EXT_FUNCS);
        if res != 0 {
            warn!("esp_register_ext_funcs failed {}", res);
            return res;
        }

        // C6 IDF sets this to `s_bt_lpclk_freq` for MAIN_XTAL (100 kHz).
        // porting_btdm's S31 stub still hardcodes 32000; the HW divider we
        // program is 100 kHz. Advertising enable busy-waits on cputime.
        (*cfg).ble.rtc_freq = crate::radio_clocks::clocks_ll::BT_LPCLK_HZ;

        let res = esp_ble_register_bb_funcs();
        if res != 0 {
            warn!("esp_ble_register_bb_funcs failed {}", res);
            return res;
        }

        let res = r_ble_controller_init(&raw mut (*cfg).ble);
        if res != 0 {
            warn!("r_ble_controller_init failed {}", res);
            return res;
        }

        r_esp_ble_change_rtc_freq(crate::radio_clocks::clocks_ll::BT_LPCLK_HZ as u32);

        let res = r_esp_ble_msys_init(
            CONFIG_BT_LE_MSYS_1_BLOCK_SIZE as u16,
            CONFIG_BT_LE_MSYS_2_BLOCK_SIZE as u16,
            CONFIG_BT_LE_MSYS_1_BLOCK_COUNT as u16,
            CONFIG_BT_LE_MSYS_2_BLOCK_COUNT as u16,
            CONFIG_BT_LE_MSYS_BUF_FROM_HEAP as u8,
        );
        if res != 0 {
            warn!("r_esp_ble_msys_init failed {}", res);
            return res;
        }

        let mut mac = [0u8; 6];
        crate::common_adapter::read_mac(mac.as_mut_ptr(), 2);
        mac.reverse();
        r_esp_ble_ll_set_public_addr(mac.as_ptr());

        let res = r_base_stack_initEnv();
        if res != 0 {
            warn!("r_base_stack_initEnv failed {}", res);
            return res;
        }

        let res = r_conn_stack_initEnv();
        if res != 0 {
            warn!("r_conn_stack_initEnv failed {}", res);
            return res;
        }
    }
    0
}

fn ble_stack_deinit() {
    unsafe {
        r_conn_stack_deinitEnv();
        let _ = r_base_stack_deinitEnv();
        r_esp_ble_msys_deinit();
        let _ = r_ble_controller_deinit();
        let _ = esp_ble_unregister_bb_funcs();
        esp_unregister_ext_funcs();
    }
}

fn ble_stack_enable() -> i32 {
    unsafe {
        let res = r_base_stack_enable();
        if res != 0 {
            return res;
        }
        // `r_conn_stack_enable` in this blob is a 2-byte `c.jr ra` that
        // never writes a0, so the "return value" is leftover register
        // garbage (we saw 8). Connection setup is done in initEnv.
        let _ = r_conn_stack_enable();
        0
    }
}

fn ble_stack_disable() {
    unsafe {
        r_conn_stack_disable();
        r_base_stack_disable();
    }
}

fn bt_controller_deinit() {
    BT_ACTIVE.store(false, Ordering::Relaxed);
    set_controller_status(esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_IDLE);
    unsafe {
        // `r_ble_controller_init` registers the two coex callbacks, but
        // `r_ble_controller_deinit` frees their environment without
        // unregistering them. The next `coex_enable` / `coex_disable` from
        // Wi-Fi would run them against a NULL environment.
        #[cfg(feature = "coex")]
        {
            coex_register_ble_cb(0, core::ptr::null_mut());
            coex_register_ble_cb(1, core::ptr::null_mut());
        }

        r_ble_hci_trans_cfg_hs(None, core::ptr::null(), None, core::ptr::null());
        ble_stack_deinit();
        r_btdm_hci_fc_env_deinit();
        r_btdm_task_deinit();
        crate::radio_clocks::clocks_ll::enable_bt(false);
    }
}

fn esp_bt_controller_init(cfg: *mut esp_bt_controller_config_t) -> esp_err_t {
    if cfg.is_null() {
        return ESP_ERR_INVALID_ARG as _;
    }
    if controller_status() != esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_IDLE {
        return ESP_ERR_INVALID_STATE as _;
    }

    unsafe {
        if (*cfg).btdm.bluetooth_mode == esp_bt_mode_t_ESP_BT_MODE_IDLE as u8 {
            return ESP_ERR_INVALID_ARG as _;
        }

        EVENTQ_DEPTH.store(eventq_depth(cfg), Ordering::Relaxed);

        crate::radio_clocks::clocks_ll::enable_bt(true);
        // IDF `btdm_lp_init` → `r_btdm_hal_rtc_freq_set(s_bt_lpclk_freq)`.
        r_btdm_hal_rtc_freq_set(crate::radio_clocks::clocks_ll::BT_LPCLK_HZ);

        let res = r_btdm_task_init(&raw mut (*cfg).btdm);
        if res != 0 {
            warn!("r_btdm_task_init failed {}", res);
            bt_controller_deinit();
            return ESP_FAIL as _;
        }

        // IDF `esp_bt_controller_init`: after `r_btdm_task_init` /
        // `btdm_lp_init`, before `ble_stack_init`.
        let res = r_btdm_hci_fc_env_init();
        if res != 0 {
            warn!("r_btdm_hci_fc_env_init failed {}", res);
            bt_controller_deinit();
            return ESP_FAIL as _;
        }

        bt_bb_v2_init_cmplx(1);
        coex_pti_v2();
        // Same ROM pointer 802.15.4 TX needs on S31. A null/stale
        // `coex_pti_tab_ptr` leaves the BB waiting for a PTI grant.
        {
            unsafe extern "C" {
                static mut coex_pti_tab_ptr: u32;
                static coex_pti_tab: u8;
            }
            (&raw mut coex_pti_tab_ptr).write_volatile(&raw const coex_pti_tab as u32);
        }

        let res = ble_stack_init(cfg);
        if res != 0 {
            bt_controller_deinit();
            return ESP_FAIL as _;
        }

        // IDF `hci_transport_init` registers this during init so
        // `r_ble_hci_trans_ll_evt_tx` has a host callback before enable.
        r_ble_hci_trans_cfg_hs(
            Some(ble_hs_hci_rx_evt),
            core::ptr::null(),
            Some(ble_hs_rx_data),
            core::ptr::null(),
        );
        // IDF also registers the BTDM-common TX path so dual-mode /
        // controller-originated packets reach the host. The blob returns
        // the previous callback pointer (often a ROM address), not an
        // error code.
        let _prev = r_btdm_hci_trans_register_tx(btdm_hci_controller_tx as *const c_void, false);
    }

    set_controller_status(esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_INITED);
    0
}

fn esp_bt_controller_deinit() -> esp_err_t {
    if controller_status() != esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_INITED {
        return ESP_ERR_INVALID_STATE as _;
    }
    bt_controller_deinit();
    0
}

fn esp_bt_controller_enable(_mode: esp_bt_mode_t) -> esp_err_t {
    if controller_status() != esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_INITED {
        return ESP_ERR_INVALID_STATE as _;
    }

    // IDF `btdm_lp_reset(true)` sets `s_bt_active`, then `esp_phy_enable`
    // + `esp_btbb_enable`, before `r_btdm_task_enable`. PHY is already
    // held by `ble_init`; re-init BB here so TX (advertising) is armed.
    BT_ACTIVE.store(true, Ordering::Relaxed);

    unsafe {
        bt_bb_v2_init_cmplx(1);

        let res = ble_stack_enable();
        if res != 0 {
            warn!("ble_stack_enable failed {}", res);
            BT_ACTIVE.store(false, Ordering::Relaxed);
            let _ = esp_bt_controller_disable();
            return ESP_FAIL as _;
        }

        let res = r_btdm_hci_fc_enable();
        if res != 0 {
            warn!("r_btdm_hci_fc_enable failed {}", res);
            BT_ACTIVE.store(false, Ordering::Relaxed);
            let _ = esp_bt_controller_disable();
            return ESP_FAIL as _;
        }

        let res = r_btdm_task_enable();
        if res != 0 {
            warn!("r_btdm_task_enable failed {}", res);
            BT_ACTIVE.store(false, Ordering::Relaxed);
            let _ = esp_bt_controller_disable();
            return ESP_FAIL as _;
        }
    }

    set_controller_status(esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_ENABLED);
    0
}

fn esp_bt_controller_disable() -> esp_err_t {
    if controller_status() != esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_ENABLED {
        return ESP_ERR_INVALID_STATE as _;
    }

    unsafe {
        r_ble_hci_trans_cfg_hs(None, core::ptr::null(), None, core::ptr::null());
        r_btdm_task_disable();
        r_btdm_hci_fc_disable();
        ble_stack_disable();
    }
    BT_ACTIVE.store(false, Ordering::Relaxed);
    set_controller_status(esp_bt_controller_status_t_ESP_BT_CONTROLLER_STATUS_INITED);
    0
}

#[unsafe(no_mangle)]
extern "C" fn e_btdm_lp_modem_clock_set(enable: bool) {
    trace!("e_btdm_lp_modem_clock_set {:?}", enable);
    // crate::radio_clocks::clocks_ll::enable_bt_clocks(enable);
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_assert(file: *const c_void, line: i32, p0: i32, p1: i32, _ty: u8) {
    unsafe {
        let name = crate::compat::common::str_from_c(file.cast());
        panic!("BLE assert {}:{} 0x{:x} 0x{:x}", name, line, p0, p1);
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_set_flags(_p0: u8) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_reset_flags(_p0: u8) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_internal_x0(_p0: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_internal_x1(_p0: u32, _p1: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_internal_x2(_p0: u32, _p1: u32, _p2: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_internal_x3(_p0: u32, _p1: u32, _p2: u32, _p3: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_internal_hex(_p0: u32, _p1: u32, _p2: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_log_raw_export(_p0: u16, _p1: u32, _p2: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_external_bb_get_tx_pwr_table(length: *mut u8, _modem_cfg: u8) -> *const i8 {
    unsafe { bt_bb_get_tx_pwr_table(length) }
}

// The controller's view of coexistence, mirroring IDF `btdm_coex.c`. Without
// the `coex` feature these keep the values IDF returns with
// `CONFIG_SW_COEXIST_ENABLE` off.

// Declared `extern` in IDF `btdm_coex.c` rather than in a coex header, so
// esp-wifi-sys does not generate bindings for them.
#[cfg(feature = "coex")]
unsafe extern "C" {
    fn coex_register_ble_cb(ty: u8, func: *mut c_void) -> i32;
    fn coex_ble_idle_time_inform(start_offset: u32, duration: u32);
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_status_bit_set(_ty: u32, _status: u32) {
    #[cfg(feature = "coex")]
    unsafe {
        coex_schm_status_bit_set(_ty, _status)
    };
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_status_bit_clear(_ty: u32, _status: u32) {
    #[cfg(feature = "coex")]
    unsafe {
        coex_schm_status_bit_clear(_ty, _status)
    };
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_register_btdm_callback(_cb: *mut c_void) -> i32 {
    cfg_select! {
        feature = "coex" => unsafe {
            coex_schm_register_callback(coex_schm_callback_type_t_COEX_SCHM_CALLBACK_TYPE_BT, _cb)
        },
        _ => 0,
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_interval_get() -> u32 {
    cfg_select! {
        feature = "coex" => unsafe { coex_schm_interval_get() },
        _ => 0,
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_curr_period_get() -> u8 {
    cfg_select! {
        // Before a scheme is running the scheduler has no period to report.
        // Clamp to the one-period value IDF uses with coex off rather than
        // handing the controller a zero it may divide by.
        feature = "coex" => unsafe { coex_schm_curr_period_get().max(1) },
        _ => 1,
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_schm_curr_phase_get() -> *mut c_void {
    cfg_select! {
        feature = "coex" => unsafe { coex_schm_curr_phase_get() },
        _ => core::ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_register_wifi_channel_change_callback(
    _cb: coex_wifi_channel_change_cb_t,
) -> i32 {
    cfg_select! {
        feature = "coex" => unsafe { coex_register_wifi_channel_change_callback(_cb) },
        _ => -1,
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_wifi_channel_get(_primary: *mut u8, _secondary: *mut u8) -> i32 {
    cfg_select! {
        feature = "coex" => unsafe { coex_wifi_channel_get(_primary, _secondary) },
        _ => -1,
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_version_get(_major: *mut u32, _minor: *mut u32, _path: *mut u32) -> i32 {
    // IDF reports success here while leaving the out-parameters untouched: its
    // `coex_version_get_value` call is `#if 0`-ed out pending a coexist
    // submodule update. Query it for real rather than reporting success over
    // whatever the controller left on its stack.
    cfg_select! {
        feature = "coex" => unsafe {
            let mut version = coex_version_t {
                major: 0,
                minor: 0,
                patch: 0,
            };
            let res = coex_version_get_value(&mut version);
            if res == 0 {
                _major.write(version.major.into());
                _minor.write(version.minor.into());
                _path.write(version.patch.into());
            }
            res
        },
        _ => -1,
    }
}

// The ISO hooks stay stubbed: IDF gates them on `CONFIG_BT_LE_ISO_SUPPORT`,
// which esp-radio does not offer.

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_iso_start(_info: *mut c_void) -> i32 {
    0
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_iso_stop(_handle: u16) -> i32 {
    0
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_iso_protect_frame_thres_get() -> u16 {
    0
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_iso_start_int_handle(_handle: u16, _duration: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_iso_end_int_handle(_handle: u16, _duration: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_ble_idle_time_inform(_start_offset: u32, _duration: u32) {
    cfg_select! {
        feature = "coex" => unsafe { coex_ble_idle_time_inform(_start_offset, _duration) },
        _ => {},
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_coex_register_ble_cb(_ty: u32, _func: *mut c_void) -> i32 {
    cfg_select! {
        feature = "coex" => unsafe { coex_register_ble_cb(_ty as u8, _func) },
        _ => 0,
    }
}

/// Hands a complete HCI packet, indicator byte first, to the host reader.
fn queue_rx_packet(header: &[u8], payload: &[u8]) {
    let mut data = alloc::vec::Vec::with_capacity(header.len() + payload.len());
    data.extend_from_slice(header);
    data.extend_from_slice(payload);

    super::dump_packet_info(&data);
    super::BT_STATE.with(|state| {
        state.rx_queue.push_back(ReceivedPacket {
            data: data.into_boxed_slice(),
        });
    });
    crate::ble::controller::hci_read_data_available();
}

extern "C" fn btdm_hci_controller_tx(pkt: *mut HciDriverPacket) -> i32 {
    let Some(pkt) = (unsafe { pkt.as_mut() }) else {
        return -1;
    };
    queue_rx_packet(&[pkt.type_], unsafe {
        core::slice::from_raw_parts(pkt.data, pkt.length as usize)
    });
    unsafe { r_btdm_hci_trans_buf_free(core::ptr::from_mut(pkt)) };
    0
}

unsafe extern "C" fn ble_hs_hci_rx_evt(cmd: *const u8, _arg: *const c_void) -> i32 {
    let event = unsafe { *cmd };
    let len = unsafe { *cmd.add(1) };
    queue_rx_packet(&[0x04, event, len], unsafe {
        core::slice::from_raw_parts(cmd.add(2), len as usize)
    });
    unsafe { r_ble_hci_trans_buf_free(cmd) };
    0
}

unsafe extern "C" fn ble_hs_rx_data(om: *const OsMbuf, _arg: *const c_void) -> i32 {
    queue_rx_packet(&[0x02], unsafe {
        core::slice::from_raw_parts((*om).om_data, (*om).om_len as usize)
    });
    unsafe { r_os_mbuf_free_chain(om.cast_mut()) };
    0
}

fn send_hci_packet(packet: &[u8]) {
    const DATA_TYPE_COMMAND: u8 = 1;
    const DATA_TYPE_ACL: u8 = 2;

    super::dump_packet_info(packet);

    unsafe {
        if packet[0] == DATA_TYPE_COMMAND {
            // IDF VHCI uses the BTDM-common packet path, not NimBLE
            // `r_ble_hci_trans_hs_cmd_tx`.
            let pkt = r_btdm_hci_trans_buf_alloc(HCI_DRIVER_TYPE_CMD, 0);
            if pkt.is_null() {
                warn!("r_btdm_hci_trans_buf_alloc failed");
                return;
            }
            (*pkt).type_ = HCI_DRIVER_TYPE_CMD;
            (*pkt).length = (packet.len() - 1) as u16;
            core::ptr::copy_nonoverlapping(packet.as_ptr().add(1), (*pkt).data, packet.len() - 1);
            let res = r_btdm_hci_trans_rx(pkt);
            if res != 0 {
                warn!("r_btdm_hci_trans_rx returned {}", res);
            }
        } else if packet[0] == DATA_TYPE_ACL {
            let om = r_os_msys_get_pkthdr(packet.len() as u16, ACL_DATA_MBUF_LEADINGSPACE as u16);
            if om.is_null() {
                warn!("r_os_msys_get_pkthdr failed");
                return;
            }
            let res = r_os_mbuf_append(om, packet.as_ptr().add(1), (packet.len() - 1) as u16);
            if res != 0 {
                panic!("r_os_mbuf_append returned {}", res);
            }
            *((*om).om_data as *mut u8).add(1) = 0;
            let res = r_ble_hci_trans_hs_acl_tx(om);
            if res != 0 {
                panic!("r_ble_hci_trans_hs_acl_tx returned {}", res);
            }
        }
    }
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    let phy_init_guard;
    unsafe {
        (*&raw mut HCI_OUT_COLLECTOR).write(HciOutCollector::new());

        chip_specific::btdm_controller_mem_init();

        let mut cfg = chip_specific::create_ble_config(config);

        #[cfg(feature = "coex")]
        {
            let res = crate::wifi::coex_init();
            assert!(res == 0, "coex_init failed");
        }

        chip_specific::bt_periph_module_enable();
        chip_specific::disable_sleep_mode();

        phy_init_guard = esp_phy::enable_phy();

        let res = esp_bt_controller_init(&mut cfg);
        assert!(res == 0, "esp_bt_controller_init returned {}", res);

        #[cfg(feature = "coex")]
        crate::sys::include::coex_enable();

        let res = esp_bt_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BLE);
        assert!(res == 0, "esp_bt_controller_enable returned {}", res);
    }

    #[cfg(rng_trng_supported)]
    unsafe {
        esp_hal::rng::TrngSource::increase_entropy_source_counter()
    };

    phy_init_guard
}

pub(crate) fn ble_deinit() {
    #[cfg(rng_trng_supported)]
    esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
        esp_hal::Internal::conjure()
    });

    let _ = esp_bt_controller_disable();
    let _ = esp_bt_controller_deinit();
}

/// Sends HCI data to the BLE controller.
///
/// Returns the number of bytes taken from `data`. At most one packet is sent per call, so the
/// caller must offer the remaining bytes again.
pub(crate) fn send_hci(data: &[u8]) -> usize {
    super::collect_and_send(data, send_hci_packet)
}

pub(crate) async fn send_hci_async(data: &[u8]) -> usize {
    super::collect_and_send(data, send_hci_packet)
}

// -------------------------------------------------------------------------
// OSAL (wr_btdm_osal_*)
//
// Event queues are local. CompatQueue's blocking receive sleeps inside
// NonReentrantMutex (IRQs off); on S31 the SWI yield is lost and the next
// schedule_wakeup panics because the task is already Sleeping.
// -------------------------------------------------------------------------

const BTDM_OSAL_OK: i32 = 0;
const BTDM_OSAL_INVALID_PARM: i32 = 3;
const BTDM_OSAL_TIMEOUT: i32 = 6;

static EVENTQ_DEPTH: AtomicU16 = AtomicU16::new(16);
static CRITICAL_NEST: AtomicU8 = AtomicU8::new(0);
static CRITICAL_TOKEN: AtomicU32 = AtomicU32::new(0);

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct BtdmOsalPtr {
    pub ptr: *mut c_void,
}

type EventFn = unsafe extern "C" fn(*mut BtdmOsalPtr);

struct Event {
    queued: bool,
    fn_ptr: Option<EventFn>,
    arg: *mut c_void,
}

struct Callout {
    timer: ets_timer,
    evq: *mut BtdmOsalPtr,
    ev: BtdmOsalPtr,
    expiry_ms: u32,
}

fn in_isr() -> bool {
    !crate::hal::interrupt::RunLevel::current().is_thread()
}

#[inline(always)]
fn event_inner(ev: *mut BtdmOsalPtr) -> &'static mut Event {
    let ev = unwrap!(unsafe { ev.as_ref() }, "event is null");
    unwrap!(
        unsafe { ev.ptr.cast::<Event>().as_mut() },
        "event inner is null"
    )
}

fn alloc<T>() -> *mut T {
    unsafe { crate::compat::malloc::calloc(1, size_of::<T>()).cast() }
}

fn free_ptr(ptr: *mut c_void) {
    unsafe { crate::compat::malloc::free(ptr.cast()) }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_eventq_init(evq: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_eventq_init {:?}", evq);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");
    if !evq.ptr.is_null() {
        return;
    }

    evq.ptr = queue_create(
        EVENTQ_DEPTH.load(Ordering::Relaxed) as i32,
        size_of::<*mut BtdmOsalPtr>() as i32,
    );
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_eventq_deinit(evq: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_eventq_deinit {:?}", evq);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");
    queue_delete(evq.ptr)
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_eventq_get(evq: *mut BtdmOsalPtr, tmo: u32) -> *mut BtdmOsalPtr {
    trace!("wr_btdm_osal_eventq_get {:?} {}", evq, tmo);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");

    let mut ev = core::ptr::null_mut::<BtdmOsalPtr>();
    let received = if in_isr() {
        if tmo != 0 {
            return core::ptr::null_mut();
        }
        queue_try_receive_from_isr(evq.ptr, (&raw mut ev).cast(), core::ptr::null_mut())
    } else {
        queue_receive(evq.ptr, (&raw mut ev).cast(), blob_ticks_to_micros(tmo))
    };

    if received != 0 {
        if let Some(inner) = unsafe { (*ev).ptr.cast::<Event>().as_mut() } {
            inner.queued = false;
        }
    }

    ev
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_eventq_put(evq: *mut BtdmOsalPtr, ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_eventq_put {:?} {:?}", evq, ev);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");

    let ret = if in_isr() {
        queue_try_send_to_back_from_isr(evq.ptr, (&raw const ev).cast(), core::ptr::null_mut())
    } else {
        queue_send_to_back(evq.ptr, (&raw const ev).cast(), OSI_FUNCS_TIME_BLOCKING)
    };

    assert_ne!(ret, 0);
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_eventq_put_to_front(evq: *mut BtdmOsalPtr, ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_eventq_put_to_front {:?} {:?}", evq, ev);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");

    let ret = if in_isr() {
        queue_try_send_to_front_from_isr(evq.ptr, (&raw const ev).cast(), core::ptr::null_mut())
    } else {
        queue_send_to_front(evq.ptr, (&raw const ev).cast(), OSI_FUNCS_TIME_BLOCKING)
    };

    assert_ne!(ret, 0);
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_eventq_remove(evq: *mut BtdmOsalPtr, ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_eventq_remove {:?} {:?}", evq, ev);
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");
    let inner = event_inner(ev);

    if inner.queued {
        queue_remove(evq.ptr, (&raw const ev).cast());
        inner.queued = false;
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_eventq_is_empty(evq: *mut BtdmOsalPtr) -> bool {
    let evq = unwrap!(unsafe { evq.as_mut() }, "eventq is null");
    queue_messages_waiting(evq.ptr) == 0
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_run(ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_event_run {:?}", ev);
    if let Some(func) = event_inner(ev).fn_ptr {
        trace!("calling event handler {:#x}", func as usize);
        unsafe { func(ev) };
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_init(
    ev: *mut BtdmOsalPtr,
    func: Option<EventFn>,
    arg: *mut c_void,
) {
    let func_p = func.map(|f| f as usize).unwrap_or(0);
    trace!("creating event with handler {:#x}", func_p);
    let ev = unwrap!(unsafe { ev.as_mut() }, "event is null");
    if ev.ptr.is_null() {
        ev.ptr = alloc::<Event>().cast();
    }
    let inner = unwrap!(
        unsafe { ev.ptr.cast::<Event>().as_mut() },
        "event alloc failed"
    );
    *inner = Event {
        queued: false,
        fn_ptr: func,
        arg,
    };
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_deinit(ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_event_deinit {:?}", ev);
    let ev = unwrap!(unsafe { ev.as_mut() }, "event is null");
    if !ev.ptr.is_null() {
        free_ptr(ev.ptr);
        ev.ptr = core::ptr::null_mut();
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_reset(ev: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_event_reset {:?}", ev);
    let ev = unwrap!(unsafe { ev.as_ref() }, "event is null");
    if let Some(inner) = unsafe { ev.ptr.cast::<Event>().as_mut() } {
        inner.queued = false;
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_is_queued(ev: *mut BtdmOsalPtr) -> bool {
    trace!("wr_btdm_osal_event_is_queued {:?}", ev);
    event_inner(ev).queued
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_get_arg(ev: *mut BtdmOsalPtr) -> *mut c_void {
    trace!("wr_btdm_osal_event_get_arg {:?}", ev);
    event_inner(ev).arg
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_event_set_arg(ev: *mut BtdmOsalPtr, arg: *mut c_void) {
    trace!("wr_btdm_osal_event_set_arg {:?} {:?}", ev, arg);
    event_inner(ev).arg = arg;
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_mutex_init(mu: *mut BtdmOsalPtr) -> i32 {
    trace!("wr_btdm_osal_mutex_init {:?}", mu);
    let mu = unwrap!(unsafe { mu.as_mut() }, "mutex is null");
    if mu.ptr.is_null() {
        mu.ptr = mutex::mutex_create(true);
    }
    if mu.ptr.is_null() {
        BTDM_OSAL_INVALID_PARM
    } else {
        BTDM_OSAL_OK
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_mutex_deinit(mu: *mut BtdmOsalPtr) -> i32 {
    trace!("wr_btdm_osal_mutex_deinit {:?}", mu);
    let mu = unwrap!(unsafe { mu.as_mut() }, "mutex is null");
    if mu.ptr.is_null() {
        return BTDM_OSAL_INVALID_PARM;
    }
    mutex::mutex_delete(mu.ptr);
    mu.ptr = core::ptr::null_mut();
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_mutex_pend(mu: *mut BtdmOsalPtr, timeout: u32) -> i32 {
    trace!("wr_btdm_osal_mutex_pend {:?} {}", mu, timeout);
    let mu = unwrap!(unsafe { mu.as_ref() }, "mutex is null");
    if mu.ptr.is_null() || in_isr() {
        return BTDM_OSAL_INVALID_PARM;
    }

    if mutex::mutex_lock_with_timeout(mu.ptr, blob_ticks_to_micros(timeout)) != 0 {
        BTDM_OSAL_OK
    } else {
        BTDM_OSAL_TIMEOUT
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_mutex_release(mu: *mut BtdmOsalPtr) -> i32 {
    trace!("wr_btdm_osal_mutex_release {:?}", mu);
    let mu = unwrap!(unsafe { mu.as_ref() }, "mutex is null");
    if mu.ptr.is_null() || in_isr() {
        return BTDM_OSAL_INVALID_PARM;
    }
    if mutex::mutex_unlock(mu.ptr) != 0 {
        BTDM_OSAL_OK
    } else {
        BTDM_OSAL_TIMEOUT
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_sem_init(sem: *mut BtdmOsalPtr, tokens: u16) -> i32 {
    trace!("wr_btdm_osal_sem_init {:?} {:?}", sem, tokens);
    let sem = unwrap!(unsafe { sem.as_mut() }, "sem is null");
    if sem.ptr.is_null() {
        sem.ptr = semaphore::sem_create(128, tokens as u32);
    }
    if sem.ptr.is_null() {
        BTDM_OSAL_INVALID_PARM
    } else {
        BTDM_OSAL_OK
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_sem_deinit(sem: *mut BtdmOsalPtr) -> i32 {
    trace!("wr_btdm_osal_sem_deinit {:?}", sem);
    let sem = unwrap!(unsafe { sem.as_mut() }, "sem is null");
    if sem.ptr.is_null() {
        return BTDM_OSAL_INVALID_PARM;
    }
    semaphore::sem_delete(sem.ptr);
    sem.ptr = core::ptr::null_mut();
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_sem_pend(sem: *mut BtdmOsalPtr, timeout: u32) -> i32 {
    trace!("wr_btdm_osal_sem_pend {:?} {}", sem, timeout);
    let sem = unwrap!(unsafe { sem.as_ref() }, "sem is null");
    if sem.ptr.is_null() {
        return BTDM_OSAL_INVALID_PARM;
    }

    let taken = if in_isr() {
        if timeout != 0 {
            return BTDM_OSAL_INVALID_PARM;
        }

        semaphore::sem_try_take_from_isr(sem.ptr, core::ptr::null_mut())
    } else {
        semaphore::sem_take(sem.ptr, blob_ticks_to_micros(timeout))
    };
    if taken != 0 {
        BTDM_OSAL_OK
    } else {
        BTDM_OSAL_TIMEOUT
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_sem_release(sem: *mut BtdmOsalPtr) -> i32 {
    trace!("wr_btdm_osal_sem_release {:?}", sem);
    let sem = unwrap!(unsafe { sem.as_ref() }, "sem is null");
    if sem.ptr.is_null() {
        return BTDM_OSAL_INVALID_PARM;
    }
    let given = if in_isr() {
        semaphore::sem_try_give_from_isr(sem.ptr, core::ptr::null_mut())
    } else {
        semaphore::sem_give(sem.ptr)
    };
    if given != 0 {
        BTDM_OSAL_OK
    } else {
        BTDM_OSAL_TIMEOUT
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_sem_get_count(sem: *mut BtdmOsalPtr) -> u16 {
    trace!("wr_btdm_osal_sem_get_count {:?}", sem);
    let Some(sem) = (unsafe { sem.as_ref() }) else {
        return 0;
    };
    if sem.ptr.is_null() {
        return 0;
    }
    semaphore::sem_count(sem.ptr) as u16
}

#[inline(always)]
fn callout_inner(co: *mut BtdmOsalPtr) -> &'static mut Callout {
    let co = unwrap!(unsafe { co.as_ref() }, "callout is null");
    unwrap!(
        unsafe { co.ptr.cast::<Callout>().as_mut() },
        "callout inner is null"
    )
}

unsafe extern "C" fn callout_timer_cb(arg: *mut c_void) {
    trace!("callout_timer_cb {:#x}", arg as usize);
    let callout = arg.cast::<Callout>();
    let callout = unwrap!(unsafe { callout.as_mut() }, "callout is null");
    if !callout.evq.is_null() {
        wr_btdm_osal_eventq_put(callout.evq, &raw mut callout.ev);
    } else {
        wr_btdm_osal_event_run(&raw mut callout.ev);
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_callout_init(
    co: *mut BtdmOsalPtr,
    evq: *mut BtdmOsalPtr,
    ev_cb: Option<EventFn>,
    ev_arg: *mut c_void,
) -> i32 {
    trace!(
        "wr_btdm_osal_callout_init co={:#x} evq={:#x}",
        co as usize, evq as usize
    );
    let co = unwrap!(unsafe { co.as_mut() }, "callout is null");
    if co.ptr.is_null() {
        let callout = alloc::<Callout>();
        if callout.is_null() {
            return -1;
        }
        unsafe {
            (*callout).ev.ptr = core::ptr::null_mut();
            compat::timer_compat::compat_timer_setfn(
                &raw mut (*callout).timer,
                callout_timer_cb,
                callout.cast(),
            );
        }
        co.ptr = callout.cast();
    }

    let callout = co.ptr.cast::<Callout>();
    unsafe {
        (*callout).evq = evq;
        wr_btdm_osal_event_init(&raw mut (*callout).ev, ev_cb, ev_arg);
    }

    0
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_callout_deinit(co: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_callout_deinit co={:#x}", co as usize);
    let co = unwrap!(unsafe { co.as_mut() }, "callout is null");
    if co.ptr.is_null() {
        return;
    }
    let callout = co.ptr.cast::<Callout>();
    let timer = unsafe { &raw mut (*callout).timer };
    unsafe {
        compat::timer_compat::compat_timer_disarm(timer);
        compat::timer_compat::compat_timer_done(timer);
        wr_btdm_osal_event_deinit(&raw mut (*callout).ev);
        free_ptr(callout.cast());
    }
    co.ptr = core::ptr::null_mut();
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_callout_reset(co: *mut BtdmOsalPtr, ticks: u32) -> i32 {
    trace!(
        "wr_btdm_osal_callout_reset co={:#x} ticks={}",
        co as usize, ticks
    );
    let callout = callout_inner(co);
    compat::timer_compat::compat_timer_disarm(&raw mut callout.timer);
    if !callout.evq.is_null() {
        wr_btdm_osal_eventq_remove(callout.evq, &raw mut callout.ev);
    }
    let millis = blob_ticks_to_millis(ticks.max(1));
    callout.expiry_ms = wr_btdm_osal_time_get().wrapping_add(millis);
    compat::timer_compat::compat_timer_arm(&raw mut callout.timer, millis, false);
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_callout_mem_reset(co: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_callout_mem_reset co={:#x}", co as usize);
    let callout = callout_inner(co);
    wr_btdm_osal_event_reset(&raw mut callout.ev);
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_callout_stop(co: *mut BtdmOsalPtr) {
    trace!("wr_btdm_osal_callout_stop co={:#x}", co as usize);
    let Some(co) = (unsafe { co.as_ref() }) else {
        return;
    };
    let Some(callout) = (unsafe { co.ptr.cast::<Callout>().as_mut() }) else {
        return;
    };
    compat::timer_compat::compat_timer_disarm(&raw mut callout.timer);
    if !callout.evq.is_null() {
        wr_btdm_osal_eventq_remove(callout.evq, &raw mut callout.ev);
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_callout_is_active(co: *mut BtdmOsalPtr) -> bool {
    trace!("wr_btdm_osal_callout_is_active co={:#x}", co as usize);
    let callout = callout_inner(co);
    compat::timer_compat::compat_timer_is_active(&raw mut callout.timer)
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_callout_get_ticks(co: *mut BtdmOsalPtr) -> u32 {
    trace!("wr_btdm_osal_callout_get_ticks co={:#x}", co as usize);
    let callout = callout_inner(co);
    callout.expiry_ms
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_callout_remaining_ticks(co: *mut BtdmOsalPtr, now: u32) -> u32 {
    trace!(
        "wr_btdm_osal_callout_remaining_ticks co={:#x} now={}",
        co as usize, now
    );
    let callout = callout_inner(co);
    if !compat::timer_compat::compat_timer_is_active(&raw mut callout.timer) {
        return 0;
    }
    callout.expiry_ms.saturating_sub(now)
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_callout_set_arg(co: *mut BtdmOsalPtr, arg: *mut c_void) {
    trace!(
        "wr_btdm_osal_callout_set_arg co={:#x} arg={:#x}",
        co as usize, arg as usize
    );
    let callout = callout_inner(co);
    wr_btdm_osal_event_set_arg(&raw mut callout.ev, arg);
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_time_get() -> u32 {
    crate::hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis() as u32
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_time_ms_to_ticks(ms: u32, out_ticks: *mut u32) -> i32 {
    unsafe { *out_ticks = millis_to_blob_ticks(ms) };
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_time_ticks_to_ms(ticks: u32, out_ms: *mut u32) -> i32 {
    unsafe { *out_ms = blob_ticks_to_millis(ticks) };
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_time_ms_to_ticks32(ms: u32) -> u32 {
    millis_to_blob_ticks(ms)
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_time_ticks_to_ms32(ticks: u32) -> u32 {
    blob_ticks_to_millis(ticks)
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_get_time_forever() -> u32 {
    OSI_FUNCS_TIME_BLOCKING
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_hw_enter_critical() -> u32 {
    // We count calls because in at least one place, the blob doesn't pass the returned number back
    // to `exit_critical`.
    let last = CRITICAL_NEST.fetch_add(1, Ordering::Release);
    if last == 0 {
        let token = unsafe { crate::ble::ESP_RADIO_LOCK.acquire().inner() };
        CRITICAL_TOKEN.store(token, Ordering::Relaxed);
    }
    0
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_hw_exit_critical(_ctx: u32) {
    if CRITICAL_NEST.fetch_sub(1, Ordering::Release) == 1 {
        let last = CRITICAL_TOKEN.load(Ordering::Relaxed);

        unsafe {
            crate::ble::ESP_RADIO_LOCK.release(esp_sync::RestoreState::new(last));
        }
    }
}

#[unsafe(no_mangle)]
#[ram]
extern "C" fn wr_btdm_osal_hw_is_in_critical() -> u8 {
    CRITICAL_NEST.load(Ordering::Acquire)
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_task_create(
    fn_ptr: *mut c_void,
    name: *const c_char,
    stack_size: u32,
    arg: *mut c_void,
    priority: u32,
    task_handle: *mut *mut c_void,
    core_id: u32,
) -> i32 {
    let name_str = unsafe { str_from_c(name) };
    unsafe {
        let task_func = transmute::<*mut c_void, extern "C" fn(*mut c_void)>(fn_ptr);
        // Sleeping waits (usleep) let embassy main run even at the blob
        // priority. Ready yield-loops at prio 29 starve main.
        let task = crate::preempt::task_create(
            name_str,
            task_func,
            arg,
            priority.min(crate::preempt::max_task_priority()),
            if core_id < Cpu::COUNT as u32 {
                Some(core_id)
            } else {
                None
            },
            stack_size as usize,
        );
        if !task_handle.is_null() {
            *task_handle = task.as_ptr().cast();
        }
    }
    BTDM_OSAL_OK
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_task_delete(task_handle: *mut c_void) {
    unsafe {
        crate::preempt::schedule_task_deletion(NonNull::new(task_handle.cast::<()>()));
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_intr_alloc(
    src: i32,
    _flags: i32,
    fn_ptr: Option<unsafe extern "C" fn(*mut c_void)>,
    arg: *mut c_void,
    _handle: *mut *mut c_void,
) -> i32 {
    let Some(func) = fn_ptr else {
        return BTDM_OSAL_INVALID_PARM;
    };
    unsafe { chip_specific::osal_intr_alloc(src as u32, func, arg) }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_intr_free(_handle: *mut c_void) -> i32 {
    0
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_malloc(size: u32, _flags: u32) -> *mut c_void {
    unsafe { crate::compat::malloc::malloc(size as usize).cast() }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_free(ptr: *mut c_void) {
    if !ptr.is_null() {
        unsafe { crate::compat::malloc::free(ptr.cast()) };
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_mmgmt_block_malloc(size: u32) -> *mut c_void {
    let raw = wr_btdm_osal_malloc(size.saturating_add(4), 0) as *mut u32;
    if raw.is_null() {
        return core::ptr::null_mut();
    }
    unsafe {
        *raw = (5 << 29) | ((size + 4) >> 2);
        raw.add(1).cast()
    }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_mmgmt_block_free(ptr: *mut c_void) {
    if ptr.is_null() {
        return;
    }
    wr_btdm_osal_free(unsafe { (ptr as *mut u32).sub(1) }.cast());
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_mmgmt_block_copy(dst: *mut c_void, src: *const c_void, size: u16) {
    unsafe extern "C" {
        fn r_ble_lll_mmgmt_block_copy(dst: *mut c_void, src: *mut c_void, size: u16);
    }
    unsafe { r_ble_lll_mmgmt_block_copy(dst, src.cast_mut(), size) };
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_read_efuse_mac(mac: *mut u8) -> i32 {
    unsafe { crate::common_adapter::read_mac(mac, 2) }
}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_srand(_seed: u32) {}

#[unsafe(no_mangle)]
extern "C" fn wr_btdm_osal_rand() -> i32 {
    unsafe { crate::common_adapter::random() as i32 }
}
