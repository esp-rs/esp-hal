use alloc::vec::Vec;
use core::{mem::transmute, ptr::NonNull};

use esp_phy::PhyInitGuard;

use super::{Config, ReceivedPacket, in_isr};
use crate::{
    compat::{self, OSI_FUNCS_TIME_BLOCKING, common::str_from_c, queue},
    hal::time::Instant,
    sys::{c_types::*, include::*},
    time::{blob_ticks_to_micros, blob_ticks_to_millis, millis_to_blob_ticks},
};

#[cfg_attr(esp32c2, path = "os_adapter_esp32c2.rs")]
#[cfg_attr(esp32c5, path = "os_adapter_esp32c5.rs")]
#[cfg_attr(esp32c6, path = "os_adapter_esp32c6.rs")]
#[cfg_attr(esp32c61, path = "os_adapter_esp32c61.rs")]
#[cfg_attr(esp32h2, path = "os_adapter_esp32h2.rs")]
pub(crate) mod chip_specific;
mod os_mempool;

const EVENT_QUEUE_SIZE: usize = 16;

const BLE_HCI_TRANS_BUF_CMD: i32 = 3;

// ACL_DATA_MBUF_LEADINGSPCAE: The leadingspace in user info header for ACL data
const ACL_DATA_MBUF_LEADINGSPACE: usize = 4;

#[repr(C)]
#[derive(Copy, Clone)]
struct Callout {
    eventq: *const ble_npl_eventq,
    timer_handle: ets_timer,
    events: ble_npl_event,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct Event {
    event_fn_ptr: *const ble_npl_event_fn,
    ev_arg_ptr: *const c_void,
    queued: bool,
}

/// Memory pool
#[repr(C)]
pub(crate) struct OsMempool {
    /// Size of the memory blocks, in bytes.
    pub(crate) mp_block_size: u32,
    /// The number of memory blocks.
    pub(crate) mp_num_blocks: u16,
    /// The number of free blocks left
    pub(crate) mp_num_free: u16,
    /// The lowest number of free blocks seen
    pub(crate) mp_min_free: u16,
    /// Bitmap of OS_MEMPOOL_F_[...] values.
    pub(crate) mp_flags: u8,
    /// Address of memory buffer used by pool
    pub(crate) mp_membuf_addr: u32,

    // STAILQ_ENTRY(os_mempool) mp_list;
    pub(crate) next: *const OsMempool,

    // SLIST_HEAD(,os_memblock);
    pub(crate) first: *const c_void,

    /// Name for memory block
    pub(crate) name: *const u8,
}

/// A mbuf pool from which to allocate mbufs. This contains a pointer to the os
/// mempool to allocate mbufs out of, the total number of elements in the pool,
/// and the amount of "user" data in a non-packet header mbuf. The total pool
/// size, in bytes, should be:
///  os_mbuf_count * (omp_databuf_len + sizeof(struct os_mbuf))
#[repr(C)]
pub(crate) struct OsMbufPool {
    /// Total length of the databuf in each mbuf.  This is the size of the
    /// mempool block, minus the mbuf header
    omp_databuf_len: u16,
    /// The memory pool which to allocate mbufs out of
    omp_pool: *const OsMempool,

    // STAILQ_ENTRY(os_mbuf_pool) omp_next;
    next: *const OsMbufPool,
}

/// Chained memory buffer.
#[repr(C)]
pub(crate) struct OsMbuf {
    /// Current pointer to data in the structure
    om_data: *const u8,
    /// Flags associated with this buffer, see OS_MBUF_F_* definitions
    om_flags: u8,
    /// Length of packet header
    om_pkthdr_len: u8,
    /// Length of data in this buffer
    om_len: u16,

    /// The mbuf pool this mbuf was allocated out of
    om_omp: *const OsMbufPool,

    // SLIST_ENTRY(os_mbuf) om_next;
    next: *const OsMbuf,

    /// Pointer to the beginning of the data, after this buffer
    om_databuf: u32,
}

unsafe extern "C" {
    cfg_select! {
        esp32c2 => {
            fn ble_controller_init(cfg: *const esp_bt_controller_config_t) -> i32;
            fn ble_controller_deinit() -> i32;
            fn ble_controller_enable(mode: u8) -> i32;
            fn ble_controller_disable();
            fn ble_get_npl_element_info(
                cfg: *const esp_bt_controller_config_t,
                npl_info: *const BleNplCountInfoT,
            ) -> i32;
            fn esp_ble_ll_set_public_addr(addr: *const u8);
        }
        _ => {
            fn esp_ble_register_bb_funcs() -> i32;
            fn r_ble_controller_disable() -> i32;
            fn r_ble_controller_deinit() -> i32;
            fn r_ble_controller_init(cfg: *const esp_bt_controller_config_t) -> i32;
            fn r_ble_controller_enable(mode: u8) -> i32;
            fn r_ble_get_npl_element_info(
                cfg: *const esp_bt_controller_config_t,
                npl_info: *const BleNplCountInfoT,
            ) -> i32;
            fn r_esp_ble_ll_set_public_addr(addr: *const u8);
            fn scan_stack_initEnv() -> i32;
            fn scan_stack_deinitEnv();

            fn r_esp_ble_msys_init(
                msys_size1: u16,
                msys_size2: u16,
                msys_cnt1: u16,
                msys_cnt2: u16,
                from_heap: u8,
            ) -> i32;

            fn base_stack_initEnv() -> i32;
            fn conn_stack_initEnv() -> i32;
            fn adv_stack_initEnv() -> i32;
            fn extAdv_stack_initEnv() -> i32;
            fn sync_stack_initEnv() -> i32;

            fn base_stack_enable() -> i32;
            fn adv_stack_enable() -> i32;
            fn extAdv_stack_enable() -> i32;
            fn scan_stack_enable() -> i32;
            fn sync_stack_enable() -> i32;

            fn base_stack_deinitEnv() -> i32;
            fn conn_stack_deinitEnv() -> i32;
            fn adv_stack_deinitEnv() -> i32;
            fn extAdv_stack_deinitEnv() -> i32;
            fn sync_stack_deinitEnv() -> i32;

            fn base_stack_disable() -> i32;
            fn adv_stack_disable() -> i32;
            fn extAdv_stack_disable() -> i32;
            fn scan_stack_disable() -> i32;
            fn sync_stack_disable() -> i32;
        }
    }

    // Sends ACL data from host to controller.
    //
    // om                    The ACL data packet to send.
    //
    // 0 on success;
    // A BLE_ERR_[...] error code on failure.
    fn r_ble_hci_trans_hs_acl_tx(om: *const OsMbuf) -> i32;

    // Sends an HCI command from the host to the controller.
    //
    // cmd                   The HCI command to send.  This buffer must be
    //                                  allocated via ble_hci_trans_buf_alloc().
    //
    // 0 on success;
    // A BLE_ERR_[...] error code on failure.
    fn r_ble_hci_trans_hs_cmd_tx(cmd: *const u8) -> i32;
    fn esp_unregister_ext_funcs();
    fn esp_register_ext_funcs(funcs: *const ExtFuncsT) -> i32;
    fn esp_register_npl_funcs(funcs: *const npl_funcs_t) -> i32;
    fn esp_unregister_npl_funcs();

    fn bt_bb_v2_init_cmplx(value: u8);

    fn r_ble_hci_trans_cfg_hs(
        // ble_hci_trans_rx_cmd_fn
        evt: Option<unsafe extern "C" fn(cmd: *const u8, arg: *const c_void) -> i32>,
        evt_arg: *const c_void,
        // ble_hci_trans_rx_acl_fn
        acl_cb: Option<unsafe extern "C" fn(om: *const OsMbuf, arg: *const c_void) -> i32>,
        acl_arg: *const c_void,
    );

    #[cfg(feature = "coex")]
    fn ble_osi_coex_funcs_register(coex_funcs: *const OsiCoexFuncsT) -> i32;

    fn r_os_msys_get_pkthdr(dsize: u16, user_hdr_len: u16) -> *mut OsMbuf;
    fn r_os_mbuf_append(om: *mut OsMbuf, src: *const u8, len: u16) -> i32;
    fn r_os_mbuf_free_chain(om: *mut OsMbuf) -> i32;
    fn r_ble_hci_trans_init(m: u8);
    fn r_ble_hci_trans_buf_alloc(typ: i32) -> *const u8;
    fn r_ble_hci_trans_buf_free(buf: *const u8);
    fn coex_pti_v2();
}

#[cfg(not(esp32c2))]
fn ble_controller_init(cfg: *const esp_bt_controller_config_t) -> i32 {
    unsafe {
        let res = esp_ble_register_bb_funcs();
        assert!(res == 0, "esp_ble_register_bb_funcs returned {}", res);

        let res = r_ble_controller_init(cfg);
        assert!(res == 0, "ble_controller_init returned {}", res);

        let res = base_stack_initEnv();
        assert!(res == 0, "base_stack_initEnv returned {}", res);

        let res = adv_stack_initEnv();
        assert!(res == 0, "adv_stack_initEnv returned {}", res);

        let res = extAdv_stack_initEnv();
        assert!(res == 0, "extAdv_stack_initEnv returned {}", res);

        let res = scan_stack_initEnv();
        assert!(res == 0, "scan_stack_initEnv returned {}", res);

        let res = conn_stack_initEnv();
        assert!(res == 0, "conn_stack_initEnv returned {}", res);

        let res = sync_stack_initEnv();
        assert!(res == 0, "sync_stack_initEnv returned {}", res);

        let res = r_esp_ble_msys_init(256, 320, 12, 24, 1);
        assert!(res == 0, "esp_ble_msys_init returned {}", res);

        let res = base_stack_enable();
        assert!(res == 0, "base_stack_enable returned {}", res);

        let res = adv_stack_enable();
        assert!(res == 0, "adv_stack_enable returned {}", res);

        let res = extAdv_stack_enable();
        assert!(res == 0, "extAdv_stack_enable returned {}", res);

        let res = scan_stack_enable();
        assert!(res == 0, "scan_stack_enable returned {}", res);

        let res = sync_stack_enable();
        assert!(res == 0, "sync_stack_enable returned {}", res);
    }
    0
}

#[cfg(not(esp32c2))]
fn ble_controller_deinit() -> i32 {
    unsafe {
        sync_stack_disable();
        scan_stack_disable();
        extAdv_stack_disable();
        adv_stack_disable();
        base_stack_disable();
        conn_stack_deinitEnv();
        sync_stack_deinitEnv();
        scan_stack_deinitEnv();
        extAdv_stack_deinitEnv();
        adv_stack_deinitEnv();
        base_stack_deinitEnv();
        r_ble_controller_deinit()
    }
}

#[repr(C)]
/// Contains pointers to external functions used by the BLE stack.
pub(crate) struct ExtFuncsT {
    ext_version: u32,
    esp_intr_alloc: Option<
        unsafe extern "C" fn(
            source: u32,
            flags: u32,
            handler: *mut c_void,
            arg: *mut c_void,
            ret_handle: *mut *mut c_void,
        ) -> i32,
    >,
    esp_intr_free: Option<unsafe extern "C" fn(ret_handle: *mut *mut c_void) -> i32>,
    malloc: Option<unsafe extern "C" fn(size: u32) -> *mut c_void>,
    free: Option<unsafe extern "C" fn(*mut c_void)>,
    #[cfg(esp32c2)]
    hal_uart_start_tx: Option<unsafe extern "C" fn(i32)>,
    #[cfg(esp32c2)]
    hal_uart_init_cbs: Option<
        unsafe extern "C" fn(i32, *const c_void, *const c_void, *const c_void, c_void) -> i32,
    >,
    #[cfg(esp32c2)]
    hal_uart_config: Option<unsafe extern "C" fn(i32, i32, u8, u8, u8, u8) -> i32>,
    #[cfg(esp32c2)]
    hal_uart_close: Option<unsafe extern "C" fn(i32) -> i32>,
    #[cfg(esp32c2)]
    hal_uart_blocking_tx: Option<unsafe extern "C" fn(i32, u8)>,
    #[cfg(esp32c2)]
    hal_uart_init: Option<unsafe extern "C" fn(i32, *const c_void) -> i32>,
    task_create: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *const c_char,
            u32,
            *mut c_void,
            u32,
            *const c_void,
            u32,
        ) -> i32,
    >,
    task_delete: Option<unsafe extern "C" fn(*mut c_void)>,
    osi_assert: Option<unsafe extern "C" fn(u32, *const c_void, u32, u32)>,
    os_random: Option<unsafe extern "C" fn() -> u32>,
    ecc_gen_key_pair: Option<unsafe extern "C" fn(*const u8, *const u8) -> i32>,
    ecc_gen_dh_key: Option<unsafe extern "C" fn(*const u8, *const u8, *const u8, *const u8) -> i32>,
    #[cfg(any(esp32c6, esp32h2))]
    esp_reset_modem: Option<unsafe extern "C" fn(mdl_opts: u8, start: u8)>,
    #[cfg(esp32c2)]
    esp_reset_rpa_moudle: Option<unsafe extern "C" fn()>,
    #[cfg(esp32c2)]
    esp_bt_track_pll_cap: Option<unsafe extern "C" fn()>,
    magic: u32,
}

static G_OSI_FUNCS: ExtFuncsT = ExtFuncsT {
    ext_version: if cfg!(esp32c2) {
        0x20221122
    } else {
        0x20250825
    },

    esp_intr_alloc: Some(chip_specific::esp_intr_alloc),
    esp_intr_free: Some(esp_intr_free),
    malloc: Some(crate::ble::malloc),
    free: Some(crate::ble::free),
    #[cfg(esp32c2)]
    hal_uart_start_tx: None,
    #[cfg(esp32c2)]
    hal_uart_init_cbs: None,
    #[cfg(esp32c2)]
    hal_uart_config: None,
    #[cfg(esp32c2)]
    hal_uart_close: None,
    #[cfg(esp32c2)]
    hal_uart_blocking_tx: None,
    #[cfg(esp32c2)]
    hal_uart_init: None,
    task_create: Some(task_create),
    task_delete: Some(task_delete),
    osi_assert: Some(osi_assert),
    os_random: Some(os_random),
    ecc_gen_key_pair: Some(ecc_gen_key_pair),
    ecc_gen_dh_key: Some(ecc_gen_dh_key),
    #[cfg(any(esp32c6, esp32h2))]
    esp_reset_modem: Some(chip_specific::reset_modem),
    #[cfg(esp32c2)]
    esp_reset_rpa_moudle: Some(chip_specific::esp_reset_rpa_moudle),
    #[cfg(esp32c2)]
    esp_bt_track_pll_cap: None,
    magic: 0xA5A5A5A5,
};

unsafe extern "C" fn ecc_gen_dh_key(_: *const u8, _: *const u8, _: *const u8, _: *const u8) -> i32 {
    todo!()
}

unsafe extern "C" fn ecc_gen_key_pair(_: *const u8, _: *const u8) -> i32 {
    todo!()
}

unsafe extern "C" fn os_random() -> u32 {
    trace!("os_random");
    unsafe { crate::common_adapter::random() as u32 }
}

unsafe extern "C" fn task_create(
    task_func: *mut c_void,
    name: *const c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *const c_void,
    core_id: u32,
) -> i32 {
    let name_str = unsafe { str_from_c(name) };
    trace!(
        "task_create {:?} {} {} {:?} {} {:?} {}",
        task_func, name_str, stack_depth, param, prio, task_handle, core_id,
    );

    unsafe {
        let task_func = transmute::<*mut c_void, extern "C" fn(*mut c_void)>(task_func);

        let task = crate::preempt::task_create(
            name_str,
            task_func,
            param,
            prio,
            if core_id < 2 { Some(core_id) } else { None },
            stack_depth as usize,
        );
        *(task_handle as *mut usize) = task.as_ptr() as usize;
    }

    1
}

unsafe extern "C" fn task_delete(task: *mut c_void) {
    trace!("task delete called for {:?}", task);

    unsafe {
        crate::preempt::schedule_task_deletion(NonNull::new(task.cast::<()>()));
    }
}

unsafe extern "C" fn osi_assert(ln: u32, fn_name: *const c_void, param1: u32, param2: u32) {
    let name_str = unsafe { str_from_c(fn_name as _) };
    panic!("ASSERT {}:{} {} {}", name_str, ln, param1, param2);
}

unsafe extern "C" fn esp_intr_free(_ret_handle: *mut *mut c_void) -> i32 {
    todo!();
}

#[repr(C)]
#[allow(non_camel_case_types)]
/// Contains pointers to functions used by the BLE NPL (Non-Preemptive Layer).
pub(crate) struct npl_funcs_t {
    p_ble_npl_os_started: Option<unsafe extern "C" fn() -> bool>,
    p_ble_npl_get_current_task_id: Option<unsafe extern "C" fn() -> *const c_void>,
    p_ble_npl_eventq_init: Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq)>,
    p_ble_npl_eventq_deinit: Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq)>,
    p_ble_npl_eventq_get: Option<
        unsafe extern "C" fn(
            queue: *mut ble_npl_eventq,
            time: ble_npl_time_t,
        ) -> *const ble_npl_event,
    >,
    p_ble_npl_eventq_put:
        Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq, event: *const ble_npl_event)>,
    p_ble_npl_eventq_remove:
        Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq, event: *const ble_npl_event)>,
    p_ble_npl_event_run: Option<unsafe extern "C" fn(event: *const ble_npl_event)>,
    p_ble_npl_eventq_is_empty: Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq) -> bool>,
    p_ble_npl_event_init: Option<
        unsafe extern "C" fn(
            event: *const ble_npl_event,
            func: *const ble_npl_event_fn,
            *const c_void,
        ),
    >,
    p_ble_npl_event_deinit: Option<unsafe extern "C" fn(event: *const ble_npl_event)>,
    p_ble_npl_event_reset: Option<unsafe extern "C" fn(event: *const ble_npl_event)>,
    p_ble_npl_event_is_queued: Option<unsafe extern "C" fn(event: *const ble_npl_event) -> bool>,
    p_ble_npl_event_get_arg:
        Option<unsafe extern "C" fn(event: *const ble_npl_event) -> *const c_void>,
    p_ble_npl_event_set_arg:
        Option<unsafe extern "C" fn(event: *const ble_npl_event, arg: *const c_void)>,
    p_ble_npl_mutex_init:
        Option<unsafe extern "C" fn(mutex: *const ble_npl_mutex) -> ble_npl_error_t>,
    p_ble_npl_mutex_deinit:
        Option<unsafe extern "C" fn(mutex: *const ble_npl_mutex) -> ble_npl_error_t>,
    p_ble_npl_mutex_pend: Option<
        unsafe extern "C" fn(mutex: *const ble_npl_mutex, time: ble_npl_time_t) -> ble_npl_error_t,
    >,
    p_ble_npl_mutex_release:
        Option<unsafe extern "C" fn(mutex: *const ble_npl_mutex) -> ble_npl_error_t>,
    p_ble_npl_sem_init:
        Option<unsafe extern "C" fn(sem: *const ble_npl_sem, val: u16) -> ble_npl_error_t>,
    p_ble_npl_sem_deinit: Option<unsafe extern "C" fn(sem: *const ble_npl_sem) -> ble_npl_error_t>,
    p_ble_npl_sem_pend: Option<
        unsafe extern "C" fn(sem: *const ble_npl_sem, time: ble_npl_time_t) -> ble_npl_error_t,
    >,
    p_ble_npl_sem_release: Option<unsafe extern "C" fn(sem: *const ble_npl_sem) -> ble_npl_error_t>,
    p_ble_npl_sem_get_count: Option<unsafe extern "C" fn(sem: *const ble_npl_sem) -> u16>,
    p_ble_npl_callout_init: Option<
        unsafe extern "C" fn(
            callout: *const ble_npl_callout,
            eventq: *const ble_npl_eventq,
            func: *const ble_npl_event_fn,
            args: *const c_void,
        ) -> i32,
    >,
    p_ble_npl_callout_reset: Option<
        unsafe extern "C" fn(
            callout: *const ble_npl_callout,
            time: ble_npl_time_t,
        ) -> ble_npl_error_t,
    >,
    p_ble_npl_callout_stop: Option<unsafe extern "C" fn(callout: *const ble_npl_callout)>,
    p_ble_npl_callout_deinit: Option<unsafe extern "C" fn(callout: *const ble_npl_callout)>,
    p_ble_npl_callout_mem_reset: Option<unsafe extern "C" fn(callout: *const ble_npl_callout)>,
    p_ble_npl_callout_is_active:
        Option<unsafe extern "C" fn(callout: *const ble_npl_callout) -> bool>,
    p_ble_npl_callout_get_ticks:
        Option<unsafe extern "C" fn(callout: *const ble_npl_callout) -> ble_npl_time_t>,
    p_ble_npl_callout_remaining_ticks:
        Option<unsafe extern "C" fn(callout: *const ble_npl_callout, time: ble_npl_time_t) -> u32>,
    p_ble_npl_callout_set_arg:
        Option<unsafe extern "C" fn(callout: *const ble_npl_callout, arg: *const c_void)>,
    p_ble_npl_time_get: Option<unsafe extern "C" fn() -> u32>,
    p_ble_npl_time_ms_to_ticks:
        Option<unsafe extern "C" fn(ms: u32, p_time: *mut ble_npl_time_t) -> ble_npl_error_t>,
    p_ble_npl_time_ticks_to_ms:
        Option<unsafe extern "C" fn(time: ble_npl_time_t, *mut u32) -> ble_npl_error_t>,
    p_ble_npl_time_ms_to_ticks32: Option<unsafe extern "C" fn(ms: u32) -> ble_npl_time_t>,
    p_ble_npl_time_ticks_to_ms32: Option<unsafe extern "C" fn(time: ble_npl_time_t) -> u32>,
    p_ble_npl_time_delay: Option<unsafe extern "C" fn(time: ble_npl_time_t)>,
    p_ble_npl_hw_set_isr: Option<unsafe extern "C" fn(no: i32, mask: u32)>,
    p_ble_npl_hw_enter_critical: Option<unsafe extern "C" fn() -> u32>,
    p_ble_npl_hw_exit_critical: Option<unsafe extern "C" fn(mask: u32)>,
    p_ble_npl_get_time_forever: Option<unsafe extern "C" fn() -> u32>,
    p_ble_npl_hw_is_in_critical: Option<unsafe extern "C" fn() -> u8>,
    p_ble_npl_eventq_put_to_front:
        Option<unsafe extern "C" fn(queue: *mut ble_npl_eventq, event: *const ble_npl_event)>,
}

static G_NPL_FUNCS: npl_funcs_t = npl_funcs_t {
    p_ble_npl_os_started: Some(ble_npl_os_started),
    p_ble_npl_get_current_task_id: Some(ble_npl_get_current_task_id),
    p_ble_npl_eventq_init: Some(ble_npl_eventq_init),
    p_ble_npl_eventq_deinit: Some(ble_npl_eventq_deinit),
    p_ble_npl_eventq_get: Some(ble_npl_eventq_get),
    p_ble_npl_eventq_put: Some(ble_npl_eventq_put),
    p_ble_npl_eventq_remove: Some(ble_npl_eventq_remove),
    p_ble_npl_event_run: Some(ble_npl_event_run),
    p_ble_npl_eventq_is_empty: Some(ble_npl_eventq_is_empty),
    p_ble_npl_event_init: Some(ble_npl_event_init),
    p_ble_npl_event_deinit: Some(ble_npl_event_deinit),
    p_ble_npl_event_reset: Some(ble_npl_event_reset),
    p_ble_npl_event_is_queued: Some(ble_npl_event_is_queued),
    p_ble_npl_event_get_arg: Some(ble_npl_event_get_arg),
    p_ble_npl_event_set_arg: Some(ble_npl_event_set_arg),
    p_ble_npl_mutex_init: Some(ble_npl_mutex_init),
    p_ble_npl_mutex_deinit: Some(ble_npl_mutex_deinit),
    p_ble_npl_mutex_pend: Some(ble_npl_mutex_pend),
    p_ble_npl_mutex_release: Some(ble_npl_mutex_release),
    p_ble_npl_sem_init: Some(ble_npl_sem_init),
    p_ble_npl_sem_deinit: Some(ble_npl_sem_deinit),
    p_ble_npl_sem_pend: Some(ble_npl_sem_pend),
    p_ble_npl_sem_release: Some(ble_npl_sem_release),
    p_ble_npl_sem_get_count: Some(ble_npl_sem_get_count),
    p_ble_npl_callout_init: Some(ble_npl_callout_init),
    p_ble_npl_callout_reset: Some(ble_npl_callout_reset),
    p_ble_npl_callout_stop: Some(ble_npl_callout_stop),
    p_ble_npl_callout_deinit: Some(ble_npl_callout_deinit),
    p_ble_npl_callout_mem_reset: Some(ble_npl_callout_mem_reset),
    p_ble_npl_callout_is_active: Some(ble_npl_callout_is_active),
    p_ble_npl_callout_get_ticks: Some(ble_npl_callout_get_ticks),
    p_ble_npl_callout_remaining_ticks: Some(ble_npl_callout_remaining_ticks),
    p_ble_npl_callout_set_arg: Some(ble_npl_callout_set_arg),
    p_ble_npl_time_get: Some(ble_npl_time_get),
    p_ble_npl_time_ms_to_ticks: Some(ble_npl_time_ms_to_ticks),
    p_ble_npl_time_ticks_to_ms: Some(ble_npl_time_ticks_to_ms),
    p_ble_npl_time_ms_to_ticks32: Some(ble_npl_time_ms_to_ticks32),
    p_ble_npl_time_ticks_to_ms32: Some(ble_npl_time_ticks_to_ms32),
    p_ble_npl_time_delay: Some(ble_npl_time_delay),
    p_ble_npl_hw_set_isr: Some(ble_npl_hw_set_isr),
    p_ble_npl_hw_enter_critical: Some(ble_npl_hw_enter_critical),
    p_ble_npl_hw_exit_critical: Some(ble_npl_hw_exit_critical),
    p_ble_npl_get_time_forever: Some(ble_npl_get_time_forever),
    p_ble_npl_hw_is_in_critical: Some(ble_npl_hw_is_in_critical),
    p_ble_npl_eventq_put_to_front: Some(ble_npl_eventq_put_to_front),
};

#[repr(C)]
#[cfg(feature = "coex")]
/// Contains pointers to functions used for BLE coexistence with Wi-Fi.
pub(crate) struct OsiCoexFuncsT {
    magic: u32,
    version: u32,
    coex_wifi_sleep_set: Option<unsafe extern "C" fn(sleep: bool)>,
    coex_core_ble_conn_dyn_prio_get:
        Option<unsafe extern "C" fn(low: *mut bool, high: *mut bool) -> i32>,
    coex_schm_status_bit_set: Option<unsafe extern "C" fn(_type: u32, status: u32)>,
    coex_schm_status_bit_clear: Option<unsafe extern "C" fn(_type: u32, status: u32)>,
}

#[cfg(feature = "coex")]
static G_COEX_FUNCS: OsiCoexFuncsT = OsiCoexFuncsT {
    magic: 0xFADEBEAD,
    version: 0x00010006,
    coex_wifi_sleep_set: Some(coex_wifi_sleep_set),
    coex_core_ble_conn_dyn_prio_get: Some(coex_core_ble_conn_dyn_prio_get),
    coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
};

#[allow(unused)]
unsafe extern "C" fn coex_wifi_sleep_set(_sleep: bool) {
    todo!()
}

#[allow(unused)]
unsafe extern "C" fn coex_core_ble_conn_dyn_prio_get(_low: *mut bool, _high: *mut bool) -> i32 {
    todo!()
}

#[allow(unused)]
unsafe extern "C" fn coex_schm_status_bit_set(_type: u32, _status: u32) {
    trace!("coex_schm_status_bit_set is an empty stub");
}

#[allow(unused)]
unsafe extern "C" fn coex_schm_status_bit_clear(_type: u32, _status: u32) {
    trace!("coex_schm_status_bit_clear is an empty stub");
}

unsafe extern "C" fn ble_npl_hw_is_in_critical() -> u8 {
    todo!()
}

unsafe extern "C" fn ble_npl_get_time_forever() -> u32 {
    OSI_FUNCS_TIME_BLOCKING
}

unsafe extern "C" fn ble_npl_hw_exit_critical(mask: u32) {
    trace!("ble_npl_hw_exit_critical {}", mask);
    unsafe {
        let token = esp_sync::RestoreState::new(mask);
        super::ESP_RADIO_LOCK.release(token);
    }
}

unsafe extern "C" fn ble_npl_hw_enter_critical() -> u32 {
    trace!("ble_npl_hw_enter_critical");
    unsafe { super::ESP_RADIO_LOCK.acquire().inner() }
}

unsafe extern "C" fn ble_npl_hw_set_isr(_no: i32, _mask: u32) {
    todo!()
}

unsafe extern "C" fn ble_npl_time_delay(time: ble_npl_time_t) {
    let time = blob_ticks_to_micros(time);
    crate::preempt::usleep(time);
}

unsafe extern "C" fn ble_npl_time_ticks_to_ms32(time: ble_npl_time_t) -> u32 {
    trace!("ble_npl_time_ticks_to_ms32 {}", time);
    blob_ticks_to_millis(time)
}

unsafe extern "C" fn ble_npl_time_ms_to_ticks32(ms: u32) -> ble_npl_time_t {
    trace!("ble_npl_time_ms_to_ticks32 {}", ms);
    millis_to_blob_ticks(ms)
}

unsafe extern "C" fn ble_npl_time_ticks_to_ms(
    time: ble_npl_time_t,
    p_ms: *mut u32,
) -> ble_npl_error_t {
    trace!("ble_npl_time_ticks_to_ms {}", time);
    unsafe { *p_ms = blob_ticks_to_millis(time) };
    0
}

unsafe extern "C" fn ble_npl_time_ms_to_ticks(
    ms: u32,
    p_time: *mut ble_npl_time_t,
) -> ble_npl_error_t {
    trace!("ble_npl_time_ms_to_ticks {}", ms);
    unsafe { *p_time = millis_to_blob_ticks(ms) };
    0
}

unsafe extern "C" fn ble_npl_time_get() -> u32 {
    trace!("ble_npl_time_get");
    Instant::now().duration_since_epoch().as_millis() as u32
}

unsafe extern "C" fn ble_npl_callout_set_arg(
    _callout: *const ble_npl_callout,
    _arg: *const c_void,
) {
    todo!()
}

unsafe extern "C" fn ble_npl_callout_remaining_ticks(
    _callout: *const ble_npl_callout,
    _time: ble_npl_time_t,
) -> u32 {
    todo!()
}

unsafe extern "C" fn ble_npl_callout_get_ticks(_callout: *const ble_npl_callout) -> ble_npl_time_t {
    todo!()
}

unsafe extern "C" fn ble_npl_callout_is_active(callout: *const ble_npl_callout) -> bool {
    debug!(
        "Missing real implementation: ble_npl_callout_is_active {:?}",
        callout
    );

    assert!(unsafe { (*callout).dummy != 0 });

    unsafe {
        let co = (*callout).dummy as *mut Callout;
        compat::timer_compat::compat_timer_is_active(&raw mut (*co).timer_handle)
    }
}

// <https://github.com/espressif/esp-idf/blob/6d835d522/components/bt/porting/npl/freertos/src/npl_os_freertos.c#L185-L194>
unsafe extern "C" fn ble_npl_callout_mem_reset(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_mem_reset {:?}", callout);

    let co = unsafe { (*callout).dummy } as *mut Callout;
    assert!(!co.is_null());

    unsafe { ble_npl_event_reset(&raw const (*co).events) };
}

// <https://github.com/espressif/esp-idf/blob/6d835d522/components/bt/porting/npl/freertos/src/npl_os_freertos.c#L962-L998>
unsafe extern "C" fn ble_npl_callout_deinit(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_deinit {:?}", callout);

    if unsafe { (*callout).dummy } == 0 {
        return;
    }

    unsafe {
        let co = (*callout).dummy as *mut Callout;
        compat::timer_compat::compat_timer_done(&raw mut (*co).timer_handle);
        ble_npl_event_deinit(&raw const (*co).events);
        crate::compat::malloc::free(co.cast());

        (*callout.cast_mut()).dummy = 0;
    }
}

unsafe extern "C" fn ble_npl_callout_stop(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_stop {:?}", callout);

    assert!(unsafe { (*callout).dummy != 0 });

    unsafe {
        let co = (*callout).dummy as *mut Callout;
        // stop timer
        compat::timer_compat::compat_timer_disarm(&raw mut (*co).timer_handle);
    }
}

unsafe extern "C" fn ble_npl_callout_reset(
    callout: *const ble_npl_callout,
    time: ble_npl_time_t,
) -> ble_npl_error_t {
    trace!("ble_npl_callout_reset {:?} {}", callout, time);

    let co = unsafe { (*callout).dummy } as *mut Callout;
    unsafe {
        // start timer
        compat::timer_compat::compat_timer_arm(
            &raw mut (*co).timer_handle,
            blob_ticks_to_millis(time),
            false,
        );
    }
    0
}

unsafe extern "C" fn ble_npl_sem_get_count(_sem: *const ble_npl_sem) -> u16 {
    todo!()
}

unsafe extern "C" fn ble_npl_sem_release(_sem: *const ble_npl_sem) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_sem_pend(
    _sem: *const ble_npl_sem,
    _time: ble_npl_time_t,
) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_sem_deinit(_sem: *const ble_npl_sem) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_sem_init(_sem: *const ble_npl_sem, _val: u16) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_mutex_release(_mutex: *const ble_npl_mutex) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_mutex_pend(
    _mutex: *const ble_npl_mutex,
    _time: ble_npl_time_t,
) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_mutex_deinit(_mutex: *const ble_npl_mutex) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_event_set_arg(event: *const ble_npl_event, arg: *const c_void) {
    trace!("ble_npl_event_set_arg {:?} {:?}", event, arg);

    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    unsafe {
        (*evt).ev_arg_ptr = arg;
    }
}

unsafe extern "C" fn ble_npl_event_get_arg(event: *const ble_npl_event) -> *const c_void {
    trace!("ble_npl_event_get_arg {:?}", event);

    unsafe {
        let evt = (*event).dummy as *mut Event;
        assert!(!evt.is_null());

        let arg_ptr = (*evt).ev_arg_ptr;

        trace!("returning arg {:x}", arg_ptr as usize);

        arg_ptr
    }
}

unsafe extern "C" fn ble_npl_event_is_queued(event: *const ble_npl_event) -> bool {
    trace!("ble_npl_event_is_queued {:?}", event);

    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    unsafe { (*evt).queued }
}

unsafe extern "C" fn ble_npl_event_reset(event: *const ble_npl_event) {
    trace!("ble_npl_event_reset {:?}", event);

    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    unsafe { (*evt).queued = false }
}

unsafe extern "C" fn ble_npl_event_deinit(event: *const ble_npl_event) {
    trace!("ble_npl_event_deinit {:?}", event);

    let event = event as *mut ble_npl_event;
    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    unsafe {
        crate::compat::malloc::free(evt.cast());
    }

    unsafe {
        (*event).dummy = 0;
    }
}

unsafe extern "C" fn ble_npl_event_init(
    event: *const ble_npl_event,
    func: *const ble_npl_event_fn,
    arg: *const c_void,
) {
    trace!("ble_npl_event_init {:?} {:?} {:?}", event, func, arg);

    if unsafe { (*event).dummy } == 0 {
        unsafe {
            let evt = crate::compat::malloc::calloc(1, core::mem::size_of::<Event>()) as *mut Event;

            (*evt).event_fn_ptr = func;
            (*evt).ev_arg_ptr = arg;
            (*evt).queued = false;

            let event = event.cast_mut();
            (*event).dummy = evt as i32;
        }
    }
}

unsafe extern "C" fn ble_npl_eventq_is_empty(queue: *mut ble_npl_eventq) -> bool {
    trace!("ble_npl_eventq_is_empty {:?}", queue);
    let wrapper = unwrap!(unsafe { queue.as_mut() }, "queue wrapper is null");

    queue::queue_messages_waiting(wrapper.dummy as _) == 0
}

unsafe extern "C" fn ble_npl_event_run(event: *const ble_npl_event) {
    trace!("ble_npl_event_run {:?}", event);

    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    trace!(
        "info {:?} with arg {:x}",
        unsafe { (*evt).event_fn_ptr },
        event as u32
    );
    unsafe {
        let func: unsafe extern "C" fn(u32) = transmute((*evt).event_fn_ptr);
        func(event as u32);
    }

    trace!("ble_npl_event_run done");
}

unsafe extern "C" fn ble_npl_eventq_remove(
    queue: *mut ble_npl_eventq,
    event: *const ble_npl_event,
) {
    trace!("ble_npl_eventq_remove {:?} {:?}", queue, event);

    unsafe {
        let evt = (*event).dummy as *mut Event;
        assert!(!evt.is_null());

        if !(*evt).queued {
            return;
        }

        let wrapper = unwrap!(queue.as_mut(), "queue wrapper is null");
        queue::queue_remove(wrapper.dummy as _, (&raw const event).cast());

        (*evt).queued = false;
    }
}

unsafe extern "C" fn ble_npl_eventq_put(queue: *mut ble_npl_eventq, event: *const ble_npl_event) {
    trace!("ble_npl_eventq_put {:?} {:?}", queue, event);

    unsafe { eventq_insert(queue, event, false) }
}

unsafe extern "C" fn ble_npl_eventq_put_to_front(
    queue: *mut ble_npl_eventq,
    event: *const ble_npl_event,
) {
    trace!("ble_npl_eventq_put_to_front {:?} {:?}", queue, event);

    unsafe { eventq_insert(queue, event, true) }
}

unsafe fn eventq_insert(queue: *mut ble_npl_eventq, event: *const ble_npl_event, front: bool) {
    let evt = unsafe { (*event).dummy } as *mut Event;
    assert!(!evt.is_null());

    if unsafe { (*evt).queued } {
        trace!("Event already queued, skipping put");
        return;
    }

    unsafe {
        (*evt).queued = true;
    }

    let wrapper = unwrap!(unsafe { queue.as_mut() }, "queue wrapper is null");
    let handle = wrapper.dummy as _;
    // Store the pointer to the ble_npl_event in the queue - this is what we'll need to dequeue.
    let item = (&raw const event).cast();

    let sent = if in_isr() {
        if front {
            queue::queue_try_send_to_front_from_isr(handle, item, core::ptr::null_mut())
        } else {
            queue::queue_try_send_to_back_from_isr(handle, item, core::ptr::null_mut())
        }
    } else if front {
        queue::queue_send_to_front(handle, item, OSI_FUNCS_TIME_BLOCKING)
    } else {
        queue::queue_send_to_back(handle, item, OSI_FUNCS_TIME_BLOCKING)
    };

    if sent == 0 {
        // The queue is full. Mark the event unqueued so that the controller can post it again.
        trace!("Event queue is full, event dropped");
        unsafe {
            (*evt).queued = false;
        }
    }
}

unsafe extern "C" fn ble_npl_eventq_get(
    queue: *mut ble_npl_eventq,
    timeout: ble_npl_time_t,
) -> *const ble_npl_event {
    trace!("ble_npl_eventq_get {:?} {}", queue, timeout);

    let mut evt = core::ptr::null_mut::<ble_npl_event>();
    let wrapper = unwrap!(unsafe { queue.as_mut() }, "queue wrapper is null");
    let item = (&raw mut evt).cast();

    let received = if in_isr() {
        if timeout != 0 {
            return core::ptr::null();
        }
        queue::queue_try_receive_from_isr(wrapper.dummy as _, item, core::ptr::null_mut())
    } else {
        queue::queue_receive(wrapper.dummy as _, item, blob_ticks_to_micros(timeout))
    };

    if received != 0 {
        trace!("got {:x}", evt as usize);
        unsafe {
            let evt = (*evt).dummy as *mut Event;
            (*evt).queued = false;
        }
    }

    evt.cast_const()
}

unsafe extern "C" fn ble_npl_eventq_init(queue: *mut ble_npl_eventq) {
    trace!("ble_npl_eventq_init {:?}", queue);

    // Keep the existing queue and empty it, the way IDF's `npl_freertos_eventq_init` calls
    // `xQueueReset` instead of allocating again.
    // <https://github.com/espressif/esp-idf/blob/6d835d522/components/bt/porting/npl/freertos/src/npl_os_freertos.c#L135-L164>
    let existing = unsafe { (*queue).dummy };
    if existing != 0 {
        let mut event: usize = 0;
        while queue::queue_receive(existing as *mut c_void, (&raw mut event).cast(), 0) != 0 {}
        return;
    }

    let queue_ptr = queue::queue_create(EVENT_QUEUE_SIZE as _, core::mem::size_of::<usize>() as _);

    unsafe {
        (*queue).dummy = queue_ptr as i32;
    }
}

unsafe extern "C" fn ble_npl_eventq_deinit(queue: *mut ble_npl_eventq) {
    trace!("ble_npl_eventq_deinit {:?}", queue);

    let wrapper = unwrap!(unsafe { queue.as_mut() }, "queue wrapper is null");
    queue::queue_delete(wrapper.dummy as _);
    wrapper.dummy = 0;
}

unsafe extern "C" fn ble_npl_callout_init(
    callout: *const ble_npl_callout,
    eventq: *const ble_npl_eventq,
    func: *const ble_npl_event_fn,
    args: *const c_void,
) -> i32 {
    trace!(
        "ble_npl_callout_init {:?} {:?} {:?} {:?}",
        callout, eventq, func, args
    );

    if unsafe { (*callout).dummy } == 0 {
        let callout = callout.cast_mut();

        unsafe {
            let new_callout =
                crate::compat::malloc::calloc(1, core::mem::size_of::<Callout>()) as *mut Callout;
            ble_npl_event_init(&raw mut (*new_callout).events, func, args);
            (*callout).dummy = new_callout as i32;

            crate::compat::timer_compat::compat_timer_setfn(
                &raw mut (*new_callout).timer_handle,
                callout_timer_callback_wrapper,
                callout as *mut c_void,
            );
        }
    }

    0
}

unsafe extern "C" fn callout_timer_callback_wrapper(arg: *mut c_void) {
    trace!("callout_timer_callback_wrapper {:?}", arg);
    let co = unsafe { (*(arg as *mut ble_npl_callout)).dummy } as *mut Callout;

    unsafe {
        if !(*co).eventq.is_null() {
            ble_npl_eventq_put((*co).eventq.cast_mut(), &raw const (*co).events);
        } else {
            ble_npl_event_run(&raw const (*co).events);
        }
    }
}

unsafe extern "C" fn ble_npl_mutex_init(_mutex: *const ble_npl_mutex) -> u32 {
    todo!()
}

unsafe extern "C" fn ble_npl_get_current_task_id() -> *const c_void {
    todo!()
}

unsafe extern "C" fn ble_npl_os_started() -> bool {
    true
}

#[repr(C)]
/// Contains information about the BLE NPL (Non-Preemptive Layer) elements.
pub(crate) struct BleNplCountInfoT {
    evt_count: u16,
    evtq_count: u16,
    co_count: u16,
    sem_count: u16,
    mutex_count: u16,
}

unsafe extern "C" {
    fn r_ble_rtc_wake_up_state_clr();
    #[cfg(not(esp32c2))]
    fn r_ble_lll_sleep_should_skip_light_sleep_check() -> bool;
}

/// Returns whether the controller refuses a light sleep, because its next event is too close.
#[cfg(not(esp32c2))]
pub(super) fn controller_skips_light_sleep() -> bool {
    unsafe { r_ble_lll_sleep_should_skip_light_sleep_check() }
}

#[crate::hal::ram]
unsafe extern "C" fn controller_sleep_cb(_enable_tick: u32, _arg: *mut c_void) {
    unsafe { r_ble_rtc_wake_up_state_clr() };
    super::modem_phy_release();
}

#[crate::hal::ram]
unsafe extern "C" fn controller_wakeup_cb(arg: *mut c_void) {
    unsafe {
        #[cfg(not(esp32c2))]
        r_ble_rtc_wake_up_state_clr();

        // The controller passes its `bt_wakeup_params_t`. Bit 31 tells it that the BLE timer
        // ended the light sleep.
        const BT_WAKEUP: u32 = 1 << 31;
        let params = arg.cast::<u32>();
        if !params.is_null() {
            let by_bt =
                esp_hal::rtc_cntl::wakeup_cause().contains(esp_hal::rtc_cntl::WakeupSource::Bt);
            let value = params.read_volatile();
            params.write_volatile(if by_bt {
                value | BT_WAKEUP
            } else {
                value & !BT_WAKEUP
            });
        }
    }
    super::modem_phy_acquire();
}

fn register_modem_sleep() {
    unsafe extern "C" {
        #[cfg(not(esp32c2))]
        fn r_ble_lll_sleep_set_sleep_cb(
            sleep_cb: unsafe extern "C" fn(u32, *mut c_void),
            wakeup_cb: unsafe extern "C" fn(*mut c_void),
            sleep_arg: *mut c_void,
            wakeup_arg: *mut c_void,
            us_to_enabled: u32,
        );
        #[cfg(esp32c2)]
        fn r_ble_lll_rfmgmt_set_sleep_cb(
            sleep_cb: unsafe extern "C" fn(u32, *mut c_void),
            wakeup_cb: unsafe extern "C" fn(*mut c_void),
            sleep_arg: *mut c_void,
            wakeup_arg: *mut c_void,
            us_to_enabled: u32,
        );
    }

    // ESP-IDF PHY enable delay. C2 adds `BLE_RTC_DELAY_US` (1800) to 500. The others use the
    // light-sleep delay, because the chip can light-sleep while the controller sleeps.
    let delay_us = cfg_select! {
        esp32c2 => 2_300,
        esp32c5 => 2_500,
        any(esp32c6, esp32c61) => 3_200,
        esp32h2 => 5_100,
    };

    unsafe {
        cfg_select! {
            esp32c2 => r_ble_lll_rfmgmt_set_sleep_cb(
                controller_sleep_cb,
                controller_wakeup_cb,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                delay_us,
            ),
            _ => r_ble_lll_sleep_set_sleep_cb(
                controller_sleep_cb,
                controller_wakeup_cb,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                delay_us,
            ),
        }
    }
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    super::set_modem_sleep(config.modem_sleep());

    let phy_init_guard;
    unsafe {
        // turn on logging
        #[allow(static_mut_refs)]
        #[cfg(all(feature = "print-logs-from-driver", esp32c2))]
        {
            unsafe extern "C" {
                static mut g_ble_plf_log_level: u32;
            }

            debug!("g_ble_plf_log_level = {}", g_ble_plf_log_level);
            g_ble_plf_log_level = 10;
        }

        self::chip_specific::ble_rtc_clk_init();

        super::lp_clk::request();

        let cfg = chip_specific::create_ble_config(config);

        let res = esp_register_ext_funcs(&G_OSI_FUNCS as *const ExtFuncsT);
        assert!(res == 0, "esp_register_ext_funcs returned {}", res);

        #[cfg(esp32c2)]
        {
            debug!("Init esp_ble_rom_func_ptr_init_all");
            unsafe extern "C" {
                fn esp_ble_rom_func_ptr_init_all() -> i32;
            }
            let res = esp_ble_rom_func_ptr_init_all();
            assert!(res == 0, "esp_ble_rom_func_ptr_init_all returned {}", res);
        }

        #[cfg(feature = "coex")]
        {
            let res = crate::wifi::coex_init();
            assert!(res == 0, "coex_init failed");
        }

        chip_specific::bt_periph_module_enable();

        chip_specific::disable_sleep_mode();

        let res = esp_register_npl_funcs(&G_NPL_FUNCS);
        assert!(res == 0, "esp_register_npl_funcs returned {}", res);

        // not really using  here ... remove it?
        let npl_info = BleNplCountInfoT {
            evt_count: 0,
            evtq_count: 0,
            co_count: 0,
            sem_count: 0,
            mutex_count: 0,
        };

        let res = cfg_select! {
            esp32c2 => ble_get_npl_element_info(&cfg, &npl_info),
            _ => r_ble_get_npl_element_info(&cfg, &npl_info),
        };
        assert!(res == 0, "ble_get_npl_element_info returned {}", res);

        // Initialize the global memory pool
        #[cfg(esp32c2)]
        chip_specific::os_msys_init();

        phy_init_guard = esp_phy::enable_phy();

        // init bb
        bt_bb_v2_init_cmplx(1);

        coex_pti_v2();

        #[cfg(feature = "coex")]
        {
            let rc = ble_osi_coex_funcs_register(&G_COEX_FUNCS);
            assert!(rc == 0, "ble_osi_coex_funcs_register returned {}", rc);
        }

        let res = ble_controller_init(&cfg);
        assert!(res == 0, "ble_controller_init returned {}", res);

        if config.modem_sleep() {
            register_modem_sleep();
        }

        #[cfg(feature = "coex")]
        crate::sys::include::coex_enable();

        let mut mac = [0u8; 6];
        crate::common_adapter::read_mac(mac.as_mut_ptr(), 2);
        mac.reverse();

        cfg_select! {
            esp32c2 => esp_ble_ll_set_public_addr(mac.as_ptr()),
            _ => r_esp_ble_ll_set_public_addr(mac.as_ptr()),
        };

        r_ble_hci_trans_init(0);

        r_ble_hci_trans_cfg_hs(
            Some(ble_hs_hci_rx_evt),
            core::ptr::null(),
            Some(ble_hs_rx_data),
            core::ptr::null(),
        );

        const BLE: u8 = 1;
        let res = cfg_select! {
            esp32c2 => ble_controller_enable(BLE),
            _ => r_ble_controller_enable(BLE),
        };
        assert!(res == 0, "ble_controller_enable returned {}", res);
    }

    if config.modem_sleep() {
        super::lp_clk::claim_wake_source();
    }

    // At some point the "High-speed ADC" entropy source became available.
    #[cfg(rng_trng_supported)]
    unsafe {
        esp_hal::rng::TrngSource::increase_entropy_source_counter()
    };

    debug!("The ble_controller_init was initialized");
    phy_init_guard
}

pub(crate) fn ble_deinit() {
    super::lp_clk::release_wake_source();
    super::modem_phy_acquire();
    super::set_modem_sleep(false);

    #[cfg(rng_trng_supported)]
    esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
        esp_hal::Internal::conjure()
    });

    unsafe {
        // HCI deinit
        r_ble_hci_trans_cfg_hs(None, core::ptr::null(), None, core::ptr::null());

        cfg_select! {
            esp32c2 => ble_controller_disable(),
            _ => r_ble_controller_disable(),
        };

        let res = ble_controller_deinit();
        assert!(res == 0, "ble_controller_deinit returned {}", res);

        #[cfg(esp32c2)]
        chip_specific::os_msys_buf_free();

        esp_unregister_npl_funcs();
        esp_unregister_ext_funcs();
    }

    super::lp_clk::release();
}

unsafe extern "C" fn ble_hs_hci_rx_evt(cmd: *const u8, arg: *const c_void) -> i32 {
    trace!("ble_hs_hci_rx_evt {:?} {:?}", cmd, arg);
    trace!("$ cmd = {:x}", unsafe { *cmd });
    trace!("$ len = {:x}", unsafe { *(cmd.offset(1)) });

    let event = unsafe { *cmd };
    let len = unsafe { *(cmd.offset(1)) } as usize;
    let payload = unsafe { core::slice::from_raw_parts(cmd.offset(2), len) };
    trace!("$ pld = {:?}", payload);

    let mut data = Vec::with_capacity(len + 3);
    data.push(0x04); // this is an event
    data.push(event);
    data.push(len as u8);
    data.extend_from_slice(payload);

    super::dump_packet_info(&data);

    super::BT_STATE.with(|state| {
        state.rx_queue.push_back(ReceivedPacket {
            data: data.into_boxed_slice(),
        });
    });

    unsafe {
        r_ble_hci_trans_buf_free(cmd);
    }

    crate::ble::controller::hci_read_data_available();

    0
}

unsafe extern "C" fn ble_hs_rx_data(om: *const OsMbuf, arg: *const c_void) -> i32 {
    trace!("ble_hs_rx_data {:?} {:?}", om, arg);

    let data_ptr = unsafe { (*om).om_data };
    let len = unsafe { (*om).om_len };
    let data_slice = unsafe { core::slice::from_raw_parts(data_ptr, len as usize) };

    let mut data = Vec::with_capacity(data_slice.len() + 1);
    data.push(0x02); // ACL
    data.extend_from_slice(data_slice);

    super::dump_packet_info(&data);

    super::BT_STATE.with(|state| {
        state.rx_queue.push_back(ReceivedPacket {
            data: data.into_boxed_slice(),
        });
    });

    unsafe {
        r_os_mbuf_free_chain(om as *mut _);
    }

    crate::ble::controller::hci_read_data_available();

    0
}

pub(crate) fn send(data: &[u8]) {
    send_packet(data)
}

pub(crate) async fn send_async(data: &[u8]) {
    send_packet(data)
}

fn send_packet(packet: &[u8]) {
    const DATA_TYPE_COMMAND: u8 = 1;
    const DATA_TYPE_ACL: u8 = 2;

    super::dump_packet_info(packet);

    unsafe {
        if packet[0] == DATA_TYPE_COMMAND {
            let cmd = r_ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_CMD);
            core::ptr::copy_nonoverlapping(
                &raw const packet[1], // don't send the TYPE
                cmd as *mut u8,
                packet.len() - 1,
            );

            let res = r_ble_hci_trans_hs_cmd_tx(cmd);

            if res != 0 {
                warn!("ble_hci_trans_hs_cmd_tx res == {}", res);
            }
        } else if packet[0] == DATA_TYPE_ACL {
            let om = r_os_msys_get_pkthdr(packet.len() as u16, ACL_DATA_MBUF_LEADINGSPACE as u16);

            let res = r_os_mbuf_append(om, packet.as_ptr().offset(1), (packet.len() - 1) as u16);
            assert!(res == 0, "r_os_mbuf_append returned {}", res);

            // this modification of the ACL data packet makes it getting sent and
            // received by the other side
            *((*om).om_data as *mut u8).offset(1) = 0;

            let res = r_ble_hci_trans_hs_acl_tx(om);
            if res != 0 {
                panic!("ble_hci_trans_hs_acl_tx returned {}", res);
            }
            trace!("ACL tx done");
        } else {
            warn!("Unknown packet kind {} dropped", packet[0]);
        }
    }
}
