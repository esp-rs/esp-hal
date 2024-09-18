use core::{
    cell::RefCell,
    ptr::{addr_of, addr_of_mut},
};

use critical_section::Mutex;

use super::*;
use crate::{
    binary::{
        c_types::{c_char, c_void},
        include::*,
    },
    compat,
    compat::{common::str_from_c, queue::SimpleQueue},
    timer::yield_task,
};

#[cfg_attr(esp32c2, path = "os_adapter_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "os_adapter_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "os_adapter_esp32h2.rs")]
pub(crate) mod ble_os_adapter_chip_specific;

const TIME_FOREVER: u32 = u32::MAX;

#[cfg(esp32c2)]
const OS_MSYS_1_BLOCK_COUNT: i32 = 24;
#[cfg(esp32c2)]
const SYSINIT_MSYS_1_MEMPOOL_SIZE: usize = 768;
#[cfg(esp32c2)]
const SYSINIT_MSYS_1_MEMBLOCK_SIZE: i32 = 128;
#[cfg(esp32c2)]
const OS_MSYS_2_BLOCK_COUNT: i32 = 24;
#[cfg(esp32c2)]
const SYSINIT_MSYS_2_MEMPOOL_SIZE: usize = 1920;
#[cfg(esp32c2)]
const SYSINIT_MSYS_2_MEMBLOCK_SIZE: i32 = 320;

const BLE_HCI_TRANS_BUF_CMD: i32 = 3;

// ACL_DATA_MBUF_LEADINGSPCAE: The leadingspace in user info header for ACL data
const ACL_DATA_MBUF_LEADINGSPACE: usize = 4;

#[derive(Copy, Clone)]
struct Callout {
    _callout: *const ble_npl_callout,
    eventq: *const ble_npl_eventq,
    timer_handle: ets_timer,
    events: ble_npl_event,
}

static mut CALLOUTS: [Option<Callout>; 18] = [None; 18];

#[derive(Copy, Clone)]
struct Event {
    event: *const ble_npl_event,
    event_fn_ptr: *const ble_npl_event_fn,
    ev_arg_ptr: *const c_void,
    queued: bool,
}

static mut EVENTS: [Option<Event>; 95] = [None; 95];

static mut EVENT_QUEUE: SimpleQueue<usize, 16> = SimpleQueue::new();

static BT_RECEIVE_QUEUE: Mutex<RefCell<SimpleQueue<ReceivedPacket, 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

#[cfg(esp32c2)]
type OsMembufT = u32;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedPacket {
    pub len: u8,
    pub data: [u8; 256],
}

/// Memory pool
#[repr(C)]
pub(crate) struct OsMempool {
    /// Size of the memory blocks, in bytes.
    mp_block_size: u32,
    /// The number of memory blocks.
    mp_num_blocks: u16,
    /// The number of free blocks left
    mp_num_free: u16,
    /// The lowest number of free blocks seen
    mp_min_free: u16,
    /// Bitmap of OS_MEMPOOL_F_[...] values.
    mp_flags: u8,
    /// Address of memory buffer used by pool
    mp_membuf_addr: u32,

    // STAILQ_ENTRY(os_mempool) mp_list;
    next: *const OsMempool,

    // SLIST_HEAD(,os_memblock);
    first: *const c_void,

    /// Name for memory block
    name: *const u8,
}

#[cfg(esp32c2)]
impl OsMempool {
    const fn zeroed() -> Self {
        Self {
            mp_block_size: 0,
            mp_num_blocks: 0,
            mp_num_free: 0,
            mp_min_free: 0,
            mp_flags: 0,
            mp_membuf_addr: 0,
            next: core::ptr::null(),
            first: core::ptr::null(),
            name: core::ptr::null(),
        }
    }
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

#[cfg(esp32c2)]
impl OsMbufPool {
    const fn zeroed() -> Self {
        Self {
            omp_databuf_len: 0,
            omp_pool: core::ptr::null(),
            next: core::ptr::null(),
        }
    }
}

/// Chained memory buffer.
#[repr(C)]
pub struct OsMbuf {
    /// Current pointer to data in the structure
    om_data: *const u8,
    /// Flags associated with this buffer, see OS_MBUF_F_* defintions
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

#[repr(C)]
pub struct OsMempoolExt {
    mpe_mp: OsMempool,

    // Callback that is executed immediately when a block is freed.
    mpe_put_cb: OsMempoolPutFn,
    mpe_put_arg: *const c_void,
}

type OsMempoolPutFn = Option<
    unsafe extern "C" fn(ome: *const OsMempoolExt, data: *const c_void, arg: *const c_void) -> i32,
>;

#[repr(C)]
pub struct BleHciTransFuncsT {
    ble_hci_trans_hs_acl_tx: Option<unsafe extern "C" fn(om: *const OsMbuf) -> i32>,
    ble_hci_trans_hs_cmd_tx: Option<unsafe extern "C" fn(cmd: *const u8) -> i32>,
    ble_hci_trans_ll_acl_tx: Option<unsafe extern "C" fn(om: *const OsMbuf) -> i32>,
    ble_hci_trans_ll_evt_tx: Option<unsafe extern "C" fn(hci_ev: *const u8) -> i32>,
    ble_hci_trans_reset: Option<unsafe extern "C" fn() -> i32>,
    ble_hci_trans_set_acl_free_cb:
        Option<unsafe extern "C" fn(cb: OsMempoolPutFn, arg: *const c_void) -> i32>,
}

#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_1_DATA: *mut OsMembufT = core::ptr::null_mut();
#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_1_MBUF_POOL: OsMbufPool = OsMbufPool::zeroed();
#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_1_MEMPOOL: OsMempool = OsMempool::zeroed();

#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_2_DATA: *mut OsMembufT = core::ptr::null_mut();
#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_2_MBUF_POOL: OsMbufPool = OsMbufPool::zeroed();
#[cfg(esp32c2)]
pub(crate) static mut OS_MSYS_INIT_2_MEMPOOL: OsMempool = OsMempool::zeroed();

extern "C" {
    static ble_hci_trans_funcs_ptr: &'static BleHciTransFuncsT;

    #[cfg(esp32c2)]
    static mut r_ble_stub_funcs_ptr: *mut u32;

    pub(crate) fn ble_controller_init(cfg: *const esp_bt_controller_config_t) -> i32;

    pub(crate) fn ble_controller_enable(mode: u8) -> i32;

    pub(crate) fn esp_register_ext_funcs(funcs: *const ext_funcs_t) -> i32;

    pub(crate) fn esp_register_npl_funcs(funcs: *const npl_funcs_t) -> i32;

    pub(crate) fn ble_get_npl_element_info(
        cfg: *const esp_bt_controller_config_t,
        npl_info: *const ble_npl_count_info_t,
    ) -> i32;

    pub(crate) fn bt_bb_v2_init_cmplx(value: u8);

    pub(crate) fn r_ble_hci_trans_cfg_hs(
        evt: Option<unsafe extern "C" fn(cmd: *const u8, arg: *const c_void)>, /* ble_hci_trans_rx_cmd_fn */
        evt_arg: *const c_void,
        acl_cb: Option<unsafe extern "C" fn(om: *const OsMbuf, arg: *const c_void)>, /* ble_hci_trans_rx_acl_fn */
        acl_arg: *const c_void,
    );

    pub(crate) fn esp_ble_ll_set_public_addr(addr: *const u8);

    #[cfg(esp32c2)]
    pub(crate) fn r_mem_init_mbuf_pool(
        mem: *const c_void,
        mempool: *const OsMempool,
        mbuf_pool: *const OsMbufPool,
        num_blocks: i32,
        block_size: i32,
        name: *const u8,
    ) -> i32;

    #[cfg(esp32c2)]
    pub(crate) fn r_os_msys_reset();

    #[cfg(esp32c2)]
    pub(crate) fn r_os_msys_register(mbuf_pool: *const OsMbufPool) -> i32;

    #[allow(unused)]
    pub(crate) fn ble_osi_coex_funcs_register(coex_funcs: *const osi_coex_funcs_t) -> i32;

    pub(crate) fn r_os_msys_get_pkthdr(dsize: u16, user_hdr_len: u16) -> *mut OsMbuf;

    pub(crate) fn r_os_mbuf_append(om: *mut OsMbuf, src: *const u8, len: u16) -> i32;

    pub(crate) fn r_os_mbuf_free_chain(om: *mut OsMbuf) -> i32;

    pub(crate) fn r_ble_hci_trans_buf_alloc(typ: i32) -> *const u8;

    pub(crate) fn r_ble_hci_trans_buf_free(buf: *const u8);

    static mut ble_hci_trans_env_p: u32;
}

#[repr(C)]
pub struct ext_funcs_t {
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
    hal_uart_start_tx: Option<unsafe extern "C" fn(i32)>,
    hal_uart_init_cbs: Option<
        unsafe extern "C" fn(i32, *const c_void, *const c_void, *const c_void, c_void) -> i32,
    >,
    hal_uart_config: Option<unsafe extern "C" fn(i32, i32, u8, u8, u8, u8) -> i32>,
    hal_uart_close: Option<unsafe extern "C" fn(i32) -> i32>,
    hal_uart_blocking_tx: Option<unsafe extern "C" fn(i32, u8)>,
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
    task_delete: Option<unsafe extern "C" fn(*const c_void)>,
    osi_assert: Option<unsafe extern "C" fn(u32, *const c_void, u32, u32)>,
    os_random: Option<unsafe extern "C" fn() -> u32>,
    ecc_gen_key_pair: Option<unsafe extern "C" fn(*const u8, *const u8) -> i32>,
    ecc_gen_dh_key: Option<unsafe extern "C" fn(*const u8, *const u8, *const u8, *const u8) -> i32>,
    esp_reset_rpa_moudle: Option<unsafe extern "C" fn()>,
    #[cfg(esp32c2)]
    esp_bt_track_pll_cap: Option<unsafe extern "C" fn()>,
    magic: u32,
}

static G_OSI_FUNCS: ext_funcs_t = ext_funcs_t {
    ext_version: 0x20221122,
    esp_intr_alloc: Some(self::ble_os_adapter_chip_specific::esp_intr_alloc),
    esp_intr_free: Some(esp_intr_free),
    malloc: Some(crate::ble::malloc),
    free: Some(crate::ble::free),
    hal_uart_start_tx: None,
    hal_uart_init_cbs: None,
    hal_uart_config: None,
    hal_uart_close: None,
    hal_uart_blocking_tx: None,
    hal_uart_init: None,
    task_create: Some(task_create),
    task_delete: Some(task_delete),
    osi_assert: Some(osi_assert),
    os_random: Some(os_random),
    ecc_gen_key_pair: Some(ecc_gen_key_pair),
    ecc_gen_dh_key: Some(ecc_gen_dh_key),
    esp_reset_rpa_moudle: Some(self::ble_os_adapter_chip_specific::esp_reset_rpa_moudle),
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
    (crate::common_adapter::random() & u32::MAX) as u32
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
    let name_str = str_from_c(name as *const u8);
    trace!(
        "task_create {:?} {} {} {:?} {} {:?} {}",
        task_func,
        name_str,
        stack_depth,
        param,
        prio,
        task_handle,
        core_id,
    );

    *(task_handle as *mut usize) = 0; // we will run it in task 0

    let task_func = core::mem::transmute::<
        *mut crate::binary::c_types::c_void,
        extern "C" fn(*mut esp_wifi_sys::c_types::c_void),
    >(task_func);

    let task = crate::preempt::arch_specific::task_create(task_func, param, stack_depth as usize);
    *(task_handle as *mut usize) = task as usize;

    1
}

unsafe extern "C" fn task_delete(_: *const c_void) {
    todo!();
}

unsafe extern "C" fn osi_assert(ln: u32, fn_name: *const c_void, param1: u32, param2: u32) {
    let name_str = str_from_c(fn_name as *const u8);
    panic!("ASSERT {}:{} {} {}", name_str, ln, param1, param2);
}

unsafe extern "C" fn esp_intr_free(_ret_handle: *mut *mut c_void) -> i32 {
    todo!();
}

#[repr(C)]
pub struct npl_funcs_t {
    p_ble_npl_os_started: Option<unsafe extern "C" fn() -> bool>,
    p_ble_npl_get_current_task_id: Option<unsafe extern "C" fn() -> *const c_void>,
    p_ble_npl_eventq_init: Option<unsafe extern "C" fn(queue: *const ble_npl_eventq)>,
    p_ble_npl_eventq_deinit: Option<unsafe extern "C" fn(queue: *const ble_npl_eventq)>,
    p_ble_npl_eventq_get: Option<
        unsafe extern "C" fn(
            queue: *const ble_npl_eventq,
            time: ble_npl_time_t,
        ) -> *const ble_npl_event,
    >,
    p_ble_npl_eventq_put:
        Option<unsafe extern "C" fn(queue: *const ble_npl_eventq, event: *const ble_npl_event)>,
    p_ble_npl_eventq_remove:
        Option<unsafe extern "C" fn(queue: *const ble_npl_eventq, event: *const ble_npl_event)>,
    p_ble_npl_event_run: Option<unsafe extern "C" fn(event: *const ble_npl_event)>,
    p_ble_npl_eventq_is_empty: Option<unsafe extern "C" fn(queue: *const ble_npl_eventq) -> bool>,
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
        Option<unsafe extern "C" fn(ms: u32, p_time: *const ble_npl_time_t) -> ble_npl_error_t>,
    p_ble_npl_time_ticks_to_ms:
        Option<unsafe extern "C" fn(time: ble_npl_time_t, *const u32) -> ble_npl_error_t>,
    p_ble_npl_time_ms_to_ticks32: Option<unsafe extern "C" fn(ms: u32) -> ble_npl_time_t>,
    p_ble_npl_time_ticks_to_ms32: Option<unsafe extern "C" fn(time: ble_npl_time_t) -> u32>,
    p_ble_npl_time_delay: Option<unsafe extern "C" fn(time: ble_npl_time_t)>,
    p_ble_npl_hw_set_isr: Option<unsafe extern "C" fn(no: i32, mask: u32)>,
    p_ble_npl_hw_enter_critical: Option<unsafe extern "C" fn() -> u32>,
    p_ble_npl_hw_exit_critical: Option<unsafe extern "C" fn(mask: u32)>,
    p_ble_npl_get_time_forever: Option<unsafe extern "C" fn() -> u32>,
    p_ble_npl_hw_is_in_critical: Option<unsafe extern "C" fn() -> u8>,
}

static mut G_NPL_FUNCS: npl_funcs_t = npl_funcs_t {
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
};

#[repr(C)]
pub struct osi_coex_funcs_t {
    magic: u32,
    version: u32,
    coex_wifi_sleep_set: Option<unsafe extern "C" fn(sleep: bool)>,
    coex_core_ble_conn_dyn_prio_get:
        Option<unsafe extern "C" fn(low: *mut bool, high: *mut bool) -> i32>,
    coex_schm_status_bit_set: Option<unsafe extern "C" fn(_type: u32, status: u32)>,
    coex_schm_status_bit_clear: Option<unsafe extern "C" fn(_type: u32, status: u32)>,
}

#[allow(unused)]
static G_COEX_FUNCS: osi_coex_funcs_t = osi_coex_funcs_t {
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
    trace!("ble_npl_get_time_forever");
    TIME_FOREVER
}

unsafe extern "C" fn ble_npl_hw_exit_critical(mask: u32) {
    trace!("ble_npl_hw_exit_critical {}", mask);
    critical_section::release(core::mem::transmute::<u8, critical_section::RestoreState>(
        mask as u8,
    ));
}

unsafe extern "C" fn ble_npl_hw_enter_critical() -> u32 {
    trace!("ble_npl_hw_enter_critical");
    let as_u8: u8 = core::mem::transmute(critical_section::acquire());
    as_u8 as u32
}

unsafe extern "C" fn ble_npl_hw_set_isr(_no: i32, _mask: u32) {
    todo!()
}

unsafe extern "C" fn ble_npl_time_delay(_time: ble_npl_time_t) {
    todo!()
}

unsafe extern "C" fn ble_npl_time_ticks_to_ms32(_time: ble_npl_time_t) -> u32 {
    todo!()
}

unsafe extern "C" fn ble_npl_time_ms_to_ticks32(ms: u32) -> ble_npl_time_t {
    trace!("ble_npl_time_ms_to_ticks32 {}", ms);
    ms
}

unsafe extern "C" fn ble_npl_time_ticks_to_ms(
    _time: ble_npl_time_t,
    _p_ms: *const u32,
) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_time_ms_to_ticks(
    _ms: u32,
    _p_time: *const ble_npl_time_t,
) -> ble_npl_error_t {
    todo!()
}

unsafe extern "C" fn ble_npl_time_get() -> u32 {
    trace!("ble_npl_time_get");
    crate::current_millis() as u32
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

unsafe extern "C" fn ble_npl_callout_is_active(_callout: *const ble_npl_callout) -> bool {
    todo!()
}

unsafe extern "C" fn ble_npl_callout_mem_reset(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_mem_reset");

    ble_npl_callout_stop(callout);
}

unsafe extern "C" fn ble_npl_callout_deinit(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_deinit");

    ble_npl_callout_stop(callout);
}

unsafe extern "C" fn ble_npl_callout_stop(callout: *const ble_npl_callout) {
    trace!("ble_npl_callout_stop {:?}", callout);

    if (*callout).dummy == 0 {
        panic!("Trying to stop an uninitialzed callout");
    }

    let co = unwrap!(CALLOUTS[((*callout).dummy - 1) as usize].as_mut());

    // stop timer
    compat::timer_compat::compat_timer_disarm(addr_of_mut!(co.timer_handle));
}

unsafe extern "C" fn ble_npl_callout_reset(
    callout: *const ble_npl_callout,
    time: ble_npl_time_t,
) -> ble_npl_error_t {
    trace!("ble_npl_callout_reset {:?} {}", callout, time);

    let co = unwrap!(CALLOUTS[((*callout).dummy - 1) as usize].as_mut());

    // start timer
    compat::timer_compat::compat_timer_arm(addr_of_mut!(co.timer_handle), time, false);

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
    if (*event).dummy == 0 {
        panic!("Call set_arg on uninitialized event");
    }

    unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).ev_arg_ptr = arg;
}

unsafe extern "C" fn ble_npl_event_get_arg(event: *const ble_npl_event) -> *const c_void {
    trace!("ble_npl_event_get_arg {:?}", event);
    if (*event).dummy == 0 {
        panic!("Call get_arg on uninitialized event");
    }

    let arg_ptr = unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).ev_arg_ptr;

    trace!("returning arg {:x}", arg_ptr as usize);

    arg_ptr
}

unsafe extern "C" fn ble_npl_event_is_queued(event: *const ble_npl_event) -> bool {
    trace!("ble_npl_event_is_queued {:?}", event);
    if (*event).dummy == 0 {
        panic!("Call is_queued on uninitialized event");
    }

    unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).queued
}

unsafe extern "C" fn ble_npl_event_reset(event: *const ble_npl_event) {
    trace!("ble_npl_event_reset {:?}", event);

    let event = event as *mut ble_npl_event;
    if (*event).dummy == 0 {
        panic!("Trying to reset an uninitialized event");
    } else {
        unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).queued = false;
    }
}

unsafe extern "C" fn ble_npl_event_deinit(_event: *const ble_npl_event) {
    todo!()
}

unsafe extern "C" fn ble_npl_event_init(
    event: *const ble_npl_event,
    func: *const ble_npl_event_fn,
    arg: *const c_void,
) {
    trace!("ble_npl_event_init {:?} {:?} {:?}", event, func, arg);

    if (*event).dummy == 0 {
        let idx = unwrap!(EVENTS.iter().position(|item| item.is_none()));
        EVENTS[idx] = Some(Event {
            event,
            event_fn_ptr: func,
            ev_arg_ptr: arg,
            queued: false,
        });

        let event = event.cast_mut();
        (*event).dummy = (idx + 1) as i32;
    }
}

unsafe extern "C" fn ble_npl_eventq_is_empty(queue: *const ble_npl_eventq) -> bool {
    trace!("ble_npl_eventq_is_empty {:?}", queue);

    if (*queue).dummy == 0 {
        panic!("Try to use uninitialized queue");
    }

    critical_section::with(|_| EVENT_QUEUE.is_empty())
}

unsafe extern "C" fn ble_npl_event_run(event: *const ble_npl_event) {
    trace!("ble_npl_event_run {:?}", event);

    let event = event as *mut ble_npl_event;
    if (*event).dummy == 0 {
        panic!("Trying to run an uninitialized event");
    } else {
        let ev = unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut());
        trace!("info {:?} with arg {:x}", ev.event_fn_ptr, event as u32);
        let func: unsafe extern "C" fn(u32) = core::mem::transmute(ev.event_fn_ptr);
        func(event as u32);
    }

    trace!("ble_npl_event_run done");
}

unsafe extern "C" fn ble_npl_eventq_remove(
    queue: *const ble_npl_eventq,
    event: *const ble_npl_event,
) {
    trace!("ble_npl_eventq_remove {:?} {:?}", queue, event);

    if (*queue).dummy == 0 {
        panic!("Try to use uninitialized queue");
    }

    if (*event).dummy == 0 {
        panic!("Try to use uninitialized event");
    }

    critical_section::with(|_| {
        unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).queued = false;
    });
}

unsafe extern "C" fn ble_npl_eventq_put(queue: *const ble_npl_eventq, event: *const ble_npl_event) {
    trace!("ble_npl_eventq_put {:?} {:?}", queue, event);

    if (*queue).dummy == 0 {
        panic!("Try to use uninitialized queue");
    }

    if (*event).dummy == 0 {
        panic!("Try to use uninitialized event");
    }

    critical_section::with(|_| {
        unwrap!(EVENTS[((*event).dummy - 1) as usize].as_mut()).queued = true;
        unwrap!(EVENT_QUEUE.enqueue((*event).dummy as usize));
    });
}

unsafe extern "C" fn ble_npl_eventq_get(
    queue: *const ble_npl_eventq,
    time: ble_npl_time_t,
) -> *const ble_npl_event {
    trace!("ble_npl_eventq_get {:?} {}", queue, time);

    if time == TIME_FOREVER {
        loop {
            let dequeued = critical_section::with(|_| EVENT_QUEUE.dequeue());

            if let Some(event_idx) = dequeued {
                let evt = unwrap!(EVENTS[event_idx - 1].as_mut());
                if evt.queued {
                    trace!("got {:x}", evt.event as usize);
                    evt.queued = false;
                    return evt.event;
                }
            }

            yield_task();
        }
    } else {
        panic!("timed eventq_get not yet supported - go implement it!");
    }
}

unsafe extern "C" fn ble_npl_eventq_deinit(_queue: *const ble_npl_eventq) {
    todo!()
}

unsafe extern "C" fn ble_npl_callout_init(
    callout: *const ble_npl_callout,
    eventq: *const ble_npl_eventq,
    func: *const ble_npl_event_fn,
    args: *const c_void,
) -> i32 {
    trace!(
        "ble_npl_callout_init {:?} {:?} {:?} {:?}",
        callout,
        eventq,
        func,
        args
    );

    if (*callout).dummy == 0 {
        let callout = callout.cast_mut();
        let idx = unwrap!(CALLOUTS.iter().position(|item| item.is_none()));

        let new_callout = CALLOUTS[idx].insert(Callout {
            _callout: callout,
            eventq,
            timer_handle: ets_timer {
                next: core::ptr::null_mut(),
                expire: 0,
                period: 0,
                func: None,
                priv_: core::ptr::null_mut(),
            },
            events: ble_npl_event { dummy: 0 },
        });

        ble_npl_event_init(addr_of_mut!(new_callout.events), func, args);

        crate::compat::timer_compat::compat_timer_setfn(
            addr_of_mut!(new_callout.timer_handle),
            callout_timer_callback_wrapper,
            idx as *mut c_void,
        );

        (*callout).dummy = (idx + 1) as i32;
    }

    0
}

unsafe extern "C" fn callout_timer_callback_wrapper(arg: *mut c_void) {
    info!("callout_timer_callback_wrapper {:?}", arg);
    let co = unwrap!(CALLOUTS[arg as usize].as_mut());

    if co.eventq.is_null() {
        ble_npl_eventq_put(addr_of!(co.events).cast(), addr_of!(co.events));
    } else {
        ble_npl_event_run(addr_of!(co.events));
    }
}

unsafe extern "C" fn ble_npl_eventq_init(queue: *const ble_npl_eventq) {
    trace!("ble_npl_eventq_init {:?}", queue);

    critical_section::with(|_cs| {
        let queue = queue as *mut ble_npl_eventq;

        if (*queue).dummy == 0 {
            (*queue).dummy = 1;
        } else {
            panic!("Only one emulated queue supported");
        }
    });
}

unsafe extern "C" fn ble_npl_mutex_init(_mutex: *const ble_npl_mutex) -> u32 {
    todo!()
}

unsafe extern "C" fn ble_npl_get_current_task_id() -> *const c_void {
    todo!()
}

unsafe extern "C" fn ble_npl_os_started() -> bool {
    todo!();
}

#[repr(C)]
pub struct ble_npl_count_info_t {
    evt_count: u16,
    evtq_count: u16,
    co_count: u16,
    sem_count: u16,
    mutex_count: u16,
}

pub(crate) fn ble_init() {
    unsafe {
        *(HCI_OUT_COLLECTOR.as_mut_ptr()) = HciOutCollector::new();

        // turn on logging
        #[cfg(feature = "sys-logs")]
        {
            extern "C" {
                static mut g_ble_plf_log_level: u32;
            }

            debug!("g_ble_plf_log_level = {}", g_ble_plf_log_level);
            g_ble_plf_log_level = 10;
        }

        self::ble_os_adapter_chip_specific::ble_rtc_clk_init();

        let cfg = ble_os_adapter_chip_specific::BLE_CONFIG;

        let res = esp_register_ext_funcs(&G_OSI_FUNCS as *const ext_funcs_t);
        if res != 0 {
            panic!("esp_register_ext_funcs returned {}", res);
        }

        #[cfg(coex)]
        {
            let res = crate::wifi::coex_init();
            if res != 0 {
                panic!("got error");
            }
        }

        ble_os_adapter_chip_specific::bt_periph_module_enable();

        ble_os_adapter_chip_specific::disable_sleep_mode();

        let res = esp_register_npl_funcs(core::ptr::addr_of!(G_NPL_FUNCS));
        if res != 0 {
            panic!("esp_register_npl_funcs returned {}", res);
        }

        let npl_info = ble_npl_count_info_t {
            evt_count: 0,
            evtq_count: 0,
            co_count: 0,
            sem_count: 0,
            mutex_count: 0,
        };
        let res = ble_get_npl_element_info(
            &cfg as *const esp_bt_controller_config_t,
            &npl_info as *const ble_npl_count_info_t,
        );
        if res != 0 {
            panic!("ble_get_npl_element_info returned {}", res);
        }
        // not really using npl_info here ... remove it?

        #[cfg(esp32c2)]
        {
            // Initialize the global memory pool
            let ret = os_msys_buf_alloc();
            if !ret {
                panic!("os_msys_buf_alloc failed");
            }

            os_msys_init();
        }

        crate::common_adapter::chip_specific::phy_enable();

        // init bb
        bt_bb_v2_init_cmplx(1);

        #[cfg(coex)]
        {
            let rc = ble_osi_coex_funcs_register(&G_COEX_FUNCS as *const osi_coex_funcs_t);
            if rc != 0 {
                panic!("ble_osi_coex_funcs_register returned {}", rc);
            }
        }

        let res = ble_controller_init(&cfg as *const esp_bt_controller_config_t);

        if res != 0 {
            panic!("ble_controller_init returned {}", res);
        }

        #[cfg(not(esp32c2))]
        {
            extern "C" {
                fn esp_ble_msys_init(
                    msys_size1: u16,
                    msys_size2: u16,
                    msys_cnt1: u16,
                    msys_cnt2: u16,
                    from_heap: u8,
                ) -> i32;
            }

            let res = esp_ble_msys_init(256, 320, 12, 24, 1);
            if res != 0 {
                panic!("esp_ble_msys_init returned {}", res);
            }
        }

        #[cfg(coex)]
        crate::binary::include::coex_enable();

        let mut mac = [0u8; 6];
        crate::common_adapter::read_mac(mac.as_mut_ptr(), 2);
        mac.reverse();

        esp_ble_ll_set_public_addr(&mac as *const u8);

        r_ble_hci_trans_cfg_hs(
            Some(ble_hs_hci_rx_evt),
            core::ptr::null(),
            Some(ble_hs_rx_data),
            core::ptr::null(),
        );

        let res = ble_controller_enable(1); // 1 = BLE
        if res != 0 {
            panic!("ble_controller_enable returned {}", res);
        }

        // "patch" r_ble_ll_random - it needs syscall_table_ptr
        // probably long term we should rather initialize syscall_table_ptr
        #[cfg(esp32c2)]
        {
            *(r_ble_stub_funcs_ptr.offset(0x7dc / 4)) =
                self::ble_os_adapter_chip_specific::ble_ll_random_override as *const u32 as u32;
        }

        // this is to avoid (ASSERT r_ble_hci_ram_hs_cmd_tx:34 0 0)
        // r_ble_hci_trans_cfg_ll initializes ble_hci_trans_env_p + 0x34
        // it's called by r_ble_ll_task which is run in another task
        // in r_ble_hci_ram_hs_cmd_tx this is checked for non-null
        while (ble_hci_trans_env_p as *const u32)
            .add(0x34 / 4)
            .read_volatile()
            == 0
        {
            // wait
        }

        debug!("The ble_controller_init was initialized");
    }
}

#[cfg(esp32c2)]
fn os_msys_buf_alloc() -> bool {
    unsafe {
        OS_MSYS_INIT_1_DATA = crate::compat::malloc::calloc(
            1,
            core::mem::size_of::<OsMembufT>() * SYSINIT_MSYS_1_MEMPOOL_SIZE,
        ) as *mut u32;
        OS_MSYS_INIT_2_DATA = crate::compat::malloc::calloc(
            1,
            core::mem::size_of::<OsMembufT>() * SYSINIT_MSYS_2_MEMPOOL_SIZE,
        ) as *mut u32;

        !(OS_MSYS_INIT_1_DATA.is_null() || OS_MSYS_INIT_2_DATA.is_null())
    }
}

#[cfg(esp32c2)]
fn os_msys_init() {
    static MSYS1: &[u8] = b"msys_1\0";
    static MSYS2: &[u8] = b"msys_2\0";

    unsafe {
        r_os_msys_reset();

        let rc = r_mem_init_mbuf_pool(
            OS_MSYS_INIT_1_DATA as *const c_void,
            addr_of!(OS_MSYS_INIT_1_MEMPOOL),
            addr_of!(OS_MSYS_INIT_1_MBUF_POOL),
            OS_MSYS_1_BLOCK_COUNT,
            SYSINIT_MSYS_1_MEMBLOCK_SIZE,
            MSYS1 as *const _ as *const u8,
        );
        if rc != 0 {
            panic!("r_mem_init_mbuf_pool failed");
        }

        let rc = r_os_msys_register(addr_of!(OS_MSYS_INIT_1_MBUF_POOL));
        if rc != 0 {
            panic!("r_os_msys_register failed");
        }

        let rc = r_mem_init_mbuf_pool(
            OS_MSYS_INIT_2_DATA as *const c_void,
            addr_of!(OS_MSYS_INIT_2_MEMPOOL),
            addr_of!(OS_MSYS_INIT_2_MBUF_POOL),
            OS_MSYS_2_BLOCK_COUNT,
            SYSINIT_MSYS_2_MEMBLOCK_SIZE,
            MSYS2 as *const _ as *const u8,
        );
        if rc != 0 {
            panic!("r_mem_init_mbuf_pool failed");
        }

        let rc = r_os_msys_register(addr_of!(OS_MSYS_INIT_2_MBUF_POOL));
        if rc != 0 {
            panic!("r_os_msys_register failed");
        }
    }
}

unsafe extern "C" fn ble_hs_hci_rx_evt(cmd: *const u8, arg: *const c_void) {
    trace!("ble_hs_hci_rx_evt {:?} {:?}", cmd, arg);
    debug!("$ cmd = {:x}", *cmd);
    debug!("$ len = {:x}", *(cmd.offset(1)));

    let event = *cmd;
    let len = *(cmd.offset(1)) as usize;
    let payload = core::slice::from_raw_parts(cmd.offset(2), len);
    debug!("$ pld = {:?}", payload);

    critical_section::with(|cs| {
        let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        let mut data = [0u8; 256];

        data[0] = 0x04; // this is an event
        data[1] = event;
        data[2] = len as u8;
        data[3..][..len].copy_from_slice(payload);

        if queue
            .enqueue(ReceivedPacket {
                len: (len + 3) as u8,
                data,
            })
            .is_err()
        {
            warn!("Dropping BLE packet");
        }

        dump_packet_info(&data[..(len + 3)]);
    });

    r_ble_hci_trans_buf_free(cmd);

    #[cfg(feature = "async")]
    crate::ble::controller::asynch::hci_read_data_available();
}

unsafe extern "C" fn ble_hs_rx_data(om: *const OsMbuf, arg: *const c_void) {
    trace!("ble_hs_rx_data {:?} {:?}", om, arg);

    let data_ptr = (*om).om_data;
    let len = (*om).om_len;
    let data_slice = core::slice::from_raw_parts(data_ptr, len as usize);

    critical_section::with(|cs| {
        let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        let mut data = [0u8; 256];

        data[0] = 0x02; // ACL
        data[1..][..data_slice.len()].copy_from_slice(data_slice);

        if queue
            .enqueue(ReceivedPacket {
                len: (len + 1) as u8,
                data,
            })
            .is_err()
        {
            warn!("Dropping BLE packet");
        }

        dump_packet_info(&data[..(len + 1) as usize]);
    });

    r_os_mbuf_free_chain(om as *mut _);

    #[cfg(feature = "async")]
    crate::ble::controller::asynch::hci_read_data_available();
}

static mut BLE_HCI_READ_DATA: [u8; 256] = [0u8; 256];
static mut BLE_HCI_READ_DATA_INDEX: usize = 0;
static mut BLE_HCI_READ_DATA_LEN: usize = 0;

#[cfg(feature = "async")]
pub fn have_hci_read_data() -> bool {
    critical_section::with(|cs| {
        let queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        !queue.is_empty()
            || unsafe {
                BLE_HCI_READ_DATA_LEN > 0 && (BLE_HCI_READ_DATA_LEN >= BLE_HCI_READ_DATA_INDEX)
            }
    })
}

pub(crate) fn read_next(data: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

        match queue.dequeue() {
            Some(packet) => {
                data[..packet.len as usize].copy_from_slice(&packet.data[..packet.len as usize]);
                packet.len as usize
            }
            None => 0,
        }
    })
}

pub fn read_hci(data: &mut [u8]) -> usize {
    unsafe {
        if BLE_HCI_READ_DATA_LEN == 0 {
            critical_section::with(|cs| {
                let mut queue = BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

                if let Some(packet) = queue.dequeue() {
                    BLE_HCI_READ_DATA[..packet.len as usize]
                        .copy_from_slice(&packet.data[..packet.len as usize]);
                    BLE_HCI_READ_DATA_LEN = packet.len as usize;
                    BLE_HCI_READ_DATA_INDEX = 0;
                }
            });
        }

        if BLE_HCI_READ_DATA_LEN > 0 {
            data[0] = BLE_HCI_READ_DATA[BLE_HCI_READ_DATA_INDEX];
            BLE_HCI_READ_DATA_INDEX += 1;

            if BLE_HCI_READ_DATA_INDEX >= BLE_HCI_READ_DATA_LEN {
                BLE_HCI_READ_DATA_LEN = 0;
                BLE_HCI_READ_DATA_INDEX = 0;
            }
            return 1;
        }
    }

    0
}

pub fn send_hci(data: &[u8]) {
    let hci_out = unsafe { &mut *HCI_OUT_COLLECTOR.as_mut_ptr() };
    hci_out.push(data);

    if hci_out.is_ready() {
        let packet = hci_out.packet();

        unsafe {
            const DATA_TYPE_COMMAND: u8 = 1;
            const DATA_TYPE_ACL: u8 = 2;

            dump_packet_info(packet);

            critical_section::with(|_cs| {
                if packet[0] == DATA_TYPE_COMMAND {
                    let cmd = r_ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_CMD);
                    core::ptr::copy_nonoverlapping(
                        &packet[1] as *const _ as *mut u8, // don't send the TYPE
                        cmd as *mut u8,
                        packet.len() - 1,
                    );

                    let res = unwrap!(ble_hci_trans_funcs_ptr.ble_hci_trans_hs_cmd_tx)(cmd);

                    if res != 0 {
                        warn!("ble_hci_trans_hs_cmd_tx res == {}", res);
                    }
                } else if packet[0] == DATA_TYPE_ACL {
                    let om = r_os_msys_get_pkthdr(
                        packet.len() as u16,
                        ACL_DATA_MBUF_LEADINGSPACE as u16,
                    );

                    let res =
                        r_os_mbuf_append(om, packet.as_ptr().offset(1), (packet.len() - 1) as u16);
                    if res != 0 {
                        panic!("r_os_mbuf_append returned {}", res);
                    }

                    // this modification of the ACL data packet makes it getting sent and
                    // received by the other side
                    *((*om).om_data as *mut u8).offset(1) = 0;

                    let res = unwrap!(ble_hci_trans_funcs_ptr.ble_hci_trans_hs_acl_tx)(om);
                    if res != 0 {
                        panic!("ble_hci_trans_hs_acl_tx returned {}", res);
                    }
                    trace!("ACL tx done");
                }
            });
        }

        hci_out.reset();
    }
}

#[allow(unreachable_code, unused_variables)]
fn dump_packet_info(buffer: &[u8]) {
    #[cfg(not(feature = "dump-packets"))]
    return;

    critical_section::with(|cs| {
        info!("@HCIFRAME {:?}", buffer);
    });
}
