use super::*;
use crate::binary::include::esp_bt_controller_config_t;
use log::trace;

pub(crate) static mut BT_INTERRUPT_FUNCTION5: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

pub(crate) static mut BT_INTERRUPT_FUNCTION8: (
    *mut crate::binary::c_types::c_void,
    *mut crate::binary::c_types::c_void,
) = (core::ptr::null_mut(), core::ptr::null_mut());

#[repr(C)]
pub(super) struct osi_funcs_s {
    magic: u32,
    version: u32,
    interrupt_set: Option<unsafe extern "C" fn(i32, i32, i32, i32) -> ()>,
    interrupt_clear: Option<unsafe extern "C" fn(i32, i32) -> ()>,
    interrupt_handler_set: Option<unsafe extern "C" fn(i32, extern "C" fn(), *const ()) -> ()>,
    interrupt_disable: Option<unsafe extern "C" fn() -> ()>,
    interrupt_enable: Option<unsafe extern "C" fn() -> ()>,
    task_yield: Option<unsafe extern "C" fn() -> ()>,
    task_yield_from_isr: Option<unsafe extern "C" fn() -> ()>,
    semphr_create: Option<unsafe extern "C" fn(u32, u32) -> *const ()>,
    semphr_delete: Option<unsafe extern "C" fn(*const ()) -> ()>,
    semphr_take_from_isr: Option<unsafe extern "C" fn(*const (), *const ()) -> i32>,
    semphr_give_from_isr: Option<unsafe extern "C" fn(*const (), *const ()) -> i32>,
    semphr_take: Option<unsafe extern "C" fn(*const (), u32) -> i32>,
    semphr_give: Option<unsafe extern "C" fn(*const ()) -> i32>,
    mutex_create: Option<unsafe extern "C" fn() -> *const ()>,
    mutex_delete: Option<unsafe extern "C" fn(*const ()) -> ()>,
    mutex_lock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    mutex_unlock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    queue_create: Option<unsafe extern "C" fn(u32, u32) -> *const ()>,
    queue_delete: Option<unsafe extern "C" fn(*const ()) -> ()>,
    queue_send: Option<unsafe extern "C" fn(*const (), *const (), u32) -> i32>,
    queue_send_from_isr: Option<unsafe extern "C" fn(*const (), *const (), *const ()) -> i32>,
    queue_recv: Option<unsafe extern "C" fn(*const (), *const (), u32) -> i32>,
    queue_recv_from_isr: Option<unsafe extern "C" fn(*const (), *const (), *const ()) -> i32>,
    task_create: Option<
        unsafe extern "C" fn(
            *mut crate::binary::c_types::c_void,
            *const u8,
            u32,
            *mut crate::binary::c_types::c_void,
            u32,
            *mut crate::binary::c_types::c_void,
            u32,
        ) -> i32,
    >,
    task_delete: Option<unsafe extern "C" fn(*const ()) -> ()>,
    is_in_isr: Option<unsafe extern "C" fn() -> i32>,
    cause_sw_intr_to_core: Option<unsafe extern "C" fn(i32, i32) -> i32>,
    malloc: Option<unsafe extern "C" fn(u32) -> *const ()>,
    malloc_internal: Option<unsafe extern "C" fn(u32) -> *const ()>,
    free: Option<unsafe extern "C" fn(*const ()) -> ()>,
    read_efuse_mac: Option<unsafe extern "C" fn(*const ()) -> i32>,
    srand: Option<unsafe extern "C" fn(u32) -> ()>,
    rand: Option<unsafe extern "C" fn() -> i32>,
    btdm_lpcycles_2_hus: Option<unsafe extern "C" fn(u32, u32) -> u32>,
    btdm_hus_2_lpcycles: Option<unsafe extern "C" fn(u32) -> u32>,
    btdm_sleep_check_duration: Option<unsafe extern "C" fn(i32) -> i32>,
    btdm_sleep_enter_phase1: Option<unsafe extern "C" fn(i32) -> ()>,
    btdm_sleep_enter_phase2: Option<unsafe extern "C" fn() -> ()>,
    btdm_sleep_exit_phase1: Option<unsafe extern "C" fn() -> ()>,
    btdm_sleep_exit_phase2: Option<unsafe extern "C" fn() -> ()>,
    btdm_sleep_exit_phase3: Option<unsafe extern "C" fn() -> ()>,
    coex_wifi_sleep_set: Option<unsafe extern "C" fn(i32) -> ()>,
    coex_core_ble_conn_dyn_prio_get: Option<unsafe extern "C" fn(*mut i32, *mut i32) -> i32>,
    coex_schm_status_bit_set: Option<unsafe extern "C" fn(i32, i32) -> ()>,
    coex_schm_status_bit_clear: Option<unsafe extern "C" fn(i32, i32) -> ()>,
    interrupt_on: Option<unsafe extern "C" fn(i32) -> ()>,
    interrupt_off: Option<unsafe extern "C" fn(i32) -> ()>,
    esp_hw_power_down: Option<unsafe extern "C" fn() -> ()>,
    esp_hw_power_up: Option<unsafe extern "C" fn() -> ()>,
    ets_backup_dma_copy: Option<unsafe extern "C" fn(u32, u32, u32, i32) -> ()>,
}

pub(super) static G_OSI_FUNCS: osi_funcs_s = osi_funcs_s {
    magic: 0xfadebead,
    version: 0x00010006,
    interrupt_set: Some(ble_os_adapter_chip_specific::interrupt_set),
    interrupt_clear: Some(ble_os_adapter_chip_specific::interrupt_clear),
    interrupt_handler_set: Some(ble_os_adapter_chip_specific::interrupt_handler_set),
    interrupt_disable: Some(interrupt_disable),
    interrupt_enable: Some(interrupt_enable),
    task_yield: Some(task_yield),
    task_yield_from_isr: Some(task_yield_from_isr),
    semphr_create: Some(semphr_create),
    semphr_delete: Some(semphr_delete),
    semphr_take_from_isr: Some(crate::common_adapter::semphr_take_from_isr),
    semphr_give_from_isr: Some(crate::common_adapter::semphr_give_from_isr),
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
    cause_sw_intr_to_core: Some(cause_sw_intr_to_core),
    malloc: Some(malloc),
    malloc_internal: Some(malloc_internal),
    free: Some(free),
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
    coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    interrupt_on: Some(ble_os_adapter_chip_specific::interrupt_on),
    interrupt_off: Some(ble_os_adapter_chip_specific::interrupt_off),
    esp_hw_power_down: Some(ble_os_adapter_chip_specific::esp_hw_power_down),
    esp_hw_power_up: Some(ble_os_adapter_chip_specific::esp_hw_power_up),
    ets_backup_dma_copy: Some(ble_os_adapter_chip_specific::ets_backup_dma_copy),
};

extern "C" {
    fn btdm_controller_rom_data_init() -> i32;
}

pub(crate) fn create_ble_config() -> esp_bt_controller_config_t {
    esp_bt_controller_config_t {
        magic: 0x5a5aa5a5,
        version: 0x02302140,
        controller_task_stack_size: 8192,
        controller_task_prio: 200,
        controller_task_run_cpu: 0,
        bluetooth_mode: 1,
        ble_max_act: 10,
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
        txant_dft: 0,
        rxant_dft: 0,
        txpwr_dft: 9,
        cfg_mask: 1,
        scan_duplicate_mode: 0,
        scan_duplicate_type: 0,
        normal_adv_size: 20,
        mesh_adv_size: 0,
        coex_phy_coded_tx_rx_time_limit: 0,
        hw_target_code: 0x01010000,
        slave_ce_len_min: 5,
        hw_recorrect_en: 1 << 0,
        cca_thresh: 20,
        dup_list_refresh_period: 0,
        scan_backoff_upperlimitmax: 0,
        ble_50_feat_supp: true, // BT_CTRL_50_FEATURE_SUPPORT
    }
}

pub(crate) unsafe extern "C" fn interrupt_on(intr_num: i32) {
    trace!("interrupt_on {}", intr_num);

    // NO-OP
}

pub(crate) unsafe extern "C" fn interrupt_off(_intr_num: i32) {
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
    interrupt_no: i32,
    interrupt_prio: i32,
) {
    trace!(
        "interrupt_set {} {} {} {}",
        cpu_no,
        intr_source,
        interrupt_no,
        interrupt_prio
    );

    /* Set the interrupt type (Edge or Level). */
    /* Map the CPU interrupt ID to the peripheral. */

    // NO-OP
}

pub(crate) unsafe extern "C" fn interrupt_clear(_interrupt_source: i32, _interrupt_no: i32) {
    todo!();
}

pub(crate) unsafe extern "C" fn interrupt_handler_set(
    interrupt_no: i32,
    func: extern "C" fn(),
    arg: *const (),
) {
    trace!(
        "interrupt_handler_set {} {:p} {:p}",
        interrupt_no,
        func,
        arg
    );
    match interrupt_no {
        5 => {
            BT_INTERRUPT_FUNCTION5 = (
                func as *mut crate::binary::c_types::c_void,
                arg as *mut crate::binary::c_types::c_void,
            )
        }
        8 => {
            BT_INTERRUPT_FUNCTION8 = (
                func as *mut crate::binary::c_types::c_void,
                arg as *mut crate::binary::c_types::c_void,
            )
        }
        _ => panic!("Unsupported interrupt number {}", interrupt_no),
    }
}

pub(crate) unsafe extern "C" fn coex_wifi_sleep_set(sleep: i32) {
    log::debug!(
        "ignored coex_wifi_sleep_set {} - because original implementation does the same",
        sleep
    );
}

#[allow(unused_variables, dead_code)]
pub(crate) unsafe extern "C" fn coex_core_ble_conn_dyn_prio_get(
    low: *mut i32,
    high: *mut i32,
) -> i32 {
    extern "C" {
        fn coex_core_ble_conn_dyn_prio_get(low: *mut i32, high: *mut i32) -> i32;
    }
    log::debug!("coex_core_ble_conn_dyn_prio_get");

    #[cfg(coex)]
    return coex_core_ble_conn_dyn_prio_get(low, high);

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
