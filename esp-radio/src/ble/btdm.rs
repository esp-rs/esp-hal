use alloc::boxed::Box;
use core::ptr::{addr_of, addr_of_mut};

use esp_phy::{PhyController, PhyInitGuard};
use esp_sync::RawMutex;
use esp_wifi_sys::c_types::*;
use portable_atomic::{AtomicBool, Ordering};

use super::{Config, ReceivedPacket};
use crate::{
    binary::include::*,
    ble::{
        HCI_OUT_COLLECTOR,
        HciOutCollector,
        btdm::ble_os_adapter_chip_specific::{G_OSI_FUNCS, osi_funcs_s},
    },
    compat::common::str_from_c,
    hal::ram,
};

#[cfg_attr(esp32c3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32s3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32, path = "os_adapter_esp32.rs")]
pub(crate) mod ble_os_adapter_chip_specific;

static PACKET_SENT: AtomicBool = AtomicBool::new(true);

#[repr(C)]
struct VhciHostCallbacks {
    // callback used to notify that the host can
    // send packet to controller
    notify_host_send_available: extern "C" fn(),
    // callback used to notify that the
    // controller has a packet to send to
    // the host
    notify_host_recv: extern "C" fn(*mut u8, u16) -> i32,
}

unsafe extern "C" {
    fn btdm_osi_funcs_register(osi_funcs: *const osi_funcs_s) -> i32;
    fn btdm_controller_get_compile_version() -> *const c_char;

    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_controller_init(config_opts: *const esp_bt_controller_config_t) -> i32;

    #[cfg(esp32)]
    fn btdm_controller_init(
        config_mask: u32,
        config_opts: *const esp_bt_controller_config_t,
    ) -> i32;

    fn btdm_controller_enable(mode: esp_bt_mode_t);

    fn API_vhci_host_check_send_available() -> bool;
    fn API_vhci_host_send_packet(data: *const u8, len: u16);
    fn API_vhci_host_register_callback(vhci_host_callbac: *const VhciHostCallbacks) -> i32;

    #[cfg(not(esp32))]
    fn coex_pti_v2();
}

static VHCI_HOST_CALLBACK: VhciHostCallbacks = VhciHostCallbacks {
    notify_host_send_available,
    notify_host_recv,
};

extern "C" fn notify_host_send_available() {
    trace!("notify_host_send_available");

    PACKET_SENT.store(true, Ordering::Relaxed);
}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    trace!("notify_host_recv {:?} {}", data, len);

    let data = unsafe { core::slice::from_raw_parts(data, len as usize) };

    let packet = ReceivedPacket {
        data: Box::from(data),
    };

    super::BT_STATE.with(|state| state.rx_queue.push_back(packet));

    super::dump_packet_info(data);

    crate::ble::controller::asynch::hci_read_data_available();

    0
}

// This is fine, we're only accessing it inside a critical section (protected by INTERRUPT_LOCK).
static mut G_INTER_FLAGS: heapless::Vec<esp_sync::RestoreState, 10> = heapless::Vec::new();

static INTERRUPT_LOCK: RawMutex = RawMutex::new();

#[ram]
unsafe extern "C" fn interrupt_enable() {
    #[allow(static_mut_refs)]
    unsafe {
        let flags = unwrap!(
            G_INTER_FLAGS.pop(),
            "interrupt_enable called without prior interrupt_disable"
        );
        trace!("interrupt_enable {:?}", flags);
        INTERRUPT_LOCK.release(flags);
    }
}

#[ram]
unsafe extern "C" fn interrupt_disable() {
    trace!("interrupt_disable");
    #[allow(static_mut_refs)]
    unsafe {
        let flags = INTERRUPT_LOCK.acquire();
        unwrap!(
            G_INTER_FLAGS.push(flags),
            "interrupt_disable was called too many times"
        );
        trace!("interrupt_disable {:?}", flags);
    }
}

#[ram]
unsafe extern "C" fn task_yield() {
    todo!();
}

unsafe extern "C" fn task_yield_from_isr() {
    // This is not called because we never set xHigherPriorityTaskWoken = true in the `_from_isr`
    // functions. This should be revisited if a scheduler needs it.
    crate::preempt::yield_task_from_isr();
}

unsafe extern "C" fn mutex_create() -> *const () {
    todo!();
}

unsafe extern "C" fn mutex_delete(_mutex: *const ()) {
    todo!();
}

unsafe extern "C" fn mutex_lock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn mutex_unlock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn task_create(
    func: *mut crate::binary::c_types::c_void,
    name_ptr: *const c_char,
    stack_depth: u32,
    param: *mut crate::binary::c_types::c_void,
    prio: u32,
    handle: *mut crate::binary::c_types::c_void,
    core_id: u32,
) -> i32 {
    let name = unsafe { str_from_c(name_ptr) };
    trace!(
        "task_create {:?} {:?} {} {} {:?} {} {:?} {}",
        func, name_ptr, name, stack_depth, param, prio, handle, core_id
    );

    unsafe {
        let task_func = core::mem::transmute::<
            *mut crate::binary::c_types::c_void,
            extern "C" fn(*mut esp_wifi_sys::c_types::c_void),
        >(func);

        let task = crate::preempt::task_create(
            name,
            task_func,
            param,
            prio,
            if core_id < 2 { Some(core_id) } else { None },
            stack_depth as usize,
        );
        *(handle as *mut usize) = task as usize;
    }

    1
}

unsafe extern "C" fn task_delete(task: *mut ()) {
    trace!("task delete called for {:?}", task);

    unsafe {
        crate::preempt::schedule_task_deletion(task.cast());
    }
}

#[ram]
unsafe extern "C" fn is_in_isr() -> i32 {
    crate::is_interrupts_disabled() as i32
}

#[cfg(esp32)]
#[ram]
unsafe extern "C" fn cause_sw_intr_to_core(_core: i32, _intr_no: i32) -> i32 {
    trace!("cause_sw_intr_to_core {} {}", _core, _intr_no);
    unsafe { xtensa_lx_rt::xtensa_lx::interrupt::set(1 << _intr_no) };
    0
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn srand(seed: u32) {
    debug!("!!!! unimplemented srand {}", seed);
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn rand() -> i32 {
    trace!("rand");
    unsafe { crate::common_adapter::random() as i32 }
}

#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(_cycles: u32, _error_corr: u32) -> u32 {
    todo!();
}

#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(us: u32) -> u32 {
    const RTC_CLK_CAL_FRACT: u32 = 19;
    let g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
    let g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

    // Converts a duration in half us into a number of low power clock cycles.
    let cycles: u64 = (us as u64) << (g_btdm_lpcycle_us_frac as u64 / g_btdm_lpcycle_us as u64);
    trace!("btdm_hus_2_lpcycles {} {}", us, cycles);

    cycles as u32
}

unsafe extern "C" fn btdm_sleep_check_duration(_slot_cnt: i32) -> i32 {
    todo!();
}

unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: i32) {
    todo!();
}

unsafe extern "C" fn btdm_sleep_enter_phase2() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase1() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase2() {
    todo!();
}

unsafe extern "C" fn btdm_sleep_exit_phase3() {
    todo!();
}

unsafe extern "C" fn coex_schm_status_bit_set(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_set {} {}", _typ, status);
    #[cfg(coex)]
    unsafe {
        crate::binary::include::coex_schm_status_bit_set(_typ as u32, status as u32)
    };
}

unsafe extern "C" fn coex_schm_status_bit_clear(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_clear {} {}", _typ, status);
    #[cfg(coex)]
    unsafe {
        crate::binary::include::coex_schm_status_bit_clear(_typ as u32, status as u32)
    };
}

#[ram]
unsafe extern "C" fn read_efuse_mac(mac: *const ()) -> i32 {
    unsafe { crate::common_adapter::read_mac(mac as *mut _, 2) }
}

#[cfg(esp32)]
unsafe extern "C" fn set_isr13(n: i32, handler: unsafe extern "C" fn(), arg: *const ()) -> i32 {
    unsafe { ble_os_adapter_chip_specific::set_isr(n, handler, arg) }
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_disable() {
    // info!("unimplemented interrupt_l3_disable");
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_restore() {
    //  info!("unimplemented interrupt_l3_restore");
}

#[cfg(esp32)]
unsafe extern "C" fn custom_queue_create(
    _len: u32,
    _item_size: u32,
) -> *mut crate::binary::c_types::c_void {
    todo!();
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    let phy_init_guard;
    unsafe {
        (*addr_of_mut!(HCI_OUT_COLLECTOR)).write(HciOutCollector::new());
        // turn on logging
        #[allow(static_mut_refs)]
        #[cfg(feature = "sys-logs")]
        {
            unsafe extern "C" {
                static mut g_bt_plf_log_level: u32;
            }

            debug!("g_bt_plf_log_level = {}", g_bt_plf_log_level);
            g_bt_plf_log_level = 10;
        }

        // esp32_bt_controller_init
        ble_os_adapter_chip_specific::btdm_controller_mem_init();

        let mut cfg = ble_os_adapter_chip_specific::create_ble_config(config);

        let res = btdm_osi_funcs_register(addr_of!(G_OSI_FUNCS));
        assert!(res == 0, "btdm_osi_funcs_register returned {}", res);

        #[cfg(coex)]
        {
            let res = crate::wifi::coex_init();
            assert!(res == 0, "coex_init failed");
        }

        let version = btdm_controller_get_compile_version();
        let version_str = str_from_c(version);
        debug!("BT controller compile version {}", version_str);

        ble_os_adapter_chip_specific::bt_periph_module_enable();

        ble_os_adapter_chip_specific::disable_sleep_mode();

        #[cfg(any(esp32c3, esp32s3))]
        let res = btdm_controller_init(&mut cfg as *mut esp_bt_controller_config_t);

        #[cfg(esp32)]
        let res = btdm_controller_init(
            (1 << 3) | (1 << 4),
            &mut cfg as *mut esp_bt_controller_config_t,
        ); // see btdm_config_mask_load for mask

        assert!(res == 0, "btdm_controller_init returned {}", res);

        debug!("The btdm_controller_init was initialized");

        #[cfg(coex)]
        crate::binary::include::coex_enable();

        phy_init_guard = esp_hal::peripherals::BT::steal().enable_phy();

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                unsafe extern "C" {
                    fn btdm_rf_bb_init_phase2();
                }

                btdm_rf_bb_init_phase2();
                coex_bt_high_prio();
            } else {
                coex_pti_v2();
            }
        }

        #[cfg(coex)]
        coex_enable();

        btdm_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BLE);

        API_vhci_host_register_callback(&VHCI_HOST_CALLBACK);
    }

    // At some point the "High-speed ADC" entropy source became available.
    unsafe { esp_hal::rng::TrngSource::increase_entropy_source_counter() };
    phy_init_guard
}

pub(crate) fn ble_deinit() {
    esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
        esp_hal::Internal::conjure()
    });

    unsafe extern "C" {
        fn btdm_controller_deinit();
    }

    unsafe {
        btdm_controller_deinit();
    }
    // Disabling the PHY happens automatically, when the BLEController gets dropped.
}
/// Sends HCI data to the BLE controller.
#[instability::unstable]
pub fn send_hci(data: &[u8]) {
    let hci_out = unsafe { (*addr_of_mut!(HCI_OUT_COLLECTOR)).assume_init_mut() };
    hci_out.push(data);

    if hci_out.is_ready() {
        let packet = hci_out.packet();

        unsafe {
            loop {
                let can_send = API_vhci_host_check_send_available();

                if !can_send {
                    trace!("can_send is false");
                    continue;
                }

                PACKET_SENT.store(false, Ordering::Relaxed);

                #[cfg(all(esp32, coex))]
                ble_os_adapter_chip_specific::async_wakeup_request(
                    ble_os_adapter_chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI,
                );

                API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);

                #[cfg(all(esp32, coex))]
                ble_os_adapter_chip_specific::async_wakeup_request_end(
                    ble_os_adapter_chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI,
                );

                trace!("sent vhci host packet");

                super::dump_packet_info(packet);

                break;
            }

            // make sure the packet buffer doesn't get touched until sent
            while !PACKET_SENT.load(Ordering::Relaxed) {}
        }

        hci_out.reset();
    }
}
