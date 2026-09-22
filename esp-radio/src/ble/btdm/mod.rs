use alloc::boxed::Box;
use core::{ptr::NonNull, task::Poll};

use esp_phy::PhyInitGuard;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use super::{Config, ReceivedPacket};
#[cfg(feature = "coex")]
use crate::sys::include;
use crate::{
    asynch::AtomicWaker,
    compat::common::str_from_c,
    hal::ram,
    sys::{c_types::*, include::*},
};

#[cfg_attr(esp32c3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32s3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32, path = "os_adapter_esp32.rs")]
pub(crate) mod chip_specific;

use chip_specific::{G_OSI_FUNCS, osi_funcs_s};

pub(crate) unsafe extern "C" fn malloc_internal(size: u32) -> *mut crate::sys::c_types::c_void {
    unsafe { crate::compat::malloc::malloc_internal(size as usize).cast() }
}

static PACKET_IN_FLIGHT: AtomicBool = AtomicBool::new(false);
static PACKET_SENT_WAKER: AtomicWaker = AtomicWaker::new();

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

    fn btdm_controller_init(
        #[cfg(esp32)] config_mask: u32,
        config_opts: *mut esp_bt_controller_config_t,
    ) -> i32;

    fn btdm_controller_enable(mode: esp_bt_mode_t);
    fn btdm_controller_deinit();

    fn API_vhci_host_check_send_available() -> bool;
    fn API_vhci_host_send_packet(data: *const u8, len: u16);
    fn API_vhci_host_register_callback(vhci_host_callbac: *const VhciHostCallbacks) -> i32;

    #[cfg(esp32)]
    fn btdm_rf_bb_init_phase2();

    #[cfg(not(esp32))]
    fn coex_pti_v2();
}

static VHCI_HOST_CALLBACK: VhciHostCallbacks = VhciHostCallbacks {
    notify_host_send_available,
    notify_host_recv,
};

extern "C" fn notify_host_send_available() {
    trace!("notify_host_send_available");

    PACKET_IN_FLIGHT.store(false, Ordering::Release);
    PACKET_SENT_WAKER.wake();
}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    trace!("notify_host_recv {:?} {}", data, len);

    let data = unsafe { core::slice::from_raw_parts(data, len as usize) };

    let packet = ReceivedPacket {
        data: Box::from(data),
    };

    super::BT_STATE.with(|state| state.rx_queue.push_back(packet));

    super::dump_packet_info(data);

    crate::ble::controller::hci_read_data_available();

    0
}

static CRITICAL_NEST: AtomicU32 = AtomicU32::new(0);
static CRITICAL_TOKEN: AtomicU32 = AtomicU32::new(0);

#[ram]
unsafe extern "C" fn interrupt_enable() {
    trace!("interrupt_enable");
    if CRITICAL_NEST.fetch_sub(1, Ordering::Release) == 1 {
        let last = CRITICAL_TOKEN.load(Ordering::Relaxed);

        unsafe {
            super::ESP_RADIO_LOCK.release(esp_sync::RestoreState::new(last));
        }
    }
}

#[ram]
unsafe extern "C" fn interrupt_disable() {
    trace!("interrupt_disable");
    let last = CRITICAL_NEST.fetch_add(1, Ordering::Release);
    if last == 0 {
        let token = unsafe { super::ESP_RADIO_LOCK.acquire().inner() };
        CRITICAL_TOKEN.store(token, Ordering::Relaxed);
    }
}

#[ram]
unsafe extern "C" fn task_yield() {
    crate::preempt::yield_task();
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
    func: *mut c_void,
    name_ptr: *const c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    handle: *mut c_void,
    core_id: u32,
) -> i32 {
    let name = unsafe { str_from_c(name_ptr) };
    trace!(
        "task_create {:?} {:?} {} {} {:?} {} {:?} {}",
        func, name_ptr, name, stack_depth, param, prio, handle, core_id
    );

    unsafe {
        let task_func = core::mem::transmute::<*mut c_void, extern "C" fn(*mut c_void)>(func);

        let task = crate::preempt::task_create(
            name,
            task_func,
            param,
            prio,
            if core_id < 2 { Some(core_id) } else { None },
            stack_depth as usize,
        );
        *(handle as *mut usize) = task.as_ptr() as usize;
    }

    1
}

unsafe extern "C" fn task_delete(task: *mut ()) {
    trace!("task delete called for {:?}", task);

    unsafe {
        crate::preempt::schedule_task_deletion(NonNull::new(task));
    }
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

const LP_CYCLE_US_FRAC: u32 = 19;
static LP_CYCLE_US: AtomicU32 = AtomicU32::new(1 << LP_CYCLE_US_FRAC);

unsafe extern "C" {
    fn btdm_lpclk_select_src(sel: u32) -> bool;
    fn btdm_lpclk_set_div(div: u32) -> bool;
    #[cfg(not(esp32))]
    fn btdm_sleep_clock_sync() -> u8;
    fn btdm_controller_enable_sleep(enable: bool);
    fn btdm_wakeup_request();
    fn btdm_in_wakeup_requesting_set(set: bool);
    fn btdm_power_state_active() -> bool;

    #[cfg(feature = "coex")]
    fn coex_update_lpclk_interval();
}

fn program_btdm_lpclk() {
    let params = super::lp_clk::btdm();
    LP_CYCLE_US.store(params.lpcycle_us, Ordering::Relaxed);
    unsafe {
        let selected = btdm_lpclk_select_src(params.select);
        let divided = btdm_lpclk_set_div(params.divider);
        assert!(selected && divided, "btdm_lpclk_select_src/set_div failed");
        #[cfg(feature = "coex")]
        coex_update_lpclk_interval();
    }
}

#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(cycles: u32, error_corr: u32) -> u32 {
    let lpcycle_us = LP_CYCLE_US.load(Ordering::Relaxed).max(1);
    cfg_select! {
        esp32 => {
            let _ = error_corr;
            let us = u64::from(lpcycle_us) * u64::from(cycles);
            ((us + (1 << (LP_CYCLE_US_FRAC - 1))) >> LP_CYCLE_US_FRAC) as u32
        }
        _ => {
            let error_corr = error_corr as *mut u32;
            let mut local = if error_corr.is_null() {
                0
            } else {
                unsafe { u64::from(*error_corr) }
            };
            let mut res = u64::from(lpcycle_us) * u64::from(cycles) * 2;
            local += res;
            res = local >> LP_CYCLE_US_FRAC;
            local -= res << LP_CYCLE_US_FRAC;
            if !error_corr.is_null() {
                unsafe { *error_corr = local as u32 };
            }
            res as u32
        }
    }
}

#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(us: u32) -> u32 {
    let lpcycle_us = u64::from(LP_CYCLE_US.load(Ordering::Relaxed).max(1));
    let cycles = (u64::from(us) << LP_CYCLE_US_FRAC) / lpcycle_us;
    cfg_select! {
        esp32 => cycles as u32,
        _ => (cycles >> 1) as u32,
    }
}

#[ram]
unsafe extern "C" fn btdm_sleep_check_duration(slot_cnt: i32) -> i32 {
    if !super::modem_sleep_enabled() {
        return 0;
    }

    let (min_sleep, wake_delay) = cfg_select! {
        esp32 => (12, 4),
        _ => (24, 8),
    };
    let slot_cnt = slot_cnt as *mut i32;
    let slots = unsafe { *slot_cnt };
    if slots < min_sleep {
        return 0;
    }
    unsafe { *slot_cnt = slots - wake_delay };
    1
}

unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: i32) {}

unsafe extern "C" fn btdm_sleep_enter_phase2() {
    if super::modem_sleep_enabled() {
        super::modem_phy_release();
    }
}

unsafe extern "C" fn btdm_sleep_exit_phase1() {}

unsafe extern "C" fn btdm_sleep_exit_phase2() {}

unsafe extern "C" fn btdm_sleep_exit_phase3() {
    if !super::modem_sleep_enabled() {
        return;
    }
    let phy_restored = super::modem_phy_acquire();
    cfg_select! {
        // The RF can be off since the last baseband initialization.
        esp32 => {
            if phy_restored {
                unsafe { btdm_rf_bb_init_phase2() };
            }
        }
        _ => {
            let _ = phy_restored;
            unsafe { while btdm_sleep_clock_sync() != 0 {} }
        }
    }
}

fn wake_controller_for_hci() {
    if !super::modem_sleep_enabled() {
        return;
    }
    unsafe {
        btdm_in_wakeup_requesting_set(true);
        if !btdm_power_state_active() {
            btdm_wakeup_request();
        }
    }
}

fn end_controller_hci_wake() {
    if !super::modem_sleep_enabled() {
        return;
    }
    unsafe { btdm_in_wakeup_requesting_set(false) };
}

/// Returns whether the controller has not yet taken the last HCI packet.
///
/// The send path requests a controller wake, and the controller takes the PHY again only later.
/// Until then, the PHY reference shows a sleeping controller.
pub(super) fn hci_packet_in_flight() -> bool {
    PACKET_IN_FLIGHT.load(Ordering::Acquire)
}

#[ram]
unsafe extern "C" fn read_efuse_mac(mac: *const ()) -> i32 {
    unsafe { crate::common_adapter::read_mac(mac as *mut _, 2) }
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    super::set_modem_sleep(config.modem_sleep());
    super::lp_clk::request();
    program_btdm_lpclk();

    let phy_init_guard;
    unsafe {
        // turn on logging
        #[allow(static_mut_refs)]
        #[cfg(feature = "print-logs-from-driver")]
        {
            unsafe extern "C" {
                static mut g_bt_plf_log_level: u32;
            }

            debug!("g_bt_plf_log_level = {}", g_bt_plf_log_level);
            g_bt_plf_log_level = 10;
        }

        // esp32_bt_controller_init
        chip_specific::btdm_controller_mem_init();

        let mut cfg = chip_specific::create_ble_config(config);

        let res = btdm_osi_funcs_register(&G_OSI_FUNCS);
        assert!(res == 0, "btdm_osi_funcs_register returned {}", res);

        #[cfg(feature = "coex")]
        {
            let res = crate::wifi::coex_init();
            assert!(res == 0, "coex_init failed");
        }

        let version = btdm_controller_get_compile_version();
        debug!("BT controller compile version {}", str_from_c(version));

        chip_specific::bt_periph_module_enable();

        chip_specific::disable_sleep_mode();

        let res = btdm_controller_init(
            #[cfg(esp32)]
            {
                // see btdm_config_mask_load for mask
                // const BTDM_CFG_BT_DATA_RELEASE: u32 = 1 << 0;
                // const BTDM_CFG_HCI_UART: u32 = 1 << 1;
                // const BTDM_CFG_CONTROLLER_RUN_APP_CPU: u32 = 1 << 2;
                const BTDM_CFG_SCAN_DUPLICATE_OPTIONS: u32 = 1 << 3;
                const BTDM_CFG_SEND_ADV_RESERVED_SIZE: u32 = 1 << 4;
                // const BTDM_CFG_BLE_FULL_SCAN_SUPPORTED: u32 = 1 << 5;
                BTDM_CFG_SCAN_DUPLICATE_OPTIONS | BTDM_CFG_SEND_ADV_RESERVED_SIZE
            },
            &mut cfg,
        );
        assert!(res == 0, "btdm_controller_init returned {}", res);

        #[cfg(feature = "coex")]
        include::coex_enable();

        phy_init_guard = esp_phy::enable_phy();

        cfg_select! {
            esp32 => {
                btdm_rf_bb_init_phase2();
                coex_bt_high_prio();
            }
            _ => {
                coex_pti_v2();
            }
        }

        btdm_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BLE);

        if config.modem_sleep() {
            btdm_controller_enable_sleep(true);
        }

        API_vhci_host_register_callback(&VHCI_HOST_CALLBACK);
    }

    if config.modem_sleep() {
        super::lp_clk::claim_wake_source();
    }

    // At some point the "High-speed ADC" entropy source became available.
    unsafe { esp_hal::rng::TrngSource::increase_entropy_source_counter() };
    phy_init_guard
}

pub(crate) fn ble_deinit() {
    super::lp_clk::release_wake_source();
    super::modem_phy_acquire();
    super::set_modem_sleep(false);

    esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
        esp_hal::Internal::conjure()
    });

    unsafe {
        btdm_controller_deinit();
    }

    super::lp_clk::release();
    // Disabling the PHY happens automatically, when the BLEController gets dropped.
}

pub(crate) fn send(data: &[u8]) {
    // make sure the packet buffer doesn't get touched until sent
    while PACKET_IN_FLIGHT.load(Ordering::Acquire) {}
    while unsafe { !API_vhci_host_check_send_available() } {
        trace!("can_send is false");
    }

    send_packet(data)
}

pub(crate) async fn send_async(data: &[u8]) {
    // make sure the packet buffer doesn't get touched until sent
    core::future::poll_fn(|cx| {
        PACKET_SENT_WAKER.register(cx.waker());
        if PACKET_IN_FLIGHT.load(Ordering::Acquire)
            || unsafe { !API_vhci_host_check_send_available() }
        {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    })
    .await;

    send_packet(data)
}

fn send_packet(packet: &[u8]) {
    unsafe {
        PACKET_IN_FLIGHT.store(true, Ordering::Relaxed);

        wake_controller_for_hci();

        #[cfg(all(esp32, feature = "coex"))]
        chip_specific::async_wakeup_request(chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI);

        API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);

        #[cfg(all(esp32, feature = "coex"))]
        chip_specific::async_wakeup_request_end(chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI);

        end_controller_hci_wake();
    }

    trace!("sent vhci host packet");

    super::dump_packet_info(packet);
}
