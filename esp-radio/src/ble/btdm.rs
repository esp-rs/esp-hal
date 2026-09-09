use alloc::boxed::Box;
use core::ptr::{NonNull, addr_of, addr_of_mut};

use esp_phy::PhyInitGuard;
use esp_sync::RawMutex;
use portable_atomic::{AtomicBool, Ordering};

use super::{Config, ReceivedPacket};
use crate::{
    ble::{
        HCI_OUT_COLLECTOR,
        HciOutCollector,
        btdm::ble_os_adapter_chip_specific::{G_OSI_FUNCS, osi_funcs_s},
    },
    compat::common::str_from_c,
    hal::ram,
    sys::{c_types::*, include::*},
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

    // BLE modem-sleep symbols (in libbtdm_app.a; GC'd until referenced). Used by
    // ble_init (lp-clock setup) and the btdm_sleep_* OS callbacks below.
    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_lpclk_select_src(sel: u32) -> bool;
    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_lpclk_set_div(div: u32) -> bool;
    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_controller_get_sleep_mode() -> u8;
    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_sleep_clock_sync() -> bool;
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

    crate::ble::controller::hci_read_data_available();

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
    func: *mut crate::sys::c_types::c_void,
    name_ptr: *const c_char,
    stack_depth: u32,
    param: *mut crate::sys::c_types::c_void,
    prio: u32,
    handle: *mut crate::sys::c_types::c_void,
    core_id: u32,
) -> i32 {
    let name = unsafe { str_from_c(name_ptr) };
    trace!(
        "task_create {:?} {:?} {} {} {:?} {} {:?} {}",
        func, name_ptr, name, stack_depth, param, prio, handle, core_id
    );

    unsafe {
        let task_func = core::mem::transmute::<
            *mut crate::sys::c_types::c_void,
            extern "C" fn(*mut crate::sys::c_types::c_void),
        >(func);

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

// Sleep-clock calibration in Q19 fixed point (mirrors bt.c btdm_lpcycle_us).
const G_BTDM_LPCYCLE_US_FRAC: u32 = 19;
// Runtime us-per-lp-cycle, set once in ble_init from the selected sleep clock.
// MAIN_XTAL @ 1 MHz is 1 << 19. Values fit u32 (min ~136 kHz RTC is ~3.86M).
static G_BTDM_LPCYCLE_US: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(1 << G_BTDM_LPCYCLE_US_FRAC);

// Convert lp-cycles to half-microseconds, carrying the fractional remainder in
// *error_corr. The slot is typed u32 but the controller passes a pointer there
// (ABI-compatible on 32-bit). Ported from bt.c btdm_lpcycles_2_hus.
#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(cycles: u32, error_corr: u32) -> u32 {
    let error_corr = error_corr as *mut u32;
    let mut local: u64 = if error_corr.is_null() {
        0
    } else {
        unsafe { *error_corr as u64 }
    };
    let lpcycle_us = G_BTDM_LPCYCLE_US.load(core::sync::atomic::Ordering::Relaxed) as u64;
    local += lpcycle_us * (cycles as u64) * 2;
    let res = local >> G_BTDM_LPCYCLE_US_FRAC;
    local -= res << G_BTDM_LPCYCLE_US_FRAC;
    if !error_corr.is_null() {
        unsafe { *error_corr = local as u32 };
    }
    res as u32
}

/// Convert a duration in half-us into low-power clock cycles. Ported from bt.c.
#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(hus: u32) -> u32 {
    let lpcycle_us = G_BTDM_LPCYCLE_US.load(core::sync::atomic::Ordering::Relaxed) as u64;
    let mut cycles: u64 = ((hus as u64) << G_BTDM_LPCYCLE_US_FRAC) / lpcycle_us;
    cycles >>= 1;
    cycles as u32
}

// BLE modem-sleep OS callbacks (ported from bt.c). PHY_ENABLED mirrors bt.c
// s_lp_stat.phy_enabled so the shared PHY is disabled/enabled once per sleep
// cycle. With a BLE-only build, gating the whole PHY during controller sleep is safe.
static PHY_ENABLED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);

// True only for a sleep clock accurate enough to let the SoC light-sleep between BLE
// events (EXT_32K, RTC_SLOW, or MAIN_XTAL kept powered). Set in ble_init. When false
// the enter/exit wrappers leave the WakeLock alone, so the SoC stays awake.
static SLEEP_CLOCK_LIGHT_SLEEP: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

const BTDM_MIN_SLEEP_DURATION: i32 = 24; // half-slots; below this, don't sleep
const BTDM_MODEM_WAKE_UP_DELAY: i32 = 8; // half-slots; wake early to re-enable PHY/RF

// The slot is typed fn(i32)->i32 but the controller passes *mut i32 (the half-slot
// count) and expects a bool return (ABI-compatible on 32-bit).
unsafe extern "C" fn btdm_sleep_check_duration(half_slot_cnt: i32) -> i32 {
    let p = half_slot_cnt as *mut i32;
    let cnt = unsafe { *p };
    if cnt < BTDM_MIN_SLEEP_DURATION {
        return 0; // false: window too short to enter modem sleep
    }
    unsafe { *p = cnt - BTDM_MODEM_WAKE_UP_DELAY };
    1 // true
}

// Modem-only sleep needs no wakeup timer (that is the CONFIG_PM system-light-sleep
// path); bt.c returns immediately when wakeup_timer_required == 0.
unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: i32) {}

// Power the RF/baseband down for the sleep window.
unsafe extern "C" fn btdm_sleep_enter_phase2() {
    if unsafe { btdm_controller_get_sleep_mode() } == 1
        && PHY_ENABLED.swap(false, core::sync::atomic::Ordering::AcqRel)
    {
        esp_phy::disable_phy();
        // The controller is sleeping the modem for the whole gap, so release the wake
        // lock; the esp-rtos idle hook may light-sleep the SoC until the pre-event wake.
        // Re-acquired in exit_phase3.
        if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
            esp_hal::rtc_cntl::WakeLock::release();
        }
    }
}

// exit_phase1/2 are NULL in bt.c's OSI table; never invoked.
unsafe extern "C" fn btdm_sleep_exit_phase1() {}

unsafe extern "C" fn btdm_sleep_exit_phase2() {}

// Re-power the RF/baseband on wake, then wait for the sleep FSM to resync.
unsafe extern "C" fn btdm_sleep_exit_phase3() {
    if unsafe { btdm_controller_get_sleep_mode() } == 1
        && !PHY_ENABLED.swap(true, core::sync::atomic::Ordering::AcqRel)
    {
        // Re-acquire the wake lock before the RF comes back, so the SoC cannot light-
        // sleep through the imminent event. Balances the enter_phase2 release, paired
        // with the PHY_ENABLED gate so the lock count stays matched.
        if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
            esp_hal::rtc_cntl::WakeLock::acquire();
        }
        // Balanced raw increment: enable_phy() returns an RAII guard; forget it
        // so the +1 persists until the matching enter_phase2 disable.
        core::mem::forget(esp_phy::enable_phy());
    }
    while unsafe { btdm_sleep_clock_sync() } {}
}

unsafe extern "C" fn coex_schm_status_bit_set(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_set {} {}", _typ, status);
    #[cfg(feature = "coex")]
    unsafe {
        crate::sys::include::coex_schm_status_bit_set(_typ as u32, status as u32)
    };
}

unsafe extern "C" fn coex_schm_status_bit_clear(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_clear {} {}", _typ, status);
    #[cfg(feature = "coex")]
    unsafe {
        crate::sys::include::coex_schm_status_bit_clear(_typ as u32, status as u32)
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
) -> *mut crate::sys::c_types::c_void {
    todo!();
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    let phy_init_guard;
    unsafe {
        (*addr_of_mut!(HCI_OUT_COLLECTOR)).write(HciOutCollector::new());
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
        ble_os_adapter_chip_specific::btdm_controller_mem_init();

        let mut cfg = ble_os_adapter_chip_specific::create_ble_config(config);

        let res = btdm_osi_funcs_register(addr_of!(G_OSI_FUNCS));
        assert!(res == 0, "btdm_osi_funcs_register returned {}", res);

        #[cfg(feature = "coex")]
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

        // BLE modem-sleep LP-clock setup from the selected sleep clock (mirrors bt.c
        // controller_init). Runs after init (cfg has sleep_mode=1), before enable.
        #[cfg(any(esp32c3, esp32s3))]
        {
            use ble_os_adapter_chip_specific::BleSleepClock;
            const BTDM_LPCLK_SEL_XTAL: u32 = 0;
            const BTDM_LPCLK_SEL_XTAL32K: u32 = 1;
            const BTDM_LPCLK_SEL_RTC_SLOW: u32 = 2;
            // (sel, div, clk_hz after div, allows-SoC-light-sleep)
            let (sel, div, clk_hz, light_sleep): (u32, u32, u32, bool) =
                match config.sleep_clock_src() {
                    // Modem gates between events but the SoC stays awake: the main XTAL is
                    // powered down in light sleep, so it cannot clock the controller across
                    // a SoC sleep. The safe default without a 32k crystal.
                    BleSleepClock::MainXtal => (BTDM_LPCLK_SEL_XTAL, 40, 1_000_000, false),
                    // MAIN_XTAL kept powered during light sleep (ESP-IDF main_xtal_pu):
                    // same clock as MainXtal but light_sleep = true. Requires the board to
                    // also call esp-rtos set_main_xtal_powered_in_light_sleep(true), or the
                    // controller loses its clock in sleep and the connection drops.
                    BleSleepClock::MainXtalPu => (BTDM_LPCLK_SEL_XTAL, 40, 1_000_000, true),
                    // 32.768 kHz crystal, undivided: exact, no runtime calibration needed.
                    BleSleepClock::Ext32kXtal => (BTDM_LPCLK_SEL_XTAL32K, 0, 32_768, true),
                    // ~136 kHz RC nominal; real RC drifts ~7% (ESP-IDF: advertising/idle only).
                    BleSleepClock::RtcSlow => (BTDM_LPCLK_SEL_RTC_SLOW, 0, 136_000, true),
                };
            // us-per-lp-cycle in Q19 fixed point: (1_000_000 << FRAC) / clk_hz.
            let lpcycle_us = ((1_000_000u64 << G_BTDM_LPCYCLE_US_FRAC) / clk_hz as u64) as u32;
            G_BTDM_LPCYCLE_US.store(lpcycle_us, core::sync::atomic::Ordering::Relaxed);
            SLEEP_CLOCK_LIGHT_SLEEP.store(light_sleep, core::sync::atomic::Ordering::Relaxed);
            let sel_ok = btdm_lpclk_select_src(sel);
            let div_ok = if div > 0 { btdm_lpclk_set_div(div) } else { true };
            if light_sleep {
                // Baseline wake lock held during events; enter_phase2 releases it in each
                // gap and exit_phase3 re-acquires it, so the SoC only light-sleeps between
                // events.
                esp_hal::rtc_cntl::WakeLock::acquire();
            }
            debug!(
                "btdm modem-sleep lpclk sel={} div={} lpcycle_us={} light_sleep={} sel_ok={} div_ok={}",
                sel, div, lpcycle_us, light_sleep, sel_ok, div_ok
            );
        }

        #[cfg(feature = "coex")]
        crate::sys::include::coex_enable();

        phy_init_guard = esp_phy::enable_phy();

        cfg_select! {
            esp32 => {
                unsafe extern "C" {
                    fn btdm_rf_bb_init_phase2();
                }

                btdm_rf_bb_init_phase2();
                coex_bt_high_prio();
            }
            _ => {
                coex_pti_v2();
            }
        }

        #[cfg(feature = "coex")]
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

                #[cfg(all(esp32, feature = "coex"))]
                ble_os_adapter_chip_specific::async_wakeup_request(
                    ble_os_adapter_chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI,
                );

                API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);

                #[cfg(all(esp32, feature = "coex"))]
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
