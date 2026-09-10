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
    fn btdm_controller_enable_sleep(enable: bool);
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

    // Track active connections from the controller->host HCI event stream so the SoC-wake
    // deadline (the pre-event margin) is armed ONLY while connected. Idle advertising does
    // not need it (the controller runs the adv events autonomously on the kept-alive XTAL),
    // so skipping it there lets the SoC light-sleep the whole advertising gap. HCI event =
    // [0x04][evt][len][params]; LE Connection Complete (0x3E/sub 0x01|0x0A, status 0) opens
    // a connection, Disconnection Complete (0x05) closes one.
    if data.len() >= 4 && data[0] == 0x04 {
        match data[1] {
            0x05 => {
                if BLE_CONN_COUNT.load(core::sync::atomic::Ordering::Relaxed) > 0
                    && BLE_CONN_COUNT.fetch_sub(1, core::sync::atomic::Ordering::Relaxed) == 1
                {
                    // last connection gone → advertising only; drop the Bt wake source.
                    #[cfg(any(esp32c3, esp32s3))]
                    esp_hal::rtc_cntl::sleep::disable_bt_wakeup();
                }
            }
            0x3E if data.len() >= 5 && matches!(data[3], 0x01 | 0x0A) && data[4] == 0x00 => {
                if BLE_CONN_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed) == 0 {
                    // first connection → arm the Bt wake source as the event backstop.
                    #[cfg(any(esp32c3, esp32s3))]
                    if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
                        esp_hal::rtc_cntl::sleep::enable_bt_wakeup();
                    }
                }
            }
            _ => {}
        }
    }

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

// Modem-sleep window diagnostics: [enter_phase1 calls, exit_phase3 wakes, sum of window
// us, min window us, max window us].
static MODEM_SLEEP_DIAG: [portable_atomic::AtomicU64; 5] = [
    portable_atomic::AtomicU64::new(0),
    portable_atomic::AtomicU64::new(0),
    portable_atomic::AtomicU64::new(0),
    portable_atomic::AtomicU64::new(u64::MAX),
    portable_atomic::AtomicU64::new(0),
];

/// Snapshot of the modem-sleep window diagnostics (see `MODEM_SLEEP_DIAG`).
pub(crate) fn modem_sleep_diag() -> [u64; 5] {
    core::array::from_fn(|i| MODEM_SLEEP_DIAG[i].load(core::sync::atomic::Ordering::Relaxed))
}

// bt.c: with CONFIG_PM the controller arms a wakeup timer slightly before the modem
// sleep window ends, so the SoC leaves light sleep and re-takes the pm lock before the
// controller wakes. Here the wake lock released in enter_phase2 is replaced by a sleep
// deadline: the idle hook sleeps at most until it, and refuses to sleep past it until
// exit_phase3 clears it. Modem-only sleep (SoC awake) needs none of this.
/// Pre-event wake margin: wake the SoC this far before the modem-sleep window ends so it is
/// fully out of light sleep in time to service the controller event. bt.c's
/// BTDM_MIN_TIMER_UNCERTAINTY_US = 1800; empirically load-bearing for a held CONNECTION
/// (events ~30 ms) — 800 µs let the SoC wake too late and the link dropped before service
/// discovery. The Bt wake source is a backstop, not a substitute for waking in time.
const MODEM_MIN_UNCERTAINTY_US: u32 = 1800;
/// `wake_in` (µs) computed by enter_phase1, consumed by enter_phase2 so the deadline is set
/// PAIRED with the wakelock release (and thus always cleared by the paired exit_phase3).
/// Setting it in enter_phase1 unconditionally left stale deadlines that held the SoC awake
/// between events whenever the enter_phase2/exit_phase3 pairing did not fire.
static PENDING_WAKE_IN_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Number of active BLE connections (from the HCI event stream). The SoC-wake deadline is
/// armed only while this is > 0; idle advertising skips it and light-sleeps the whole gap.
static BLE_CONN_COUNT: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(0);

#[ram]
unsafe extern "C" fn btdm_sleep_enter_phase1(lpcycles: i32) {
    if !SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
        return;
    }
    let us_to_sleep = unsafe { btdm_lpcycles_2_hus(lpcycles as u32, 0) } >> 1;
    let uncertainty = (us_to_sleep >> 11).max(MODEM_MIN_UNCERTAINTY_US);
    let wake_in = us_to_sleep.saturating_sub(uncertainty);
    {
        use core::sync::atomic::Ordering::Relaxed;
        MODEM_SLEEP_DIAG[0].fetch_add(1, Relaxed);
        MODEM_SLEEP_DIAG[2].fetch_add(us_to_sleep as u64, Relaxed);
        MODEM_SLEEP_DIAG[3].fetch_min(us_to_sleep as u64, Relaxed);
        MODEM_SLEEP_DIAG[4].fetch_max(us_to_sleep as u64, Relaxed);
    }
    // Don't set the deadline here — hand wake_in to enter_phase2, which sets it in the same
    // branch that releases the wakelock, so exit_phase3 always clears it.
    PENDING_WAKE_IN_US.store(wake_in, core::sync::atomic::Ordering::Relaxed);
}

// Power the RF/baseband down for the sleep window.
// Modem sleep is only wired on the c3/s3 BTDM controllers (the blob sleep symbols and
// the lp-clock setup are c3/s3-only). On esp32-classic these callbacks are never invoked
// (the controller is left in sleep_mode 0), so they are no-ops there, which also keeps the
// esp32-classic build free of the c3/s3-only externs.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_enter_phase2() {
    if unsafe { btdm_controller_get_sleep_mode() } == 1
        && PHY_ENABLED.swap(false, core::sync::atomic::Ordering::AcqRel)
    {
        esp_phy::disable_phy();
        // The controller is sleeping the modem for the whole gap, so release the wake
        // lock; the esp-rtos idle hook may light-sleep the SoC until the pre-event wake.
        // Set the deadline HERE (paired with the release), so exit_phase3's paired
        // re-acquire always clears it — no stale deadline can strand the SoC awake.
        // Re-acquired in exit_phase3.
        if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
            esp_hal::rtc_cntl::WakeLock::release();
            // Arm the pre-event wake deadline ONLY while connected. A held CONNECTION needs
            // the SoC awake for its periodic events (without it the link drops before service
            // discovery — HW-proven); idle advertising does not, so skipping the deadline
            // there lets the SoC light-sleep the whole gap (~55% -> ~87% residency).
            if BLE_CONN_COUNT.load(core::sync::atomic::Ordering::Relaxed) > 0 {
                let wake_in = PENDING_WAKE_IN_US.load(core::sync::atomic::Ordering::Relaxed);
                if wake_in > 0 {
                    esp_hal::rtc_cntl::WakeLock::set_sleep_deadline(
                        esp_hal::time::Instant::now()
                            + esp_hal::time::Duration::from_micros(wake_in as u64),
                    );
                }
            }
        }
    }
}

#[cfg(not(any(esp32c3, esp32s3)))]
unsafe extern "C" fn btdm_sleep_enter_phase2() {}

// exit_phase1/2 are NULL in bt.c's OSI table; never invoked.
unsafe extern "C" fn btdm_sleep_exit_phase1() {}

unsafe extern "C" fn btdm_sleep_exit_phase2() {}

// Re-power the RF/baseband on wake, then wait for the sleep FSM to resync.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_exit_phase3() {
    if unsafe { btdm_controller_get_sleep_mode() } == 1
        && !PHY_ENABLED.swap(true, core::sync::atomic::Ordering::AcqRel)
    {
        // Re-acquire the wake lock before the RF comes back, so the SoC cannot light-
        // sleep through the imminent event. Balances the enter_phase2 release, paired
        // with the PHY_ENABLED gate so the lock count stays matched.
        if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
            esp_hal::rtc_cntl::WakeLock::acquire();
            // The lock is held again, so the pre-event deadline has done its job.
            esp_hal::rtc_cntl::WakeLock::clear_sleep_deadline();
            MODEM_SLEEP_DIAG[1].fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        // Balanced raw increment: enable_phy() returns an RAII guard; forget it
        // so the +1 persists until the matching enter_phase2 disable.
        core::mem::forget(esp_phy::enable_phy());
    }
    while unsafe { btdm_sleep_clock_sync() } {}
}

#[cfg(not(any(esp32c3, esp32s3)))]
unsafe extern "C" fn btdm_sleep_exit_phase3() {}

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
            // With a light-sleep capable sleep clock, the wake lock esp-radio took in `init()`
            // doubles as the controller's per-event lock: enter_phase2 releases it in each
            // gap and exit_phase3 re-acquires it, so the SoC only light-sleeps between events.
            // Without one, that lock stays held for the controller's lifetime.
            // The Bt light-sleep wake source is armed per-CONNECTION (in notify_host_recv),
            // NOT here: during idle advertising the controller runs autonomously on the
            // kept-alive XTAL and does not need to wake the SoC, so leaving Bt-wake off there
            // avoids fragmenting the advertising sleep gap with per-event wakeups.
            let _ = light_sleep;
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

        // Turn modem sleep on at runtime. The config's sleep_mode only selects the mode;
        // ESP-IDF's esp_bt_controller_enable issues this to actually start it. Paired with
        // the VHCI-send wakeup guard in send_hci, which wakes a sleeping controller before
        // a host->controller send (otherwise the send hangs).
        #[cfg(any(esp32c3, esp32s3))]
        btdm_controller_enable_sleep(true);

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
    // If the controller was in a modem-sleep gap, enter_phase2 released the wake lock that
    // `deinit()` is about to release again; re-take it so the count stays balanced.
    if SLEEP_CLOCK_LIGHT_SLEEP.load(core::sync::atomic::Ordering::Relaxed) {
        // Bt wakeup is only ever enabled on the light-sleep-capable btdm chips.
        #[cfg(any(esp32c3, esp32s3))]
        esp_hal::rtc_cntl::sleep::disable_bt_wakeup();
        if !PHY_ENABLED.swap(true, core::sync::atomic::Ordering::AcqRel) {
            esp_hal::rtc_cntl::WakeLock::acquire();
            esp_hal::rtc_cntl::WakeLock::clear_sleep_deadline();
        }
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
                // Wake a modem-sleeping controller before the send (c3/s3).
                #[cfg(any(esp32c3, esp32s3))]
                ble_os_adapter_chip_specific::hci_wakeup_request();

                API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);

                #[cfg(any(esp32c3, esp32s3))]
                ble_os_adapter_chip_specific::hci_wakeup_request_end();
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
