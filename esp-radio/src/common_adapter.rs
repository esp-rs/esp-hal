#![allow(dead_code)]

use crate::{
    compat::{common::*, semaphore::*},
    hal::{self, ram},
    sys::{
        c_types::{c_char, c_int, c_ulong, c_void},
        include::{esp_event_base_t, timeval},
    },
    time::blob_ticks_to_micros,
};

/// **************************************************************************
/// Name: esp_semphr_create
///
/// Description:
///   Create and initialize semaphore
///
/// Input Parameters:
///   max  - No mean
///   init - semaphore initialization value
///
/// Returned Value:
///   Semaphore data pointer
///
/// *************************************************************************
#[allow(unused)]
pub unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *mut c_void {
    trace!("semphr_create - max {} init {}", max, init);
    sem_create(max, init)
}

/// **************************************************************************
/// Name: esp_semphr_delete
///
/// Description:
///   Delete semaphore
///
/// Input Parameters:
///   semphr - Semaphore data pointer
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[allow(unused)]
pub unsafe extern "C" fn semphr_delete(semphr: *mut c_void) {
    trace!("semphr_delete {:?}", semphr);
    sem_delete(semphr);
}

/// **************************************************************************
/// Name: esp_semphr_take
///
/// Description:
///   Wait semaphore within a certain period of time
///
/// Input Parameters:
///   semphr - Semaphore data pointer
///   ticks  - Wait system ticks
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
#[ram]
pub unsafe extern "C" fn semphr_take(semphr: *mut c_void, tick: u32) -> i32 {
    trace!(">>>> semphr_take {:?} block_time_tick {}", semphr, tick);
    sem_take(semphr, blob_ticks_to_micros(tick))
}

#[ram]
pub unsafe extern "C" fn semphr_take_from_isr(
    semphr: *mut c_void,
    higher_priority_task_waken: *mut bool,
) -> i32 {
    trace!(">>>> semphr_take_from_isr {:?}", semphr);
    sem_try_take_from_isr(semphr, higher_priority_task_waken)
}

/// **************************************************************************
/// Name: esp_semphr_give
///
/// Description:
///   Post semaphore
///
/// Input Parameters:
///   semphr - Semaphore data pointer
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
#[ram]
pub unsafe extern "C" fn semphr_give(semphr: *mut c_void) -> i32 {
    trace!(">>>> semphr_give {:?}", semphr);
    sem_give(semphr)
}

#[ram]
pub unsafe extern "C" fn semphr_give_from_isr(
    semphr: *mut c_void,
    higher_priority_task_waken: *mut bool,
) -> i32 {
    trace!(">>>> semphr_give_from_isr {:?}", semphr);
    sem_try_give_from_isr(semphr, higher_priority_task_waken)
}

/// **************************************************************************
/// Name: esp_random_ulong
/// *************************************************************************
#[allow(unused)]
#[ram]
pub unsafe extern "C" fn random() -> c_ulong {
    trace!("random");

    let rng = hal::rng::Rng::new();
    rng.random()
}

/// **************************************************************************
/// Name: esp_wifi_read_mac
///
/// Description:
///   Read MAC address from efuse
///
/// Input Parameters:
///   mac  - MAC address buffer pointer
///   type - MAC address type
///
/// Returned Value:
///   0 if success or -1 if fail
///
/// *************************************************************************
pub unsafe extern "C" fn read_mac(mac: *mut u8, type_: u32) -> c_int {
    trace!("read_mac {:?} {}", mac, type_);
    use hal::efuse::InterfaceMacAddress;

    let kind = match type_ {
        0 => InterfaceMacAddress::Station,
        1 => InterfaceMacAddress::AccessPoint,
        2 => InterfaceMacAddress::Bluetooth,
        _ => {
            warn!(
                "Invalid interface type: {} (expected 0=STA, 1=AP, 2=BT)",
                type_
            );
            return -1;
        }
    };

    let addr = hal::efuse::Efuse::interface_mac_address(kind);

    unsafe {
        core::ptr::copy_nonoverlapping(addr.as_bytes().as_ptr(), mac, 6);
    }

    0
}

// other functions
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_puts(s: *const c_char) {
    unsafe {
        let cstr = str_from_c(s);
        info!("{}", cstr);
    }
}

// #define ESP_EVENT_DEFINE_BASE(id) esp_event_base_t id = #id
#[unsafe(no_mangle)]
static mut __ESP_RADIO_WIFI_EVENT: esp_event_base_t = c"WIFI_EVENT".as_ptr();

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_disarm(timer: *mut c_void) {
    crate::compat::timer_compat::compat_timer_disarm(timer.cast());
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_done(timer: *mut c_void) {
    crate::compat::timer_compat::compat_timer_done(timer.cast());
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_setfn(
    ptimer: *mut c_void,
    pfunction: *mut c_void,
    parg: *mut c_void,
) {
    unsafe {
        crate::compat::timer_compat::compat_timer_setfn(
            ptimer.cast(),
            core::mem::transmute::<*mut c_void, unsafe extern "C" fn(*mut c_void)>(pfunction),
            parg,
        );
    }
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_arm(timer: *mut c_void, ms: u32, repeat: bool) {
    crate::compat::timer_compat::compat_timer_arm(timer.cast(), ms, repeat);
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_arm_us(timer: *mut c_void, us: u32, repeat: bool) {
    crate::compat::timer_compat::compat_timer_arm_us(timer.cast(), us, repeat);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_gettimeofday(tv: *mut timeval, _tz: *mut ()) -> i32 {
    if !tv.is_null() {
        unsafe {
            let microseconds = __esp_radio_esp_timer_get_time();
            (*tv).tv_sec = (microseconds / 1_000_000) as u64;
            (*tv).tv_usec = (microseconds % 1_000_000) as u32;
        }
    }

    0
}

/// **************************************************************************
/// Name: esp_timer_get_time
///
/// Description:
///   Get time in microseconds since boot.
///
/// Returned Value:
///   System time in micros
///
/// *************************************************************************
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_esp_timer_get_time() -> i64 {
    trace!("esp_timer_get_time");
    // Just using IEEE802.15.4 doesn't need the current time. If we don't use `preempt::now`, users
    // will not need to have a scheduler in their firmware.
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "wifi", feature = "ble"))] {
            crate::preempt::now() as i64
        } else {
            // In this case we don't have a scheduler, we can return esp-hal's timestamp.
            esp_hal::time::Instant::now().duration_since_epoch().as_micros() as i64
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_esp_fill_random(dst: *mut u8, len: u32) {
    trace!("esp_fill_random");
    unsafe {
        let dst = core::slice::from_raw_parts_mut(dst, len as usize);

        let rng = esp_hal::rng::Rng::new();
        for chunk in dst.chunks_mut(4) {
            let bytes = rng.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_strrchr(_s: *const (), _c: u32) -> *const u8 {
    todo!("strrchr");
}

#[unsafe(no_mangle)]
static mut __ESP_RADIO_G_LOG_LEVEL: i32 = 0;

#[unsafe(no_mangle)]
pub static mut __ESP_RADIO_G_MISC_NVS: *mut u32 = &raw mut NVS as *mut u32;

pub static mut NVS: [u32; 15] = [0u32; 15];

// For some reason these are only necessary on Xtensa chips.
#[cfg(xtensa)]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_deinit() {
    trace!("misc_nvs_deinit")
}

#[cfg(xtensa)]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
    trace!("misc_nvs_init");
    0
}

#[cfg(xtensa)]
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_restore() -> i32 {
    todo!("misc_nvs_restore")
}

// We're use either WIFI or BT here, since esp-radio also supports the ESP32-H2 as the only
// chip, with BT but without WIFI.
#[cfg(not(esp32h2))]
type ModemClockControllerPeripheral = esp_hal::peripherals::WIFI<'static>;
#[cfg(esp32h2)]
type ModemClockControllerPeripheral = esp_hal::peripherals::BT<'static>;

// Clock control is no-op because the wifi blobs don't symmetrically enable/disable the clock,
// causing an eventual overflow. Currently we are holding onto a guard ourselves while Wi-Fi/BT is
// active, so the blobs should not be able to disable the clock anyway.
//
// This might have some low-power issues, but we're not there yet anyway.
#[allow(unused)]
pub(crate) unsafe fn phy_enable_clock() {
    // Stealing the peripheral is safe here, as they must have been passed into the relevant
    // initialization functions for the Wi-Fi or BLE controller, if this code gets executed.
    // let clock_guard = unsafe { ModemClockControllerPeripheral::steal() }.enable_phy_clock();
    // core::mem::forget(clock_guard);
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    // unsafe { ModemClockControllerPeripheral::steal() }.decrease_phy_clock_ref_count();
}

pub(crate) fn enable_wifi_power_domain() {
    #[cfg(not(any(soc_has_pmu, esp32c2)))]
    {
        // C5, C6 have `LP_CLKRST`, but they're cfg'd out with `not(soc_has_pmu)`
        // TODO: revisit this code (https://github.com/esp-rs/esp-hal/pull/5066#discussion_r2858978902)
        let rtc_cntl = regs!(RTC_CNTL);

        rtc_cntl
            .dig_pwc()
            .modify(|_, w| w.wifi_force_pd().clear_bit());

        #[cfg(not(esp32))]
        cfg_if::cfg_if! {
            if #[cfg(soc_has_apb_ctrl)] {
                let syscon = regs!(APB_CTRL);
            } else { // S2
                let syscon = regs!(SYSCON);
            }
        }
        #[cfg(not(esp32))]
        unsafe {
            const WIFIBB_RST: u32 = 1 << 0; // Wi-Fi baseband
            const FE_RST: u32 = 1 << 1; // RF Frontend RST
            const WIFIMAC_RST: u32 = 1 << 2; // Wi-Fi MAC

            const BTBB_RST: u32 = 1 << 3; // Bluetooth Baseband
            const BTMAC_RST: u32 = 1 << 4; // deprecated
            const RW_BTMAC_RST: u32 = 1 << 9; // Bluetooth MAC
            const RW_BTMAC_REG_RST: u32 = 1 << 11; // Bluetooth MAC Regsiters
            const BTBB_REG_RST: u32 = 1 << 13; // Bluetooth Baseband Registers

            const MODEM_RESET_FIELD_WHEN_PU: u32 = WIFIBB_RST
                | FE_RST
                | WIFIMAC_RST
                | if cfg!(soc_has_bt) {
                    BTBB_RST | BTMAC_RST | RW_BTMAC_RST | RW_BTMAC_REG_RST | BTBB_RST
                } else {
                    0
                };

            syscon
                .wifi_rst_en()
                .modify(|r, w| w.bits(r.bits() | MODEM_RESET_FIELD_WHEN_PU));
            syscon
                .wifi_rst_en()
                .modify(|r, w| w.bits(r.bits() & !MODEM_RESET_FIELD_WHEN_PU));
        }

        rtc_cntl
            .dig_iso()
            .modify(|_, w| w.wifi_force_iso().clear_bit());
    }
}

/// **************************************************************************
/// Name: esp_queue_create
///
/// Description:
///   Create message queue
///
/// Input Parameters:
///   queue_len - queue message number
///   item_size - message size
///
/// Returned Value:
///   Message queue data pointer
///
/// *************************************************************************
pub unsafe extern "C" fn queue_create(queue_len: u32, item_size: u32) -> *mut c_void {
    crate::compat::queue::queue_create(queue_len as i32, item_size as i32).cast()
}

/// **************************************************************************
/// Name: esp_queue_delete
///
/// Description:
///   Delete message queue
///
/// Input Parameters:
///   queue - Message queue data pointer
///
/// Returned Value:
///   None
///
/// *************************************************************************
pub unsafe extern "C" fn queue_delete(queue: *mut c_void) {
    crate::compat::queue::queue_delete(queue.cast());
}

/// **************************************************************************
/// Name: esp_queue_send
///
/// Description:
///   Send message of low priority to queue within a certain period of time
///
/// Input Parameters:
///   queue - Message queue data pointer
///   item  - Message data pointer
///   ticks - Wait ticks
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn queue_send(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    crate::compat::queue::queue_send_to_back(
        queue.cast(),
        item.cast_const(),
        blob_ticks_to_micros(block_time_tick),
    )
}

/// **************************************************************************
/// Name: esp_queue_send_from_isr
///
/// Description:
///   Send message of low priority to queue in ISR within
///   a certain period of time
///
/// Input Parameters:
///   queue - Message queue data pointer
///   item  - Message data pointer
///   hptw  - No mean
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn queue_send_from_isr(
    queue: *mut c_void,
    item: *mut c_void,
    higher_priority_task_waken: *mut c_void,
) -> i32 {
    crate::compat::queue::queue_try_send_to_back_from_isr(
        queue.cast(),
        item.cast_const(),
        higher_priority_task_waken.cast(),
    )
}

/// **************************************************************************
/// Name: esp_queue_send_to_back
///
/// Description:
///   Send message of low priority to queue within a certain period of time
///
/// Input Parameters:
///   queue - Message queue data pointer
///   item  - Message data pointer
///   ticks - Wait ticks
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn queue_send_to_back(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    crate::compat::queue::queue_send_to_back(
        queue.cast(),
        item,
        blob_ticks_to_micros(block_time_tick),
    )
}

/// **************************************************************************
/// Name: esp_queue_send_from_to_front
///
/// Description:
///   Send message of high priority to queue within a certain period of time
///
/// Input Parameters:
///   queue - Message queue data pointer
///   item  - Message data pointer
///   ticks - Wait ticks
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn queue_send_to_front(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    crate::compat::queue::queue_send_to_front(
        queue.cast(),
        item,
        blob_ticks_to_micros(block_time_tick),
    )
}

/// **************************************************************************
/// Name: esp_queue_recv
///
/// Description:
///   Receive message from queue within a certain period of time
///
/// Input Parameters:
///   queue - Message queue data pointer
///   item  - Message data pointer
///   ticks - Wait ticks
///
/// Returned Value:
///   True if success or false if fail
///
/// *************************************************************************
pub unsafe extern "C" fn queue_recv(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_ms: u32,
) -> i32 {
    crate::compat::queue::queue_receive(queue.cast(), item, blob_ticks_to_micros(block_time_ms))
}

pub unsafe extern "C" fn queue_recv_from_isr(
    queue: *mut c_void,
    item: *mut c_void,
    higher_priority_task_waken: *mut c_void,
) -> i32 {
    crate::compat::queue::queue_try_receive_from_isr(
        queue.cast(),
        item,
        higher_priority_task_waken.cast(),
    )
}

/// **************************************************************************
/// Name: esp_queue_msg_waiting
///
/// Description:
///   Get message number in the message queue
///
/// Input Parameters:
///   queue - Message queue data pointer
///
/// Returned Value:
///   Message number
///
/// *************************************************************************
pub unsafe extern "C" fn queue_msg_waiting(queue: *mut c_void) -> u32 {
    crate::compat::queue::queue_messages_waiting(queue.cast())
}

#[allow(unused)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_esp_event_post(
    event_base: *const c_char,
    event_id: i32,
    event_data: *mut c_void,
    event_data_size: usize,
    ticks_to_wait: u32,
) -> i32 {
    #[cfg(feature = "wifi")]
    return unsafe {
        crate::wifi::event_post(
            event_base,
            event_id,
            event_data,
            event_data_size,
            ticks_to_wait,
        )
    };

    #[cfg(not(feature = "wifi"))]
    return -1;
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_vTaskDelay(ticks: u32) {
    unsafe {
        crate::compat::common::__esp_radio_usleep(crate::time::blob_ticks_to_micros(ticks));
    }
}
