#![allow(dead_code)]

use esp_sync::NonReentrantMutex;
use esp_wifi_sys::{
    c_types::c_char,
    include::{
        esp_phy_calibration_data_t,
        esp_phy_calibration_mode_t,
        get_phy_version_str,
        register_chipv7_phy,
        timeval,
    },
};
use portable_atomic::{AtomicU32, Ordering};

use crate::{
    binary::{
        c_types::{c_int, c_ulong, c_void},
        include::{esp_event_base_t, esp_timer_get_time},
    },
    compat::{common::*, semaphore::*},
    hal::{self, clock::ModemClockController, ram},
};

#[cfg_attr(esp32c3, path = "common_adapter_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "common_adapter_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "common_adapter_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "common_adapter_esp32h2.rs")]
#[cfg_attr(esp32, path = "common_adapter_esp32.rs")]
#[cfg_attr(esp32s3, path = "common_adapter_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "common_adapter_esp32s2.rs")]
pub(crate) mod chip_specific;

#[cfg_attr(esp32c3, path = "phy_init_data_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "phy_init_data_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "phy_init_data_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "phy_init_data_esp32h2.rs")]
#[cfg_attr(esp32, path = "phy_init_data_esp32.rs")]
#[cfg_attr(esp32s3, path = "phy_init_data_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "phy_init_data_esp32s2.rs")]
pub(crate) mod phy_init_data;

static PHY_ACCESS_REF: NonReentrantMutex<usize> = NonReentrantMutex::new(0);

pub(crate) unsafe fn phy_enable() {
    PHY_ACCESS_REF.with(|ref_count| {
        *ref_count += 1;
        if *ref_count == 1 {
            unsafe { chip_specific::phy_enable_inner() };
        }
    })
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable() {
    PHY_ACCESS_REF.with(|ref_count| {
        *ref_count -= 1;
        if *ref_count == 0 {
            unsafe { chip_specific::phy_disable_inner() };
        }
    })
}

static CAL_DATA: esp_sync::NonReentrantMutex<
    [u8; core::mem::size_of::<esp_phy_calibration_data_t>()],
> = esp_sync::NonReentrantMutex::new([0u8; core::mem::size_of::<esp_phy_calibration_data_t>()]);

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
    sem_take(semphr, tick)
}

#[ram]
pub unsafe extern "C" fn semphr_take_from_isr(
    semphr: *mut c_void,
    higher_priority_task_waken: *mut bool,
) -> i32 {
    trace!(">>>> semphr_take_from_isr {:?}", semphr);
    sem_take_from_isr(semphr, higher_priority_task_waken)
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
    sem_give_from_isr(semphr, higher_priority_task_waken)
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

    let base_mac = hal::efuse::Efuse::mac_address();

    for (i, &byte) in base_mac.iter().enumerate() {
        unsafe {
            mac.add(i).write_volatile(byte);
        }
    }

    unsafe {
        // ESP_MAC_WIFI_SOFTAP
        if type_ == 1 {
            let tmp = mac.offset(0).read_volatile();
            for i in 0..64 {
                mac.offset(0).write_volatile(tmp | 0x02);
                mac.offset(0)
                    .write_volatile(mac.offset(0).read_volatile() ^ (i << 2));

                if mac.offset(0).read_volatile() != tmp {
                    break;
                }
            }
        }

        // ESP_MAC_BT
        if type_ == 2 {
            let tmp = mac.offset(0).read_volatile();
            for i in 0..64 {
                mac.offset(0).write_volatile(tmp | 0x02);
                mac.offset(0)
                    .write_volatile(mac.offset(0).read_volatile() ^ (i << 2));

                if mac.offset(0).read_volatile() != tmp {
                    break;
                }
            }

            mac.offset(5)
                .write_volatile(mac.offset(5).read_volatile() + 1);
        }
    }

    0
}

// other functions
#[unsafe(no_mangle)]
pub unsafe extern "C" fn puts(s: *const c_char) {
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
pub unsafe extern "C" fn ets_timer_arm(timer: *mut c_void, tmout: u32, repeat: bool) {
    crate::compat::timer_compat::compat_timer_arm(timer.cast(), tmout, repeat);
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_arm_us(timer: *mut c_void, tmout: u32, repeat: bool) {
    crate::compat::timer_compat::compat_timer_arm_us(timer.cast(), tmout, repeat);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_gettimeofday(tv: *mut timeval, _tz: *mut ()) -> i32 {
    if !tv.is_null() {
        unsafe {
            let microseconds = esp_timer_get_time();
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
    crate::time::ticks_to_micros(crate::preempt::now()) as i64
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

static PHY_CLOCK_ENABLE_REF: AtomicU32 = AtomicU32::new(0);

// We're use either WIFI or BT here, since esp-radio also supports the ESP32-H2 as the only
// chip, with BT but without WIFI.
#[cfg(not(esp32h2))]
type ModemClockControllerPeripheral = esp_hal::peripherals::WIFI<'static>;
#[cfg(esp32h2)]
type ModemClockControllerPeripheral = esp_hal::peripherals::BT<'static>;

pub(crate) unsafe fn phy_enable_clock() {
    let count = PHY_CLOCK_ENABLE_REF.fetch_add(1, Ordering::Acquire);
    if count == 0 {
        // Stealing the peripheral is safe here, as they must have been passed into the relevant
        // initialization functions for the Wi-Fi or BLE controller, if this code gets executed.
        let clock_guard = unsafe { ModemClockControllerPeripheral::steal() }.enable_phy_clock();
        core::mem::forget(clock_guard);

        trace!("phy_enable_clock done!");
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    let count = PHY_CLOCK_ENABLE_REF.fetch_sub(1, Ordering::Release);
    if count == 1 {
        unsafe { ModemClockControllerPeripheral::steal() }.decrease_phy_clock_ref_count();
        trace!("phy_disable_clock done!");
    }
}

pub(crate) fn phy_calibrate() {
    let phy_version = unsafe { get_phy_version_str() };
    trace!("phy_version {}", unsafe { str_from_c(phy_version) });

    let init_data = &phy_init_data::PHY_INIT_DATA_DEFAULT;

    unsafe {
        chip_specific::bbpll_en_usb();
    }

    #[cfg(phy_full_calibration)]
    const CALIBRATION_MODE: esp_phy_calibration_mode_t =
        esp_wifi_sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_FULL;
    #[cfg(not(phy_full_calibration))]
    const CALIBRATION_MODE: esp_phy_calibration_mode_t =
        esp_wifi_sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_PARTIAL;

    #[cfg(phy_skip_calibration_after_deep_sleep)]
    let calibration_mode = if crate::hal::system::reset_reason()
        == Some(crate::hal::rtc_cntl::SocResetReason::CoreDeepSleep)
    {
        esp_wifi_sys::include::esp_phy_calibration_mode_t_PHY_RF_CAL_NONE
    } else {
        CALIBRATION_MODE
    };
    #[cfg(not(phy_skip_calibration_after_deep_sleep))]
    let calibration_mode = CALIBRATION_MODE;

    debug!("Using calibration mode {}", calibration_mode);

    let res = CAL_DATA.with(|cal_data| unsafe {
        register_chipv7_phy(
            init_data,
            cal_data as *mut _ as *mut crate::binary::include::esp_phy_calibration_data_t,
            calibration_mode,
        )
    });

    debug!("register_chipv7_phy result = {}", res);
}

/// Get calibration data.
///
/// Returns the last calibration result.
///
/// If you see the data is different than what was persisted before, consider persisting the new
/// data.
pub fn phy_calibration_data() -> [u8; core::mem::size_of::<esp_phy_calibration_data_t>()] {
    CAL_DATA.with(|cal_data| *cal_data)
}

/// Set calibration data.
///
/// This will be used next time the phy gets initialized.
pub fn set_phy_calibration_data(data: &[u8; core::mem::size_of::<esp_phy_calibration_data_t>()]) {
    CAL_DATA.with(|cal_data| {
        *cal_data = *data;
    });
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
    // TODO remove this once fixed in esp_supplicant AND we updated to the fixed
    // version - JIRA: WIFI-6676
    let (queue_len, item_size) = if queue_len != 3 && item_size != 4 {
        (queue_len, item_size)
    } else {
        warn!("Fixing queue item_size");
        (3, 8)
    };

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
    crate::compat::queue::queue_send_to_back(queue.cast(), item.cast_const(), block_time_tick)
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
    _higher_priority_task_waken: *mut c_void,
) -> i32 {
    crate::compat::queue::queue_try_send_to_back(queue.cast(), item.cast_const())
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
    crate::compat::queue::queue_send_to_back(queue.cast(), item, block_time_tick)
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
    crate::compat::queue::queue_send_to_front(queue.cast(), item, block_time_tick)
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
    block_time_tick: u32,
) -> i32 {
    crate::compat::queue::queue_receive(queue.cast(), item, block_time_tick)
}

pub unsafe extern "C" fn queue_recv_from_isr(
    queue: *mut c_void,
    item: *mut c_void,
    _higher_priority_task_waken: *mut c_void,
) -> i32 {
    crate::compat::queue::queue_try_receive(queue.cast(), item)
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
