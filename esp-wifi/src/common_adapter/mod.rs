use core::ptr::addr_of;

use esp_wifi_sys::include::timespec;
use hal::{macros::ram, rng::Rng};

use crate::{
    binary::include::{
        esp_event_base_t,
        esp_timer_create_args_t,
        esp_timer_get_time,
        esp_timer_handle_t,
    },
    compat::{common::*, timer_compat::*},
    hal,
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

pub(crate) static mut RANDOM_GENERATOR: Option<Rng> = None;

pub(crate) static mut RADIO_CLOCKS: Option<hal::peripherals::RADIO_CLK> = None;

pub(crate) fn init_rng(rng: Rng) {
    unsafe { RANDOM_GENERATOR = Some(rng) };
}

pub(crate) fn init_radio_clock_control(rcc: hal::peripherals::RADIO_CLK) {
    unsafe { RADIO_CLOCKS = Some(rcc) };
}

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
pub unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *mut crate::binary::c_types::c_void {
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
pub unsafe extern "C" fn semphr_delete(semphr: *mut crate::binary::c_types::c_void) {
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
pub unsafe extern "C" fn semphr_take(
    semphr: *mut crate::binary::c_types::c_void,
    tick: u32,
) -> i32 {
    sem_take(semphr, tick)
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
pub unsafe extern "C" fn semphr_give(semphr: *mut crate::binary::c_types::c_void) -> i32 {
    sem_give(semphr)
}

/// **************************************************************************
/// Name: esp_random_ulong
/// *************************************************************************
#[allow(unused)]
#[ram]
#[no_mangle]
pub unsafe extern "C" fn random() -> crate::binary::c_types::c_ulong {
    trace!("random");

    if let Some(ref mut rng) = RANDOM_GENERATOR {
        rng.random()
    } else {
        0
    }
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
pub unsafe extern "C" fn read_mac(mac: *mut u8, type_: u32) -> crate::binary::c_types::c_int {
    trace!("read_mac {:?} {}", mac, type_);

    let base_mac = crate::hal::efuse::Efuse::get_mac_address();

    for (i, &byte) in base_mac.iter().enumerate() {
        mac.add(i).write_volatile(byte);
    }

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

    0
}

#[allow(unused)]
#[ram]
pub(crate) unsafe extern "C" fn semphr_take_from_isr(sem: *const (), hptw: *const ()) -> i32 {
    trace!("sem take from isr");
    (hptw as *mut u32).write_volatile(0);
    crate::common_adapter::semphr_take(sem as *mut crate::binary::c_types::c_void, 0)
}

#[allow(unused)]
#[ram]
pub(crate) unsafe extern "C" fn semphr_give_from_isr(sem: *const (), hptw: *const ()) -> i32 {
    trace!("sem give from isr");
    (hptw as *mut u32).write_volatile(0);
    crate::common_adapter::semphr_give(sem as *mut crate::binary::c_types::c_void)
}

// other functions
#[no_mangle]
pub unsafe extern "C" fn puts(s: *const u8) {
    let cstr = str_from_c(s);
    info!("{}", cstr);
}

#[cfg(any(feature = "wifi-logs", nightly))]
#[no_mangle]
pub unsafe extern "C" fn sprintf(dst: *mut u8, format: *const u8, args: ...) -> i32 {
    let str = str_from_c(format);
    trace!("sprintf format: {}", str);

    let len = crate::compat::syslog::vsnprintf(dst, 512, format, args);

    let s = str_from_c(dst);
    trace!("sprintf result: {}", s);

    len
}

#[cfg(all(not(feature = "wifi-logs"), not(nightly)))]
#[no_mangle]
pub unsafe extern "C" fn sprintf(dst: *mut u8, format: *const u8, _args: *const ()) -> i32 {
    let str = str_from_c(format);

    let res = match str {
        "ESP_%02X%02X%02X" => "ESP_0000",
        "%d,%s,%s,%s" => "????",
        "unknown id:%d" => "unknown id:??",
        _ => {
            warn!("Unexpected call to `sprintf`: {}", str);
            "???"
        }
    };

    dst.copy_from_nonoverlapping(res.as_ptr(), res.len());
    dst.add(res.len()).write_volatile(0);

    res.len() as i32
}

#[cfg(feature = "wifi-logs")]
mod log {
    #[no_mangle]
    pub unsafe extern "C" fn printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn rtc_printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn phy_printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn coexist_printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn net80211_printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn pp_printf(s: *const u8, args: ...) {
        crate::compat::syslog::syslog(0, s, args);
    }
}

#[cfg(not(feature = "wifi-logs"))]
mod log {
    #[no_mangle]
    pub unsafe extern "C" fn printf(_s: *const u8, _args: *const ()) {}

    #[no_mangle]
    pub unsafe extern "C" fn rtc_printf(_s: *const u8, _args: *const ()) {}

    #[no_mangle]
    pub unsafe extern "C" fn phy_printf(_s: *const u8, _args: *const ()) {}

    #[no_mangle]
    pub unsafe extern "C" fn coexist_printf(_s: *const u8, _args: *const ()) {}

    #[no_mangle]
    pub unsafe extern "C" fn net80211_printf(_s: *const u8, _args: *const ()) {}

    #[no_mangle]
    pub unsafe extern "C" fn pp_printf(_s: *const u8, _args: *const ()) {}
}

// #define ESP_EVENT_DEFINE_BASE(id) esp_event_base_t id = #id
static mut EVT: i8 = 0;
#[no_mangle]
#[allow(unused_unsafe)]
static mut WIFI_EVENT: esp_event_base_t = unsafe { addr_of!(EVT) };

// stuff needed by wpa-supplicant
#[no_mangle]
pub unsafe extern "C" fn __assert_func(
    file: *const u8,
    line: u32,
    func: *const u8,
    failed_expr: *const u8,
) {
    let file = str_from_c(file);
    let (func_pre, func) = if func.is_null() {
        ("", "")
    } else {
        (", function: ", str_from_c(func))
    };
    let expr = str_from_c(failed_expr);

    panic!(
        "assertion \"{}\" failed: file \"{}\", line {}{}{}",
        expr, file, line, func_pre, func
    );
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_disarm(timer: *mut crate::binary::c_types::c_void) {
    compat_timer_disarm(timer.cast());
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_done(timer: *mut crate::binary::c_types::c_void) {
    compat_timer_done(timer.cast());
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_setfn(
    ptimer: *mut crate::binary::c_types::c_void,
    pfunction: *mut crate::binary::c_types::c_void,
    parg: *mut crate::binary::c_types::c_void,
) {
    compat_timer_setfn(
        ptimer.cast(),
        core::mem::transmute::<
            *mut crate::binary::c_types::c_void,
            unsafe extern "C" fn(*mut crate::binary::c_types::c_void),
        >(pfunction),
        parg,
    );
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_arm(
    timer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
    compat_timer_arm(timer.cast(), tmout, repeat);
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_arm_us(
    timer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
    compat_timer_arm_us(timer.cast(), tmout, repeat);
}

#[no_mangle]
pub unsafe extern "C" fn gettimeofday(tv: *mut timespec, _tz: *mut ()) -> i32 {
    if !tv.is_null() {
        unsafe {
            let microseconds = esp_timer_get_time();
            (*tv).tv_sec = (microseconds / 1_000_000) as i32;
            (*tv).tv_nsec = (microseconds % 1_000_000) as i32 * 1000;
        }
    }

    0
}

#[no_mangle]
pub unsafe extern "C" fn esp_fill_random(dst: *mut u8, len: u32) {
    trace!("esp_fill_random");
    let dst = core::slice::from_raw_parts_mut(dst, len as usize);

    if let Some(ref mut rng) = RANDOM_GENERATOR {
        for chunk in dst.chunks_mut(4) {
            let bytes = rng.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn esp_timer_stop(_handle: *mut ()) {
    todo!("esp_timer_stop");
}

#[no_mangle]
pub unsafe extern "C" fn esp_timer_delete(_handle: *mut ()) {
    todo!("esp_timer_delete");
}

#[no_mangle]
pub unsafe extern "C" fn esp_timer_start_once(_handle: *mut (), _timeout_us: u64) -> i32 {
    todo!("esp_timer_start_once");
}

#[no_mangle]
pub unsafe extern "C" fn esp_timer_create(
    args: *const esp_timer_create_args_t,
    out_handle: *mut esp_timer_handle_t,
) -> i32 {
    compat_esp_timer_create(args, out_handle)
}

#[no_mangle]
pub unsafe extern "C" fn strrchr(_s: *const (), _c: u32) -> *const u8 {
    todo!("strrchr");
}

// this will result in a duplicate symbol error once `floor` is available
// ideally we would use weak linkage but that is not stabilized
// see https://github.com/esp-rs/esp-wifi/pull/191
#[cfg(feature = "esp32c6")]
#[no_mangle]
pub unsafe extern "C" fn floor(v: f64) -> f64 {
    libm::floor(v)
}
