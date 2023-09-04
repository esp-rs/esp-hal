use crate::binary::include::esp_event_base_t;
use crate::binary::include::esp_timer_create_args_t;
use crate::binary::include::esp_timer_handle_t;
use crate::compat::timer_compat::*;
use crate::trace;

use crate::compat::common::*;

#[cfg(esp32)]
use esp32_hal as hal;
#[cfg(esp32c2)]
use esp32c2_hal as hal;
#[cfg(esp32c3)]
use esp32c3_hal as hal;
#[cfg(esp32c6)]
use esp32c6_hal as hal;
#[cfg(esp32s2)]
use esp32s2_hal as hal;
#[cfg(esp32s3)]
use esp32s3_hal as hal;

use hal::system::RadioClockControl;
use hal::Rng;

use hal::macros::ram;

#[cfg_attr(esp32c3, path = "common_adapter_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "common_adapter_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "common_adapter_esp32c6.rs")]
#[cfg_attr(esp32, path = "common_adapter_esp32.rs")]
#[cfg_attr(esp32s3, path = "common_adapter_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "common_adapter_esp32s2.rs")]
pub(crate) mod chip_specific;

#[cfg_attr(esp32c3, path = "phy_init_data_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "phy_init_data_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "phy_init_data_esp32c6.rs")]
#[cfg_attr(esp32, path = "phy_init_data_esp32.rs")]
#[cfg_attr(esp32s3, path = "phy_init_data_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "phy_init_data_esp32s2.rs")]
pub(crate) mod phy_init_data;

pub(crate) static mut RANDOM_GENERATOR: Option<Rng> = None;

pub(crate) static mut RADIO_CLOCKS: Option<RadioClockControl> = None;

pub fn init_rng(rng: Rng) {
    unsafe {
        crate::common_adapter::RANDOM_GENERATOR = Some(core::mem::transmute(rng));
    }
}

pub fn init_radio_clock_control(rcc: RadioClockControl) {
    unsafe {
        crate::common_adapter::RADIO_CLOCKS = Some(core::mem::transmute(rcc));
    }
}

/****************************************************************************
 * Name: esp_semphr_create
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No mean
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/
#[allow(unused)]
pub unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *mut crate::binary::c_types::c_void {
    trace!("semphr_create - max {} init {}", max, init);
    sem_create(max, init)
}

/****************************************************************************
 * Name: esp_semphr_delete
 *
 * Description:
 *   Delete semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#[allow(unused)]
pub unsafe extern "C" fn semphr_delete(semphr: *mut crate::binary::c_types::c_void) {
    trace!("semphr_delete {:?}", semphr);
    sem_delete(semphr);
}

/****************************************************************************
 * Name: esp_semphr_take
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *   ticks  - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
#[ram]
pub unsafe extern "C" fn semphr_take(
    semphr: *mut crate::binary::c_types::c_void,
    tick: u32,
) -> i32 {
    sem_take(semphr, tick)
}

/****************************************************************************
 * Name: esp_semphr_give
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/
#[ram]
pub unsafe extern "C" fn semphr_give(semphr: *mut crate::binary::c_types::c_void) -> i32 {
    sem_give(semphr)
}

/****************************************************************************
 * Name: esp_random_ulong
 ****************************************************************************/
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

/****************************************************************************
 * Name: ets_timer_arm
 *
 * Description:
 *   Set timer timeout period and repeat flag
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *   ms     - millim seconds
 *   repeat - true: run cycle, false: run once
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn timer_arm(
    ptimer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
    compat_timer_arm(ptimer, tmout, repeat);
}

/****************************************************************************
 * Name: ets_timer_disarm
 *
 * Description:
 *   Disable timer
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn timer_disarm(ptimer: *mut crate::binary::c_types::c_void) {
    compat_timer_disarm(ptimer);
}

/****************************************************************************
 * Name: ets_timer_done
 *
 * Description:
 *   Disable and free timer
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn timer_done(ptimer: *mut crate::binary::c_types::c_void) {
    compat_timer_done(ptimer);
}

/****************************************************************************
 * Name: ets_timer_setfn
 *
 * Description:
 *   Set timer callback function and private data
 *
 * Input Parameters:
 *   ptimer    - Timer data pointer
 *   pfunction - Callback function
 *   parg      - Callback function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
pub unsafe extern "C" fn timer_setfn(
    ptimer: *mut crate::binary::c_types::c_void,
    pfunction: *mut crate::binary::c_types::c_void,
    parg: *mut crate::binary::c_types::c_void,
) {
    compat_timer_setfn(ptimer, pfunction, parg);
}

/****************************************************************************
 * Name: ets_timer_arm_us
 *
 * Description:
 *   Set timer timeout period and repeat flag
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *   us     - micro seconds
 *   repeat - true: run cycle, false: run once
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#[allow(unused)]
pub unsafe extern "C" fn timer_arm_us(
    ptimer: *mut crate::binary::c_types::c_void,
    us: u32,
    repeat: bool,
) {
    compat_timer_arm_us(ptimer, us, repeat);
}

/****************************************************************************
 * Name: esp_wifi_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/
pub unsafe extern "C" fn read_mac(mac: *mut u8, type_: u32) -> crate::binary::c_types::c_int {
    trace!("read_mac {:?} {}", mac, type_);

    let base_mac = crate::hal::efuse::Efuse::get_mac_address();

    for i in 0..6 {
        mac.offset(i as isize).write_volatile(base_mac[i]);
    }

    /* ESP_MAC_WIFI_SOFTAP */
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
    let cstr = StrBuf::from(s);
    trace!("{}", cstr.as_str_ref());
}

#[no_mangle]
pub unsafe extern "C" fn sprintf(dst: *mut u8, format: *const u8, args: ...) -> i32 {
    let str = StrBuf::from(format);
    trace!("sprintf {}", str.as_str_ref());

    let len = crate::compat::common::vsnprintf(dst, 511, format, args);

    let s = StrBuf::from(dst);
    trace!("sprintf {}", s.as_str_ref());

    len
}

#[no_mangle]
pub unsafe extern "C" fn printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

#[no_mangle]
pub unsafe extern "C" fn rtc_printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

#[no_mangle]
pub unsafe extern "C" fn phy_printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

#[no_mangle]
pub unsafe extern "C" fn coexist_printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

#[no_mangle]
pub unsafe extern "C" fn net80211_printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

#[no_mangle]
pub unsafe extern "C" fn pp_printf(s: *const u8, args: ...) {
    syslog(0, s, args);
}

// #define ESP_EVENT_DEFINE_BASE(id) esp_event_base_t id = #id
static mut EVT: i8 = 0;
#[no_mangle]
static mut WIFI_EVENT: esp_event_base_t = unsafe { &EVT };

// stuff needed by wpa-supplicant
#[no_mangle]
pub unsafe extern "C" fn __assert_func(
    _file: *const u8,
    _line: u32,
    _func: *const u8,
    _failed_expr: *const u8,
) {
    todo!("__assert_func");
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_disarm(timer: *mut crate::binary::c_types::c_void) {
    timer_disarm(timer);
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_done(timer: *mut crate::binary::c_types::c_void) {
    timer_done(timer);
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_setfn(
    ptimer: *mut crate::binary::c_types::c_void,
    pfunction: *mut crate::binary::c_types::c_void,
    parg: *mut crate::binary::c_types::c_void,
) {
    timer_setfn(ptimer, pfunction, parg);
}

#[no_mangle]
pub unsafe extern "C" fn ets_timer_arm(
    timer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
    timer_arm(timer, tmout, repeat);
}

#[no_mangle]
pub unsafe extern "C" fn gettimeofday(_tv: *const (), _tz: *const ()) {
    todo!("gettimeofday");
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

#[no_mangle]
#[linkage = "weak"]
pub unsafe extern "C" fn floor(_v: f64) -> f64 {
    todo!("floor")
}
