use esp_wifi_sys::{
    c_types::c_char,
    include::{esp_phy_calibration_data_t, timeval},
};

use crate::{
    binary::include::{esp_event_base_t, esp_timer_get_time},
    compat::common::*,
    hal::{self, clock::ModemClockController, ram},
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
pub unsafe extern "C" fn random() -> crate::binary::c_types::c_ulong {
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
pub unsafe extern "C" fn read_mac(mac: *mut u8, type_: u32) -> crate::binary::c_types::c_int {
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

#[allow(unused)]
#[ram]
pub(crate) unsafe extern "C" fn semphr_take_from_isr(sem: *const (), hptw: *const ()) -> i32 {
    trace!("sem take from isr");
    unsafe {
        (hptw as *mut u32).write_volatile(0);
        crate::common_adapter::semphr_take(sem as *mut crate::binary::c_types::c_void, 0)
    }
}

#[allow(unused)]
#[ram]
pub(crate) unsafe extern "C" fn semphr_give_from_isr(sem: *const (), hptw: *const ()) -> i32 {
    trace!("sem give from isr");
    unsafe {
        (hptw as *mut u32).write_volatile(0);
        crate::common_adapter::semphr_give(sem as *mut crate::binary::c_types::c_void)
    }
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
pub unsafe extern "C" fn ets_timer_disarm(timer: *mut crate::binary::c_types::c_void) {
    crate::compat::timer_compat::compat_timer_disarm(timer.cast());
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_done(timer: *mut crate::binary::c_types::c_void) {
    crate::compat::timer_compat::compat_timer_done(timer.cast());
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_setfn(
    ptimer: *mut crate::binary::c_types::c_void,
    pfunction: *mut crate::binary::c_types::c_void,
    parg: *mut crate::binary::c_types::c_void,
) {
    unsafe {
        crate::compat::timer_compat::compat_timer_setfn(
            ptimer.cast(),
            core::mem::transmute::<
                *mut crate::binary::c_types::c_void,
                unsafe extern "C" fn(*mut crate::binary::c_types::c_void),
            >(pfunction),
            parg,
        );
    }
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_arm(
    timer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
    crate::compat::timer_compat::compat_timer_arm(timer.cast(), tmout, repeat);
}

#[cfg(feature = "wifi")]
pub unsafe extern "C" fn ets_timer_arm_us(
    timer: *mut crate::binary::c_types::c_void,
    tmout: u32,
    repeat: bool,
) {
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
    crate::time::ticks_to_micros(crate::time::systimer_count()) as i64
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
pub static mut __ESP_RADIO_G_MISC_NVS: u32 = 0;

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

#[allow(unused)]
pub(crate) unsafe fn phy_enable_clock() {
    // Stealing the peripheral is safe here, as they must have been passed into the relevant
    // initialization functions for the Wi-Fi or BLE controller, if this code gets executed.
    let clock_guard = unsafe { ModemClockControllerPeripheral::steal() }.enable_phy_clock();
    core::mem::forget(clock_guard);
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    unsafe { ModemClockControllerPeripheral::steal() }.decrease_phy_clock_ref_count();
}
pub(crate) fn enable_wifi_power_domain() {
    #[cfg(not(any(soc_has_pmu, esp32c2)))]
    {
        cfg_if::cfg_if! {
            if #[cfg(soc_has_lpwr)] {
                let rtc_cntl = esp_hal::peripherals::LPWR::regs();
            } else {
                let rtc_cntl = esp_hal::peripherals::RTC_CNTL::regs();
            }
        }

        rtc_cntl
            .dig_pwc()
            .modify(|_, w| w.wifi_force_pd().clear_bit());

        rtc_cntl
            .dig_iso()
            .modify(|_, w| w.wifi_force_iso().clear_bit());
    }
}

/// Get calibration data.
///
/// Returns the last calibration result.
///
/// If you see the data is different than what was persisted before, consider persisting the new
/// data.
pub fn phy_calibration_data(data: &mut [u8; esp_phy::PHY_CALIBRATION_DATA_LENGTH]) {
    // Although we're ignoring the result here, this doesn't change the behavior, as this just
    // doesn't do anything in case an error is returned.
    let _ = esp_phy::backup_phy_calibration_data(data);
}

/// Set calibration data.
///
/// This will be used next time the phy gets initialized.
pub fn set_phy_calibration_data(data: &[u8; core::mem::size_of::<esp_phy_calibration_data_t>()]) {
    // Although we're ignoring the result here, this doesn't change the behavior, as this just
    // doesn't do anything in case an error is returned.
    let _ = esp_phy::set_phy_calibration_data(data);
}
