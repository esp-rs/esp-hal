use portable_atomic::{AtomicU32, Ordering};

use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::{
    binary::include::*,
    hal::{peripherals::LPWR, ram},
};

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;
static PHY_ACCESS_REF: AtomicU32 = AtomicU32::new(0);

pub(crate) fn enable_wifi_power_domain() {
    LPWR::regs()
        .dig_pwc()
        .modify(|_, w| w.wifi_force_pd().clear_bit());

    LPWR::regs()
        .dig_iso()
        .modify(|_, w| w.wifi_force_iso().clear_bit());
}

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = core::ptr::addr_of_mut!(SOC_PHY_DIG_REGS_MEM).cast();
    }
}

pub(crate) unsafe fn phy_enable() {
    let count = PHY_ACCESS_REF.fetch_add(1, Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            // #if CONFIG_IDF_TARGET_ESP32
            //     // Update time stamp
            //     s_phy_rf_en_ts = esp_timer_get_time();
            //     // Update WiFi MAC time before WiFi/BT common clock is enabled
            //     phy_update_wifi_mac_time(false, s_phy_rf_en_ts);
            // #endif

            unsafe {
                super::phy_enable_clock();
            }

            if !unsafe { G_IS_PHY_CALIBRATED } {
                let mut cal_data: [u8; core::mem::size_of::<esp_phy_calibration_data_t>()] =
                    [0u8; core::mem::size_of::<esp_phy_calibration_data_t>()];

                let init_data = &PHY_INIT_DATA_DEFAULT;

                unsafe {
                    register_chipv7_phy(
                        init_data,
                        &mut cal_data as *mut _
                            as *mut crate::binary::include::esp_phy_calibration_data_t,
                        esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
                    );

                    G_IS_PHY_CALIBRATED = true;
                }
            } else {
                unsafe {
                    phy_wakeup_init();
                }
                phy_digital_regs_load();
            }

            #[cfg(coex)]
            unsafe {
                coex_bt_high_prio();
            }

            trace!("PHY ENABLE");
        });
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable() {
    let count = PHY_ACCESS_REF.fetch_sub(1, Ordering::SeqCst);
    if count == 1 {
        critical_section::with(|_| {
            phy_digital_regs_store();
            unsafe {
                // Disable PHY and RF.
                phy_close_rf();

                // #if CONFIG_IDF_TARGET_ESP32
                //         // Update WiFi MAC time before disalbe WiFi/BT common peripheral
                // clock         phy_update_wifi_mac_time(true,
                // esp_timer_get_time()); #endif

                // Disable WiFi/BT common peripheral clock. Do not disable clock for hardware
                // RNG
                super::phy_disable_clock();
            }
            trace!("PHY DISABLE");
        });
    }
}

fn phy_digital_regs_load() {
    unsafe {
        if S_IS_PHY_REG_STORED && !G_PHY_DIGITAL_REGS_MEM.is_null() {
            phy_dig_reg_backup(false, G_PHY_DIGITAL_REGS_MEM);
        }
    }
}

fn phy_digital_regs_store() {
    unsafe {
        if !G_PHY_DIGITAL_REGS_MEM.is_null() {
            phy_dig_reg_backup(true, G_PHY_DIGITAL_REGS_MEM);
            S_IS_PHY_REG_STORED = true;
        }
    }
}

/// **************************************************************************
/// Name: esp_dport_access_reg_read
///
/// Description:
///   Read regitser value safely in SMP
///
/// Input Parameters:
///   reg - Register address
///
/// Returned Value:
///   Register value
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_dport_access_reg_read(reg: u32) -> u32 {
    unsafe {
        // trace!("esp_dport_access_reg_read {:x} => {:x}", reg, res);
        (reg as *mut u32).read_volatile()
    }
}

/// **************************************************************************
/// Name: phy_enter_critical
///
/// Description:
///   Enter critical state
///
/// Input Parameters:
///   None
///
/// Returned Value:
///   CPU PS value
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_enter_critical() -> u32 {
    trace!("phy_enter_critical");

    unsafe { core::mem::transmute(critical_section::acquire()) }
}

/// **************************************************************************
/// Name: phy_exit_critical
///
/// Description:
///   Exit from critical state
///
/// Input Parameters:
///   level - CPU PS value
///
/// Returned Value:
///   None
///
/// *************************************************************************
#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn phy_exit_critical(level: u32) {
    trace!("phy_exit_critical {}", level);

    unsafe {
        critical_section::release(core::mem::transmute::<u32, critical_section::RestoreState>(
            level,
        ));
    }
}

#[ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn rtc_get_xtal() -> u32 {
    use esp_hal::clock::Clock;

    let xtal = crate::hal::rtc_cntl::RtcClock::xtal_freq();
    xtal.mhz()
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_deinit() {
    trace!("misc_nvs_deinit")
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_init() -> i32 {
    trace!("misc_nvs_init");
    0
}

#[unsafe(no_mangle)]
unsafe extern "C" fn misc_nvs_restore() -> i32 {
    todo!("misc_nvs_restore")
}

#[unsafe(no_mangle)]
static mut g_log_mod: i32 = 0;

#[unsafe(no_mangle)]
static mut g_log_level: i32 = 0;

#[unsafe(no_mangle)]
pub static mut g_misc_nvs: u32 = 0;
