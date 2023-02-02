use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::binary::include::*;
use atomic_polyfill::AtomicU32;
use esp32_hal::prelude::ram;
use log::trace;

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;
static mut PHY_ACCESS_REF: AtomicU32 = AtomicU32::new(0);
static mut PHY_CLOCK_ENABLE_REF: AtomicU32 = AtomicU32::new(0);

// Mask for clock bits used by both WIFI and Bluetooth
const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;

const DR_REG_DPORT_BASE: u32 = 0x3ff00000;
const DPORT_WIFI_CLK_EN_REG: u32 = DR_REG_DPORT_BASE + 0x0CC;

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = SOC_PHY_DIG_REGS_MEM.as_ptr() as *mut u32;
    }
}

pub(crate) unsafe fn phy_enable() {
    let count = PHY_ACCESS_REF.fetch_add(1, atomic_polyfill::Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            // #if CONFIG_IDF_TARGET_ESP32
            //     // Update time stamp
            //     s_phy_rf_en_ts = esp_timer_get_time();
            //     // Update WiFi MAC time before WiFi/BT common clock is enabled
            //     phy_update_wifi_mac_time(false, s_phy_rf_en_ts);
            // #endif

            phy_enable_clock();

            if G_IS_PHY_CALIBRATED == false {
                let mut cal_data: [u8; core::mem::size_of::<esp_phy_calibration_data_t>()] =
                    [0u8; core::mem::size_of::<esp_phy_calibration_data_t>()];

                let init_data = &PHY_INIT_DATA_DEFAULT;

                register_chipv7_phy(
                    init_data,
                    &mut cal_data as *mut _
                        as *mut crate::binary::include::esp_phy_calibration_data_t,
                    esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
                );

                G_IS_PHY_CALIBRATED = true;
            } else {
                phy_wakeup_init();
                phy_digital_regs_load();
            }

            #[cfg(coex)]
            coex_bt_high_prio();

            log::trace!("PHY ENABLE");
        });
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable() {
    let count = PHY_ACCESS_REF.fetch_sub(1, atomic_polyfill::Ordering::SeqCst);
    if count == 1 {
        critical_section::with(|_| {
            phy_digital_regs_store();
            // Disable PHY and RF.
            phy_close_rf();

            // #if CONFIG_IDF_TARGET_ESP32
            //         // Update WiFi MAC time before disalbe WiFi/BT common peripheral clock
            //         phy_update_wifi_mac_time(true, esp_timer_get_time());
            // #endif

            // Disable WiFi/BT common peripheral clock. Do not disable clock for hardware RNG
            phy_disable_clock();
            log::trace!("PHY DISABLE");
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

pub(crate) unsafe fn phy_enable_clock() {
    trace!("phy_enable_clock");

    let count = PHY_CLOCK_ENABLE_REF.fetch_add(1, atomic_polyfill::Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            let ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
            let old = ptr.read_volatile();
            ptr.write_volatile(old | DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
        });
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    trace!("phy_disable_clock");

    let count = PHY_CLOCK_ENABLE_REF.fetch_sub(1, atomic_polyfill::Ordering::SeqCst);
    if count == 1 {
        critical_section::with(|_| {
            let ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
            let old = ptr.read_volatile();
            ptr.write_volatile(old & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
        });
    }
}

#[ram]
pub(crate) unsafe extern "C" fn read_mac(
    mac: *mut u8,
    type_: u32,
) -> crate::binary::c_types::c_int {
    trace!("read_mac {:p} {}", mac, type_);

    let mut regval = [0u32; 2];
    let data = &regval as *const _ as *const u8;
    regval[0] = (0x3ff5a004 as *const u32).read_volatile();
    regval[1] = (0x3ff5a008 as *const u32).read_volatile();

    for i in 0..6 {
        mac.offset(i)
            .write_volatile(data.offset(5 - i).read_volatile());
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

#[ram]
pub(crate) fn init_clocks() {
    unsafe {
        let mut regval = getreg32(0x6001f048); /* DR_REG_BB_BASE+48 */
        regval &= !(1 << 14);
        putreg32(regval, 0x6001f048);
    }
}

#[inline(always)]
unsafe fn putreg32(v: u32, r: u32) {
    (r as *mut u32).write_volatile(v);
}

#[inline(always)]
unsafe fn getreg32(r: u32) -> u32 {
    (r as *mut u32).read_volatile()
}

#[allow(unused)]
pub(crate) fn wifi_reset_mac() {
    const SYSCON_WIFI_RST_EN_REG: *mut u32 = (0x3ff00000 + 0xD0) as *mut u32;
    const SYSTEM_MAC_RST: u32 = 1 << 2;

    unsafe {
        SYSCON_WIFI_RST_EN_REG
            .write_volatile(SYSCON_WIFI_RST_EN_REG.read_volatile() | SYSTEM_MAC_RST);
        SYSCON_WIFI_RST_EN_REG
            .write_volatile(SYSCON_WIFI_RST_EN_REG.read_volatile() & !SYSTEM_MAC_RST);
    }
}

/****************************************************************************
 * Name: esp_dport_access_reg_read
 *
 * Description:
 *   Read regitser value safely in SMP
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

#[ram]
#[no_mangle]
unsafe extern "C" fn esp_dport_access_reg_read(reg: u32) -> u32 {
    let res = (reg as *mut u32).read_volatile();
    //trace!("esp_dport_access_reg_read {:x} => {:x}", reg, res);
    res
}

/****************************************************************************
 * Name: phy_enter_critical
 *
 * Description:
 *   Enter critical state
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU PS value
 *
 ****************************************************************************/
#[ram]
#[no_mangle]
unsafe extern "C" fn phy_enter_critical() -> u32 {
    trace!("phy_enter_critical");

    core::mem::transmute(critical_section::acquire())
}

/****************************************************************************
 * Name: phy_exit_critical
 *
 * Description:
 *   Exit from critical state
 *
 * Input Parameters:
 *   level - CPU PS value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#[ram]
#[no_mangle]
unsafe extern "C" fn phy_exit_critical(level: u32) {
    trace!("phy_exit_critical {}", level);

    critical_section::release(core::mem::transmute(level));
}

#[ram]
#[no_mangle]
unsafe extern "C" fn rtc_get_xtal() -> u32 {
    trace!("rtc_get_xtal - just hardcoded value 40 for now");
    40
}

#[no_mangle]
unsafe extern "C" fn misc_nvs_deinit() {
    trace!("misc_nvs_deinit")
}

#[no_mangle]
unsafe extern "C" fn misc_nvs_init() -> i32 {
    trace!("misc_nvs_init");
    0
}

#[no_mangle]
unsafe extern "C" fn misc_nvs_restore() -> i32 {
    todo!("misc_nvs_restore")
}

#[no_mangle]
static mut g_log_mod: i32 = 0;

#[no_mangle]
static mut g_log_level: i32 = 0;

#[no_mangle]
pub static mut g_misc_nvs: u32 = 0;
