use portable_atomic::{AtomicU32, Ordering};

use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::{
    binary::include::*,
    common_adapter::RADIO_CLOCKS,
    hal::{
        prelude::ram,
        system::{RadioClockController, RadioPeripherals},
    },
};

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;
static mut PHY_ACCESS_REF: AtomicU32 = AtomicU32::new(0);
static mut PHY_CLOCK_ENABLE_REF: AtomicU32 = AtomicU32::new(0);

pub(crate) fn enable_wifi_power_domain() {
    const DPORT_WIFIBB_RST: u32 = 1 << 0;
    const DPORT_FE_RST: u32 = 1 << 1;
    const DPORT_WIFIMAC_RST: u32 = 1 << 2;
    const DPORT_BTBB_RST: u32 = 1 << 3;
    const DPORT_BTMAC_RST: u32 = 1 << 4;
    const DPORT_RW_BTMAC_RST: u32 = 1 << 9;

    const MODEM_RESET_FIELD_WHEN_PU: u32 = DPORT_WIFIBB_RST
        | DPORT_FE_RST
        | DPORT_WIFIMAC_RST
        | DPORT_BTBB_RST
        | DPORT_BTMAC_RST
        | DPORT_RW_BTMAC_RST;

    unsafe {
        let rtc_cntl = &*crate::hal::peripherals::LPWR::ptr();

        let syscon = &*crate::hal::peripherals::SYSCON::ptr();

        rtc_cntl
            .dig_pwc()
            .modify(|_, w| w.wifi_force_pd().clear_bit());

        syscon
            .wifi_rst_en()
            .modify(|r, w| w.bits(r.bits() | MODEM_RESET_FIELD_WHEN_PU));
        syscon
            .wifi_rst_en()
            .modify(|r, w| w.bits(r.bits() & !MODEM_RESET_FIELD_WHEN_PU));

        rtc_cntl
            .dig_iso()
            .modify(|_, w| w.wifi_force_iso().clear_bit());
    }
}

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = SOC_PHY_DIG_REGS_MEM.as_ptr() as *mut u32;
    }
}

pub(crate) unsafe fn phy_enable() {
    let count = PHY_ACCESS_REF.fetch_add(1, Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            phy_enable_clock();

            if !G_IS_PHY_CALIBRATED {
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

            #[cfg(feature = "ble")]
            {
                extern "C" {
                    fn coex_pti_v2();
                }
                coex_pti_v2();
            }
        });
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable() {
    let count = PHY_ACCESS_REF.fetch_sub(1, Ordering::SeqCst);
    if count == 1 {
        critical_section::with(|_| {
            phy_digital_regs_store();
            // Disable PHY and RF.
            phy_close_rf();

            // Disable PHY temperature sensor
            phy_xpd_tsens();

            // Disable WiFi/BT common peripheral clock. Do not disable clock for hardware
            // RNG
            phy_disable_clock();
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
    let count = PHY_CLOCK_ENABLE_REF.fetch_add(1, Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            unwrap!(RADIO_CLOCKS.as_mut()).enable(RadioPeripherals::Phy);
        });

        trace!("phy_enable_clock done!");
    }
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    let count = PHY_CLOCK_ENABLE_REF.fetch_sub(1, Ordering::SeqCst);
    if count == 1 {
        critical_section::with(|_| {
            unwrap!(RADIO_CLOCKS.as_mut()).disable(RadioPeripherals::Phy);
        });

        trace!("phy_disable_clock done!");
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
#[no_mangle]
unsafe extern "C" fn phy_enter_critical() -> u32 {
    trace!("phy_enter_critical");

    core::mem::transmute(critical_section::acquire())
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
#[no_mangle]
unsafe extern "C" fn phy_exit_critical(level: u32) {
    trace!("phy_exit_critical {}", level);

    critical_section::release(core::mem::transmute::<u32, critical_section::RestoreState>(
        level,
    ));
}

#[no_mangle]
unsafe extern "C" fn abort() {
    trace!("misc_nvs_deinit")
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
pub static mut g_misc_nvs: &u32 = unsafe { &*core::ptr::addr_of!(NVS) };

pub static mut NVS: u32 = 0;
