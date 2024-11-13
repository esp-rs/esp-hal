use portable_atomic::{AtomicU32, Ordering};

use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::{
    binary::include::*,
    compat::common::str_from_c,
    hal::system::{RadioClockController, RadioPeripherals},
};

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;
static PHY_ACCESS_REF: AtomicU32 = AtomicU32::new(0);

pub(crate) fn enable_wifi_power_domain() {
    // In esp-idf, SOC_PMU_SUPPORTED is set which makes
    // `esp_wifi_bt_power_domain_on` a no-op.
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
            phy_enable_clock();

            if !G_IS_PHY_CALIBRATED {
                let mut cal_data: [u8; core::mem::size_of::<esp_phy_calibration_data_t>()] =
                    [0u8; core::mem::size_of::<esp_phy_calibration_data_t>()];

                let phy_version = get_phy_version_str();
                trace!("phy_version {}", str_from_c(phy_version as *const u8));

                let init_data = &PHY_INIT_DATA_DEFAULT;

                #[cfg(phy_enable_usb)]
                {
                    extern "C" {
                        fn phy_bbpll_en_usb(param: bool);
                    }

                    phy_bbpll_en_usb(true);
                }
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
            // Disable PHY and RF.
            phy_close_rf();

            // Disable PHY temperature sensor
            phy_xpd_tsens();

            // #if CONFIG_IDF_TARGET_ESP32
            //         // Update WiFi MAC time before disable WiFi/BT common peripheral
            // clock         phy_update_wifi_mac_time(true,
            // esp_timer_get_time()); #endif

            // Disable WiFi/BT common peripheral clock. Do not disable clock for hardware
            // RNG
            phy_disable_clock();
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

pub(crate) unsafe fn phy_enable_clock() {
    trace!("phy_enable_clock");
    // stealing RADIO_CLK is safe since it is passed (as mutable reference or by
    // value) into `init`
    let mut radio_clocks = unsafe { esp_hal::peripherals::RADIO_CLK::steal() };
    radio_clocks.enable(RadioPeripherals::Phy);
    trace!("phy_enable_clock done!");
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    trace!("phy_disable_clock");
    // stealing RADIO_CLK is safe since it is passed (as mutable reference or by
    // value) into `init`
    let mut radio_clocks = unsafe { esp_hal::peripherals::RADIO_CLK::steal() };
    radio_clocks.disable(RadioPeripherals::Phy);
    trace!("phy_disable_clock done!");
}
