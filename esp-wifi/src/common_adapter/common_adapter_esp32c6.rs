use portable_atomic::{AtomicU32, Ordering};

use crate::binary::include::*;

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

pub(crate) unsafe fn bbpll_en_usb() {
    #[cfg(phy_enable_usb)]
    {
        unsafe extern "C" {
            fn phy_bbpll_en_usb(param: bool);
        }

        unsafe {
            phy_bbpll_en_usb(true);
        }
    }
}

pub(crate) unsafe fn phy_enable() {
    let count = PHY_ACCESS_REF.fetch_add(1, Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            unsafe {
                super::phy_enable_clock();
            }

            if unsafe { !G_IS_PHY_CALIBRATED } {
                super::phy_calibrate();
                unsafe { G_IS_PHY_CALIBRATED = true };
            } else {
                unsafe {
                    phy_wakeup_init();
                }
                phy_digital_regs_load();
            }

            #[cfg(feature = "ble")]
            {
                unsafe extern "C" {
                    fn coex_pti_v2();
                }
                unsafe {
                    coex_pti_v2();
                }
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

                // Disable PHY temperature sensor
                phy_xpd_tsens();

                // #if CONFIG_IDF_TARGET_ESP32
                //         // Update WiFi MAC time before disable WiFi/BT common peripheral
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
