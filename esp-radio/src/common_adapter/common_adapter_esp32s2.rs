use crate::{
    binary::include::*,
    hal::{
        peripherals::{LPWR, SYSCON},
        ram,
    },
};

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;

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

    LPWR::regs()
        .dig_pwc()
        .modify(|_, w| w.wifi_force_pd().clear_bit());

    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | MODEM_RESET_FIELD_WHEN_PU) });

    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !MODEM_RESET_FIELD_WHEN_PU) });

    LPWR::regs()
        .dig_iso()
        .modify(|_, w| w.wifi_force_iso().clear_bit());
}

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = core::ptr::addr_of_mut!(SOC_PHY_DIG_REGS_MEM).cast();
    }
}

pub(crate) unsafe fn bbpll_en_usb() {
    // nothing for ESP32-S2
}

pub(super) unsafe fn phy_enable_inner() {
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
        extern "C" {
            fn coex_pti_v2();
        }
        unsafe {
            coex_pti_v2();
        }
    }
}

#[allow(unused)]
pub(super) unsafe fn phy_disable_inner() {
    phy_digital_regs_store();
    unsafe {
        // Disable PHY and RF.
        phy_close_rf();

        // Disable PHY temperature sensor
        phy_xpd_tsens();

        // Disable WiFi/BT common peripheral clock. Do not disable clock for hardware
        // RNG
        super::phy_disable_clock();
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
unsafe extern "C" fn __esp_radio_phy_enter_critical() -> u32 {
    trace!("phy_enter_critical");

    unsafe { crate::ESP_RADIO_LOCK.acquire().inner() }
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
unsafe extern "C" fn __esp_radio_phy_exit_critical(level: u32) {
    trace!("phy_exit_critical {}", level);

    unsafe {
        let token = esp_sync::RestoreState::new(level);
        crate::ESP_RADIO_LOCK.release(token);
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_deinit() {
    trace!("misc_nvs_deinit")
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
    trace!("misc_nvs_init");
    0
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_misc_nvs_restore() -> i32 {
    todo!("misc_nvs_restore")
}

#[unsafe(no_mangle)]
static mut __ESP_RADIO_G_LOG_LEVEL: i32 = 0;

#[unsafe(no_mangle)]
pub static mut __ESP_RADIO_G_MISC_NVS: &u32 = unsafe { &*core::ptr::addr_of!(NVS) };

pub static mut NVS: u32 = 0;
