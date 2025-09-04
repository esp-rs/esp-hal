use crate::binary::include::*;

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = core::ptr::addr_of_mut!(SOC_PHY_DIG_REGS_MEM).cast();
    }
}

pub(crate) fn enable_wifi_power_domain() {
    const SYSTEM_WIFIBB_RST: u32 = 1 << 0;
    const SYSTEM_FE_RST: u32 = 1 << 1;
    const SYSTEM_WIFIMAC_RST: u32 = 1 << 2;
    const SYSTEM_BTBB_RST: u32 = 1 << 3; // Bluetooth Baseband
    const SYSTEM_BTMAC_RST: u32 = 1 << 4; // deprecated
    const SYSTEM_RW_BTMAC_RST: u32 = 1 << 9; // Bluetooth MAC
    const SYSTEM_RW_BTMAC_REG_RST: u32 = 1 << 11; // Bluetooth MAC Regsiters
    const SYSTEM_BTBB_REG_RST: u32 = 1 << 13; // Bluetooth Baseband Registers

    const MODEM_RESET_FIELD_WHEN_PU: u32 = SYSTEM_WIFIBB_RST
        | SYSTEM_FE_RST
        | SYSTEM_WIFIMAC_RST
        | SYSTEM_BTBB_RST
        | SYSTEM_BTMAC_RST
        | SYSTEM_RW_BTMAC_RST
        | SYSTEM_RW_BTMAC_REG_RST
        | SYSTEM_BTBB_REG_RST;

    unsafe {
        let rtc_cntl = &*crate::hal::peripherals::LPWR::ptr();
        let syscon = &*crate::hal::peripherals::APB_CTRL::ptr();

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
        unsafe extern "C" {
            fn coex_pti_v2();
        }
        unsafe {
            coex_pti_v2();
        }
    }

    trace!("PHY ENABLE");
}

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

    trace!("PHY DISABLE");
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
pub static mut __ESP_RADIO_G_MISC_NVS: u32 = 0;
