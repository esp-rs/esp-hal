use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::binary::include::*;
use atomic_polyfill::AtomicU32;
use log::trace;

const SOC_PHY_DIG_REGS_MEM_SIZE: usize = 21 * 4;

static mut SOC_PHY_DIG_REGS_MEM: [u8; SOC_PHY_DIG_REGS_MEM_SIZE] = [0u8; SOC_PHY_DIG_REGS_MEM_SIZE];
static mut G_IS_PHY_CALIBRATED: bool = false;
static mut G_PHY_DIGITAL_REGS_MEM: *mut u32 = core::ptr::null_mut();
static mut S_IS_PHY_REG_STORED: bool = false;
static mut PHY_ACCESS_REF: AtomicU32 = AtomicU32::new(0);

pub(crate) fn phy_mem_init() {
    unsafe {
        G_PHY_DIGITAL_REGS_MEM = SOC_PHY_DIG_REGS_MEM.as_ptr() as *mut u32;
    }
}

pub(crate) unsafe fn phy_enable() {
    let count = PHY_ACCESS_REF.fetch_add(1, atomic_polyfill::Ordering::SeqCst);
    if count == 0 {
        critical_section::with(|_| {
            phy_enable_clock();

            if G_IS_PHY_CALIBRATED == false {
                let mut cal_data: [u8; core::mem::size_of::<esp_phy_calibration_data_t>()] =
                    [0u8; core::mem::size_of::<esp_phy_calibration_data_t>()];

                let init_data = &PHY_INIT_DATA_DEFAULT;

                #[cfg(feature = "phy_enable_usb")]
                {
                    extern "C" {
                        pub fn phy_bbpll_en_usb(param: bool);
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

            // Disable PHY temperature sensor
            phy_xpd_tsens();

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
    const SYSTEM_WIFI_CLK_EN_REG: u32 = 0x60026000 + 0x014;
    critical_section::with(|_| {
        (SYSTEM_WIFI_CLK_EN_REG as *mut u32)
            .write_volatile((SYSTEM_WIFI_CLK_EN_REG as *mut u32).read_volatile() | 0x78078F);
    });

    trace!("phy_enable_clock done!");
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    trace!("phy_disable_clock");
    const SYSTEM_WIFI_CLK_EN_REG: u32 = 0x60026000 + 0x014;
    critical_section::with(|_| {
        (SYSTEM_WIFI_CLK_EN_REG as *mut u32)
            .write_volatile((SYSTEM_WIFI_CLK_EN_REG as *mut u32).read_volatile() & !0x78078F);
    });

    trace!("phy_enable_clock done!");
}

pub(crate) unsafe extern "C" fn read_mac(
    mac: *mut u8,
    type_: u32,
) -> crate::binary::c_types::c_int {
    trace!("read_mac {:p} {}", mac, type_);

    let mut regval = [0u32; 2];
    let data = &regval as *const _ as *const u8;
    regval[0] = ((0x60007000 + 0x44) as *const u32).read_volatile();
    regval[1] = ((0x60007000 + 0x48) as *const u32).read_volatile();

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

pub(crate) fn init_clocks() {
    unsafe {
        // PERIP_CLK_EN0
        ((0x600c0000 + 0x18) as *mut u32).write_volatile(0xffffffff);
        // PERIP_CLK_EN1
        ((0x600c0000 + 0x1C) as *mut u32).write_volatile(0xffffffff);
    }

    // APB_CTRL_WIFI_CLK_EN_REG
    unsafe {
        ((0x60026000 + 0x14) as *mut u32).write_volatile(0xffffffff);
    }

    unsafe {
        const RTC_CNTL_DIG_PWC_REG: *mut u32 = (0x60008000 + 0x90) as *mut u32;
        const RTC_CNTL_WIFI_FORCE_PD: u32 = 1 << 17;
        const SYSCON_WIFI_RST_EN_REG: *mut u32 = (0x60026000 + 0x18) as *mut u32;
        const SYSTEM_WIFIBB_RST: u32 = 1 << 0;
        const SYSTEM_FE_RST: u32 = 1 << 1;
        const RTC_CNTL_DIG_ISO_REG: *mut u32 = (0x60008000 + 0x94) as *mut u32;
        const RTC_CNTL_WIFI_FORCE_ISO: u32 = 1 << 28;

        critical_section::with(|_| {
            RTC_CNTL_DIG_PWC_REG
                .write_volatile(RTC_CNTL_DIG_PWC_REG.read_volatile() & !RTC_CNTL_WIFI_FORCE_PD);
            SYSCON_WIFI_RST_EN_REG.write_volatile(
                SYSCON_WIFI_RST_EN_REG.read_volatile() | SYSTEM_WIFIBB_RST | SYSTEM_FE_RST,
            );
            SYSCON_WIFI_RST_EN_REG.write_volatile(
                SYSCON_WIFI_RST_EN_REG.read_volatile() & !(SYSTEM_WIFIBB_RST | SYSTEM_FE_RST),
            );
            RTC_CNTL_DIG_ISO_REG
                .write_volatile(RTC_CNTL_DIG_ISO_REG.read_volatile() & !RTC_CNTL_WIFI_FORCE_ISO);

            RTC_CNTL_DIG_PWC_REG.write_volatile(0x545010);
            RTC_CNTL_DIG_ISO_REG.write_volatile(0xaa805080);
        });
    }
}

#[allow(unused)]
pub(crate) fn wifi_reset_mac() {
    const SYSCON_WIFI_RST_EN_REG: *mut u32 = (0x60026000 + 0x18) as *mut u32;
    const SYSTEM_MAC_RST: u32 = 1 << 2;

    unsafe {
        SYSCON_WIFI_RST_EN_REG
            .write_volatile(SYSCON_WIFI_RST_EN_REG.read_volatile() | SYSTEM_MAC_RST);
        SYSCON_WIFI_RST_EN_REG
            .write_volatile(SYSCON_WIFI_RST_EN_REG.read_volatile() & !SYSTEM_MAC_RST);
    }
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
pub static mut g_misc_nvs: u32 = 0;
