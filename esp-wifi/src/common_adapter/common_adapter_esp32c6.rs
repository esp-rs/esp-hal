use super::phy_init_data::PHY_INIT_DATA_DEFAULT;
use crate::binary::include::*;
use crate::compat::common::StrBuf;
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

                let phy_version = get_phy_version_str();
                trace!("phy_version {}", StrBuf::from(phy_version).as_str_ref());

                let init_data = &PHY_INIT_DATA_DEFAULT;

                #[cfg(feature = "phy-enable-usb")]
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
    const MODEM_LPCON: u32 = 0x600AF000;
    const CLK_CONF: u32 = MODEM_LPCON + 6 * 4;
    const I2C_MST_CLK_CONF: u32 = MODEM_LPCON + 4 * 4;

    unsafe {
        let clk_conf = CLK_CONF as *mut u32;
        clk_conf.write_volatile(
            clk_conf.read_volatile() | 1 << 2, // clk_i2c_mst_en
        );

        let i2c_mst_clk_conf = I2C_MST_CLK_CONF as *mut u32;
        i2c_mst_clk_conf.write_volatile(
            i2c_mst_clk_conf.read_volatile() | 1 << 0, // clk_i2c_mst_sel_160m
        );
    }

    trace!("phy_enable_clock done!");
}

#[allow(unused)]
pub(crate) unsafe fn phy_disable_clock() {
    trace!("phy_disable_clock");
    const MODEM_LPCON: u32 = 0x600AF000;
    const CLK_CONF: u32 = MODEM_LPCON + 6 * 4;
    const I2C_MST_CLK_CONF: u32 = MODEM_LPCON + 4 * 4;

    unsafe {
        let clk_conf = CLK_CONF as *mut u32;
        clk_conf.write_volatile(
            clk_conf.read_volatile()
                & !(
                    1 << 2
                    // clk_i2c_mst_en
                ),
        );

        let i2c_mst_clk_conf = I2C_MST_CLK_CONF as *mut u32;
        i2c_mst_clk_conf.write_volatile(
            i2c_mst_clk_conf.read_volatile()
                & !(
                    1 << 0
                    // clk_i2c_mst_sel_160m
                ),
        );
    }

    trace!("phy_enable_clock done!");
}

pub(crate) fn init_clocks() {
    unsafe {
        let pmu = &*esp32c6::PMU::PTR;

        pmu.hp_sleep_icg_modem
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().variant(0));
        pmu.hp_modem_icg_modem
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().variant(1));
        pmu.hp_active_icg_modem
            .modify(|_, w| w.hp_active_dig_icg_modem_code().variant(2));
        pmu.imm_modem_icg
            .as_ptr()
            .write_volatile(pmu.imm_modem_icg.as_ptr().read_volatile() | 1 << 31);
        pmu.imm_sleep_sysclk
            .as_ptr()
            .write_volatile(pmu.imm_sleep_sysclk.as_ptr().read_volatile() | 1 << 28);

        let syscon_clk_conf_power_st = (0x600A9800 + 12) as *mut u32;
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 6 << 28);
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 4 << 24);
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 6 << 20);
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 6 << 16);
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 6 << 12);
        syscon_clk_conf_power_st.write_volatile(syscon_clk_conf_power_st.read_volatile() | 6 << 8);

        let lp_clk_conf_power_st = (MODEM_LPCON + 8 * 4) as *mut u32;
        lp_clk_conf_power_st.write_volatile(lp_clk_conf_power_st.read_volatile() | 6 << 28);
        lp_clk_conf_power_st.write_volatile(lp_clk_conf_power_st.read_volatile() | 6 << 24);
        lp_clk_conf_power_st.write_volatile(lp_clk_conf_power_st.read_volatile() | 6 << 20);
        lp_clk_conf_power_st.write_volatile(lp_clk_conf_power_st.read_volatile() | 6 << 16);

        const MODEM_LPCON: u32 = 0x600AF000;
        let wifi_lp_clk_con = (MODEM_LPCON + 4 * 3) as *mut u32;
        const CLK_WIFIPWR_LP_SEL_OSC_SLOW: u32 = 0;
        const CLK_WIFIPWR_LP_SEL_OSC_FAST: u32 = 1;
        const CLK_WIFIPWR_LP_SEL_XTAL32K: u32 = 3;
        const CLK_WIFIPWR_LP_SEL_XTAL: u32 = 2;
        const CLK_WIFIPWR_LP_DIV_NUM_SHIFT: u32 = 4;
        const CLK_WIFIPWR_LP_DIV_NUM_MASK: u32 = 0b1111_1111_1111;
        const CLK_WIFIPWR_EN: u32 = 0;

        // modem_clock_hal_deselect_all_wifi_lpclk_source
        wifi_lp_clk_con.write_volatile(
            wifi_lp_clk_con.read_volatile()
                & !(1 << CLK_WIFIPWR_LP_SEL_OSC_SLOW
                    | 1 << CLK_WIFIPWR_LP_SEL_OSC_FAST
                    | 1 << CLK_WIFIPWR_LP_SEL_XTAL32K
                    | 1 << CLK_WIFIPWR_LP_SEL_XTAL),
        );

        // modem_clock_hal_select_wifi_lpclk_source
        wifi_lp_clk_con
            .write_volatile(wifi_lp_clk_con.read_volatile() | 1 << CLK_WIFIPWR_LP_SEL_OSC_SLOW);

        // modem_lpcon_ll_set_wifi_lpclk_divisor_value
        wifi_lp_clk_con.write_volatile(
            wifi_lp_clk_con.read_volatile()
                & !(CLK_WIFIPWR_LP_DIV_NUM_MASK << CLK_WIFIPWR_LP_DIV_NUM_SHIFT)
                | 0 << CLK_WIFIPWR_LP_DIV_NUM_SHIFT,
        );

        // modem_lpcon_ll_enable_wifipwr_clock
        let clk_conf = (MODEM_LPCON + 6 * 3) as *mut u32;
        clk_conf.write_volatile(clk_conf.read_volatile() | 1 << CLK_WIFIPWR_EN);
    }
}

#[allow(unused)]
pub(crate) fn wifi_reset_mac() {
    // empty
}

#[no_mangle]
pub extern "C" fn rtc_clk_xtal_freq_get() -> i32 {
    // JUST SUPPORT 40MHz XTAL for now
    40
}
