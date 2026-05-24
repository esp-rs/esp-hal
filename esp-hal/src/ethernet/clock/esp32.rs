//! EMAC clock configuration for ESP32.
//!
//! # RMII reference clock sources
//!
//! The ESP32 EMAC needs a 50 MHz reference clock for RMII.  There are two
//! hardware modes:
//!
//! - **[`ExternalRefClock`]** — the PHY drives `EMAC_TX_CLK`. Set
//!   [`RmiiPinBundle::clock`][crate::ethernet::RmiiPinBundle::clock] when calling
//!   [`Ethernet::new`][crate::ethernet::Ethernet::new]; use this when the PHY has an oscillator.
//!
//! - **[`ApllClock`]** — the ESP32 APLL is tuned to 50 MHz and the output is routed through
//!   `EMAC_CLK_OUT` or `EMAC_CLK_180`. The EMAC_EXT block feeds this clock back into the MAC. Use
//!   this when the PHY requires an external clock reference (e.g. LAN8720A in clock-output mode).
//!
//!   **Warning**: the APLL is shared with I²S and LCD. Requesting 50 MHz while another subsystem
//!   already uses the APLL at a different frequency will result in silent corruption. There is
//!   currently no shared-APLL arbitration in esp-hal. Also, APLL is unstable when Wi-Fi is
//!   active, so Wi-Fi must not be used together with this clock source.
//!
//! # MII clock source
//!
//! Using [`Ethernet::new`][crate::ethernet::Ethernet::new] with
//! [`MiiPinBundle`][crate::ethernet::MiiPinBundle] enables the EMAC_EXT MII clock buffers. In MII
//! mode the PHY drives both `TX_CLK` and `RX_CLK` at 25 MHz (100 Mbps) or 2.5 MHz (10 Mbps) — no
//! internal clock generation is required.

use esp_rom_sys::rom::ets_delay_us;

use crate::{
    efuse::ChipRevision,
    ethernet::{RmiiClkIn, RmiiClkOut, RmiiClockConfig},
    peripherals::{EMAC_EXT, LPWR},
    private::Sealed,
    soc::regi2c,
};

/// PHY interface selection for RMII mode in `EMAC_EXT.ex_phyinf_conf`.
pub(super) const PHY_INTF_RMII: u8 = 4;
/// PHY interface selection for MII mode in `EMAC_EXT.ex_phyinf_conf`.
pub(super) const PHY_INTF_MII: u8 = 0;

/// RMII reference clock provided externally by the PHY.
pub struct ExternalRefClock<P>(P);

impl<P> ExternalRefClock<P> {
    /// Wraps the GPIO pin that receives the PHY reference clock.
    pub fn new(pin: P) -> Self {
        Self(pin)
    }
}

impl<P> Sealed for ExternalRefClock<P> {}

impl<P: RmiiClkIn> RmiiClockConfig for ExternalRefClock<P> {
    fn configure(self) {
        // Configure the pad (IOMUX AF5, input buffer enabled).
        self.0.configure_iomux();

        EMAC_EXT::regs()
            .ex_phyinf_conf()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(PHY_INTF_RMII) });

        EMAC_EXT::regs().ex_oscclk_conf().modify(|_, w| {
            // clk_sel = 1: select external clock input
            w.clk_sel().set_bit()
        });

        EMAC_EXT::regs().ex_clk_ctrl().modify(|_, w| {
            w.ext_en().set_bit();
            w.int_en().clear_bit()
        });
    }
}

/// RMII reference clock generated internally by the ESP32 APLL at 50 MHz.
///
/// Should not be used together with Wi-Fi. No other APLL consumer must be active.
///
/// The APLL output is routed to GPIO16 (`EMAC_CLK_OUT`) or GPIO17
/// (`EMAC_CLK_180`).  Wrap the chosen GPIO:
/// ```rust,ignore
/// ApllClock::new(peripherals.GPIO16)
/// ```
pub struct ApllClock<P>(P);

impl<P> ApllClock<P> {
    /// Wraps the GPIO pin that outputs the APLL-generated reference clock.
    pub fn new(pin: P) -> Self {
        Self(pin)
    }
}

impl<P> Sealed for ApllClock<P> {}

impl<P: RmiiClkOut> RmiiClockConfig for ApllClock<P> {
    fn configure(self) {
        // Configure the APLL clock output pad.
        self.0.configure_iomux();

        // TODO: refactor into clock tree code

        // Reference formula:
        // apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536) / ((o_div + 2) * 2)
        //             ----------------------------------------------   -----------------
        //                  350 MHz <= Numerator <= 500 MHz                Denominator

        const APLL_ODIV_50MHZ: u8 = 2; // Fixed - prescribes numerator = 400 MHz

        // SDM = (400MHz / f_xtal) - 4; Q6.16
        let f_xtal = crate::clock::ll::xtal_clk_frequency();
        let f_xtal_mhz = f_xtal / 1_000_000;

        // SDM is a Q6.16 fixed-point number
        let sdm = ((400 << 16) / f_xtal_mhz) - (4 << 16);

        let sdm2 = (sdm >> 16) as u8;
        let sdm1 = ((sdm >> 8) & 0xff) as u8;
        let sdm0 = (sdm & 0xff) as u8;

        trace!("SDM2: {}, SDM1: {}, SDM0: {}", sdm2, sdm1, sdm0);

        // Power up the APLL.
        LPWR::regs().ana_conf().modify(|_, w| {
            w.plla_force_pd().clear_bit();
            w.plla_force_pu().set_bit()
        });

        regi2c::I2C_APLL_DSDM2.write_field(sdm2);
        regi2c::I2C_APLL_DSDM1.write_field(sdm1);
        regi2c::I2C_APLL_DSDM0.write_field(sdm0);
        // APLL configuration parameters
        const CLK_LL_APLL_SDM_STOP_VAL_1: u8 = 0x09;
        const CLK_LL_APLL_SDM_STOP_VAL_2_REV0: u8 = 0x69;
        const CLK_LL_APLL_SDM_STOP_VAL_2_REV1: u8 = 0x49;
        regi2c::I2C_APLL_SDM_CTRL.write_reg(CLK_LL_APLL_SDM_STOP_VAL_1);
        if crate::soc::chip_revision_above(ChipRevision::from_combined(100)) {
            regi2c::I2C_APLL_SDM_CTRL.write_reg(CLK_LL_APLL_SDM_STOP_VAL_2_REV1);
        } else {
            regi2c::I2C_APLL_SDM_CTRL.write_reg(CLK_LL_APLL_SDM_STOP_VAL_2_REV0);
        }
        regi2c::I2C_APLL_OR_OUTPUT_DIV.write_field(APLL_ODIV_50MHZ);

        // Trigger IR calibration / start SDM.
        const APLL_CALIBRATION_DELAY: u8 = 0x0F;
        const APLL_CALIBRATION_RSTB: u8 = 0x10;
        const APLL_CALIBRATION_START: u8 = 0x20;
        regi2c::I2C_APLL_IR_CAL.write_reg(APLL_CALIBRATION_DELAY);
        regi2c::I2C_APLL_IR_CAL
            .write_reg(APLL_CALIBRATION_DELAY | APLL_CALIBRATION_RSTB | APLL_CALIBRATION_START);
        // This seems wrong, is RSTB and START swapped?
        regi2c::I2C_APLL_IR_CAL.write_reg(APLL_CALIBRATION_DELAY | APLL_CALIBRATION_RSTB);

        // Wait for calibration to complete.
        while regi2c::I2C_APLL_OR_CAL_END.read() == 0 {
            // use ets_delay_us so the RTC bus doesn't get flooded
            ets_delay_us(1);
        }

        EMAC_EXT::regs()
            .ex_phyinf_conf()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(PHY_INTF_RMII) });

        EMAC_EXT::regs().ex_clkout_conf().modify(|_, w| unsafe {
            // div_num=0, h_div_num=0 → output divider = 1 (no division)
            w.div_num().bits(0);
            w.h_div_num().bits(0)
        });

        EMAC_EXT::regs().ex_oscclk_conf().modify(|_, w| {
            // clk_sel = 0: select internal (APLL) clock path
            w.clk_sel().clear_bit()
        });

        EMAC_EXT::regs().ex_clk_ctrl().modify(|_, w| {
            w.int_en().set_bit();
            w.ext_en().clear_bit()
        });
    }
}

// ── MiiClock ──────────────────────────────────────────────────────────────

/// MII clock configuration.
///
/// In MII mode the PHY drives both `TX_CLK` (GPIO0) and `RX_CLK` (GPIO5).
/// The EMAC_EXT block only needs the MII clock buffers enabled;
/// no internal clock source is required.
pub(crate) struct MiiClock;

impl MiiClock {
    pub(super) fn configure(&self) {
        EMAC_EXT::regs()
            .ex_phyinf_conf()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(PHY_INTF_MII) });

        EMAC_EXT::regs().ex_clk_ctrl().modify(|_, w| {
            w.mii_clk_tx_en().set_bit();
            w.mii_clk_rx_en().set_bit()
        });
    }
}
