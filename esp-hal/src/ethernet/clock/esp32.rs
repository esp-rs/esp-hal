//! EMAC clock configuration for ESP32.
//!
//! # RMII reference clock sources
//!
//! The ESP32 EMAC needs a 50 MHz reference clock for RMII.  There are two
//! hardware modes:
//!
//! - **[`ExternalRefClock`]** — the PHY drives the reference clock into `EMAC_TX_CLK`. Pass an
//!   instance of this struct wrapping the chosen GPIO to
//!   [`Ethernet::new_rmii`][crate::ethernet::Ethernet::new_rmii]. This is the recommended
//!   configuration when the PHY has an integrated oscillator.
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
//! Using [`Ethernet::new_mii`][crate::ethernet::Ethernet::new_mii] enables the EMAC_EXT MII clock
//! buffers. In MII mode the PHY drives both `TX_CLK` and `RX_CLK` at 25 MHz (100 Mbps)
//! or 2.5 MHz (10 Mbps) — no internal clock generation is required.

use crate::{
    ethernet::{EmacClkOut, EmacRmiiClkIn, RmiiClockConfig},
    peripherals::{EMAC_EXT, LPWR},
    private::Sealed,
    soc::regi2c,
};

/// Pre-computed APLL parameters for 50 MHz output from a 40 MHz crystal.
///
/// Formula: `f_apll = f_xtal * (4 + sdm2 + sdm1/256 + sdm0/65536) / ((o_div + 2) * 2)`
/// → `40 * (4 + 6 + 0 + 0) / ((2 + 2) * 2) = 40 * 10 / 8 = 50 MHz` ✓
const APLL_SDM2_50MHZ: u8 = 6;
const APLL_SDM1_50MHZ: u8 = 0;
const APLL_SDM0_50MHZ: u8 = 0;
const APLL_ODIV_50MHZ: u8 = 2;

/// RMII reference clock provided externally by the PHY.
pub struct ExternalRefClock<P>(P);

impl<P> ExternalRefClock<P> {
    /// Wraps the GPIO pin that receives the PHY reference clock.
    pub fn new(pin: P) -> Self {
        Self(pin)
    }
}

impl<P> Sealed for ExternalRefClock<P> {}

impl<P: EmacRmiiClkIn> RmiiClockConfig for ExternalRefClock<P> {
    fn configure(self) {
        // Configure the pad (IOMUX AF5, input buffer enabled).
        self.0.configure_iomux();

        EMAC_EXT::regs()
            .ex_phyinf_conf()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(super::mac::PHY_INTF_RMII) });

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

impl<P: EmacClkOut> RmiiClockConfig for ApllClock<P> {
    fn configure(self) {
        // Configure the APLL clock output pad.
        self.0.configure_iomux();

        // Power up the APLL.
        LPWR::regs().ana_conf().modify(|_, w| {
            w.plla_force_pd().clear_bit();
            w.plla_force_pu().set_bit()
        });

        // Program Σ-Δ modulator coefficients for 50 MHz (40 MHz XTAL assumed).
        regi2c::I2C_APLL_DSDM2.write_field(APLL_SDM2_50MHZ);
        regi2c::I2C_APLL_DSDM1.write_field(APLL_SDM1_50MHZ);
        regi2c::I2C_APLL_DSDM0.write_field(APLL_SDM0_50MHZ);
        regi2c::I2C_APLL_OR_OUTPUT_DIV.write_field(APLL_ODIV_50MHZ);

        // Release SDM reset, then start SDM.
        regi2c::I2C_APLL_SDM_RSTB.write_field(1);
        regi2c::I2C_APLL_SDM_STOP.write_field(0);

        // Trigger IR calibration.
        regi2c::I2C_APLL_IR_CAL_START.write_field(1);

        EMAC_EXT::regs()
            .ex_phyinf_conf()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(super::mac::PHY_INTF_RMII) });

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
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(super::mac::PHY_INTF_MII) });

        EMAC_EXT::regs().ex_clk_ctrl().modify(|_, w| {
            w.mii_clk_tx_en().set_bit();
            w.mii_clk_rx_en().set_bit()
        });
    }
}
