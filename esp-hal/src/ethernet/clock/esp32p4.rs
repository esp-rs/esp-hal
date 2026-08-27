//! EMAC clock configuration for ESP32-P4.
//!
//! # RMII reference clock sources
//!
//! The ESP32-P4 EMAC needs a 50 MHz reference clock for RMII.
//!
//! - **[`ExternalRefClock`]** — the PHY drives the reference clock into one of the `EMAC_RMII_CLK`
//!   input pads (GPIO32, GPIO44, or GPIO50). This is the recommended configuration when the PHY has
//!   an integrated oscillator.
//!
//! - **[`InternalRefClock`]** — the ESP32-P4 MPLL generates a 50 MHz clock and drives it out on a
//!   `REF_50M_CLK` pad (GPIO23 or GPIO39). This signal must be looped back externally into one of
//!   the `EMAC_RMII_CLK` input pads.
//!
//! # MII clock source
//!
//! The PHY drives `TX_CLK` and `RX_CLK` separately at 25 MHz (100 Mbps) or
//! 2.5 MHz (10 Mbps). No internal clock generation is required.

use esp_rom_sys::rom::ets_delay_us;

use crate::{
    ethernet::{RmiiClkIn, RmiiClkOut, RmiiClockConfig},
    peripherals::{HP_SYS, HP_SYS_CLKRST, LP_AON_CLKRST},
    private::Sealed,
};

/// Number of spin-loop iterations to wait after enabling the EMAC clock tree.
const CLOCK_STABILIZE_US: u32 = 300;

// ── ExternalRefClock ─────────────────────────────────────────────────────────

/// RMII reference clock provided externally by the PHY.
///
/// The PHY must drive a 50 MHz clock into one of the `EMAC_RMII_CLK` input
/// pads: GPIO32, GPIO44, or GPIO50.
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
        self.0.configure_iomux();
        configure_rmii_input();
    }
}

// ── InternalRefClock ─────────────────────────────────────────────────────────

/// RMII reference clock derived from the ESP32-P4 MPLL at 50 MHz.
///
/// The MPLL output is driven out on a `REF_50M_CLK` output pad (GPIO23 or
/// GPIO39) and must be looped back externally into one of the `EMAC_RMII_CLK`
/// input pads (GPIO32, GPIO44, or GPIO50).
pub struct InternalRefClock<POut, PIn> {
    ref_clk_out: POut,
    ref_clk_in: PIn,
}

impl<POut, PIn> InternalRefClock<POut, PIn> {
    /// Creates an internal MPLL clock configuration.
    ///
    /// - `ref_clk_out` — the pad that will output the 50 MHz `REF_50M_CLK` signal (GPIO23 or
    ///   GPIO39).
    /// - `ref_clk_in` — the pad that receives the looped-back signal (GPIO32, GPIO44, or GPIO50).
    pub fn new(ref_clk_out: POut, ref_clk_in: PIn) -> Self {
        Self {
            ref_clk_out,
            ref_clk_in,
        }
    }
}

impl<POut, PIn> Sealed for InternalRefClock<POut, PIn> {}

impl<POut: RmiiClkOut, PIn: RmiiClkIn> RmiiClockConfig for InternalRefClock<POut, PIn> {
    fn configure(self) {
        self.ref_clk_out.configure_iomux();
        self.ref_clk_in.configure_iomux();
        enable_mpll_50m_output();
        configure_rmii_input();
        // Override pad_emac_ref_clk_en to enable the clock output pad
        // (configure_rmii_input clears it; the MPLL topology needs it set).
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl00()
            .modify(|_, w| w.pad_emac_ref_clk_en().set_bit());
    }
}

// ── MiiClock ─────────────────────────────────────────────────────────────────

/// MII clock configuration.
///
/// In MII mode the PHY drives `TX_CLK` and `RX_CLK` separately
/// (25 MHz at 100 Mbps, 2.5 MHz at 10 Mbps). No internal clock generation is
/// required.
pub(crate) struct MiiClock;

impl MiiClock {
    pub(super) fn configure(&self) {
        configure_mii();
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Configures the EMAC clock tree for RMII with an external clock input.
///
/// Matches `emac_ll_clock_enable_rmii_input()` in esp-idf.
fn configure_rmii_input() {
    HP_SYS::regs()
        .gmac_ctrl0()
        .modify(|_, w| unsafe { w.phy_intf_sel().bits(4) }); // 4 = RMII

    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl00()
        .modify(|_, w| unsafe {
            // pad_emac_ref_clk_en stays clear for external-input topology;
            // for MPLL output it is set afterwards by the caller.
            w.pad_emac_ref_clk_en().clear_bit();
            // Source: 0 = pad_emac_txrx_clk (the combined RMII ref pad).
            w.emac_rmii_clk_src_sel().bits(0);
            w.emac_rmii_clk_en().set_bit();
            w.emac_rx_clk_src_sel().clear_bit(); // 0 = pad_emac_txrx_clk
            w.emac_rx_clk_en().set_bit()
        });

    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl01()
        .modify(|_, w| unsafe {
            w.emac_tx_clk_src_sel().clear_bit(); // 0 = pad_emac_txrx_clk
            w.emac_tx_clk_en().set_bit();
            w.emac_rx_clk_div_num().bits(1); // div 1 = 100 Mbps default
            w.emac_tx_clk_div_num().bits(1)
        });

    // LP pad gate: use the combined txrx clock pad, not the separate tx/rx pads.
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| {
            w.lp_aonclkrst_hp_pad_emac_tx_clk_en().clear_bit();
            w.lp_aonclkrst_hp_pad_emac_rx_clk_en().clear_bit();
            w.lp_aonclkrst_hp_pad_emac_txrx_clk_en().set_bit()
        });

    deassert_reset();
    clock_stabilize();
}

/// Enables the MPLL 500 MHz source and derives the 50 MHz `REF_50M_CLK` output.
///
/// Divider: 500 MHz / (9 + 1) = 50 MHz.
///
/// Must be called before `configure_rmii_input()` for the MPLL topology, since
/// `configure_rmii_input()` clears `pad_emac_ref_clk_en` and the caller must
/// restore it afterwards.
fn enable_mpll_50m_output() {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| w.lp_aonclkrst_hp_mpll_500m_clk_en().set_bit());

    HP_SYS_CLKRST::regs()
        .ref_clk_ctrl0()
        .modify(|_, w| unsafe { w.ref_50m_clk_div_num().bits(9) });

    HP_SYS_CLKRST::regs()
        .ref_clk_ctrl1()
        .modify(|_, w| w.ref_50m_clk_en().set_bit());
}

/// Configures the EMAC clock tree for MII mode.
///
/// Matches `emac_ll_clock_enable_mii()` in esp-idf. In MII mode the RX and TX
/// clocks come from separate pads (not the combined RMII pad), so
/// `emac_rx_clk_src_sel = 1` and `emac_tx_clk_src_sel = 1`.
fn configure_mii() {
    HP_SYS::regs()
        .gmac_ctrl0()
        .modify(|_, w| unsafe { w.phy_intf_sel().bits(0) }); // 0 = MII

    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl00()
        .modify(|_, w| unsafe {
            w.pad_emac_ref_clk_en().clear_bit();
            w.emac_rmii_clk_en().clear_bit();
            w.emac_rmii_clk_src_sel().bits(0);
            w.emac_rx_clk_src_sel().set_bit(); // 1 = pad_emac_rx_clk
            w.emac_rx_clk_en().set_bit()
        });

    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl01()
        .modify(|_, w| unsafe {
            w.emac_tx_clk_src_sel().set_bit(); // 1 = pad_emac_tx_clk
            w.emac_tx_clk_en().set_bit();
            w.emac_rx_clk_div_num().bits(0); // div 0 = 25 MHz (no division)
            w.emac_tx_clk_div_num().bits(0)
        });

    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| {
            w.lp_aonclkrst_hp_pad_emac_tx_clk_en().set_bit();
            w.lp_aonclkrst_hp_pad_emac_rx_clk_en().set_bit()
        });

    deassert_reset();
    clock_stabilize();
}

/// Pulses the EMAC peripheral reset via `LP_AON_CLKRST` (assert then deassert).
///
/// Matches `emac_ll_reset_register()` in esp-idf. The `GenericPeripheralGuard`
/// for EMAC on P4 does not perform a hardware reset (the reset handler is a
/// no-op), so this must be called explicitly during clock configuration.
fn deassert_reset() {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_sdmmc_emac_rst_ctrl()
        .modify(|_, w| {
            w.lp_aonclkrst_rst_en_emac().set_bit();
            w.lp_aonclkrst_force_norst_emac().clear_bit()
        });
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_sdmmc_emac_rst_ctrl()
        .modify(|_, w| w.lp_aonclkrst_rst_en_emac().clear_bit());
}

/// Spin-waits for the EMAC clock tree to stabilise after configuration.
fn clock_stabilize() {
    ets_delay_us(CLOCK_STABILIZE_US);
}
