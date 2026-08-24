//! EMAC clock configuration for ESP32-S31.
//!
//! # RMII reference clock sources
//!
//! The ESP32-S31 EMAC needs a 50 MHz reference clock for RMII.
//!
//! - **[`ExternalRefClock`]** — the PHY drives the reference clock into one of the `EMAC_RMII_CLK`
//!   input pads (GPIO13 or GPIO43). This is the recommended configuration when the PHY has an
//!   integrated oscillator.
//!
//! # MII clock source
//!
//! The PHY drives `TX_CLK` and `RX_CLK` separately at 25 MHz (100 Mbps) or
//! 2.5 MHz (10 Mbps). No internal clock generation is required.
//!
//! # RGMII clock sources
//!
//! RGMII uses PHY-driven `RX_CLK`. The MAC drives `TX_CLK` from the MPLL
//! (125 MHz at 1000 Mbps, 25 MHz at 100 Mbps, 2.5 MHz at 10 Mbps).

use esp_rom_sys::rom::ets_delay_us;

use crate::{
    clock::ll::{ClockTree, mpll_clk_frequency, request_mpll_clk},
    ethernet::{RmiiClkIn, RmiiClockConfig, mac::Speed},
    peripherals::{CNNT_IO_MUX, CNNT_SYS},
    private::Sealed,
};

/// Number of spin-loop iterations to wait after enabling the EMAC clock tree.
const CLOCK_STABILIZE_US: u32 = 300;

/// RMII PHY interface select (`CNNT_SYS.sys_gmac_ctrl0.sys_phy_intf_sel`).
const PHY_INTF_RMII: u8 = 4;
/// RGMII PHY interface select.
const PHY_INTF_RGMII: u8 = 1;
/// MII PHY interface select.
const PHY_INTF_MII: u8 = 0;

/// MPLL source for `SYS_HP_EMAC_REF_CTRL.sys_emac_ref_clk_sel`.
const REF_CLK_SEL_MPLL: u8 = 0;

/// RGMII TX clock: 125 MHz (1000 Mbps), 25 MHz (100 Mbps), 2.5 MHz (10 Mbps).
const RGMII_TX_CLK_1000M_HZ: u32 = 125_000_000;
const RGMII_TX_CLK_100M_HZ: u32 = 25_000_000;
const RGMII_TX_CLK_10M_HZ: u32 = 2_500_000;

// ── ExternalRefClock ─────────────────────────────────────────────────────────

/// RMII reference clock provided externally by the PHY.
///
/// The PHY must drive a 50 MHz clock into one of the `EMAC_RMII_CLK` input
/// pads: GPIO13 or GPIO43.
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

// ── RgmiiClock ───────────────────────────────────────────────────────────────

/// RGMII clock configuration.
///
/// The PHY drives `RX_CLK`. The MAC generates `TX_CLK` from the MPLL.
pub(crate) struct RgmiiClock;

impl RgmiiClock {
    pub(super) fn configure(&self) {
        configure_rgmii();
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
fn configure_rmii_input() {
    let cnnt = CNNT_SYS::regs();

    cnnt.sys_gmac_ctrl0()
        .modify(|_, w| unsafe { w.sys_phy_intf_sel().bits(PHY_INTF_RMII) });

    cnnt.sys_hp_emac_rmii_ctrl().modify(|_, w| unsafe {
        w.sys_emac_rmii_clk_en().set_bit();
        // 1 = external clock
        w.sys_emac_rmii_clk_sel().set_bit();
        // 50 / 2 = 25 MHz for 100 Mbps
        w.sys_emac_rxtx_clk_div_num().bits(1);
        w.sys_emac_rmii_pad_out_clk_en().clear_bit()
    });

    cnnt.sys_hp_emac_rx_ctrl().modify(|_, w| {
        w.sys_emac_rx_pad_clk_en().clear_bit();
        w.sys_emac_rx_clk_sel().clear_bit()
    });

    cnnt.sys_hp_emac_tx_ctrl().modify(|_, w| {
        w.sys_emac_tx_pad_clk_en().clear_bit();
        w.sys_emac_tx_clk_sel().clear_bit()
    });

    cnnt.sys_hp_emac_rmii_pad_ctrl().modify(|_, w| {
        w.sys_emac_rmii_pad_clk_en().set_bit();
        w.sys_emac_rmii_pad_clk_inv_en().clear_bit()
    });

    enable_common();
}

/// Configures the EMAC clock tree for RGMII.
fn configure_rgmii() {
    ClockTree::with(request_mpll_clk);

    let cnnt = CNNT_SYS::regs();

    cnnt.sys_gmac_ctrl0()
        .modify(|_, w| unsafe { w.sys_phy_intf_sel().bits(PHY_INTF_RGMII) });

    cnnt.sys_hp_emac_rmii_pad_ctrl().modify(|_, w| {
        w.sys_emac_rmii_pad_clk_en().clear_bit();
        w.sys_emac_rmii_pad_clk_inv_en().clear_bit()
    });

    cnnt.sys_hp_emac_rmii_ctrl().modify(|_, w| {
        w.sys_emac_rmii_clk_sel().clear_bit();
        w.sys_emac_rmii_clk_en().clear_bit();
        w.sys_emac_rmii_pad_out_clk_en().set_bit()
    });

    cnnt.sys_hp_emac_rx_ctrl().modify(|_, w| {
        w.sys_emac_rx_pad_clk_en().set_bit();
        w.sys_emac_rx_clk_sel().set_bit();
        w.sys_emac_rx_pad_clk_inv_en().clear_bit();
        w.sys_emac_rx_180_clk_en().set_bit()
    });

    cnnt.sys_hp_emac_tx_ctrl().modify(|_, w| {
        w.sys_emac_tx_pad_clk_en().clear_bit();
        w.sys_emac_tx_clk_sel().clear_bit();
        w.sys_emac_tx_180_clk_en().set_bit()
    });

    set_rgmii_tx_divider(RGMII_TX_CLK_1000M_HZ);
    enable_common();
}

/// Updates the RGMII TX clock divider for the negotiated link speed.
///
/// Only meaningful for RGMII, where the MAC drives `TX_CLK`; the RGMII pin
/// bundle is the only caller.
pub(super) fn apply_rgmii_tx_speed(speed: Speed) {
    let out_hz = match speed {
        Speed::_10M => RGMII_TX_CLK_10M_HZ,
        Speed::_100M => RGMII_TX_CLK_100M_HZ,
        Speed::_1000M => RGMII_TX_CLK_1000M_HZ,
    };
    set_rgmii_tx_divider(out_hz);
}

fn set_rgmii_tx_divider(out_hz: u32) {
    let src_hz = mpll_clk_frequency();
    let div = (src_hz / out_hz).saturating_sub(1) as u8;

    CNNT_SYS::regs()
        .sys_hp_emac_ref_ctrl()
        .modify(|_, w| unsafe {
            w.sys_emac_ref_clk_sel().bits(REF_CLK_SEL_MPLL);
            w.sys_emac_ref_clk_div_num().bits(div);
            w.sys_emac_ref_clk_en().set_bit()
        });
}

/// Configures the EMAC clock tree for MII mode.
fn configure_mii() {
    let cnnt = CNNT_SYS::regs();

    cnnt.sys_gmac_ctrl0()
        .modify(|_, w| unsafe { w.sys_phy_intf_sel().bits(PHY_INTF_MII) });

    cnnt.sys_hp_emac_rmii_ctrl()
        .modify(|_, w| w.sys_emac_rmii_clk_en().set_bit());

    cnnt.sys_hp_emac_rx_ctrl().modify(|_, w| {
        w.sys_emac_rx_pad_clk_en().set_bit();
        w.sys_emac_rx_clk_sel().set_bit()
    });

    cnnt.sys_hp_emac_tx_ctrl().modify(|_, w| {
        w.sys_emac_tx_pad_clk_en().set_bit();
        w.sys_emac_tx_clk_sel().set_bit()
    });

    enable_common();
}

fn enable_common() {
    CNNT_SYS::regs()
        .sys_gmac_ctrl0()
        .modify(|_, w| w.sys_gmac_mem_clk_force_on().set_bit());

    // GPIO13-19 live in the CNNT domain. Dedicated pad control is required when
    // those pads are used as GMAC I/O.
    CNNT_IO_MUX::regs()
        .ctrl()
        .modify(|_, w| w.gmac_pad_pin_ctrl_ded_sel().set_bit());

    deassert_reset();
    clock_stabilize();
}

/// Pulses the EMAC peripheral reset via `CNNT_SYS` (assert then deassert).
///
/// The `GenericPeripheralGuard` for EMAC on S31 does not perform a hardware reset, so this must be
/// called during clock configuration.
fn deassert_reset() {
    CNNT_SYS::regs()
        .sys_hp_emac_ctrl()
        .modify(|_, w| w.sys_emac_rst_en().set_bit());
    CNNT_SYS::regs()
        .sys_hp_emac_ctrl()
        .modify(|_, w| w.sys_emac_rst_en().clear_bit());
}

fn clock_stabilize() {
    ets_delay_us(CLOCK_STABILIZE_US);
}
