//! PHY abstraction layer.
//!
//! Defines the [`Phy`] trait, standard IEEE 802.3 Clause 22 MDIO register
//! addresses, and a thin [`MdioDriver`] wrapper around the EMAC's built-in
//! MDIO controller.

use core::task::Context;

use crate::ethernet::mac::{EmacRegs, LinkState};

pub mod generic;

// ── Standard Clause 22 register addresses ──────────────────────────────────

/// Basic Mode Control Register.
pub const BMCR: u8 = 0x00;
/// Basic Mode Status Register.
pub const BMSR: u8 = 0x01;
/// PHY Identifier 1.
pub const PHYIDR1: u8 = 0x02;
/// PHY Identifier 2.
pub const PHYIDR2: u8 = 0x03;
/// Auto-Negotiation Advertisement Register.
pub const ANAR: u8 = 0x04;
/// Auto-Negotiation Link Partner Ability Register.
pub const ANLPAR: u8 = 0x05;
/// Auto-Negotiation Expansion Register.
pub const ANER: u8 = 0x06;

/// Bit definitions for [`BMCR`].
pub mod bmcr {
    /// Software reset.
    pub const RESET: u16 = 1 << 15;
    /// Select 100 Mbps (when auto-neg disabled).
    pub const SPEED_100: u16 = 1 << 13;
    /// Enable auto-negotiation.
    pub const ANEN: u16 = 1 << 12;
    /// Restart auto-negotiation.
    pub const RESTART_AN: u16 = 1 << 9;
    /// Full-duplex (when auto-neg disabled).
    pub const FULL_DUPLEX: u16 = 1 << 8;
}

/// Bit definitions for [`BMSR`].
pub mod bmsr {
    /// Link status (latch-low on many PHYs — read twice to get current value).
    pub const LINK_STATUS: u16 = 1 << 2;
    /// Auto-negotiation complete.
    pub const AN_COMPLETE: u16 = 1 << 5;
    /// Auto-negotiation ability.
    pub const AN_ABILITY: u16 = 1 << 3;
}

/// Bit definitions for [`ANAR`] and [`ANLPAR`].
pub mod an {
    /// 10BASE-T half-duplex.
    pub const BASE_10_HALF: u16 = 1 << 5;
    /// 10BASE-T full-duplex.
    pub const BASE_10_FULL: u16 = 1 << 6;
    /// 100BASE-TX half-duplex.
    pub const BASE_100_HALF: u16 = 1 << 7;
    /// 100BASE-TX full-duplex.
    pub const BASE_100_FULL: u16 = 1 << 8;
}

// ── MdioBus trait ───────────────────────────────────────────────────────────

/// Generic MDIO bus interface for Clause 22 PHY register access.
///
/// PHY drivers are generic over this trait so they can live in external
/// crates without depending on the concrete [`MdioDriver`]. PHY and
/// register addresses are 5-bit Clause 22 fields (0-31).
pub trait MdioBus {
    /// Read a 16-bit PHY register.
    fn read(&mut self, phy_addr: u8, reg_addr: u8) -> u16;

    /// Write a 16-bit PHY register.
    fn write(&mut self, phy_addr: u8, reg_addr: u8, value: u16);
}

// ── MdioDriver ──────────────────────────────────────────────────────────────

/// Thin wrapper that exposes Clause 22 MDIO read/write over the EMAC MAC's
/// built-in MDIO controller.
///
/// Implements [`MdioBus`], so PHY drivers generic over `MdioBus` can run
/// directly on top of this MAC without any adapter.
pub struct MdioDriver<'a> {
    regs: &'a EmacRegs,
}

impl<'a> MdioDriver<'a> {
    pub(super) fn new(regs: &'a EmacRegs) -> Self {
        Self { regs }
    }

    /// Reads one PHY register (Clause 22).
    pub fn read(&mut self, phy_addr: u8, reg: u8) -> u16 {
        // Clause 22 frame fields are 5 bits — out-of-range values are silently
        // truncated by the PAC writer (`FieldWriter<_, 5>`) and would hit a
        // different PHY/register without any error. Catch this in debug builds.
        debug_assert!(phy_addr < 32, "Clause 22 PHY address must be < 32");
        debug_assert!(reg < 32, "Clause 22 register address must be < 32");
        self.regs.mdio_read(phy_addr, reg)
    }

    /// Writes one PHY register (Clause 22).
    pub fn write(&mut self, phy_addr: u8, reg: u8, data: u16) {
        debug_assert!(phy_addr < 32, "Clause 22 PHY address must be < 32");
        debug_assert!(reg < 32, "Clause 22 register address must be < 32");
        self.regs.mdio_write(phy_addr, reg, data)
    }
}

impl<'a> MdioBus for MdioDriver<'a> {
    fn read(&mut self, phy_addr: u8, reg_addr: u8) -> u16 {
        MdioDriver::read(self, phy_addr, reg_addr)
    }

    fn write(&mut self, phy_addr: u8, reg_addr: u8, value: u16) {
        MdioDriver::write(self, phy_addr, reg_addr, value);
    }
}

// ── Phy trait ───────────────────────────────────────────────────────────────

/// PHY error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhyError {
    /// The PHY did not respond within the expected time.
    Timeout,
    /// No PHY was found on the MDIO bus during auto-address discovery.
    NotFound,
}

/// Ethernet PHY driver.
pub trait Phy {
    /// One-time hardware initialization.
    ///
    /// Should reset the PHY, configure auto-negotiation, and wait until the
    /// hardware is ready to respond to further MDIO transactions.
    fn init<M: MdioBus>(&mut self, mdio: &mut M) -> Result<(), PhyError>;

    /// Polls the current link state.
    ///
    /// Returns [`LinkState`] with `up == true` once auto-negotiation is
    /// complete and the link partner is detected. The context can be used
    /// to wake the caller when the link state should be re-polled, if the PHY chip
    /// supports interrupt-driven notifications.
    fn poll_link<M: MdioBus>(&mut self, mdio: &mut M, _cx: Option<&mut Context<'_>>) -> LinkState;

    /// Returns the Clause 22 PHY address on the MDIO bus.
    fn address(&self) -> u8;
}
