//! Generic PHY driver.
//!
//! Standard IEEE 802.3 Clause 22 PHYs with auto-negotiation support. This driver works with either
//! variant by relying only on the standard registers.

use core::task::Context;

use crate::ethernet::{
    mac::LinkState,
    phy::{ANAR, BMCR, BMSR, MdioDriver, Phy, PhyError, an, bmcr, bmsr},
};

/// Maximum iterations to wait for the PHY reset bit to self-clear.
const RESET_POLL_LIMIT: u32 = 1_000_000;

/// Generic PHY driver.
#[derive(Clone, Copy, Debug)]
pub struct GenericPhy {
    /// Clause 22 PHY address (typically 0 or 1 depending on strap).
    addr: u8,
}

impl GenericPhy {
    /// Creates a driver instance for the given MDIO bus address.
    pub const fn new(addr: u8) -> Self {
        Self { addr }
    }
}

impl Phy for GenericPhy {
    fn address(&self) -> u8 {
        self.addr
    }

    fn init(&mut self, mdio: &MdioDriver<'_>) -> Result<(), PhyError> {
        // Advertise all four capabilities (10/100 half/full).
        let adv = an::BASE_10_HALF | an::BASE_10_FULL | an::BASE_100_HALF | an::BASE_100_FULL;
        // Selector field = 0b00001 (IEEE 802.3).
        mdio.write(self.addr, ANAR, adv | 0x0001);

        // Assert software reset + restart auto-negotiation atomically.
        mdio.write(self.addr, BMCR, bmcr::RESET | bmcr::ANEN | bmcr::RESTART_AN);

        // Wait for the RESET bit to self-clear (≤ ~0.5 ms per IEEE 802.3).
        for _ in 0..RESET_POLL_LIMIT {
            if mdio.read(self.addr, BMCR) & bmcr::RESET == 0 {
                return Ok(());
            }
        }

        Err(PhyError::Timeout)
    }

    fn poll_link(&mut self, mdio: &MdioDriver<'_>, cx: Option<&mut Context<'_>>) -> LinkState {
        if let Some(cx) = cx {
            cx.waker().wake_by_ref(); // Schedule another poll after yielding to the executor.
        }

        // Read BMSR twice: first read clears the latch-low LINK_STATUS bit on
        // some PHYs (including LAN8720A); the second read gives the real state.
        let _ = mdio.read(self.addr, BMSR);
        let bmsr_val = mdio.read(self.addr, BMSR);

        if bmsr_val & bmsr::LINK_STATUS == 0 {
            return LinkState { up: false };
        }

        if bmsr_val & bmsr::AN_COMPLETE == 0 {
            return LinkState { up: false };
        }

        LinkState { up: true }
    }
}
