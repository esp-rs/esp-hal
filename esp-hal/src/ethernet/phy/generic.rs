//! Generic PHY driver.
//!
//! Standard IEEE 802.3 Clause 22 PHYs with auto-negotiation support.

use core::task::Context;

use crate::ethernet::{
    mac::{Duplex, LinkState, Speed},
    phy::{ANAR, ANLPAR, BMCR, BMSR, MdioDriver, PHYIDR1, Phy, PhyError, an, bmcr, bmsr},
};

/// Maximum iterations to wait for the PHY reset bit to self-clear.
const RESET_POLL_LIMIT: u32 = 500_000; // us

/// Generic PHY driver.
///
/// Can be constructed with a fixed address ([`GenericPhy::new`]) or with
/// automatic address discovery ([`GenericPhy::new_auto`]).  When using
/// `new_auto`, the MDIO bus is scanned during [`Phy::init`] and the first
/// responding PHY address is adopted.
#[derive(Clone, Copy, Debug)]
pub struct GenericPhy {
    /// `Some(addr)` for a fixed address; `None` until auto-discovery runs.
    addr: Option<u8>,
}

impl GenericPhy {
    /// Creates a driver instance for the given MDIO bus address.
    pub const fn new(addr: u8) -> Self {
        Self { addr: Some(addr) }
    }

    /// Creates a driver instance that discovers the PHY address automatically
    /// by scanning the MDIO bus during [`Phy::init`].
    ///
    /// Returns [`PhyError::NotFound`] from `init` if no PHY responds.
    pub const fn new_auto() -> Self {
        Self { addr: None }
    }

    /// Scans all 32 Clause 22 addresses and returns the first one that holds a
    /// valid `PHYIDR1` value (not 0x0000 or 0xFFFF).
    ///
    /// The scan is repeated up to 3 times to handle transient bus noise,
    /// matching the esp-idf `esp_eth_phy_802_3_detect_phy_addr` strategy.
    fn discover(mdio: &MdioDriver<'_>) -> Option<u8> {
        for _ in 0..3 {
            for addr in 0..32_u8 {
                let id = mdio.read(addr, PHYIDR1);
                if id != 0x0000 && id != 0xFFFF {
                    debug!("phy found at addr {} - id: {:x}", addr, id);
                    return Some(addr);
                }
            }
        }
        None
    }
}

impl Phy for GenericPhy {
    fn address(&self) -> u8 {
        self.addr
            .expect("GenericPhy address not yet resolved — call init() first")
    }

    fn init(&mut self, mdio: &MdioDriver<'_>) -> Result<(), PhyError> {
        // Resolve address if auto-discovery was requested.
        if self.addr.is_none() {
            self.addr = Some(Self::discover(mdio).ok_or(PhyError::NotFound)?);
        }
        let addr = self.addr.unwrap();

        mdio.write(addr, BMCR, bmcr::RESET);

        // Wait for the RESET bit to self-clear (≤ ~0.5 ms per IEEE 802.3).
        for _ in 0..RESET_POLL_LIMIT {
            if mdio.read(addr, BMCR) & bmcr::RESET == 0 {
                // Reset complete. Configure advertisement then restart AN.
                let adv =
                    an::BASE_10_HALF | an::BASE_10_FULL | an::BASE_100_HALF | an::BASE_100_FULL;
                mdio.write(addr, ANAR, adv | 0x0001); // selector = IEEE 802.3

                // RMW so we don't disturb speed/duplex bits restored by reset.
                let ctrl = mdio.read(addr, BMCR);
                mdio.write(addr, BMCR, ctrl | bmcr::ANEN | bmcr::RESTART_AN);

                return Ok(());
            }
            crate::rom::ets_delay_us(1);
        }

        Err(PhyError::Timeout)
    }

    fn poll_link(&mut self, mdio: &MdioDriver<'_>, cx: Option<&mut Context<'_>>) -> LinkState {
        const LINK_STATE_DOWN: LinkState = LinkState {
            up: false,
            speed: Speed::_100M,
            duplex: Duplex::Full,
        };

        if let Some(cx) = cx {
            cx.waker().wake_by_ref();
        }

        let Some(addr) = self.addr else {
            debug!("poll_link called before init");
            return LINK_STATE_DOWN;
        };

        // Read BMSR twice: first read clears the latch-low LINK_STATUS bit on
        // some PHYs; the second read gives the real state.
        let _ = mdio.read(addr, BMSR);
        let bmsr_val = mdio.read(addr, BMSR);

        trace!("bmsr_val: {:x}", bmsr_val);

        if bmsr_val & bmsr::LINK_STATUS == 0 {
            return LINK_STATE_DOWN;
        }

        if bmsr_val & bmsr::AN_COMPLETE == 0 {
            return LINK_STATE_DOWN;
        }

        // Link is up and auto-negotiation is complete. read ANLPAR to
        // determine the negotiated speed/duplex.

        let anlpar_val = mdio.read(addr, ANLPAR);
        let speed = if anlpar_val & an::BASE_100_FULL != 0 || anlpar_val & an::BASE_100_HALF != 0 {
            Speed::_100M
        } else {
            Speed::_10M
        };
        let duplex = if anlpar_val & an::BASE_10_FULL != 0 || anlpar_val & an::BASE_100_FULL != 0 {
            Duplex::Full
        } else {
            Duplex::Half
        };

        LinkState {
            up: true,
            speed,
            duplex,
        }
    }
}
