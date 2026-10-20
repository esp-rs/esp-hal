//! Generic PHY driver.
//!
//! Standard IEEE 802.3 Clause 22 PHYs with auto-negotiation support.

use core::task::Context;

use crate::ethernet::{
    mac::{Duplex, LinkState, Speed},
    phy::{
        ANAR,
        ANLPAR,
        BMCR,
        BMSR,
        EXSR,
        GBCR,
        GBSR,
        MdioBus,
        PHYIDR1,
        PHYIDR2,
        Phy,
        PhyError,
        an,
        bmcr,
        bmsr,
        exsr,
        gbcr,
        gbsr,
    },
};

/// Maximum iterations to wait for the PHY reset bit to self-clear.
const RESET_POLL_LIMIT: u32 = 500_000; // us

/// Motorcomm YT8531 identifier (`PHYIDR1`/`PHYIDR2`).
const YT8531_ID: (u16, u16) = (0x4F51, 0xE91B);

/// Extended-register address window (Motorcomm).
const REG_EXT_ADDR: u8 = 0x1E;
/// Extended-register data window (Motorcomm).
const REG_EXT_DATA: u8 = 0x1F;

/// Chip config (extended). Bit 8 adds a coarse ~1.9 ns RXC delay.
const EXT_CHIP_CFG: u16 = 0xA001;
const RXC_DLY_EN: u16 = 1 << 8;

/// RGMII delay config (extended).
/// `[13:10]` RX, `[7:4]` 100M TX, `[3:0]` 1000M TX. Unit 150 ps; 13 ≈ 1.95 ns.
const EXT_RGMII_CFG1: u16 = 0xA003;
const RGMII_TX_DLY_MASK: u16 = 0x00FF;
const RGMII_DLY_2NS: u16 = 13;

fn ext_read<M: MdioBus>(mdio: &mut M, addr: u8, ext: u16) -> u16 {
    mdio.write(addr, REG_EXT_ADDR, ext);
    mdio.read(addr, REG_EXT_DATA)
}

fn ext_write<M: MdioBus>(mdio: &mut M, addr: u8, ext: u16, val: u16) {
    mdio.write(addr, REG_EXT_ADDR, ext);
    mdio.write(addr, REG_EXT_DATA, val);
}

/// Generic PHY driver.
///
/// Initialization and link handling follow IEEE 802.3 Clause 22, matching
/// esp-idf `esp_eth_phy_802_3`. PHYs that need chip-specific setup get it
/// applied from [`Phy::init`] based on their identifier.
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
    /// Creates a new driver instance for the given MDIO bus address.
    pub const fn new(addr: u8) -> Self {
        Self { addr: Some(addr) }
    }

    /// Creates a new driver instance that discovers the PHY address automatically
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
    fn discover<M: MdioBus>(mdio: &mut M) -> Option<u8> {
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

    /// Removes advertisements faster than `max_speed`.
    ///
    /// A gigabit PHY wired to an RMII or MII MAC would otherwise negotiate 1000
    /// Mbps with the link partner and the MAC could not carry it. Returns
    /// whether an advertisement was withdrawn, in which case auto-negotiation
    /// has to be restarted to take effect.
    fn limit_advertisement<M: MdioBus>(mdio: &mut M, addr: u8, max_speed: Speed) -> bool {
        let mut changed = false;

        if max_speed != Speed::_1000M {
            let gbcr_val = mdio.read(addr, GBCR);
            if gbcr_val != 0xFFFF {
                let limited = gbcr_val & !(gbcr::ADV_1000_FULL | gbcr::ADV_1000_HALF);
                if limited != gbcr_val {
                    mdio.write(addr, GBCR, limited);
                    changed = true;
                }
            }
        }

        if max_speed == Speed::_10M {
            let anar_val = mdio.read(addr, ANAR);
            let limited = anar_val & !(an::BASE_100_FULL | an::BASE_100_HALF);
            if limited != anar_val {
                mdio.write(addr, ANAR, limited);
                changed = true;
            }
        }

        changed
    }

    /// Programs the Motorcomm YT8531 RGMII clock delays.
    ///
    /// RGMII needs roughly 2 ns of delay on both paths. Do not write the YT8521 `0xA000` UTP/fiber
    /// page select: the YT8531 is UTP-only and that write maps the Clause 22 registers to dead
    /// space.
    fn init_yt8531<M: MdioBus>(mdio: &mut M, addr: u8) {
        let chip = ext_read(mdio, addr, EXT_CHIP_CFG);
        ext_write(mdio, addr, EXT_CHIP_CFG, chip | RXC_DLY_EN);

        let mut rgmii = ext_read(mdio, addr, EXT_RGMII_CFG1);
        rgmii = (rgmii & !RGMII_TX_DLY_MASK) | (RGMII_DLY_2NS << 4) | RGMII_DLY_2NS;
        ext_write(mdio, addr, EXT_RGMII_CFG1, rgmii);

        // Restart auto-negotiation so the delays are in force before it resolves.
        let ctrl = mdio.read(addr, BMCR);
        mdio.write(addr, BMCR, ctrl | bmcr::RESTART_AN);
    }
}

impl Phy for GenericPhy {
    fn address(&self) -> u8 {
        self.addr
            .expect("GenericPhy address not yet resolved — call init() first")
    }

    fn init<M: MdioBus>(&mut self, mdio: &mut M, max_speed: Speed) -> Result<(), PhyError> {
        // Resolve address if auto-discovery was requested.
        if self.addr.is_none() {
            self.addr = Some(Self::discover(mdio).ok_or(PhyError::NotFound)?);
        }
        let addr = self.addr.unwrap();

        // Match esp-idf `esp_eth_phy_802_3_basic_phy_init`: power up, then
        // software reset. ANAR/GBCR keep their post-reset defaults except for
        // abilities the MAC interface cannot carry.
        let ctrl = mdio.read(addr, BMCR);
        if ctrl & bmcr::POWER_DOWN != 0 {
            mdio.write(addr, BMCR, ctrl & !bmcr::POWER_DOWN);
            for _ in 0..RESET_POLL_LIMIT {
                if mdio.read(addr, BMCR) & bmcr::POWER_DOWN == 0 {
                    break;
                }
                crate::rom::ets_delay_us(1);
            }
        }

        mdio.write(addr, BMCR, bmcr::RESET);
        for _ in 0..RESET_POLL_LIMIT {
            if mdio.read(addr, BMCR) & bmcr::RESET == 0 {
                // Some PHYs clear ANEN across a reset. Only restore the bit;
                // advertisements stay at their post-reset defaults.
                let ctrl = mdio.read(addr, BMCR);
                if ctrl & bmcr::ANEN == 0 {
                    mdio.write(addr, BMCR, (ctrl & !bmcr::POWER_DOWN) | bmcr::ANEN);
                }

                let restart_an = Self::limit_advertisement(mdio, addr, max_speed);

                let id = (mdio.read(addr, PHYIDR1), mdio.read(addr, PHYIDR2));
                if id == YT8531_ID {
                    // Also restarts auto-negotiation.
                    Self::init_yt8531(mdio, addr);
                } else if restart_an {
                    let ctrl = mdio.read(addr, BMCR);
                    mdio.write(addr, BMCR, ctrl | bmcr::RESTART_AN);
                }

                return Ok(());
            }
            crate::rom::ets_delay_us(1);
        }

        Err(PhyError::Timeout)
    }

    fn poll_link<M: MdioBus>(&mut self, mdio: &mut M, cx: Option<&mut Context<'_>>) -> LinkState {
        const LINK_STATE_DOWN: LinkState = LinkState {
            up: false,
            speed: Speed::_100M,
            duplex: Duplex::Full,
        };

        // No PHY interrupt is available, and esp-hal has no timer to defer to,
        // so wake immediately to keep being polled. Wrap this driver to
        // rate-limit the polling (see `Phy::poll_link`).
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

        // Link is BMSR.LINK_STATUS alone, as in IDF
        // `esp_eth_phy_802_3_updt_link_dup_spd`. AN_COMPLETE may still be clear
        // while Clause 40 finishes even though copper is already up.
        if bmsr_val & bmsr::LINK_STATUS == 0 {
            return LINK_STATE_DOWN;
        }

        let bmcr_val = mdio.read(addr, BMCR);
        if bmcr_val & bmcr::ANEN == 0 {
            let speed = if bmcr_val & bmcr::SPEED_1000 != 0 {
                Speed::_1000M
            } else if bmcr_val & bmcr::SPEED_100 != 0 {
                Speed::_100M
            } else {
                Speed::_10M
            };
            let duplex = if bmcr_val & bmcr::FULL_DUPLEX != 0 {
                Duplex::Full
            } else {
                Duplex::Half
            };
            return LinkState {
                up: true,
                speed,
                duplex,
            };
        }

        if bmsr_val & bmsr::EXTENDED_STATUS != 0 {
            let exsr_val = mdio.read(addr, EXSR);
            if exsr_val != 0xFFFF && (exsr_val & (exsr::BASE1000_T_HD | exsr::BASE1000_T_FD) != 0) {
                let gbcr_val = mdio.read(addr, GBCR);
                let gbsr_val = mdio.read(addr, GBSR);
                if gbcr_val != 0xFFFF && gbsr_val != 0xFFFF {
                    if gbcr_val & gbcr::ADV_1000_FULL != 0 && gbsr_val & gbsr::LP_1000_FULL != 0 {
                        return LinkState {
                            up: true,
                            speed: Speed::_1000M,
                            duplex: Duplex::Full,
                        };
                    }
                    if gbcr_val & gbcr::ADV_1000_HALF != 0 && gbsr_val & gbsr::LP_1000_HALF != 0 {
                        return LinkState {
                            up: true,
                            speed: Speed::_1000M,
                            duplex: Duplex::Half,
                        };
                    }
                }
            }
        }

        // IDF: resolved mode is ANAR ∩ ANLPAR, highest common ability first.
        let anar_val = mdio.read(addr, ANAR);
        let anlpar_val = mdio.read(addr, ANLPAR);
        let common = anar_val & anlpar_val;
        let (speed, duplex) = if common & an::BASE_100_FULL != 0 {
            (Speed::_100M, Duplex::Full)
        } else if common & an::BASE_100_HALF != 0 {
            (Speed::_100M, Duplex::Half)
        } else if common & an::BASE_10_FULL != 0 {
            (Speed::_10M, Duplex::Full)
        } else if common & an::BASE_10_HALF != 0 {
            (Speed::_10M, Duplex::Half)
        } else {
            (Speed::_100M, Duplex::Full)
        };

        LinkState {
            up: true,
            speed,
            duplex,
        }
    }
}
