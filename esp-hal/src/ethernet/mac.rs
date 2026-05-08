//! MAC register abstraction for the EMAC driver.
//!
//! Provides higher-level helpers for MAC/DMA initialization and runtime
//! control, all accessing the three EMAC register blocks via their
//! `::regs()` static accessors.

use crate::peripherals::{EMAC_DMA, EMAC_MAC};

/// MDC clock range for the MDIO clock divider (`emacgmiiaddr.miicsrclk`).
///
/// Value 3 selects APB/42 ≈ 1.9 MHz MDC clock (APB = 80 MHz on ESP32).
const MDC_CSR_CLOCK_RANGE: u8 = 3;

/// PHY interface selection for RMII mode in `EMAC_EXT.ex_phyinf_conf`.
pub(super) const PHY_INTF_RMII: u8 = 4;
/// PHY interface selection for MII mode in `EMAC_EXT.ex_phyinf_conf`.
pub(super) const PHY_INTF_MII: u8 = 0;

/// Link speed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10 Mbit/s
    _10M,
    /// 100 Mbit/s
    _100M,
}

/// Link duplex mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duplex {
    /// Half duplex
    Half,
    /// Full duplex
    Full,
}

/// Link state reported by the PHY.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkState {
    /// Whether the link is established.
    pub up: bool,
}

/// Zero-sized handle that provides register-level operations on the three EMAC
/// blocks.
///
/// All methods use `EMAC_MAC::regs()` / `EMAC_DMA::regs()` / `EMAC_EXT::regs()`
/// static accessors; the singleton ownership is tracked by the `Ethernet` struct.
#[derive(Clone, Copy)]
pub(super) struct EmacRegs;

impl EmacRegs {
    pub const fn new() -> Self {
        Self
    }

    // ── DMA soft-reset ────────────────────────────────────────────────────

    /// Issues a DMA soft-reset and spins until the hardware clears the bit.
    pub fn dma_soft_reset(&self) {
        EMAC_DMA::regs()
            .dmabusmode()
            .modify(|_, w| w.sw_rst().set_bit());

        while EMAC_DMA::regs().dmabusmode().read().sw_rst().bit_is_set() {}

        // Enable enhanced 32-byte descriptor format.
        EMAC_DMA::regs()
            .dmabusmode()
            .modify(|_, w| w.alt_desc_size().set_bit());
    }

    // ── DMA operation ─────────────────────────────────────────────────────

    /// Enables RX store-and-forward mode and starts both DMA engines.
    pub fn dma_start(&self) {
        EMAC_DMA::regs().dmaoperation_mode().modify(|_, w| {
            w.rx_store_forward().set_bit();
            w.start_stop_rx().set_bit();
            w.start_stop_transmission_command().set_bit()
        });
    }

    /// Stops both DMA engines.
    #[expect(dead_code)]
    pub fn dma_stop(&self) {
        EMAC_DMA::regs().dmaoperation_mode().modify(|_, w| {
            w.start_stop_rx().clear_bit();
            w.start_stop_transmission_command().clear_bit()
        });
    }

    /// Programs the TX and RX descriptor list base addresses.
    pub fn set_descriptor_lists(&self, tx_base: u32, rx_base: u32) {
        unsafe {
            EMAC_DMA::regs().dmatxbaseaddr().write(|w| w.bits(tx_base));
            EMAC_DMA::regs().dmarxbaseaddr().write(|w| w.bits(rx_base));
        }
    }

    /// Issues a TX poll demand to resume a suspended TX engine.
    pub fn demand_tx_poll(&self) {
        // Write any value to demand a TX poll; PAC doesn't expose Writable for
        // this register so we use a direct raw write.
        unsafe {
            core::ptr::write_volatile(EMAC_DMA::regs().dmatxpolldemand().as_ptr(), 0);
        }
    }

    /// Issues an RX poll demand to resume a suspended RX engine.
    pub fn demand_rx_poll(&self) {
        unsafe {
            core::ptr::write_volatile(EMAC_DMA::regs().dmarxpolldemand().as_ptr(), 0);
        }
    }

    // ── DMA interrupt control ─────────────────────────────────────────────

    /// Enables Normal/Abnormal summary interrupts and the RX interrupt.
    /// Optionally enables the TX interrupt (required in async mode).
    pub fn dma_enable_interrupts(&self, enable_tx: bool) {
        EMAC_DMA::regs().dmain_en().modify(|_, w| {
            w.dmain_rie().set_bit();
            w.dmain_aise().set_bit();
            w.dmain_nise().set_bit();
            w.dmain_tie().bit(enable_tx)
        });
    }

    /// Disables all DMA interrupt sources.
    pub fn dma_disable_interrupts(&self) {
        unsafe {
            EMAC_DMA::regs().dmain_en().write(|w| w.bits(0));
        }
    }

    /// Reads and clears all pending DMA interrupt status bits.
    #[expect(dead_code)]
    pub fn dma_clear_interrupts(&self) -> u32 {
        let status = EMAC_DMA::regs().dmastatus().read().bits();

        EMAC_DMA::regs()
            .dmastatus()
            .write(|w| unsafe { w.bits(status) });

        status
    }

    // ── MAC configuration ─────────────────────────────────────────────────

    /// Configures the MAC for the given speed/duplex and enables TX/RX.
    pub fn mac_init(&self, speed: Speed, duplex: Duplex) {
        EMAC_MAC::regs().emacconfig().modify(|_, w| {
            w.mii().set_bit();
            w.fespeed().bit(speed == Speed::_100M);
            w.duplex().bit(duplex == Duplex::Full);
            w.padcrcstrip().set_bit();
            w.rx().set_bit();
            w.tx().set_bit()
        });

        // Enable pass-all-multicast mode.
        EMAC_MAC::regs().emacff().modify(|_, w| w.pam().set_bit());
    }

    // ── MAC address ───────────────────────────────────────────────────────

    /// Programs the unicast MAC address (filter slot 0).
    pub fn set_mac_address(&self, addr: &[u8; 6]) {
        let hi = (addr[5] as u32) << 8 | (addr[4] as u32);
        let lo = (addr[3] as u32) << 24
            | (addr[2] as u32) << 16
            | (addr[1] as u32) << 8
            | (addr[0] as u32);

        EMAC_MAC::regs().emacaddr0high().write(|w| unsafe {
            w.address0_hi().bits(hi as u16);
            w.address_enable0().set_bit()
        });
        EMAC_MAC::regs()
            .emacaddr0low()
            .write(|w| unsafe { w.bits(lo) });
    }

    // ── MDIO ─────────────────────────────────────────────────────────────

    /// Reads one PHY register via the MDIO interface (Clause 22).
    pub fn mdio_read(&self, phy_addr: u8, reg: u8) -> u16 {
        EMAC_MAC::regs().emacgmiiaddr().write(|w| unsafe {
            w.miidev().bits(phy_addr);
            w.miireg().bits(reg);
            w.miicsrclk().bits(MDC_CSR_CLOCK_RANGE);
            w.miiwrite().clear_bit();
            w.miibusy().set_bit()
        });

        self.mdio_wait();
        EMAC_MAC::regs().emacmiidata().read().mii_data().bits()
    }

    /// Writes one PHY register via the MDIO interface (Clause 22).
    pub fn mdio_write(&self, phy_addr: u8, reg: u8, data: u16) {
        EMAC_MAC::regs()
            .emacmiidata()
            .write(|w| unsafe { w.mii_data().bits(data) });
        EMAC_MAC::regs().emacgmiiaddr().write(|w| unsafe {
            w.miidev().bits(phy_addr);
            w.miireg().bits(reg);
            w.miicsrclk().bits(MDC_CSR_CLOCK_RANGE);
            w.miiwrite().set_bit();
            w.miibusy().set_bit()
        });

        self.mdio_wait();
    }

    fn mdio_wait(&self) {
        while EMAC_MAC::regs()
            .emacgmiiaddr()
            .read()
            .miibusy()
            .bit_is_set()
        {}
    }
}
