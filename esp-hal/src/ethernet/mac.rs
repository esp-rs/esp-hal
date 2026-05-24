//! MAC register abstraction for the EMAC driver.
//!
//! Provides higher-level helpers for MAC/DMA initialization and runtime
//! control, all accessing the three EMAC register blocks via their
//! `::regs()` static accessors.

use crate::peripherals::{EMAC_DMA, EMAC_MAC};

/// Returns the `miicsrclk` value for the MDIO management clock divider.
///
/// On ESP32 the MDC CSR clock source is the APB clock (fixed at 80 MHz), so
/// value 3 (35–60 MHz range → /26) has always been used and works in practice.
///
/// On ESP32-P4 the CSR clock source is the SYS (CPU) clock, which varies with
/// the selected [`crate::clock::CpuClock`] preset (100–400 MHz). The divider
/// is computed at runtime to keep MDC within the 1–2.5 MHz range required by
/// IEEE 802.3 clause 22. The 4-bit `miicsrclk` field on P4 supports extended
/// divider values (up to 6 = 300–500 MHz → /204).
fn mdc_csr_clock_range() -> u8 {
    #[cfg(esp32p4)]
    {
        // Matches emac_hal_set_csr_clock_range() in esp-idf (emac_hal.c).
        // The P4 EMAC (DWC_gmac) only defines encoding values 0–5; value 6+
        // is reserved and must not be used. The CSR clock source is the SYS
        // (CPU) clock.
        //
        // emac_crs_div_table = {42, 62, 16, 26, 102, 124}
        //   encoding 0 → /42  (60–100 MHz)
        //   encoding 1 → /62  (100–150 MHz)
        //   encoding 2 → /16  (20–35 MHz)
        //   encoding 3 → /26  (35–60 MHz)
        //   encoding 4 → /102 (150–250 MHz)
        //   encoding 5 → /124 (≥ 250 MHz, slightly over 2.5 MHz spec at high SYS clocks)
        match crate::clock::ll::sys_clk_frequency() {
            hz if hz >= 250_000_000 => 5, // /124
            hz if hz >= 150_000_000 => 4, // /102
            hz if hz >= 100_000_000 => 1, // /62
            hz if hz >= 60_000_000 => 0,  // /42
            hz if hz >= 35_000_000 => 3,  // /26
            _ => 2,                       // /16
        }
    }
    #[cfg(not(esp32p4))]
    {
        3 // ESP32: 80 MHz APB → /26, works in practice
    }
}

/// Link speed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Speed {
    /// 10 Mbit/s
    _10M,
    /// 100 Mbit/s
    _100M,
}

/// Link duplex mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
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
    /// Link speed (valid if `up` is true).
    pub speed: Speed,
    /// Link duplex mode (valid if `up` is true).
    pub duplex: Duplex,
}

/// Zero-sized handle that provides register-level operations on the three EMAC
/// blocks.
///
/// All methods use `EMAC_MAC::regs()` / `EMAC_DMA::regs()` static accessors; the singleton
/// ownership is tracked by the `Ethernet` struct.
#[derive(Clone, Copy)]
pub(super) struct EmacRegs;

impl EmacRegs {
    // ── DMA soft-reset ────────────────────────────────────────────────────

    /// Issues a DMA soft-reset and spins until the hardware clears the bit.
    pub fn dma_soft_reset(&self) {
        EMAC_DMA::regs()
            .dmabusmode()
            .modify(|_, w| w.sw_rst().set_bit());

        while EMAC_DMA::regs().dmabusmode().read().sw_rst().bit_is_set() {}

        // Enhanced 32-byte descriptors, fixed burst, address-aligned beats, PBL=8.
        EMAC_DMA::regs().dmabusmode().modify(|_, w| unsafe {
            w.alt_desc_size().set_bit();
            w.fixed_burst().set_bit();
            w.dmaaddralibea().set_bit();
            w.use_sep_pbl().set_bit();
            w.prog_burst_len().bits(8);
            w.rx_dma_pbl().bits(8);
            w
        });
    }

    // ── DMA operation ─────────────────────────────────────────────────────

    /// Starts both DMA engines.
    ///
    /// P4's RX FIFO is too small (~256 B) to store full frames, so RSF must
    /// not be set — it silently drops frames larger than the FIFO. Use
    /// cut-through receive instead.
    pub fn dma_start(&self) {
        EMAC_DMA::regs().dmaoperation_mode().modify(|_, w| {
            #[cfg(esp32p4)]
            {
                w.tx_str_fwd().set_bit();
                w.fwd_under_gf().set_bit();
            }
            #[cfg(not(esp32p4))]
            {
                w.rx_store_forward().set_bit();
            }
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

    /// Configures the MAC for the given speed.
    pub fn set_speed(&self, speed: Speed) {
        EMAC_MAC::regs()
            .emacconfig()
            .modify(|_, w| w.fespeed().bit(speed == Speed::_100M));
    }

    /// Configures the MAC for the given duplex mode.
    pub fn set_duplex(&self, duplex: Duplex) {
        EMAC_MAC::regs()
            .emacconfig()
            .modify(|_, w| w.duplex().bit(duplex == Duplex::Full));
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
            w.miicsrclk().bits(mdc_csr_clock_range());
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
            w.miicsrclk().bits(mdc_csr_clock_range());
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
