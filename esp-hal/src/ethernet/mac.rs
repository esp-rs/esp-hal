//! MAC register abstraction for the EMAC driver.
//!
//! Provides higher-level helpers for MAC/DMA initialization and runtime
//! control, all accessing the three EMAC register blocks via their
//! `::regs()` static accessors.

use crate::peripherals::{EMAC_DMA, EMAC_MAC};

/// Programmable DMA burst length, matching the esp-idf per-target defaults.
///
/// A 32-beat burst overflows the AXI bus on ESP32-S31.
const DMA_BURST_LEN: u8 = cfg_select! {
    esp32s31 => 16,
    _ => 32,
};

/// Returns the `miicsrclk` value for the MDIO management clock divider.
///
/// On ESP32 the MDC CSR clock source is the APB clock (fixed at 80 MHz), so
/// value 3 (35–60 MHz range → /26) has always been used and works in practice.
///
/// On ESP32-P4 and ESP32-S31 the CSR clock source is the SYS clock. Both vary with the selected
/// [`crate::clock::CpuClock`] preset. The divider is computed at runtime to keep MDC within the
/// 1–2.5 MHz range required by IEEE 802.3 clause 22.
fn mdc_csr_clock_range() -> u8 {
    // Matches emac_hal_set_csr_clock_range() in esp-idf (emac_hal.c).
    // The DWC_gmac only defines encoding values 0–5; value 6+
    // is reserved and must not be used.
    //
    // emac_crs_div_table = {42, 62, 16, 26, 102, 124}
    //   encoding 0 → /42  (60–100 MHz)
    //   encoding 1 → /62  (100–150 MHz)
    //   encoding 2 → /16  (20–35 MHz)
    //   encoding 3 → /26  (35–60 MHz)
    //   encoding 4 → /102 (150–250 MHz)
    //   encoding 5 → /124 (≥ 250 MHz, slightly over 2.5 MHz spec at high SYS clocks)
    let hz = cfg_select! {
        soc_has_clock_node_sys_clk => crate::clock::ll::sys_clk_frequency(),
        esp32 => 40_000_000,
    };
    match hz {
        hz if hz >= 250_000_000 => 5, // /124
        hz if hz >= 150_000_000 => 4, // /102
        hz if hz >= 100_000_000 => 1, // /62
        hz if hz >= 60_000_000 => 0,  // /42
        hz if hz >= 35_000_000 => 3,  // /26
        _ => 2,                       // /16
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
    /// 1000 Mbit/s (RGMII)
    _1000M,
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

        // Bus mode per esp-idf `emac_hal_init_dma_default`: enhanced 32-byte
        // descriptors, mixed burst, address-aligned beats, one PBL for both
        // directions, round-robin arbitration with a 1:1 RX/TX ratio.
        EMAC_DMA::regs().dmabusmode().modify(|_, w| unsafe {
            w.alt_desc_size().set_bit();
            w.dmamixedburst().set_bit();
            w.dmaaddralibea().set_bit();
            w.fixed_burst().clear_bit();
            w.use_sep_pbl().clear_bit();
            w.prog_burst_len().bits(DMA_BURST_LEN);
            w.desc_skip_len().bits(0);
            w.dma_arb_sch().clear_bit();
            w.pri_ratio().bits(0);
            w
        });
    }

    /// Configures the DMA operation mode, matching esp-idf
    /// `emac_hal_init_dma_default`.
    ///
    /// Store-and-forward is off in both directions with 64-byte thresholds: the
    /// FIFOs cannot hold a full frame on every chip, and enabling it there
    /// stalls the affected direction.
    pub fn dma_init_op_mode(&self) {
        // The TX FIFO flush must complete before any other write to this
        // register, so it is issued on its own first.
        EMAC_DMA::regs()
            .dmaoperation_mode()
            .modify(|_, w| w.flush_tx_fifo().set_bit());
        for _ in 0..1000 {
            if !EMAC_DMA::regs()
                .dmaoperation_mode()
                .read()
                .flush_tx_fifo()
                .bit_is_set()
            {
                break;
            }
        }

        EMAC_DMA::regs().dmaoperation_mode().modify(|_, w| unsafe {
            // Drop frames with a checksum error instead of forwarding them.
            w.dis_drop_tcpip_err_fram().clear_bit();
            // Flush received frames when descriptors or buffers run out.
            w.dis_flush_recv_frames().clear_bit();
            cfg_select! {
                esp32 => {
                    w.rx_store_forward().set_bit();
                }
                _ => {
                    // RX FIFO is only 256 B, too small for a full frame.
                    w.rx_store_forward().clear_bit();
                }
            }
            w.tx_str_fwd().clear_bit();
            w.tx_thresh_ctrl().bits(0); // 64 bytes
            w.rx_thresh_ctrl().bits(0); // 64 bytes
            w.fwd_err_frame().clear_bit();
            w.fwd_under_gf().clear_bit();
            // Start on a second frame before the first one's status is read.
            w.opt_second_frame().set_bit();
            w
        });
    }

    // ── DMA operation ─────────────────────────────────────────────────────

    /// Starts the MAC and DMA transmit and receive paths.
    ///
    /// The order follows esp-idf `emac_hal_start`. The EMAC Databook allows
    /// starting the DMA before enabling the MAC transmitter, but doing so has
    /// been observed to hang the transmitter.
    pub fn dma_start(&self) {
        EMAC_MAC::regs().config().modify(|_, w| w.tx().set_bit());
        EMAC_DMA::regs()
            .dmaoperation_mode()
            .modify(|_, w| w.start_stop_transmission_command().set_bit());

        EMAC_DMA::regs()
            .dmaoperation_mode()
            .modify(|_, w| w.start_stop_rx().set_bit());
        EMAC_MAC::regs().config().modify(|_, w| w.rx().set_bit());
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

    /// Enables the normal summary and RX interrupts, optionally TX as well
    /// (required in async mode).
    ///
    /// The abnormal summary stays masked: the ISR does not handle FBE/RBU, so
    /// a sticky error would re-enter it forever.
    pub fn dma_enable_interrupts(&self, enable_tx: bool) {
        EMAC_DMA::regs().dmain_en().modify(|_, w| {
            w.dmain_rie().set_bit();
            w.dmain_nise().set_bit();
            w.dmain_tie().bit(enable_tx)
        });
    }

    /// Masks every MAC-level interrupt source (PMT, LPI, timestamp, and
    /// RGMII/PCS on chips that have them).
    ///
    /// These share the peripheral interrupt line with the DMA but are not
    /// gated by `dmain_en`, and the ISR only acknowledges `dmastatus`. Leaving
    /// them unmasked lets a single PMT/LPI event re-enter the ISR forever.
    pub fn mask_mac_interrupts(&self) {
        unsafe {
            EMAC_MAC::regs().intmask().write(|w| w.bits(u32::MAX));
        }
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

    /// Configures the MAC for the given speed/duplex.
    ///
    /// TX/RX are left disabled; [`Self::dma_start`] enables them in the order
    /// the hardware requires.
    pub fn mac_init(&self, speed: Speed, duplex: Duplex) {
        EMAC_MAC::regs().config().modify(|_, w| {
            match speed {
                Speed::_10M => {
                    w.mii().set_bit();
                    w.fespeed().clear_bit();
                }
                Speed::_100M => {
                    w.mii().set_bit();
                    w.fespeed().set_bit();
                }
                Speed::_1000M => {
                    w.mii().clear_bit();
                }
            }
            w.duplex().bit(duplex == Duplex::Full);
            w.padcrcstrip().clear_bit();
            w.rxipcoffload().set_bit();
            w.retry().set_bit();
            w.watchdog().set_bit();
            w.rxown().set_bit();
            w.loopback().clear_bit();
            w.deferralcheck().clear_bit();
            w.rx().clear_bit();
            w.tx().clear_bit()
        });

        // Enable pass-all-multicast mode.
        EMAC_MAC::regs().ff().modify(|_, w| w.pam().set_bit());
    }

    /// Configures the MAC for the given speed.
    ///
    /// 10/100 Mbps sets `mii` (port select) and `fespeed`. 1000 Mbps clears `mii`.
    pub fn set_speed(&self, speed: Speed) {
        EMAC_MAC::regs().config().modify(|_, w| {
            match speed {
                Speed::_10M => {
                    w.mii().set_bit();
                    w.fespeed().clear_bit();
                }
                Speed::_100M => {
                    w.mii().set_bit();
                    w.fespeed().set_bit();
                }
                Speed::_1000M => {
                    w.mii().clear_bit();
                }
            }
            w
        });
    }

    /// Configures the MAC for the given duplex mode.
    pub fn set_duplex(&self, duplex: Duplex) {
        EMAC_MAC::regs()
            .config()
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

        EMAC_MAC::regs().addr0high().write(|w| unsafe {
            w.address0_hi().bits(hi as u16);
            w.address_enable0().set_bit()
        });
        EMAC_MAC::regs().addr0low().write(|w| unsafe { w.bits(lo) });
    }

    // ── MDIO ─────────────────────────────────────────────────────────────

    /// Reads one PHY register via the MDIO interface (Clause 22).
    pub fn mdio_read(&self, phy_addr: u8, reg: u8) -> u16 {
        EMAC_MAC::regs().gmiiaddr().write(|w| unsafe {
            w.miidev().bits(phy_addr);
            w.miireg().bits(reg);
            w.miicsrclk().bits(mdc_csr_clock_range());
            w.miiwrite().clear_bit();
            w.miibusy().set_bit()
        });

        self.mdio_wait();
        EMAC_MAC::regs().miidata().read().mii_data().bits()
    }

    /// Writes one PHY register via the MDIO interface (Clause 22).
    pub fn mdio_write(&self, phy_addr: u8, reg: u8, data: u16) {
        EMAC_MAC::regs()
            .miidata()
            .write(|w| unsafe { w.mii_data().bits(data) });
        EMAC_MAC::regs().gmiiaddr().write(|w| unsafe {
            w.miidev().bits(phy_addr);
            w.miireg().bits(reg);
            w.miicsrclk().bits(mdc_csr_clock_range());
            w.miiwrite().set_bit();
            w.miibusy().set_bit()
        });

        self.mdio_wait();
    }

    fn mdio_wait(&self) {
        while EMAC_MAC::regs().gmiiaddr().read().miibusy().bit_is_set() {}
    }
}
