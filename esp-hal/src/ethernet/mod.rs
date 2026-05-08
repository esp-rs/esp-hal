#![cfg_attr(docsrs, procmacros::doc_replace)]
//! EMAC Ethernet driver for ESP32.
//!
//! The driver wraps the three ESP32 EMAC PAC singletons (`EMAC_MAC`,
//! `EMAC_DMA`, `EMAC_EXT`) and provides blocking and async Ethernet APIs.
//!
//! # Quick start (RMII, external reference clock)
//!
//! ```rust,no_run
//! # {before_snippet}
//! use esp_hal::ethernet::{
//!     Ethernet,
//!     EthernetDmaStorage,
//!     clock::ExternalRefClock,
//!     phy::generic::GenericPhy,
//! };
//!
//! let mut storage: EthernetDmaStorage<10, 10> = EthernetDmaStorage::new();
//!
//! let mut eth = Ethernet::new_rmii(
//!     &mut storage,
//!     [0x02, 0x00, 0x00, 0x12, 0x34, 0x56],
//!     ExternalRefClock::new(peripherals.GPIO0), // REF_CLK from PHY on GPIO0
//!     GenericPhy::new(1),
//!     peripherals.GPIO25,
//!     peripherals.GPIO26,
//!     peripherals.GPIO27,
//!     peripherals.GPIO19,
//!     peripherals.GPIO22,
//!     peripherals.GPIO21,
//!     peripherals.GPIO23,
//!     peripherals.GPIO18, // MDC, MDIO
//!     peripherals.EMAC_MAC,
//!     peripherals.EMAC_DMA,
//!     peripherals.EMAC_EXT,
//! )
//! .unwrap();
//! // `eth` has type `Ethernet<'_, Blocking, GenericPhy>`
//! # {after_snippet}
//! ```

use core::{marker::PhantomData, task::Context};

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    gpio::{
        AlternateFunction,
        DriveMode,
        DriveStrength,
        OutputConfig,
        Pin,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    interrupt,
    peripherals::{EMAC_DMA, ETH, Interrupt},
    private::Sealed,
    system::{GenericPeripheralGuard, Peripheral},
};

#[cfg_attr(esp32, path = "clock/esp32.rs")]
pub mod clock;
pub(crate) mod dma;
pub(crate) mod embassy_net;
pub mod mac;
pub mod phy;

use core::task::Poll;

pub use dma::EthernetDmaStorage;
use dma::{RDesRing, TDesRing};
use mac::{EmacRegs, LinkState, Speed};
use phy::{MdioDriver, Phy};

// ── Driver error ─────────────────────────────────────────────────────────────

/// Error type returned by `Ethernet` operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// TX ring has no available slot.
    TxFull,
    /// Frame is larger than the DMA buffer.
    FrameTooLarge,
    /// No received frame is currently available.
    NoFrame,
    /// PHY initialization timed out.
    PhyTimeout,
}

/// Sealed trait implemented by all RMII clock configurations.
pub trait RmiiClockConfig: Sealed {
    /// Configures EMAC_EXT and any related hardware (e.g. APLL) for RMII,
    /// including the clock input/output GPIO pad.
    ///
    /// Consumes `self` so that the pin carried inside the struct is configured
    /// and ownership is transferred to the EMAC peripheral.
    fn configure(self);
}

// ── IOMUX pin-signal traits ───────────────────────────────────────────────────
//
// EMAC data-plane pins can only be connected via IO_MUX (function 5 on ESP32)
// and cannot use the GPIO matrix.  The traits below are implemented only for
// the GPIO numbers documented in the ESP32 TRM, making wrong-pin errors
// compile-time failures.

macro_rules! emac_iomux_trait {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        pub trait $name: crate::private::Sealed {
            #[doc(hidden)]
            fn configure_iomux(self);
        }
    };
}

emac_iomux_trait!(EmacRxd0, "GPIO that carries `EMAC_RXD0`.");
emac_iomux_trait!(EmacRxd1, "GPIO that carries `EMAC_RXD1`.");
emac_iomux_trait!(EmacRxDv, "GPIO that carries `EMAC_RX_DV`.");
emac_iomux_trait!(EmacTxd0, "GPIO that carries `EMAC_TXD0`.");
emac_iomux_trait!(EmacTxd1, "GPIO that carries `EMAC_TXD1`.");
emac_iomux_trait!(EmacTxEn, "GPIO that carries `EMAC_TX_EN`.");
emac_iomux_trait!(
    EmacClkOut,
    "GPIO that carries `EMAC_CLK_OUT` / `EMAC_CLK_180`."
);
emac_iomux_trait!(EmacRxd2, "GPIO that carries `EMAC_RXD2`.");
emac_iomux_trait!(EmacRxd3, "GPIO that carries `EMAC_RXD3`.");
emac_iomux_trait!(EmacTxd2, "GPIO that carries `EMAC_TXD2`.");
emac_iomux_trait!(EmacTxd3, "GPIO that carries `EMAC_TXD3`.");
emac_iomux_trait!(EmacTxClk, "GPIO that carries `EMAC_TX_CLK`.");
emac_iomux_trait!(EmacRxClk, "GPIO that carries `EMAC_RX_CLK`.");
emac_iomux_trait!(EmacTxEr, "GPIO that carries `EMAC_TX_ER`.");
emac_iomux_trait!(EmacRxEr, "GPIO that carries `EMAC_RX_ER`.");

emac_iomux_trait!(
    EmacRmiiClkIn,
    "GPIO suitable for use as the RMII reference clock input pin."
);

macro_rules! implement_trait {
    ($trait:ident, $gpio:ident, $af:ident) => {
        impl $trait for crate::peripherals::$gpio<'_> {
            fn configure_iomux(self) {
                // Set the IOMUX alternate function and enable the input buffer.
                // fun_ie is required for RX/clock-input pins; it is harmless on
                // TX-direction pads where no peripheral reads the feedback path.
                crate::gpio::io_mux_reg(self.number()).modify(|_, w| {
                    unsafe { w.mcu_sel().bits(AlternateFunction::$af as u8) };
                    w.fun_ie().set_bit()
                });
            }
        }
    };
}

// FIXME: signal names are ESP32-specific
for_each_iomux_function! {
    (EMAC_RXD0, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxd0, $gpio, $af);
    };
    (EMAC_RXD1, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxd1, $gpio, $af);
    };
    (EMAC_RX_DV, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxDv, $gpio, $af);
    };
    (EMAC_TXD0, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxd0, $gpio, $af);
    };
    (EMAC_TXD1, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxd1, $gpio, $af);
    };
    (EMAC_TX_EN, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxEn, $gpio, $af);
    };
    (EMAC_RX_CLK, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxClk, $gpio, $af);
    };
    (EMAC_CLK_OUT, $gpio:ident, $af:ident) => {
        implement_trait!(EmacClkOut, $gpio, $af);
    };
    (EMAC_CLK_180, $gpio:ident, $af:ident) => {
        implement_trait!(EmacClkOut, $gpio, $af);
    };
    (EMAC_RXD2, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxd2, $gpio, $af);
    };
    (EMAC_RXD3, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxd3, $gpio, $af);
    };
    (EMAC_TXD2, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxd2, $gpio, $af);
    };
    (EMAC_TXD3, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxd3, $gpio, $af);
    };
    (EMAC_TX_CLK, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRmiiClkIn, $gpio, $af);
        implement_trait!(EmacTxClk, $gpio, $af);
    };
    (EMAC_TX_ER, $gpio:ident, $af:ident) => {
        implement_trait!(EmacTxEr, $gpio, $af);
    };
    (EMAC_RX_ER, $gpio:ident, $af:ident) => {
        implement_trait!(EmacRxEr, $gpio, $af);
    };
}

// ── Async wakers ──────────────────────────────────────────────────────────────

pub(super) static RX_WAKER: AtomicWaker = AtomicWaker::new();
pub(super) static TX_WAKER: AtomicWaker = AtomicWaker::new();
/// DMASTATUS bit mask for the RX-complete interrupt.
const DMASTATUS_RI: u32 = 1 << 6;
/// DMASTATUS bit mask for the TX-complete interrupt.
const DMASTATUS_TI: u32 = 1 << 0;

#[crate::handler]
fn eth_mac_isr() {
    let status = EMAC_DMA::regs().dmastatus().read().bits();
    unsafe {
        EMAC_DMA::regs().dmastatus().write(|w| w.bits(status));
    }
    if status & DMASTATUS_RI != 0 {
        RX_WAKER.wake();
    }
    if status & DMASTATUS_TI != 0 {
        TX_WAKER.wake();
    }
}

// ── Ethernet struct ───────────────────────────────────────────────────────────

/// EMAC Ethernet driver.
pub struct Ethernet<'d, DM: DriverMode, P: Phy> {
    regs: EmacRegs,
    _clock_guard: GenericPeripheralGuard<{ Peripheral::Emac as u8 }>,
    tx: TDesRing<'d>,
    rx: RDesRing<'d>,
    phy: P,
    mac_addr: [u8; 6],
    _eth: ETH<'d>,
    _mode: PhantomData<DM>,
}

impl<'d, P: Phy> Ethernet<'d, Blocking, P> {
    /// Creates an RMII Ethernet driver.
    #[expect(clippy::too_many_arguments)]
    pub fn new_rmii<const RX: usize, const TX: usize>(
        eth: ETH<'d>,
        storage: &'d mut EthernetDmaStorage<RX, TX>,
        mac_addr: [u8; 6],
        clock: impl RmiiClockConfig,
        phy: P,
        rxd0: impl EmacRxd0 + 'd,
        rxd1: impl EmacRxd1 + 'd,
        rx_dv: impl EmacRxDv + 'd,
        txd0: impl EmacTxd0 + 'd,
        txd1: impl EmacTxd1 + 'd,
        tx_en: impl EmacTxEn + 'd,
        mdc: impl PeripheralOutput<'d>,
        mdio: impl PeripheralInput<'d> + PeripheralOutput<'d>,
    ) -> Result<Self, Error> {
        let clock_guard = GenericPeripheralGuard::new();

        // Configure IOMUX-only data pins (alt function 5).
        rxd0.configure_iomux();
        rxd1.configure_iomux();
        rx_dv.configure_iomux();
        txd0.configure_iomux();
        txd1.configure_iomux();
        tx_en.configure_iomux();

        configure_mdio(mdc, mdio);

        // Configure ref-clock pin + EMAC_EXT clock source + PHY interface.
        clock.configure();

        init_common(clock_guard, storage, mac_addr, phy, eth)
    }

    /// Creates a MII Ethernet driver.
    #[expect(clippy::too_many_arguments)]
    pub fn new_mii<const RX: usize, const TX: usize>(
        eth: ETH<'d>,
        storage: &'d mut EthernetDmaStorage<RX, TX>,
        mac_addr: [u8; 6],
        phy: P,
        rxd0: impl EmacRxd0 + 'd,
        rxd1: impl EmacRxd1 + 'd,
        rx_dv: impl EmacRxDv + 'd,
        txd0: impl EmacTxd0 + 'd,
        txd1: impl EmacTxd1 + 'd,
        tx_en: impl EmacTxEn + 'd,
        rxd2: impl EmacRxd2 + 'd,
        rxd3: impl EmacRxd3 + 'd,
        txd2: impl EmacTxd2 + 'd,
        txd3: impl EmacTxd3 + 'd,
        tx_clk: impl EmacTxClk + 'd,
        rx_clk: impl EmacRxClk + 'd,
        tx_er: impl EmacTxEr + 'd,
        rx_er: impl EmacRxEr + 'd,
        crs: impl PeripheralInput<'d>,
        col: impl PeripheralInput<'d>,
        mdc: impl PeripheralOutput<'d>,
        mdio: impl PeripheralInput<'d> + PeripheralOutput<'d>,
    ) -> Result<Self, Error> {
        let clock_guard = GenericPeripheralGuard::new();

        rxd0.configure_iomux();
        rxd1.configure_iomux();
        rx_dv.configure_iomux();
        txd0.configure_iomux();
        txd1.configure_iomux();
        tx_en.configure_iomux();
        rxd2.configure_iomux();
        rxd3.configure_iomux();
        txd2.configure_iomux();
        txd3.configure_iomux();
        tx_clk.configure_iomux();
        rx_clk.configure_iomux();
        tx_er.configure_iomux();
        rx_er.configure_iomux();

        configure_mdio(mdc, mdio);

        let crs: interconnect::InputSignal<'_> = crs.into();
        crs.set_input_enable(true);
        crate::gpio::InputSignal::EMAC_CRS.connect_to(&crs);

        let col: interconnect::InputSignal<'_> = col.into();
        col.set_input_enable(true);
        crate::gpio::InputSignal::EMAC_COL.connect_to(&col);

        clock::MiiClock.configure();

        init_common(clock_guard, storage, mac_addr, phy, eth)
    }

    /// Returns the received frame data if one is ready.
    ///
    /// Call [`pop_rx`][Self::pop_rx] after processing the frame.
    pub fn receive(&mut self) -> Result<&mut [u8], Error> {
        match self.rx.receive() {
            Some(frame) => Ok(frame),
            None => {
                // receive() may have recycled error frames back to DMA ownership
                // without a poll-demand write. Poke the RX DMA so it resumes.
                self.regs.demand_rx_poll();
                Err(Error::NoFrame)
            }
        }
    }

    /// Releases the current RX descriptor back to DMA.
    pub fn pop_rx(&mut self) {
        self.rx.pop();
        self.regs.demand_rx_poll();
    }

    /// Converts to async mode, enabling DMA interrupts and binding the ISR.
    pub fn into_async(self) -> Ethernet<'d, Async, P> {
        self.regs.dma_enable_interrupts(true);
        interrupt::bind_handler(Interrupt::ETH_MAC, eth_mac_isr);
        Ethernet {
            regs: self.regs,
            tx: self.tx,
            rx: self.rx,
            phy: self.phy,
            mac_addr: self.mac_addr,
            _clock_guard: self._clock_guard,
            _eth: self._eth,
            _mode: PhantomData,
        }
    }
}

fn configure_mdio<'d>(
    mdc: impl PeripheralOutput<'d>,
    mdio: impl PeripheralInput<'d> + PeripheralOutput<'d>,
) {
    // MDC is output-only; MDIO is bidirectional open-drain.
    let mdc: interconnect::OutputSignal<'_> = mdc.into();
    mdc.apply_output_config(&OutputConfig::default().with_drive_strength(DriveStrength::_20mA));
    crate::gpio::OutputSignal::EMAC_MDC.connect_to(&mdc);

    let mdio: interconnect::OutputSignal<'_> = mdio.into();
    mdio.apply_output_config(
        &OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_drive_strength(DriveStrength::_20mA),
    );
    crate::gpio::InputSignal::EMAC_MDI.connect_to(&mdio);
    crate::gpio::OutputSignal::EMAC_MDO.connect_to(&mdio);
    mdio.set_input_enable(true);
    mdio.set_output_enable(true);
}

impl<'d, Dm: DriverMode, P: Phy> Ethernet<'d, Dm, P> {
    /// Copies `frame` into the next TX slot and initiates transmission.
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.tx.transmit(frame).map_err(|e| match e {
            dma::TxError::RingFull => Error::TxFull,
            dma::TxError::FrameTooLarge => Error::FrameTooLarge,
        })?;
        self.regs.demand_tx_poll();
        Ok(())
    }

    /// Returns the current PHY link state.
    ///
    /// Async callers are encouraged to pass the context to allow the PHY driver to wake them when
    /// the link state needs to be re-polled.
    pub fn poll_link(&mut self, cx: Option<&mut Context<'_>>) -> LinkState {
        let mdio = MdioDriver::new(&self.regs);
        self.phy.poll_link(&mdio, cx)
    }

    /// Returns the MAC address configured during construction.
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }
}

impl<'d, P: Phy> Ethernet<'d, Async, P> {
    /// Asynchronously waits for a TX slot and transmits `frame`.
    pub async fn transmit_async(&mut self, frame: &[u8]) -> Result<(), Error> {
        loop {
            match self.tx.transmit(frame) {
                Ok(()) => {
                    self.regs.demand_tx_poll();
                    return Ok(());
                }
                Err(dma::TxError::FrameTooLarge) => return Err(Error::FrameTooLarge),
                Err(dma::TxError::RingFull) => {
                    core::future::poll_fn(|cx| {
                        TX_WAKER.register(cx.waker());
                        if self.tx.has_capacity() {
                            Poll::Ready(())
                        } else {
                            Poll::Pending
                        }
                    })
                    .await;
                }
            }
        }
    }

    /// Asynchronously waits for a frame and copies it into `buf`.
    pub async fn receive_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        loop {
            core::future::poll_fn(|cx| {
                RX_WAKER.register(cx.waker());
                if self.rx.has_packet() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            if let Some(data) = self.rx.receive() {
                let len = data.len();
                if len > buf.len() {
                    self.rx.pop();
                    self.regs.demand_rx_poll();
                    return Err(Error::FrameTooLarge);
                }
                buf[..len].copy_from_slice(data);
                self.rx.pop();
                self.regs.demand_rx_poll();
                return Ok(len);
            }
        }
    }

    /// Converts back to blocking mode, disabling all DMA interrupts.
    pub fn into_blocking(self) -> Ethernet<'d, Blocking, P> {
        self.regs.dma_disable_interrupts();
        Ethernet {
            regs: self.regs,
            tx: self.tx,
            rx: self.rx,
            phy: self.phy,
            mac_addr: self.mac_addr,
            _clock_guard: self._clock_guard,
            _eth: self._eth,
            _mode: PhantomData,
        }
    }

    /// Returns `true` if there is at least one free TX descriptor slot.
    pub fn tx_has_capacity(&self) -> bool {
        self.tx.has_capacity()
    }

    /// Peeks at the next received frame without consuming it.
    ///
    /// Returns `Some((len, data_slice))` if a valid frame is ready, or `None`.
    /// The caller must call [`release_rx`][Self::release_rx] after processing.
    pub fn peek_rx(&mut self) -> Option<&mut [u8]> {
        self.rx.receive()
    }

    /// Releases the current RX descriptor back to the DMA engine.
    pub fn release_rx(&mut self) {
        self.rx.pop();
        self.regs.demand_rx_poll();
    }
}

// ── Shared init ───────────────────────────────────────────────────────────────

fn init_common<'d, P: Phy, const RX: usize, const TX: usize>(
    clock_guard: GenericPeripheralGuard<{ Peripheral::Emac as u8 }>,
    storage: &'d mut EthernetDmaStorage<RX, TX>,
    mac_addr: [u8; 6],
    mut phy: P,
    eth: ETH<'d>,
) -> Result<Ethernet<'d, Blocking, P>, Error> {
    let regs = EmacRegs::new();

    // DMA soft-reset (also enables enhanced 32B descriptor format).
    regs.dma_soft_reset();

    // Init PHY (MDIO requires DMA/MAC clocks to be running after reset).
    let mdio = MdioDriver::new(&regs);
    phy.init(&mdio).map_err(|_| Error::PhyTimeout)?;

    // Configure MAC defaults.
    regs.mac_init(Speed::_100M, mac::Duplex::Full);
    regs.set_mac_address(&mac_addr);

    // Build descriptor rings from the erased slice references.
    let tx = TDesRing::new(&mut storage.tx_descs, &mut storage.tx_bufs);
    let rx = RDesRing::new(&mut storage.rx_descs, &mut storage.rx_bufs);

    // Program descriptor list addresses into DMA.
    regs.set_descriptor_lists(tx.base_ptr() as u32, rx.base_ptr() as u32);

    // In blocking mode all DMA interrupts stay disabled; async mode enables
    // them via into_async().
    regs.dma_disable_interrupts();

    // Start both DMA engines.
    regs.dma_start();

    Ok(Ethernet {
        regs,
        tx,
        rx,
        phy,
        mac_addr,
        _clock_guard: clock_guard,
        _eth: eth,
        _mode: PhantomData,
    })
}
