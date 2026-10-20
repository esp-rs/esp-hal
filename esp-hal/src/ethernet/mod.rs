#![cfg_attr(docsrs, procmacros::doc_replace(
    "clk_in_gpio" => {
        cfg(esp32) => "GPIO0",
        cfg(esp32p4) => "GPIO50",
    },
    "rxd0_gpio" => {
        cfg(esp32) => "GPIO25",
        cfg(esp32p4) => "GPIO29",
    },
    "rxd1_gpio" => {
        cfg(esp32) => "GPIO26",
        cfg(esp32p4) => "GPIO30",
    },
    "rxdv_gpio" => {
        cfg(esp32) => "GPIO27",
        cfg(esp32p4) => "GPIO28",
    },
    "txd0_gpio" => {
        cfg(esp32) => "GPIO19",
        cfg(esp32p4) => "GPIO34",
    },
    "txd1_gpio" => {
        cfg(esp32) => "GPIO22",
        cfg(esp32p4) => "GPIO35",
    },
    "txen_gpio" => {
        cfg(esp32) => "GPIO21",
        cfg(esp32p4) => "GPIO49",
    },
    "mdc_gpio" => {
        cfg(esp32) => "GPIO23",
        cfg(esp32p4) => "GPIO31",
    },
    "mdio_gpio" => {
        cfg(esp32) => "GPIO18",
        cfg(esp32p4) => "GPIO52",
    }
))]
//! EMAC Ethernet driver for ESP32.
//!
//! The driver provides blocking and async Ethernet APIs.
//!
//! # Quick start (RMII, external reference clock)
//!
//! ```rust,no_run
//! # {before_snippet}
//! use esp_hal::ethernet::{
//!     Ethernet,
//!     EthernetDmaStorage,
//!     RmiiPinBundle,
//!     clock::ExternalRefClock,
//!     phy::generic::GenericPhy,
//! };
//!
//! let mut storage: EthernetDmaStorage<10, 10> = EthernetDmaStorage::new();
//!
//! let mut eth = Ethernet::new(
//!     peripherals.ETH,
//!     &mut storage,
//!     [0x02, 0x00, 0x00, 0x12, 0x34, 0x56],
//!     GenericPhy::new_auto(),
//!     RmiiPinBundle {
//!         clock: ExternalRefClock::new(peripherals.__clk_in_gpio__), // REF_CLK from PHY
//!         rxd0: peripherals.__rxd0_gpio__,
//!         rxd1: peripherals.__rxd1_gpio__,
//!         rx_dv: peripherals.__rxdv_gpio__,
//!         txd0: peripherals.__txd0_gpio__,
//!         txd1: peripherals.__txd1_gpio__,
//!         tx_en: peripherals.__txen_gpio__,
//!         mdc: peripherals.__mdc_gpio__,
//!         mdio: peripherals.__mdio_gpio__,
//!     },
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
    ethernet::phy::PhyError,
    gpio::{
        AlternateFunction,
        DriveStrength,
        InputConfig,
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
#[cfg_attr(esp32p4, path = "clock/esp32p4.rs")]
pub mod clock;
pub(crate) mod dma;
pub(crate) mod embassy_net;
pub mod mac;
pub mod phy;

use core::task::Poll;

pub use dma::EthernetDmaStorage;
use dma::{RDesRing, TDesRing};
use mac::{Duplex, EmacRegs, LinkState, Speed};
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
    /// PHY (initialization) error.
    Phy(PhyError),
}

/// Sealed trait implemented by all RMII clock configurations.
pub trait RmiiClockConfig: Sealed {
    /// Configures the hardware for RMII, including the clock input/output GPIO pad.
    ///
    /// Consumes `self` so that the pin carried inside the struct is configured
    /// and ownership is transferred to the peripheral.
    fn configure(self);
}

// ── IOMUX pin-signal traits ───────────────────────────────────────────────────
//
// EMAC data-plane pins can only be connected via IO_MUX (function 5 on ESP32)
// and cannot use the GPIO matrix.  The traits below are implemented only for
// the GPIO numbers documented in the ESP32 TRM, making wrong-pin errors
// compile-time failures.

macro_rules! emac_pin {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        pub trait $name: crate::private::Sealed {
            #[doc(hidden)]
            fn configure_iomux(self);
        }
    };
}

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

// MII traits
cfg_if::cfg_if! {
    if #[cfg(ethernet_mii_via_gpio_matrix)] {
        macro_rules! mii_pin {
            ($name:ident, $doc:literal $(, in=$input:ident)? $(, out=$output:ident)?) => {
                #[doc = $doc]
                pub trait $name: crate::gpio::InputPin + crate::gpio::OutputPin + Sized {
                    #[doc(hidden)]
                    fn configure_iomux(self) { // Intentionally consumes to allow converting
                        let peri_signal: interconnect::OutputSignal<'_> = self.into();
                        $(
                            let signal = crate::gpio::InputSignal::$input;

                            signal.connect_to(&peri_signal);
                            peri_signal.set_input_enable(true);
                        )?
                        $(
                            let signal = crate::gpio::OutputSignal::$output;
                            peri_signal.apply_output_config(
                                &OutputConfig::default()
                                    .with_drive_strength(DriveStrength::_20mA),
                            );
                            signal.connect_to(&peri_signal);
                            peri_signal.set_output_enable(true);
                        )?
                    }
                }
            };
        }
        mii_pin!(MiiTxClk, "MII TX clock pin", in = EMAC_TX_CLK);
        mii_pin!(MiiTxEn, "MII TX enable pin", out = EMAC_TXEN);
        mii_pin!(MiiTxd0, "MII TXD0 pin", out = EMAC_TXD0);
        mii_pin!(MiiTxd1, "MII TXD1 pin", out = EMAC_TXD1);
        mii_pin!(MiiTxd2, "MII TXD2 pin", out = EMAC_TXD2);
        mii_pin!(MiiTxd3, "MII TXD3 pin", out = EMAC_TXD3);
        mii_pin!(MiiRxClk, "MII RX clock pin", in = EMAC_RX_CLK);
        mii_pin!(MiiRxDv, "MII RX data valid pin", in = EMAC_RXDV);
        mii_pin!(MiiRxd0, "MII RXD0 pin", in = EMAC_RXD0);
        mii_pin!(MiiRxd1, "MII RXD1 pin", in = EMAC_RXD1);
        mii_pin!(MiiRxd2, "MII RXD2 pin", in = EMAC_RXD2);
        mii_pin!(MiiRxd3, "MII RXD3 pin", in = EMAC_RXD3);

        for_each_gpio! {
            ($n:literal, $gpio:ident $($_rest:tt)*) => {
                impl MiiTxClk for crate::peripherals::$gpio<'_> {}
                impl MiiTxEn for crate::peripherals::$gpio<'_> {}
                impl MiiTxd0 for crate::peripherals::$gpio<'_> {}
                impl MiiTxd1 for crate::peripherals::$gpio<'_> {}
                impl MiiTxd2 for crate::peripherals::$gpio<'_> {}
                impl MiiTxd3 for crate::peripherals::$gpio<'_> {}
                impl MiiRxClk for crate::peripherals::$gpio<'_> {}
                impl MiiRxDv for crate::peripherals::$gpio<'_> {}
                impl MiiRxd0 for crate::peripherals::$gpio<'_> {}
                impl MiiRxd1 for crate::peripherals::$gpio<'_> {}
                impl MiiRxd2 for crate::peripherals::$gpio<'_> {}
                impl MiiRxd3 for crate::peripherals::$gpio<'_> {}
            };
        }
    } else {
        emac_pin!(MiiTxClk, "MII TX clock pin");
        emac_pin!(MiiTxEn,  "MII TX enable pin");
        emac_pin!(MiiTxd0,  "MII TXD0 pin");
        emac_pin!(MiiTxd1,  "MII TXD1 pin");
        emac_pin!(MiiTxd2,  "MII TXD2 pin");
        emac_pin!(MiiTxd3,  "MII TXD3 pin");
        emac_pin!(MiiRxClk, "MII RX clock pin");
        emac_pin!(MiiRxDv,  "MII RX data valid pin");
        emac_pin!(MiiRxd0,  "MII RXD0 pin");
        emac_pin!(MiiRxd1,  "MII RXD1 pin");
        emac_pin!(MiiRxd2,  "MII RXD2 pin");
        emac_pin!(MiiRxd3,  "MII RXD3 pin");

        for_each_iomux_function! {
            (EMAC_TX_CLK, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxClk, $gpio, $af);
            };
            (EMAC_TXEN, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxEn, $gpio, $af);
            };
            (EMAC_TXD0, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxd0, $gpio, $af);
            };
            (EMAC_TXD1, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxd1, $gpio, $af);
            };
            (EMAC_TXD2, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxd2, $gpio, $af);
            };
            (EMAC_TXD3, $gpio:ident, $af:ident) => {
                implement_trait!(MiiTxd3, $gpio, $af);
            };

            (EMAC_RX_CLK, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxClk, $gpio, $af);
            };
            (EMAC_RXDV, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxDv, $gpio, $af);
            };
            (EMAC_RXD0, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxd0, $gpio, $af);
            };
            (EMAC_RXD1, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxd1, $gpio, $af);
            };
            (EMAC_RXD2, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxd2, $gpio, $af);
            };
            (EMAC_RXD3, $gpio:ident, $af:ident) => {
                implement_trait!(MiiRxd3, $gpio, $af);
            };
        }
    }
}

// RMII traits
emac_pin!(RmiiClkIn, "RMII CLK input pin");
emac_pin!(RmiiClkOut, "RMII CLK output pin");
emac_pin!(RmiiTxEn, "RMII TX enable pin");
emac_pin!(RmiiTxd0, "RMII TXD0 pin");
emac_pin!(RmiiTxd1, "RMII TXD1 pin");
emac_pin!(RmiiCrsDv, "RMII CRS/DV pin");
emac_pin!(RmiiRxd0, "RMII RXD0 pin");
emac_pin!(RmiiRxd1, "RMII RXD1 pin");

// RMII traits
for_each_iomux_function! {
    (EMAC_TXEN, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiTxEn, $gpio, $af);
    };
    (EMAC_TXD0, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiTxd0, $gpio, $af);
    };
    (EMAC_TXD1, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiTxd1, $gpio, $af);
    };
    (EMAC_RXDV, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiCrsDv, $gpio, $af);
    };
    (EMAC_RXD0, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiRxd0, $gpio, $af);
    };
    (EMAC_RXD1, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiRxd1, $gpio, $af);
    };

    // ESP32-specific
    (EMAC_TX_CLK, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiClkIn, $gpio, $af);
    };
    (EMAC_CLK_OUT, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiClkOut, $gpio, $af);
    };
    (EMAC_CLK_180, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiClkOut, $gpio, $af);
    };

    // ESP32-P4-specific: MPLL-derived 50 MHz reference clock output pad.
    (REF_50M_CLK, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiClkOut, $gpio, $af);
    };
    (EMAC_RMII_CLK, $gpio:ident, $af:ident) => {
        implement_trait!(RmiiClkIn, $gpio, $af);
    };
}

/// RMII or MII pad wiring for [`Ethernet::new`].
pub trait EthernetPinBundle: crate::private::Sealed {
    /// Applies the required configuration. Intended to be
    /// called by the Ethernet driver internally.
    fn apply(self);
}

/// Wired RMII pads and RMII clock (pass to [`Ethernet::new`]).
#[allow(
    missing_docs,
    reason = "The field names are indicative of their function."
)]
pub struct RmiiPinBundle<C, Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Mdc, Mdio> {
    pub clock: C,
    pub rxd0: Rxd0,
    pub rxd1: Rxd1,
    pub rx_dv: RxDv,
    pub txd0: Txd0,
    pub txd1: Txd1,
    pub tx_en: TxEn,
    pub mdc: Mdc,
    pub mdio: Mdio,
}

impl<C, Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Mdc, Mdio> crate::private::Sealed
    for RmiiPinBundle<C, Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Mdc, Mdio>
{
}

impl<'d, C, Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Mdc, Mdio> EthernetPinBundle
    for RmiiPinBundle<C, Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Mdc, Mdio>
where
    C: RmiiClockConfig,
    Rxd0: RmiiRxd0 + 'd,
    Rxd1: RmiiRxd1 + 'd,
    RxDv: RmiiCrsDv + 'd,
    Txd0: RmiiTxd0 + 'd,
    Txd1: RmiiTxd1 + 'd,
    TxEn: RmiiTxEn + 'd,
    Mdc: PeripheralOutput<'d>,
    Mdio: PeripheralInput<'d> + PeripheralOutput<'d>,
{
    fn apply(self) {
        self.rxd0.configure_iomux();
        self.rxd1.configure_iomux();
        self.rx_dv.configure_iomux();
        self.txd0.configure_iomux();
        self.txd1.configure_iomux();
        self.tx_en.configure_iomux();

        configure_mdio(self.mdc, self.mdio);
        self.clock.configure();
    }
}

/// Wired MII pads (pass to [`Ethernet::new`]).
#[allow(
    missing_docs,
    reason = "The field names are indicative of their function."
)]
pub struct MiiPinBundle<
    Rxd0,
    Rxd1,
    RxDv,
    Txd0,
    Txd1,
    TxEn,
    Rxd2,
    Rxd3,
    Txd2,
    Txd3,
    TxClk,
    RxClk,
    Crs,
    Col,
    Mdc,
    Mdio,
> {
    pub rxd0: Rxd0,
    pub rxd1: Rxd1,
    pub rx_dv: RxDv,
    pub txd0: Txd0,
    pub txd1: Txd1,
    pub tx_en: TxEn,
    pub rxd2: Rxd2,
    pub rxd3: Rxd3,
    pub txd2: Txd2,
    pub txd3: Txd3,
    pub tx_clk: TxClk,
    pub rx_clk: RxClk,
    pub crs: Crs,
    pub col: Col,
    pub mdc: Mdc,
    pub mdio: Mdio,
}

impl<Rxd0, Rxd1, RxDv, Txd0, Txd1, TxEn, Rxd2, Rxd3, Txd2, Txd3, TxClk, RxClk, Crs, Col, Mdc, Mdio>
    crate::private::Sealed
    for MiiPinBundle<
        Rxd0,
        Rxd1,
        RxDv,
        Txd0,
        Txd1,
        TxEn,
        Rxd2,
        Rxd3,
        Txd2,
        Txd3,
        TxClk,
        RxClk,
        Crs,
        Col,
        Mdc,
        Mdio,
    >
{
}

impl<
    'd,
    Rxd0,
    Rxd1,
    RxDv,
    Txd0,
    Txd1,
    TxEn,
    Rxd2,
    Rxd3,
    Txd2,
    Txd3,
    TxClk,
    RxClk,
    Crs,
    Col,
    Mdc,
    Mdio,
> EthernetPinBundle
    for MiiPinBundle<
        Rxd0,
        Rxd1,
        RxDv,
        Txd0,
        Txd1,
        TxEn,
        Rxd2,
        Rxd3,
        Txd2,
        Txd3,
        TxClk,
        RxClk,
        Crs,
        Col,
        Mdc,
        Mdio,
    >
where
    Rxd0: MiiRxd0 + 'd,
    Rxd1: MiiRxd1 + 'd,
    RxDv: MiiRxDv + 'd,
    Txd0: MiiTxd0 + 'd,
    Txd1: MiiTxd1 + 'd,
    TxEn: MiiTxEn + 'd,
    Rxd2: MiiRxd2 + 'd,
    Rxd3: MiiRxd3 + 'd,
    Txd2: MiiTxd2 + 'd,
    Txd3: MiiTxd3 + 'd,
    TxClk: MiiTxClk + 'd,
    RxClk: MiiRxClk + 'd,
    Crs: PeripheralInput<'d>,
    Col: PeripheralInput<'d>,
    Mdc: PeripheralOutput<'d>,
    Mdio: PeripheralInput<'d> + PeripheralOutput<'d>,
{
    fn apply(self) {
        self.rxd0.configure_iomux();
        self.rxd1.configure_iomux();
        self.rx_dv.configure_iomux();
        self.txd0.configure_iomux();
        self.txd1.configure_iomux();
        self.tx_en.configure_iomux();
        self.rxd2.configure_iomux();
        self.rxd3.configure_iomux();
        self.txd2.configure_iomux();
        self.txd3.configure_iomux();
        self.tx_clk.configure_iomux();
        self.rx_clk.configure_iomux();

        configure_mdio(self.mdc, self.mdio);

        let crs: interconnect::InputSignal<'_> = self.crs.into();
        crs.set_input_enable(true);
        crate::gpio::InputSignal::EMAC_CRS.connect_to(&crs);

        let col: interconnect::InputSignal<'_> = self.col.into();
        col.set_input_enable(true);
        crate::gpio::InputSignal::EMAC_COL.connect_to(&col);

        clock::MiiClock.configure();
    }
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
    _eth: ETH<'d>,
    _clock_guard: GenericPeripheralGuard<{ Peripheral::Emac as u8 }>,
    tx: TDesRing<'d>,
    rx: RDesRing<'d>,
    phy: P,
    mac_addr: [u8; 6],
    speed: Speed,
    duplex: Duplex,
    _mode: PhantomData<DM>,
}

impl<'d, P: Phy> Ethernet<'d, Blocking, P> {
    /// Creates an Ethernet driver using RMII ([`RmiiPinBundle`]) or MII ([`MiiPinBundle`]).
    pub fn new<const RX: usize, const TX: usize>(
        eth: ETH<'d>,
        storage: &'d mut EthernetDmaStorage<RX, TX>,
        mac_addr: [u8; 6],
        phy: P,
        pins: impl EthernetPinBundle + 'd,
    ) -> Result<Self, Error> {
        let clock_guard = GenericPeripheralGuard::new();

        pins.apply();

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
                EmacRegs.demand_rx_poll();
                Err(Error::NoFrame)
            }
        }
    }

    /// Releases the current RX descriptor back to DMA.
    pub fn pop_rx(&mut self) {
        self.rx.pop();
        EmacRegs.demand_rx_poll();
    }

    /// Converts to async mode, enabling DMA interrupts and binding the ISR.
    pub fn into_async(self) -> Ethernet<'d, Async, P> {
        EmacRegs.dma_enable_interrupts(true);
        interrupt::bind_handler(Interrupt::ETH_MAC, eth_mac_isr);
        Ethernet {
            tx: self.tx,
            rx: self.rx,
            phy: self.phy,
            mac_addr: self.mac_addr,
            speed: self.speed,
            duplex: self.duplex,
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
    // MDC is output-only; MDIO is bidirectional, controlled by the peripheral.
    let mdc: interconnect::OutputSignal<'_> = mdc.into();
    mdc.apply_output_config(&OutputConfig::default().with_drive_strength(DriveStrength::_20mA));
    crate::gpio::OutputSignal::EMAC_MDC.connect_to(&mdc);

    let mdio: interconnect::OutputSignal<'_> = mdio.into();
    mdio.apply_output_config(&OutputConfig::default().with_drive_strength(DriveStrength::_20mA));
    mdio.apply_input_config(&InputConfig::default());
    crate::gpio::InputSignal::EMAC_MDI.connect_to(&mdio);
    crate::gpio::OutputSignal::EMAC_MDO.connect_to(&mdio);
    mdio.set_input_enable(true);
}

impl<'d, Dm: DriverMode, P: Phy> Ethernet<'d, Dm, P> {
    /// Configures the MAC for the given speed.
    pub fn set_speed(&mut self, speed: Speed) {
        if speed != self.speed {
            EmacRegs.set_speed(speed);
            self.speed = speed;
        }
    }

    /// Configures the MAC for the given duplex mode.
    pub fn set_duplex(&mut self, duplex: Duplex) {
        if duplex != self.duplex {
            EmacRegs.set_duplex(duplex);
            self.duplex = duplex;
        }
    }

    /// Copies `frame` into the next TX slot and initiates transmission.
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.tx.transmit(frame).map_err(|e| match e {
            dma::TxError::RingFull => Error::TxFull,
            dma::TxError::FrameTooLarge => Error::FrameTooLarge,
        })?;
        EmacRegs.demand_tx_poll();
        Ok(())
    }

    /// Returns the current PHY link state.
    ///
    /// Async callers are encouraged to pass the context to allow the PHY driver to wake them when
    /// the link state needs to be re-polled.
    pub fn poll_link(&mut self, cx: Option<&mut Context<'_>>) -> LinkState {
        let mut mdio = MdioDriver::new(&EmacRegs);
        self.phy.poll_link(&mut mdio, cx)
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
                    EmacRegs.demand_tx_poll();
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
                    EmacRegs.demand_rx_poll();
                    return Err(Error::FrameTooLarge);
                }
                buf[..len].copy_from_slice(data);
                self.rx.pop();
                EmacRegs.demand_rx_poll();
                return Ok(len);
            }

            // receive() may have recycled error frames back to DMA ownership
            // without a poll-demand write. Poke the RX DMA so it resumes.
            EmacRegs.demand_rx_poll();
        }
    }

    /// Converts back to blocking mode, disabling all DMA interrupts.
    pub fn into_blocking(self) -> Ethernet<'d, Blocking, P> {
        EmacRegs.dma_disable_interrupts();
        Ethernet {
            tx: self.tx,
            rx: self.rx,
            phy: self.phy,
            mac_addr: self.mac_addr,
            speed: self.speed,
            duplex: self.duplex,
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
    /// Returns `Some(data_slice)` if a valid frame is ready, or `None`.
    /// The caller must call [`release_rx`][Self::release_rx] after processing.
    pub fn peek_rx(&mut self) -> Option<&mut [u8]> {
        self.rx.receive()
    }

    /// Releases the current RX descriptor back to the DMA engine.
    pub fn release_rx(&mut self) {
        self.rx.pop();
        EmacRegs.demand_rx_poll();
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
    // DMA soft-reset (also enables enhanced 32B descriptor format).
    EmacRegs.dma_soft_reset();

    // Init PHY (MDIO requires DMA/MAC clocks to be running after reset).
    let mut mdio = MdioDriver::new(&EmacRegs);
    phy.init(&mut mdio).map_err(Error::Phy)?;

    // Build descriptor rings from the erased slice references.
    let tx = TDesRing::new(&mut storage.tx_descs, &mut storage.tx_bufs);
    let rx = RDesRing::new(&mut storage.rx_descs, &mut storage.rx_bufs);

    // Program descriptor list addresses into DMA.
    EmacRegs.set_descriptor_lists(tx.base_ptr() as u32, rx.base_ptr() as u32);

    let eth = Ethernet {
        tx,
        rx,
        phy,
        mac_addr,
        speed: Speed::_100M,
        duplex: Duplex::Full,
        _clock_guard: clock_guard,
        _eth: eth,
        _mode: PhantomData,
    };

    // Configure MAC defaults.
    EmacRegs.mac_init(eth.speed, eth.duplex);
    EmacRegs.set_mac_address(&eth.mac_addr);

    // In blocking mode all DMA interrupts stay disabled; async mode enables
    // them via into_async().
    EmacRegs.dma_disable_interrupts();

    // Start both DMA engines.
    EmacRegs.dma_start();

    Ok(eth)
}
