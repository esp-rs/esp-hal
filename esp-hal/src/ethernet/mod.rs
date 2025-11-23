//! Ethernet MAC (EMAC) peripheral driver for ESP32 and ESP32-P4
//!
//! This module provides support for the EMAC peripheral with RMII interface.
//! It supports both ESP32 and ESP32-P4 chips which have internal EMAC controllers.
//!
//! # Example
//!
//! ```no_run
//! let peripherals = esp_hal::init(Config::default());
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! let ethernet = Ethernet::new(
//!     peripherals.EMAC,
//!     RmiiPins {
//!         ref_clk: io.pins.gpio0,
//!         tx_en: io.pins.gpio21,
//!         txd0: io.pins.gpio19,
//!         txd1: io.pins.gpio22,
//!         rxd0: io.pins.gpio25,
//!         rxd1: io.pins.gpio26,
//!         crs_dv: io.pins.gpio27,
//!         mdc: io.pins.gpio23,
//!         mdio: io.pins.gpio18,
//!     },
//!     [0x02, 0x00, 0x00, 0x12, 0x34, 0x56], // MAC address
//! );
//! ```

use core::marker::PhantomData;

use crate::{
    gpio::{InputPin, OutputPin, Pin},
    peripheral::{Peripheral, PeripheralRef},
    Blocking,
};

/// RMII pin configuration
pub struct RmiiPins<'d, REF_CLK, TX_EN, TXD0, TXD1, RXD0, RXD1, CRS_DV, MDC, MDIO>
where
    REF_CLK: Pin,
    TX_EN: Pin,
    TXD0: Pin,
    TXD1: Pin,
    RXD0: Pin,
    RXD1: Pin,
    CRS_DV: Pin,
    MDC: Pin,
    MDIO: Pin,
{
    pub ref_clk: PeripheralRef<'d, REF_CLK>,
    pub tx_en: PeripheralRef<'d, TX_EN>,
    pub txd0: PeripheralRef<'d, TXD0>,
    pub txd1: PeripheralRef<'d, TXD1>,
    pub rxd0: PeripheralRef<'d, RXD0>,
    pub rxd1: PeripheralRef<'d, RXD1>,
    pub crs_dv: PeripheralRef<'d, CRS_DV>,
    pub mdc: PeripheralRef<'d, MDC>,
    pub mdio: PeripheralRef<'d, MDIO>,
}

/// Ethernet MAC address
pub type MacAddress = [u8; 6];

/// PHY address on MDIO bus (0-31)
#[derive(Debug, Clone, Copy)]
pub struct PhyAddress(u8);

impl PhyAddress {
    pub const fn new(addr: u8) -> Option<Self> {
        if addr < 32 {
            Some(PhyAddress(addr))
        } else {
            None
        }
    }

    pub const fn addr(&self) -> u8 {
        self.0
    }
}

/// Ethernet driver errors
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// Invalid configuration
    InvalidConfig,
    /// PHY initialization failed
    PhyInitFailed,
    /// Transmission timeout
    TransmitTimeout,
    /// Reception timeout  
    ReceiveTimeout,
    /// Buffer too small
    BufferTooSmall,
    /// No frame available
    NoFrame,
}

/// Ethernet MAC peripheral driver
pub struct Ethernet<'d, DM: crate::DriverMode> {
    _peripheral: PeripheralRef<'d, crate::peripherals::EMAC>,
    mac_addr: MacAddress,
    _mode: PhantomData<DM>,
}

impl<'d> Ethernet<'d, Blocking> {
    /// Create a new Ethernet driver instance
    pub fn new<REF_CLK, TX_EN, TXD0, TXD1, RXD0, RXD1, CRS_DV, MDC, MDIO>(
        peripheral: impl Peripheral<P = crate::peripherals::EMAC> + 'd,
        pins: RmiiPins<'d, REF_CLK, TX_EN, TXD0, TXD1, RXD0, RXD1, CRS_DV, MDC, MDIO>,
        mac_addr: MacAddress,
    ) -> Self
    where
        REF_CLK: OutputPin,
        TX_EN: OutputPin,
        TXD0: OutputPin,
        TXD1: OutputPin,
        RXD0: InputPin,
        RXD1: InputPin,
        CRS_DV: InputPin,
        MDC: OutputPin,
        MDIO: InputPin + OutputPin,
    {
        crate::into_ref!(peripheral);

        // TODO: Configure RMII pins via IO_MUX
        // TODO: Enable EMAC clock
        // TODO: Initialize DMA descriptors
        // TODO: Configure MAC registers
        // TODO: Set MAC address

        Self {
            _peripheral: peripheral,
            mac_addr,
            _mode: PhantomData,
        }
    }

    /// Initialize PHY chip
    pub fn init_phy(&mut self, phy_addr: PhyAddress) -> Result<(), Error> {
        // TODO: Implement PHY initialization via MDIO
        // - Reset PHY
        // - Configure auto-negotiation
        // - Wait for link up
        Ok(())
    }

    /// Send an Ethernet frame
    pub fn send_frame(&mut self, data: &[u8]) -> Result<(), Error> {
        // TODO: Implement frame transmission
        // - Setup DMA descriptor
        // - Copy data to TX buffer
        // - Trigger transmission
        // - Wait for completion
        Ok(())
    }

    /// Receive an Ethernet frame
    pub fn receive_frame(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        // TODO: Implement frame reception
        // - Check for available frame
        // - Copy from RX buffer
        // - Return frame length
        Ok(0)
    }

    /// Get MAC address
    pub fn mac_address(&self) -> &MacAddress {
        &self.mac_addr
    }

    /// Check if link is up
    pub fn is_link_up(&self) -> bool {
        // TODO: Read PHY status via MDIO
        false
    }
}

// TODO: Implement async support
// impl<'d> Ethernet<'d, Async> { ... }
