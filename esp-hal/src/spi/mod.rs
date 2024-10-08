//! Serial Peripheral Interface (SPI)
//!
//! ## Overview
//! The Serial Peripheral Interface (SPI) is a synchronous serial interface
//! useful for communication with external peripherals.
//!
//! ## Configuration
//! This peripheral is capable of operating in either master or slave mode. For
//! more information on these modes, please refer to the documentation in their
//! respective modules.

use crate::dma::{DmaError, PeripheralMarker};

pub mod master;
pub mod slave;

/// SPI errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Error occurred due to a DMA-related issue.
    DmaError(DmaError),
    /// Error indicating that the maximum DMA transfer size was exceeded.
    MaxDmaTransferSizeExceeded,
    /// Error indicating that the FIFO size was exceeded during SPI
    /// communication.
    FifoSizeExeeded,
    /// Error indicating that the operation is unsupported by the current
    /// implementation.
    Unsupported,
    /// An unknown error occurred during SPI communication.
    Unknown,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

/// SPI communication modes, defined by clock polarity (CPOL) and clock phase
/// (CPHA).
///
/// These modes control the clock signal's idle state and when data is sampled
/// and shifted.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    /// Mode 0 (CPOL = 0, CPHA = 0): Clock is low when idle, data is captured on
    /// the rising edge and propagated on the falling edge.
    Mode0,
    /// Mode 1 (CPOL = 0, CPHA = 1): Clock is low when idle, data is captured on
    /// the falling edge and propagated on the rising edge.
    Mode1,
    /// Mode 2 (CPOL = 1, CPHA = 0): Clock is high when idle, data is captured
    /// on the falling edge and propagated on the rising edge.
    Mode2,
    /// Mode 3 (CPOL = 1, CPHA = 1): Clock is high when idle, data is captured
    /// on the rising edge and propagated on the falling edge.
    Mode3,
}

/// SPI Bit Order
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiBitOrder {
    /// Most Significant Bit (MSB) is transmitted first.
    MSBFirst,
    /// Least Significant Bit (LSB) is transmitted first.
    LSBFirst,
}

/// Trait marker for defining SPI duplex modes.
pub trait DuplexMode: crate::private::Sealed {}

/// SPI data mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiDataMode {
    /// `Single` Data Mode - 1 bit, 2 wires.
    Single,
    /// `Dual` Data Mode - 2 bit, 2 wires
    Dual,
    /// `Quad` Data Mode - 4 bit, 4 wires
    Quad,
}

/// Full-duplex operation
pub struct FullDuplexMode {}
impl DuplexMode for FullDuplexMode {}
impl crate::private::Sealed for FullDuplexMode {}

/// Half-duplex operation
pub struct HalfDuplexMode {}
impl DuplexMode for HalfDuplexMode {}
impl crate::private::Sealed for HalfDuplexMode {}

#[cfg(spi2)]
impl PeripheralMarker for crate::peripherals::SPI2 {
    #[inline(always)]
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Spi2
    }
}

#[cfg(spi3)]
impl PeripheralMarker for crate::peripherals::SPI3 {
    #[inline(always)]
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Spi3
    }
}
