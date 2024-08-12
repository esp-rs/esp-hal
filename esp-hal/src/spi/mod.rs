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

#![allow(missing_docs)] // TODO: Remove when able

use crate::dma::DmaError;

pub mod master;
#[cfg(not(esp32))]
pub mod slave;

/// SPI errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    DmaError(DmaError),
    MaxDmaTransferSizeExceeded,
    FifoSizeExeeded,
    Unsupported,
    Unknown,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

/// SPI modes
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

/// SPI Bit Order
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiBitOrder {
    MSBFirst,
    LSBFirst,
}

pub trait DuplexMode {}
pub trait IsFullDuplex: DuplexMode {}
pub trait IsHalfDuplex: DuplexMode {}

/// SPI data mode
///
/// Single = 1 bit, 2 wires
/// Dual = 2 bit, 2 wires
/// Quad = 4 bit, 4 wires
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiDataMode {
    Single,
    Dual,
    Quad,
}

/// Full-duplex operation
pub struct FullDuplexMode {}
impl DuplexMode for FullDuplexMode {}
impl IsFullDuplex for FullDuplexMode {}

/// Half-duplex operation
pub struct HalfDuplexMode {}
impl DuplexMode for HalfDuplexMode {}
impl IsHalfDuplex for HalfDuplexMode {}
