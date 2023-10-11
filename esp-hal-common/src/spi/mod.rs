use crate::dma::DmaError;

pub mod master;
#[cfg(all(any(spi0, spi1, spi2, spi3), not(pdma)))]
pub mod slave;

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

#[cfg(feature = "eh1")]
impl embedded_hal_1::spi::Error for Error {
    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
        embedded_hal_1::spi::ErrorKind::Other
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
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

pub struct FullDuplexMode {}
impl DuplexMode for FullDuplexMode {}
impl IsFullDuplex for FullDuplexMode {}

pub struct HalfDuplexMode {}
impl DuplexMode for HalfDuplexMode {}
impl IsHalfDuplex for HalfDuplexMode {}
