//! # Inter-IC Sound (I2S)

use crate::dma::DmaEligible;

pub mod master;

#[cfg(esp32)]
pub mod parallel;

crate::any_peripheral! {
    /// Any I2S peripheral.
    pub peripheral AnyI2s {
        #[cfg(i2s0)]
        I2s0(crate::peripherals::I2S0),
        #[cfg(i2s1)]
        I2s1(crate::peripherals::I2S1),
    }
}

impl DmaEligible for AnyI2s {
    #[cfg(gdma)]
    type Dma = crate::dma::AnyGdmaChannel;
    #[cfg(pdma)]
    type Dma = crate::dma::AnyI2sDmaChannel;

    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        match &self.0 {
            AnyI2sInner::I2s0(_) => crate::dma::DmaPeripheral::I2s0,
            #[cfg(i2s1)]
            AnyI2sInner::I2s1(_) => crate::dma::DmaPeripheral::I2s1,
        }
    }
}
