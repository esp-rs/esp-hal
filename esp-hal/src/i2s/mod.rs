//! # Inter-IC Sound (I2S)

pub mod master;

#[cfg(esp32)]
pub mod parallel;

crate::any_peripheral! {
    /// Any I2S peripheral.
    pub peripheral AnyI2s<'d> {
        #[cfg(soc_has_i2s0)]
        I2s0(crate::peripherals::I2S0<'d>),
        #[cfg(soc_has_i2s1)]
        I2s1(crate::peripherals::I2S1<'d>),
    }
}

impl AnyI2s<'_> {
    pub(crate) fn dma_peripheral_num(&self) -> u8 {
        match &self.0 {
            #[cfg(soc_has_i2s0)]
            any::Inner::I2s0(_) => crate::dma::DmaPeripheral::I2s0.0,
            #[cfg(soc_has_i2s1)]
            any::Inner::I2s1(_) => crate::dma::DmaPeripheral::I2s1.0,
        }
    }
}
