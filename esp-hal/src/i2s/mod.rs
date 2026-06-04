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

with_i2s_dma_engine! {
    ($engine:tt, $any_ch:ident) => {
        use crate::dma::DmaEligiblePeripheral;

        impl<'d> DmaEligiblePeripheral<crate::dma::$any_ch<'d>> for AnyI2s<'d> {
            fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
                any::delegate!(self, i2s => { i2s.dma_peripheral() })
            }
        }
    };
}
