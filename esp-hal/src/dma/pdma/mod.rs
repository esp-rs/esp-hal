//! # Direct Memory Access
//!
//! ## Overview
//! The `pdma` module is part of the DMA driver of `ESP32` and `ESP32-S2`.
//!
//! This module provides efficient direct data transfer capabilities between
//! peripherals and memory without involving the CPU. It enables bidirectional
//! data transfers through DMA channels, making it particularly useful for
//! high-speed data transfers, such as [SPI] and [I2S] communication.
//!
//! [SPI]: ../spi/index.html
//! [I2S]: ../i2s/index.html

for_each_dma_engine! {
    ("COPY_DMA") => {
        mod copy;
        pub use copy::{CopyDmaRxChannel, CopyDmaTxChannel};
    };
    ("CRYPTO_DMA") => {
        mod crypto;
        pub use crypto::{CryptoDmaRxChannel, CryptoDmaTxChannel};
    };
    ("I2S_DMA") => {
        mod i2s;
        pub use i2s::{AnyI2sDmaChannel, AnyI2sDmaRxChannel, AnyI2sDmaTxChannel};
    };
    ("SPI_DMA") => {
        mod spi;
        pub use spi::{AnySpiDmaChannel, AnySpiDmaRxChannel, AnySpiDmaTxChannel};
    };
}

pub(super) fn init_dma_racey() {
    #[cfg(esp32)]
    {
        // (only) on ESP32 we need to configure DPORT for the SPI DMA channels
        // This assigns the DMA channels to the SPI peripherals, which is more
        // restrictive than necessary but we currently support the same
        // number of SPI peripherals as SPI DMA channels so it's not a big
        // deal.
        use crate::peripherals::DPORT;

        DPORT::regs().spi_dma_chan_sel().modify(|_, w| unsafe {
            w.spi2_dma_chan_sel().bits(1);
            w.spi3_dma_chan_sel().bits(2)
        });
    }
}
