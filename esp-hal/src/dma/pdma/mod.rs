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

use portable_atomic::AtomicBool;

use crate::{
    DriverMode,
    asynch::AtomicWaker,
    dma::{Channel, DmaChannel, DmaEligible, DmaPeripheral, InterruptHandler, RegisterAccess},
    peripherals::Interrupt,
};

#[cfg(soc_has_dma_copy)]
mod copy;
#[cfg(soc_has_dma_crypto)]
mod crypto;
mod i2s;
mod spi;

#[cfg(soc_has_dma_copy)]
pub use copy::{CopyDmaRxChannel, CopyDmaTxChannel};
#[cfg(soc_has_dma_crypto)]
pub use crypto::{CryptoDmaRxChannel, CryptoDmaTxChannel};
pub use i2s::{AnyI2sDmaChannel, AnyI2sDmaRxChannel, AnyI2sDmaTxChannel};
pub use spi::{AnySpiDmaChannel, AnySpiDmaRxChannel, AnySpiDmaTxChannel};

/// Immutable per-channel metadata.
#[doc(hidden)]
pub struct ChannelInfo {
    pub(crate) peripheral_interrupt: Interrupt,
    pub(crate) async_handler: InterruptHandler,
    pub(crate) compatible_peripherals: &'static [DmaPeripheral],
}

impl ChannelInfo {
    pub(crate) fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.compatible_peripherals.contains(&peripheral)
    }
}

/// Mutable per-channel runtime state.
#[doc(hidden)]
pub struct ChannelState {
    pub(crate) tx_waker: AtomicWaker,
    pub(crate) rx_waker: AtomicWaker,
    pub(crate) tx_async_flag: AtomicBool,
    pub(crate) rx_async_flag: AtomicBool,
}

macro_rules! impl_pdma_channel {
    ($peri:ident, $instance:ident, $int:ident, [$($compatible:ident),*]) => {
        paste::paste! {
            use $crate::peripherals::$instance;
            impl $instance<'_> {
                pub(super) fn info(&self) -> &'static ChannelInfo {
                    #[crate::handler(priority = crate::interrupt::Priority::max())]
                    fn interrupt_handler() {
                        crate::dma::asynch::handle_in_interrupt::<$instance<'static>>();
                        crate::dma::asynch::handle_out_interrupt::<$instance<'static>>();
                    }
                    static INFO: ChannelInfo = ChannelInfo {
                        peripheral_interrupt: crate::peripherals::Interrupt::$int,
                        async_handler: interrupt_handler,
                        compatible_peripherals: &[$(crate::dma::DmaPeripheral::$compatible),*],
                    };
                    &INFO
                }

                pub(super) fn state(&self) -> &'static ChannelState {
                    static STATE: ChannelState = ChannelState {
                        tx_waker: crate::asynch::AtomicWaker::new(),
                        rx_waker: crate::asynch::AtomicWaker::new(),
                        tx_async_flag: portable_atomic::AtomicBool::new(false),
                        rx_async_flag: portable_atomic::AtomicBool::new(false),
                    };
                    &STATE
                }
            }

            impl<'d> DmaChannelConvert<[<$peri Channel>]<'d>> for $instance<'d> {
                fn degrade(self) -> [<$peri Channel>]<'d> {
                    self.into()
                }
            }
        }

        crate::dma::impl_channel_common!($peri, $instance);
    };
}
pub(crate) use impl_pdma_channel;

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

impl<CH, Dm> Channel<Dm, CH>
where
    CH: DmaChannel,
    Dm: DriverMode,
{
    /// Asserts that the channel is compatible with the given peripheral.
    #[instability::unstable]
    pub fn runtime_ensure_compatible(&self, peripheral: &impl DmaEligible) {
        assert!(
            self.tx
                .tx_impl
                .is_compatible_with(peripheral.dma_peripheral()),
            "This DMA channel is not compatible with {:?}",
            peripheral.dma_peripheral()
        );
    }
}
