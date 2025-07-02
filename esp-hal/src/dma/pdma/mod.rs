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

use critical_section::CriticalSection;
use portable_atomic::AtomicBool;

use crate::{
    DriverMode,
    asynch::AtomicWaker,
    dma::{
        Channel,
        DmaChannel,
        DmaChannelConvert,
        DmaChannelExt,
        DmaEligible,
        DmaPeripheral,
        DmaRxInterrupt,
        DmaTxInterrupt,
        InterruptAccess,
        InterruptHandler,
        RegisterAccess,
    },
    handler,
    interrupt::Priority,
    peripherals::Interrupt,
};

#[cfg(soc_has_copy_dma)]
mod copy;
#[cfg(soc_has_crypto_dma)]
mod crypto;
mod i2s;
mod spi;

#[cfg(soc_has_copy_dma)]
pub use copy::{CopyDmaRxChannel, CopyDmaTxChannel};
#[cfg(soc_has_crypto_dma)]
pub use crypto::{CryptoDmaRxChannel, CryptoDmaTxChannel};
use i2s::I2sRegisterBlock;
pub use i2s::{AnyI2sDmaChannel, AnyI2sDmaRxChannel, AnyI2sDmaTxChannel};
use spi::SpiRegisterBlock;
pub use spi::{AnySpiDmaChannel, AnySpiDmaRxChannel, AnySpiDmaTxChannel};

#[doc(hidden)]
pub trait PdmaChannel: crate::private::Sealed {
    type RegisterBlock;

    fn register_block(&self) -> &Self::RegisterBlock;
    fn tx_waker(&self) -> &'static AtomicWaker;
    fn rx_waker(&self) -> &'static AtomicWaker;
    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;

    fn peripheral_interrupt(&self) -> Interrupt;
    fn async_handler(&self) -> InterruptHandler;
    fn rx_async_flag(&self) -> &'static AtomicBool;
    fn tx_async_flag(&self) -> &'static AtomicBool;
}

macro_rules! impl_pdma_channel {
    ($peri:ident, $register_block:ident, $instance:ident, $int:ident, [$($compatible:ident),*]) => {
        paste::paste! {
            use $crate::peripherals::[< $instance >];
            impl<'d> DmaChannel for $instance<'d> {
                type Rx = [<$peri DmaRxChannel>]<'d>;
                type Tx = [<$peri DmaTxChannel>]<'d>;

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) { unsafe {
                    (
                        [<$peri DmaRxChannel>](Self::steal().into()),
                        [<$peri DmaTxChannel>](Self::steal().into()),
                    )
                }}
            }

            impl DmaChannelExt for $instance<'_> {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    [<$peri DmaRxChannel>](unsafe { Self::steal() }.into())
                }
                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    [<$peri DmaTxChannel>](unsafe { Self::steal() }.into())
                }
            }

            impl PdmaChannel for $instance<'_> {
                type RegisterBlock = $register_block;

                fn register_block(&self) -> &Self::RegisterBlock {
                    $crate::peripherals::[< $instance >]::regs()
                }
                fn tx_waker(&self) -> &'static AtomicWaker {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                fn rx_waker(&self) -> &'static AtomicWaker {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
                    let compatible_peripherals = [$(DmaPeripheral::$compatible),*];
                    compatible_peripherals.contains(&peripheral)
                }

                fn peripheral_interrupt(&self) -> Interrupt {
                    Interrupt::$int
                }

                fn async_handler(&self) -> InterruptHandler {
                    #[handler(priority = Priority::max())]
                    pub(crate) fn interrupt_handler() {
                        super::asynch::handle_in_interrupt::<$instance<'static>>();
                        super::asynch::handle_out_interrupt::<$instance<'static>>();
                    }

                    interrupt_handler
                }
                fn rx_async_flag(&self) -> &'static AtomicBool {
                    static FLAG: AtomicBool = AtomicBool::new(false);
                    &FLAG
                }
                fn tx_async_flag(&self) -> &'static AtomicBool {
                    static FLAG: AtomicBool = AtomicBool::new(false);
                    &FLAG
                }
            }

            impl<'d> DmaChannelConvert<[<$peri DmaChannel>]<'d>> for $instance<'d> {
                fn degrade(self) -> [<$peri DmaChannel>]<'d> {
                    self.into()
                }
            }

            impl<'d> DmaChannelConvert<[<$peri DmaRxChannel>]<'d>> for $instance<'d> {
                fn degrade(self) -> [<$peri DmaRxChannel>]<'d> {
                    [<$peri DmaRxChannel>](self.into())
                }
            }

            impl<'d> DmaChannelConvert<[<$peri DmaTxChannel>]<'d>> for $instance<'d> {
                fn degrade(self) -> [<$peri DmaTxChannel>]<'d> {
                    [<$peri DmaTxChannel>](self.into())
                }
            }
        }
    };
}

impl_pdma_channel!(AnySpi, SpiRegisterBlock, DMA_SPI2, SPI2_DMA, [Spi2]);
impl_pdma_channel!(AnySpi, SpiRegisterBlock, DMA_SPI3, SPI3_DMA, [Spi3]);

#[cfg(soc_has_i2s0)]
impl_pdma_channel!(AnyI2s, I2sRegisterBlock, DMA_I2S0, I2S0, [I2s0]);
#[cfg(soc_has_i2s1)]
impl_pdma_channel!(AnyI2s, I2sRegisterBlock, DMA_I2S1, I2S1, [I2s1]);

// Specific peripherals use specific channels. Note that this may be overly
// restrictive (ESP32 allows configuring 2 SPI DMA channels between 3 different
// peripherals), but for the current set of restrictions this is sufficient.
#[cfg(soc_has_spi2)]
crate::dma::impl_dma_eligible!([DMA_SPI2] SPI2 => Spi2);
#[cfg(soc_has_spi3)]
crate::dma::impl_dma_eligible!([DMA_SPI3] SPI3 => Spi3);
#[cfg(soc_has_i2s0)]
crate::dma::impl_dma_eligible!([DMA_I2S0] I2S0 => I2s0);
#[cfg(soc_has_i2s1)]
crate::dma::impl_dma_eligible!([DMA_I2S1] I2S1 => I2s1);
#[cfg(esp32s2)]
use crate::peripherals::DMA_CRYPTO;
#[cfg(esp32s2)]
crate::dma::impl_dma_eligible!([DMA_CRYPTO] AES => Aes);
#[cfg(esp32s2)]
crate::dma::impl_dma_eligible!([DMA_CRYPTO] SHA => Sha);

pub(super) fn init_dma(_cs: CriticalSection<'_>) {
    #[cfg(esp32)]
    {
        // (only) on ESP32 we need to configure DPORT for the SPI DMA channels
        // This assignes the DMA channels to the SPI peripherals, which is more
        // restrictive than necessary but we currently support the same
        // number of SPI peripherals as SPI DMA channels so it's not a big
        // deal.
        use crate::peripherals::DPORT;

        DPORT::regs()
            .spi_dma_chan_sel()
            .modify(|_, w| unsafe { w.spi2_dma_chan_sel().bits(1).spi3_dma_chan_sel().bits(2) });
    }

    #[cfg(esp32s2)]
    {
        // This is the only DMA channel on the S2 that needs to be enabled this way
        // (using its own registers). Ideally this should be enabled only when
        // the DMA channel is in use but we don't have a good mechanism for that
        // yet. For now, we shall just turn in on forever once any DMA channel is used.

        use crate::peripherals::DMA_COPY;

        DMA_COPY::regs().conf().modify(|_, w| w.clk_en().set_bit());
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
