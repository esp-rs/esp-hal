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
    asynch::AtomicWaker,
    dma::*,
    handler,
    interrupt::Priority,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
};

#[cfg(esp32s2)]
mod crypto;
mod i2s;
mod spi;

#[cfg(esp32s2)]
pub use crypto::*;
pub use i2s::*;
pub use spi::*;

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
            #[doc = concat!("DMA channel suitable for ", stringify!([< $instance:upper >]))]
            #[non_exhaustive]
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct [<$instance DmaChannel>] {}

            impl $crate::private::Sealed for [<$instance DmaChannel>] {}

            unsafe impl Peripheral for [<$instance DmaChannel>] {
                type P = Self;

                unsafe fn clone_unchecked(&self) -> Self::P {
                    Self::steal()
                }
            }

            impl [<$instance DmaChannel>] {
                /// Unsafely constructs a new DMA channel.
                ///
                /// # Safety
                ///
                /// The caller must ensure that only a single instance is used.
                pub unsafe fn steal() -> Self {
                    Self {}
                }
            }

            impl DmaChannel for [<$instance DmaChannel>] {
                type Rx = [<$peri DmaRxChannel>];
                type Tx = [<$peri DmaTxChannel>];

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) {
                    ([<$peri DmaRxChannel>](Self {}.into()), [<$peri DmaTxChannel>](Self {}.into()))
                }
            }

            impl DmaChannelExt for [<$instance DmaChannel>] {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    [<$peri DmaRxChannel>](Self {}.into())
                }
                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    [<$peri DmaTxChannel>](Self {}.into())
                }
            }

            impl PdmaChannel for [<$instance DmaChannel>] {
                type RegisterBlock = $register_block;

                fn register_block(&self) -> &Self::RegisterBlock {
                    crate::peripherals::[< $instance:upper >]::regs()
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
                        super::asynch::handle_in_interrupt::<[< $instance DmaChannel >]>();
                        super::asynch::handle_out_interrupt::<[< $instance DmaChannel >]>();
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

            impl DmaChannelConvert<[<$peri DmaChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaChannel>] {
                    self.into()
                }
            }

            impl DmaChannelConvert<[<$peri DmaRxChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaRxChannel>] {
                    [<$peri DmaRxChannel>](self.into())
                }
            }

            impl DmaChannelConvert<[<$peri DmaTxChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaTxChannel>] {
                    [<$peri DmaTxChannel>](self.into())
                }
            }
        }
    };
}

impl_pdma_channel!(AnySpi, SpiRegisterBlock, Spi2, SPI2_DMA, [Spi2]);
impl_pdma_channel!(AnySpi, SpiRegisterBlock, Spi3, SPI3_DMA, [Spi3]);

impl_pdma_channel!(AnyI2s, I2sRegisterBlock, I2s0, I2S0, [I2s0]);
#[cfg(i2s1)]
impl_pdma_channel!(AnyI2s, I2sRegisterBlock, I2s1, I2S1, [I2s1]);

// Specific peripherals use specific channels. Note that this may be overly
// restrictive (ESP32 allows configuring 2 SPI DMA channels between 3 different
// peripherals), but for the current set of restrictions this is sufficient.
crate::dma::impl_dma_eligible!([Spi2DmaChannel] SPI2 => Spi2);
crate::dma::impl_dma_eligible!([Spi3DmaChannel] SPI3 => Spi3);
crate::dma::impl_dma_eligible!([I2s0DmaChannel] I2S0 => I2s0);
#[cfg(i2s1)]
crate::dma::impl_dma_eligible!([I2s1DmaChannel] I2S1 => I2s1);
#[cfg(esp32s2)]
crate::dma::impl_dma_eligible!([CryptoDmaChannel] AES => Aes);
#[cfg(esp32s2)]
crate::dma::impl_dma_eligible!([CryptoDmaChannel] SHA => Sha);

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
}

impl<CH, Dm> Channel<'_, Dm, CH>
where
    CH: DmaChannel,
    Dm: DriverMode,
{
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible(&self, peripheral: &PeripheralRef<'_, impl DmaEligible>) {
        assert!(
            self.tx
                .tx_impl
                .is_compatible_with(peripheral.dma_peripheral()),
            "This DMA channel is not compatible with {:?}",
            peripheral.dma_peripheral()
        );
    }
}
