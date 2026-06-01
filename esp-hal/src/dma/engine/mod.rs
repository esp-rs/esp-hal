use enumset::{EnumSet, EnumSetType};

#[cfg(dma_kind = "gdma")]
use crate::dma::DmaPriority;
use crate::{
    asynch::AtomicWaker,
    dma::{BurstConfig, DmaRxInterrupt, DmaTxInterrupt},
    interrupt::InterruptHandler,
    peripherals::Interrupt,
    private::{Internal, Sealed},
    system::PeripheralGuard,
};

for_each_dma_engine! {
    ("AHB_GDMA") => {
        mod gdma;
        pub use gdma::*;
    };
    ("COPY_DMA") => {
        mod copy;
        pub use copy::{CopyDmaChannel, CopyDmaRxChannel, CopyDmaTxChannel};
    };
    ("CRYPTO_DMA") => {
        mod crypto;
        pub use crypto::{CryptoDmaChannel, CryptoDmaRxChannel, CryptoDmaTxChannel};
    };
    ("I2S_DMA") => {
        mod i2s;
        pub use i2s::{I2sDmaChannel, I2sDmaRxChannel, I2sDmaTxChannel};
    };
    ("SPI_DMA") => {
        mod spi;
        pub use spi::{SpiDmaChannel, SpiDmaRxChannel, SpiDmaTxChannel};
    };
}

/// Implemented by peripheral singletons that can be used with a DMA engine.
pub trait DmaEligiblePeripheral {
    /// The erased DMA channel type for the engine this peripheral belongs to.
    type ErasedChannel<'a>: DmaChannel;

    /// Returns the `DmaPeripheral` ID for runtime compatibility checks.
    fn dma_peripheral(&self) -> DmaPeripheral;
}

for_each_peripheral! {
    (dma_eligible $(( $peri:ident, $name:ident, $id:literal, $any_ch:ident )),*) => {
        /// DMA-eligible peripheral selector values; values are engine-local (matching hardware where applicable).
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub struct DmaPeripheral(pub u8);
        impl DmaPeripheral {
            $(
                #[doc = concat!("DMA accesses ", stringify!($name))]
                pub const $peri: Self = Self($id);
            )*
        }

        $(
            impl DmaEligiblePeripheral for crate::peripherals::$peri<'_> {
                type ErasedChannel<'a> = $any_ch<'a>;

                fn dma_peripheral(&self) -> DmaPeripheral {
                    DmaPeripheral::$peri
                }
            }
        )*
    };
}

#[doc(hidden)]
pub trait RegisterAccess: Sealed {
    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard>;

    /// Reset the state machine of the channel and FIFO pointer.
    fn reset(&self);

    /// Enable/Disable INCR burst transfer for channel reading
    /// accessing data in internal RAM.
    fn set_burst_mode(&self, burst_mode: BurstConfig);

    /// Enable/Disable burst transfer for channel reading
    /// descriptors in internal RAM.
    fn set_descr_burst_mode(&self, burst_mode: bool);

    /// The priority of the channel. The larger the value, the higher the
    /// priority.
    #[cfg(dma_kind = "gdma")]
    fn set_priority(&self, priority: DmaPriority);

    /// Select a peripheral for the channel.
    fn set_peripheral(&self, _peripheral: u8) {}

    /// Set the address of the first descriptor.
    fn set_link_addr(&self, address: u32);

    /// Enable the channel for data transfer.
    fn start(&self);

    /// Stop the channel from transferring data.
    fn stop(&self);

    /// Mount a new descriptor.
    fn restart(&self);

    /// Configure the bit to enable checking the owner attribute of the
    /// descriptor.
    fn set_check_owner(&self, check_owner: Option<bool>);

    #[cfg(dma_ext_mem_configurable_block_size)]
    fn set_ext_mem_block_size(&self, size: crate::dma::DmaExtMemBKSize);

    #[cfg(dma_can_access_psram)]
    fn can_access_psram(&self) -> bool;

    fn compatible_peripherals(&self) -> &[u8];

    fn runtime_ensure_compatible(&self, peripheral: DmaPeripheral) {
        let peripherals = self.compatible_peripherals();
        assert!(
            peripherals.contains(&peripheral.0),
            "This DMA channel is not compatible with peripheral id {}",
            peripheral.0
        );
    }
}

#[doc(hidden)]
pub trait RxRegisterAccess: RegisterAccess {
    #[cfg(dma_kind = "gdma")]
    fn set_mem2mem_mode(&self, value: bool);

    fn peripheral_interrupt(&self) -> Option<Interrupt>;
    fn async_handler(&self) -> Option<InterruptHandler>;
}

#[doc(hidden)]
pub trait TxRegisterAccess: RegisterAccess {
    /// Returns whether the DMA's FIFO is empty.
    fn is_fifo_empty(&self) -> bool;

    /// Enable/disable outlink-writeback
    fn set_auto_write_back(&self, enable: bool);

    /// Outlink descriptor address when EOF occurs of Tx channel.
    fn last_dscr_address(&self) -> usize;

    fn peripheral_interrupt(&self) -> Option<Interrupt>;
    fn async_handler(&self) -> Option<InterruptHandler>;
}

#[doc(hidden)]
pub trait InterruptAccess<T: EnumSetType>: Sealed {
    fn listen(&self, interrupts: impl Into<EnumSet<T>>) {
        self.enable_listen(interrupts.into(), true)
    }
    fn unlisten(&self, interrupts: impl Into<EnumSet<T>>) {
        self.enable_listen(interrupts.into(), false)
    }

    fn clear_all(&self) {
        self.clear(EnumSet::all());
    }

    fn enable_listen(&self, interrupts: EnumSet<T>, enable: bool);
    fn is_listening(&self) -> EnumSet<T>;
    fn clear(&self, interrupts: impl Into<EnumSet<T>>);
    fn pending_interrupts(&self) -> EnumSet<T>;
    fn waker(&self) -> &'static AtomicWaker;

    fn is_async(&self) -> bool;
    fn set_async(&self, is_async: bool);
}

#[instability::unstable]
pub trait DmaRxChannel: RxRegisterAccess + InterruptAccess<DmaRxInterrupt> {}

#[instability::unstable]
pub trait DmaTxChannel: TxRegisterAccess + InterruptAccess<DmaTxInterrupt> {}

/// A description of a DMA Channel.
pub trait DmaChannel: Sized + crate::private::Sealed {
    /// A description of the RX half of a DMA Channel.
    type Rx: DmaRxChannel + From<Self>;

    /// A description of the TX half of a DMA Channel.
    type Tx: DmaTxChannel + From<Self>;

    /// Splits the DMA channel into its RX and TX halves.
    #[cfg(any(esp32c5, esp32c6, esp32h2, esp32s3))] // TODO relax this to allow splitting on all chips
    fn split(self) -> (Self::Rx, Self::Tx) {
        // This function is exposed safely on chips that have separate IN and OUT
        // interrupt handlers.
        // TODO: this includes the P4 as well.
        unsafe { self.split_internal(Internal) }
    }

    /// Splits the DMA channel into its RX and TX halves.
    ///
    /// # Safety
    ///
    /// This function must only be used if the separate halves are used by the
    /// same peripheral.
    unsafe fn split_internal(self, _: Internal) -> (Self::Rx, Self::Tx);
}

#[doc(hidden)]
pub trait DmaChannelExt: DmaChannel {
    fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt>;
    fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt>;
}

macro_rules! impl_channel_common {
    ($peri:ident, $instance:ident) => {
        paste::paste! {
            impl<'d> DmaChannel for $instance<'d> {
                type Rx = [<$peri RxChannel>]<'d>;
                type Tx = [<$peri TxChannel>]<'d>;

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) {
                    unsafe {
                        (
                            [<$peri RxChannel>](Self::steal().into()),
                            [<$peri TxChannel>](Self::steal().into()),
                        )
                    }
                }
            }

            // Convert concrete channel into erased TX/RX half structs
            impl<'d> From<$instance<'d>> for [<$peri RxChannel>]<'d> {
                fn from(this: $instance<'d>) -> [<$peri RxChannel>]<'d> {
                    [<$peri RxChannel>](this.into())
                }
            }

            impl<'d> From<$instance<'d>> for [<$peri TxChannel>]<'d> {
                fn from(this: $instance<'d>) -> [<$peri TxChannel>]<'d> {
                    [<$peri TxChannel>](this.into())
                }
            }

            impl crate::dma::DmaChannelExt for $instance<'_> {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    [<$peri RxChannel>]::from(unsafe { Self::steal() })
                }

                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    [<$peri TxChannel>]::from(unsafe { Self::steal() })
                }
            }
        }
    };
}
pub(crate) use impl_channel_common;
