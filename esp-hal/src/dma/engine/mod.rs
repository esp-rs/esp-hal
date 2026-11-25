use enumset::{EnumSet, EnumSetType};

use crate::{
    asynch::AtomicWaker,
    dma::{BurstConfig, DmaRxInterrupt, DmaTxInterrupt},
    interrupt::InterruptHandler,
    peripherals::Interrupt,
    private::{Internal, Sealed},
    system::PeripheralGuard,
};

// Unused on chips without engine priorities.
#[allow(unused_macros)]
macro_rules! priority_enum {
    ($name:ident, $engine:literal, $($level:literal),+) => {
        paste::paste! {
            #[doc = concat!("Channel priority of the ", $engine, " engine.")]
            ///
            /// The arbiter serves the channel with the highest priority first. Level 0 is the
            /// lowest priority and the default.
            #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[non_exhaustive]
            pub enum $name {
                $(
                    #[doc = concat!("Priority level ", stringify!($level), ".")]
                    [<Priority $level>] = $level,
                )+
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::Priority0
            }
        }
    };
}

for_each_dma_engine! {
    ("AHB_GDMA") => {
        mod gdma;
        pub use gdma::*;
    };
    ("AXI_GDMA") => {
        mod axi_gdma;
        pub use axi_gdma::*;
    };
    ("COPY_DMA") => {
        mod copy;
        pub use copy::*;
    };
    ("CRYPTO_DMA") => {
        mod crypto;
        pub use crypto::*;
    };
    ("I2S_DMA") => {
        mod i2s;
        pub use i2s::*;
    };
    ("SPI_DMA") => {
        mod spi;
        pub use spi::*;
    };
}

/// Names the configuration type of a DMA engine.
macro_rules! engine_config {
    ("AHB_GDMA") => {
        crate::dma::AhbGdmaConfig
    };
    ("AXI_GDMA") => {
        crate::dma::AxiGdmaConfig
    };
    ("SPI_DMA") => {
        crate::dma::SpiDmaConfig
    };
    ("I2S_DMA") => {
        crate::dma::I2sDmaConfig
    };
    ("CRYPTO_DMA") => {
        crate::dma::CryptoDmaConfig
    };
    ("COPY_DMA") => {
        crate::dma::CopyDmaConfig
    };
}
pub(crate) use engine_config;

/// Declares the `DmaConfig` alias of a driver module, and `DmaPriority` on engines with
/// priority registers.
macro_rules! driver_dma_config {
    (@config $engine:tt) => {
        /// The DMA configuration type of this driver's DMA engine.
        ///
        /// `apply_dma_config` applies this configuration.
        #[instability::unstable]
        pub type DmaConfig = $crate::dma::engine_config!($engine);
    };
    ("AHB_GDMA") => {
        $crate::dma::driver_dma_config!(@config "AHB_GDMA");

        /// The channel priority type of this driver's DMA engine.
        #[instability::unstable]
        pub type DmaPriority = $crate::dma::AhbPriority;
    };
    ("AXI_GDMA") => {
        $crate::dma::driver_dma_config!(@config "AXI_GDMA");

        /// The channel priority type of this driver's DMA engine.
        #[instability::unstable]
        pub type DmaPriority = $crate::dma::AxiPriority;
    };
    ($engine:tt) => {
        $crate::dma::driver_dma_config!(@config $engine);
    };
}
pub(crate) use driver_dma_config;

/// Implemented by peripheral singletons that can be used with a DMA engine.
pub trait DmaEligiblePeripheral<D: DmaChannel> {
    /// Returns the `DmaPeripheral` ID for runtime compatibility checks.
    fn dma_peripheral(&self) -> DmaPeripheral;
}

for_each_peripheral! {
    (dma_eligible $(( $peri:ident, $name:ident, $id:literal, $any_ch:ident )),*) => {
        /// DMA-eligible peripheral selector values; values are engine-local (matching hardware where applicable).
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub struct DmaPeripheral(pub(crate) u8);
        impl DmaPeripheral {
            $(
                #[doc = concat!("DMA accesses ", stringify!($name))]
                pub const $peri: Self = Self($id);
            )*
        }

        $(
            impl<'d> DmaEligiblePeripheral<$any_ch<'d>> for crate::peripherals::$peri<'d> {
                fn dma_peripheral(&self) -> DmaPeripheral {
                    DmaPeripheral::$peri
                }
            }
        )*
    };
}

#[doc(hidden)]
pub trait RegisterAccess: Sealed {
    /// The configuration type of the channel's DMA engine.
    type Config: Copy + Default;

    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard>;

    /// Resets the state machine of the channel and FIFO pointer.
    fn reset(&self);

    /// Enables or disables INCR burst transfer for channel reading
    /// accessing data in internal RAM.
    fn set_burst_mode(&self, burst_mode: BurstConfig);

    /// Enables or disables burst transfer for channel reading
    /// descriptors in internal RAM.
    fn set_descr_burst_mode(&self, burst_mode: bool);

    /// Writes the engine configuration to the channel half.
    fn apply_config(&self, config: &Self::Config);

    /// Selects a peripheral for the channel.
    fn set_peripheral(&self, _peripheral: u8) {}

    /// Sets the address of the first descriptor.
    fn set_link_addr(&self, address: u32);

    /// Enables the channel for data transfer.
    fn start(&self);

    /// Stops the channel from transferring data.
    fn stop(&self);

    /// Mounts a new descriptor.
    fn restart(&self);

    /// Configures the bit to enable checking the owner attribute of the
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
    #[cfg(dma_supports_mem2mem)]
    fn set_mem2mem_mode(&self, value: bool);

    fn peripheral_interrupt(&self) -> Option<Interrupt>;
    fn async_handler(&self) -> Option<InterruptHandler>;
}

#[doc(hidden)]
pub trait TxRegisterAccess: RegisterAccess {
    /// Returns whether the DMA's FIFO is empty.
    fn is_fifo_empty(&self) -> bool;

    /// Enables or disables outlink-writeback.
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
    /// The configuration type of the channel's DMA engine.
    type Config: Copy + Default;

    /// A description of the RX half of a DMA Channel.
    type Rx: DmaRxChannel<Config = Self::Config> + From<Self>;

    /// A description of the TX half of a DMA Channel.
    type Tx: DmaTxChannel<Config = Self::Config> + From<Self>;

    /// Splits the DMA channel into its RX and TX halves.
    #[cfg(dma_separate_in_out_interrupts)] // TODO relax this to allow splitting on all chips
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
    /// Must only be used if the separate halves are used by the same peripheral.
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
                type Config = <[<$peri RxChannel>]<'d> as RegisterAccess>::Config;
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
