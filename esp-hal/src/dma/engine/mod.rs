use enumset::{EnumSet, EnumSetType};

use crate::{
    asynch::AtomicWaker,
    dma::{DmaConfigError, DmaRxInterrupt, DmaTxInterrupt},
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

/// The configuration of a DMA engine, as the channel uses it.
#[doc(hidden)]
pub trait DmaEngineConfig: Copy + Default + crate::private::Sealed {
    /// The log2 of the buffer alignment that the internal RAM burst needs.
    fn internal_burst_log2(&self) -> u8;

    /// The log2 of the buffer alignment that the PSRAM burst needs.
    fn external_burst_log2(&self) -> u8;

    /// Checks the conditions that the types cannot express.
    fn validate(&self) -> Result<(), DmaConfigError>;
}

/// The discriminant of a burst type variant: the log2 of the buffer alignment that it needs.
macro_rules! burst_log2 {
    (Disabled) => {
        0
    };
    (Enabled) => {
        2
    };
    (Size4) => {
        2
    };
    (Size8) => {
        3
    };
    (Size16) => {
        4
    };
    (Size32) => {
        5
    };
    (Size64) => {
        6
    };
    (Size128) => {
        7
    };
}

macro_rules! burst_doc {
    (Disabled) => {
        "Single transfers."
    };
    (Enabled) => {
        "Burst transfers. A receive buffer needs 4-byte alignment to use them."
    };
    (Size4) => {
        "Bursts of 4 bytes."
    };
    (Size8) => {
        "Bursts of up to 8 bytes."
    };
    (Size16) => {
        "Bursts of up to 16 bytes."
    };
    (Size32) => {
        "Bursts of up to 32 bytes."
    };
    (Size64) => {
        "Bursts of up to 64 bytes."
    };
    (Size128) => {
        "Bursts of up to 128 bytes."
    };
}

// Unused on chips with a configurable PSRAM block size.
#[allow(unused_macros)]
macro_rules! is_on_off {
    (Disabled, Enabled) => {
        true
    };
    ($($variant:ident),*) => {
        false
    };
}

macro_rules! burst_type {
    ($(#[$attr:meta])* $name:ident, default = $default:ident, [$($variant:ident),+]) => {
        $(#[$attr])*
        ///
        /// The variants are the bursts that the engine can use on this chip. Convert a value
        /// into a [`BurstSize`](crate::dma::BurstSize) to create a buffer that does not limit
        /// the burst.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[non_exhaustive]
        #[repr(u8)]
        pub enum $name {
            $(
                #[doc = burst_doc!($variant)]
                $variant = burst_log2!($variant),
            )+
        }

        impl Default for $name {
            fn default() -> Self {
                Self::$default
            }
        }

        impl From<$name> for crate::dma::BurstSize {
            fn from(burst: $name) -> Self {
                crate::dma::BurstSize::from_log2(burst as u8)
            }
        }
    };
}

/// Declares an engine configuration and the burst types of its fields.
// Unused on chips whose only engine is the copy DMA.
#[allow(unused_macros)]
macro_rules! declare_engine_config {
    (
        $(#[$attr:meta])*
        $config:ident, $engine:literal, $prefix:ident,
        internal = [$first:ident $(, $internal:ident)*],
        external = [],
        external_default = $external_default:ident
        $(, priority = $priority:ident)?
    ) => {
        paste::paste! {
            burst_type!(
                #[doc = concat!("Data burst of ", $engine, " for buffers in internal RAM.")]
                [<$prefix InternalBurst>],
                default = $first,
                [$first $(, $internal)*]
            );

            $(#[$attr])*
            #[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[non_exhaustive]
            pub struct $config {
                /// The largest data burst for buffers in internal RAM.
                internal_burst: [<$prefix InternalBurst>],

                $(
                    /// The channel priority.
                    priority: $priority,
                )?
            }

            impl crate::private::Sealed for $config {}

            impl crate::dma::DmaEngineConfig for $config {
                fn internal_burst_log2(&self) -> u8 {
                    self.internal_burst as u8
                }

                fn external_burst_log2(&self) -> u8 {
                    0
                }

                fn validate(&self) -> Result<(), crate::dma::DmaConfigError> {
                    Ok(())
                }
            }
        }
    };
    (
        $(#[$attr:meta])*
        $config:ident, $engine:literal, $prefix:ident,
        internal = [$first:ident $(, $internal:ident)*],
        external = [$($external:ident),+],
        external_default = $external_default:ident
        $(, priority = $priority:ident)?
    ) => {
        paste::paste! {
            burst_type!(
                #[doc = concat!("Data burst of ", $engine, " for buffers in internal RAM.")]
                [<$prefix InternalBurst>],
                default = $first,
                [$first $(, $internal)*]
            );

            burst_type!(
                #[doc = concat!("Data burst of ", $engine, " for buffers in PSRAM.")]
                ///
                /// The default is 32 bytes, or the largest size if the PSRAM accepts less. With flash
                /// encryption enabled, the burst must be at least 16 bytes.
                #[cfg_attr(
                    esp32s2,
                    doc = "",
                    doc = "The ESP32-S2 PSRAM accepts only 16-byte bursts. With 64-byte bursts \
                        the DMA moves only the first 32 bytes of each block, and with 32-byte \
                        bursts a full-duplex SPI transfer with both buffers in PSRAM can lose \
                        data."
                )]
                [<$prefix ExternalBurst>],
                default = $external_default,
                [$($external),+]
            );

            $(#[$attr])*
            #[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[non_exhaustive]
            pub struct $config {
                /// The largest data burst for buffers in internal RAM.
                internal_burst: [<$prefix InternalBurst>],

                /// The largest data burst for buffers in PSRAM.
                external_burst: [<$prefix ExternalBurst>],

                $(
                    /// The channel priority.
                    priority: $priority,
                )?
            }

            impl crate::private::Sealed for $config {}

            impl crate::dma::DmaEngineConfig for $config {
                fn internal_burst_log2(&self) -> u8 {
                    self.internal_burst as u8
                }

                fn external_burst_log2(&self) -> u8 {
                    self.external_burst as u8
                }

                fn validate(&self) -> Result<(), crate::dma::DmaConfigError> {
                    if self.external_burst_log2() < burst_log2!(Size16)
                        && crate::efuse::flash_encryption()
                    {
                        return Err(crate::dma::DmaConfigError::UnsupportedBurstSize);
                    }
                    Ok(())
                }
            }
        }
    };
}

for_each_dma_engine! {
    (burst $( (
        $engine:tt,
        internal = [$($internal:ident),+],
        external = [$($external:ident),*],
        external_default = $external_default:ident
    ) ),*) => {
        /// The largest burst on this chip that needs an aligned buffer, in bytes.
        ///
        /// On/off data burst bits need 4-byte aligned receive buffers. A configurable PSRAM block
        /// size needs receive buffers aligned to the block.
        pub(crate) const MAX_ALIGNED_BURST: usize = cfg_select! {
            dma_ext_mem_configurable_block_size => property!("dma.psram_max_burst"),
            _ => if false $(|| is_on_off!($($internal),+))* { 4 } else { 1 },
        };
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

/// Declares the `DmaConfig` alias of a driver module and the aliases of its field types.
macro_rules! driver_dma_config {
    (@config $engine:tt, $prefix:ident) => {
        /// The DMA configuration type of this driver's DMA engine.
        ///
        /// `apply_dma_config` applies this configuration.
        #[instability::unstable]
        pub type DmaConfig = $crate::dma::engine_config!($engine);

        paste::paste! {
            /// The internal RAM burst type of this driver's DMA engine.
            #[instability::unstable]
            pub type DmaInternalBurst = $crate::dma::[<$prefix InternalBurst>];

            /// The PSRAM burst type of this driver's DMA engine.
            #[cfg(dma_can_access_psram)]
            #[instability::unstable]
            pub type DmaExternalBurst = $crate::dma::[<$prefix ExternalBurst>];
        }
    };
    ("AHB_GDMA") => {
        $crate::dma::driver_dma_config!(@config "AHB_GDMA", Ahb);

        /// The channel priority type of this driver's DMA engine.
        #[instability::unstable]
        pub type DmaPriority = $crate::dma::AhbPriority;
    };
    ("AXI_GDMA") => {
        $crate::dma::driver_dma_config!(@config "AXI_GDMA", Axi);

        /// The channel priority type of this driver's DMA engine.
        #[instability::unstable]
        pub type DmaPriority = $crate::dma::AxiPriority;
    };
    ("SPI_DMA") => {
        $crate::dma::driver_dma_config!(@config "SPI_DMA", SpiDma);
    };
    ("I2S_DMA") => {
        $crate::dma::driver_dma_config!(@config "I2S_DMA", I2sDma);
    };
    ("CRYPTO_DMA") => {
        $crate::dma::driver_dma_config!(@config "CRYPTO_DMA", CryptoDma);
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
    type Config: DmaEngineConfig;

    /// Whether a data burst needs a buffer that is aligned to it. A transfer then lowers the
    /// burst to the alignment that the buffer guarantees.
    const NEEDS_ALIGNED_BURST: bool = false;

    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard>;

    /// Resets the state machine of the channel and FIFO pointer.
    fn reset(&self);

    /// Sets the data burst of a transfer.
    ///
    /// `internal` and `external` are the log2 of the buffer alignment that the bursts for
    /// internal RAM and for PSRAM need (the discriminants of the engine burst types, or lower).
    /// `psram` is true when the transfer accesses PSRAM.
    fn set_burst(&self, internal: u8, external: u8, psram: bool);

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

    /// Inlink descriptor address when a successful EOF occurs on the Rx channel.
    fn last_dscr_address(&self) -> usize;

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
    type Config: DmaEngineConfig;

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
