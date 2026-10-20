//! # General Direct Memory Access (GDMA)
//!
//! ## Overview
//! GDMA is a feature that allows peripheral-to-memory, memory-to-peripheral,
//! and memory-to-memory data transfer at high speed. The CPU is not involved in
//! the GDMA transfer and therefore is more efficient with less workload.
//!
//! The `GDMA` module provides multiple DMA channels, each capable of managing
//! data transfer for various peripherals.

use core::marker::PhantomData;

use crate::{
    asynch::AtomicWaker,
    dma::*,
    handler,
    interrupt::Priority,
    peripherals::{DMA, Interrupt, pac},
    system::{Peripheral, PeripheralGuard},
};

#[cfg_attr(dma_gdma_version = "1", path = "ahb_v1.rs")]
#[cfg_attr(dma_gdma_version = "2", path = "ahb_v2.rs")]
mod implementation;

/// Immutable per-channel metadata owned by each `DMA_CH*` singleton.
pub(crate) struct ChannelInfo {
    /// Hardware channel index used to select the register bank.
    pub(crate) channel: u8,
    /// Interrupt handler for the RX (in) direction.
    pub(crate) handler_in: Option<InterruptHandler>,
    /// Interrupt handler for the TX (out) direction.
    pub(crate) handler_out: Option<InterruptHandler>,
    /// Peripheral interrupt for the RX (in) direction.
    pub(crate) isr_in: Option<Interrupt>,
    /// Peripheral interrupt for the TX (out) direction.
    pub(crate) isr_out: Option<Interrupt>,
    /// List of compatible peripheral IDs for this channel.
    pub(crate) compatible_peripherals: &'static [u8],
}

/// Mutable per-channel runtime state (wakers and async-mode flags).
pub(crate) struct ChannelState {
    /// Async waker for the TX (out) half of this channel.
    pub(crate) tx_waker: AtomicWaker,

    /// Async waker for the RX (in) half of this channel.
    pub(crate) rx_waker: AtomicWaker,

    /// Whether the TX half is currently in async mode (shared-interrupt chips only).
    #[cfg(not(dma_separate_in_out_interrupts))]
    pub(crate) tx_is_async: portable_atomic::AtomicBool,

    /// Whether the RX half is currently in async mode (shared-interrupt chips only).
    #[cfg(not(dma_separate_in_out_interrupts))]
    pub(crate) rx_is_async: portable_atomic::AtomicBool,
}

/// An arbitrary GDMA channel
pub struct AhbGdmaChannel<'d> {
    info: &'static ChannelInfo,
    state: &'static ChannelState,
    _lifetime: PhantomData<&'d mut ()>,
}

impl core::fmt::Debug for AhbGdmaChannel<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AhbGdmaChannel")
            .field("channel", &self.info.channel)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AhbGdmaChannel<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "AhbGdmaChannel {{ channel: {} }}", self.info.channel)
    }
}

impl AhbGdmaChannel<'_> {
    pub(crate) unsafe fn clone_unchecked(&self) -> Self {
        Self {
            info: self.info,
            state: self.state,
            _lifetime: PhantomData,
        }
    }
}

impl crate::private::Sealed for AhbGdmaChannel<'_> {}
impl<'d> DmaChannel for AhbGdmaChannel<'d> {
    type Rx = AhbGdmaRxChannel<'d>;
    type Tx = AhbGdmaTxChannel<'d>;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AhbGdmaRxChannel(unsafe { self.clone_unchecked() }),
            AhbGdmaTxChannel(self),
        )
    }
}

/// An arbitrary GDMA RX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AhbGdmaRxChannel<'d>(AhbGdmaChannel<'d>);

/// An arbitrary GDMA TX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AhbGdmaTxChannel<'d>(AhbGdmaChannel<'d>);

impl crate::private::Sealed for AhbGdmaTxChannel<'_> {}
impl DmaTxChannel for AhbGdmaTxChannel<'_> {}

impl crate::private::Sealed for AhbGdmaRxChannel<'_> {}
impl DmaRxChannel for AhbGdmaRxChannel<'_> {}

macro_rules! impl_channel {
    // Single shared interrupt: one handler drives both the in and out paths.
    ($ch:ident, $num:literal, $interrupt_in:ident, compatible = [$($compatible:ident),*]) => {
        use $crate::peripherals::$ch;
        impl $ch<'_> {
            pub(super) fn info() -> &'static ChannelInfo {
                #[handler(priority = Priority::max())]
                fn interrupt_handler() {
                    asynch::handle_in_interrupt::<$ch<'static>>();
                    asynch::handle_out_interrupt::<$ch<'static>>();
                }
                static INFO: ChannelInfo = ChannelInfo {
                    channel: $num,
                    handler_in: Some(interrupt_handler),
                    handler_out: None,
                    isr_in: Some(Interrupt::$interrupt_in),
                    isr_out: None,
                    compatible_peripherals: &[$(crate::dma::DmaPeripheral::$compatible.0),*],
                };
                &INFO
            }

            pub(super) fn state() -> &'static ChannelState {
                static STATE: ChannelState = ChannelState {
                    tx_waker: AtomicWaker::new(),
                    rx_waker: AtomicWaker::new(),
                    tx_is_async: portable_atomic::AtomicBool::new(false),
                    rx_is_async: portable_atomic::AtomicBool::new(false),
                };
                &STATE
            }
        }

        impl<'d> From<$ch<'d>> for AhbGdmaChannel<'d> {
            fn from(_ch: $ch<'d>) -> AhbGdmaChannel<'d> {
                AhbGdmaChannel {
                    info: $ch::info(),
                    state: $ch::state(),
                    _lifetime: core::marker::PhantomData,
                }
            }
        }
        crate::dma::impl_channel_common!(AhbGdma, $ch);
    };

    // Split interrupts: separate handlers for the in and out paths.
    ($ch:ident, $num:literal, $interrupt_in:ident, $interrupt_out:ident, compatible = [$($compatible:ident),*]) => {
        use $crate::peripherals::$ch;
        impl $ch<'_> {
            pub(super) fn info() -> &'static ChannelInfo {
                #[handler(priority = Priority::max())]
                fn interrupt_handler_in() {
                    asynch::handle_in_interrupt::<$ch<'static>>();
                }

                #[handler(priority = Priority::max())]
                fn interrupt_handler_out() {
                    asynch::handle_out_interrupt::<$ch<'static>>();
                }

                static INFO: ChannelInfo = ChannelInfo {
                    channel: $num,
                    handler_in: Some(interrupt_handler_in),
                    handler_out: Some(interrupt_handler_out),
                    isr_in: Some(Interrupt::$interrupt_in),
                    isr_out: Some(Interrupt::$interrupt_out),
                    compatible_peripherals: &[$(crate::dma::DmaPeripheral::$compatible.0),*],
                };
                &INFO
            }

            pub(super) fn state() -> &'static ChannelState {
                static STATE: ChannelState = ChannelState {
                    tx_waker: AtomicWaker::new(),
                    rx_waker: AtomicWaker::new(),
                };
                &STATE
            }
        }

        impl<'d> From<$ch<'d>> for AhbGdmaChannel<'d> {
            fn from(_ch: $ch<'d>) -> AhbGdmaChannel<'d> {
                AhbGdmaChannel {
                    info: $ch::info(),
                    state: $ch::state(),
                    _lifetime: core::marker::PhantomData,
                }
            }
        }
        crate::dma::impl_channel_common!(AhbGdma, $ch);
    };
}

// Convert erased channel into erased TX/RX half structs
impl<'d> From<AhbGdmaChannel<'d>> for AhbGdmaRxChannel<'d> {
    fn from(this: AhbGdmaChannel<'d>) -> AhbGdmaRxChannel<'d> {
        AhbGdmaRxChannel(this)
    }
}

impl<'d> From<AhbGdmaChannel<'d>> for AhbGdmaTxChannel<'d> {
    fn from(this: AhbGdmaChannel<'d>) -> AhbGdmaTxChannel<'d> {
        AhbGdmaTxChannel(this)
    }
}

for_each_dma_channel! {
    ("AHB_GDMA", $ch:ident, $num:literal, interrupt = $interrupt:ident, compatible = [$($compatible:ident),*]) => {
        impl_channel!($ch, $num, $interrupt, compatible = [$($compatible),*]);
    };
    ("AHB_GDMA", $ch:ident, $num:literal, interrupt_in = $interrupt_in:ident, interrupt_out = $interrupt_out:ident, compatible = [$($compatible:ident),*]) => {
        impl_channel!($ch, $num, $interrupt_in, $interrupt_out, compatible = [$($compatible),*]);
    };
}

fn init_dma_racey() {
    // FIXME: reset/clock enable belongs to metadata
    use crate::RegisterToggle;
    DMA::regs()
        .misc_conf()
        .toggle(|w, en| w.ahbm_rst_inter().bit(en));
    DMA::regs().misc_conf().modify(|_, w| w.clk_en().set_bit());

    implementation::setup();
}
