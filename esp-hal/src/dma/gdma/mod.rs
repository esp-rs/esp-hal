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
};

#[cfg_attr(dma_gdma_version = "1", path = "ahb_v1.rs")]
#[cfg_attr(dma_gdma_version = "2", path = "ahb_v2.rs")]
mod implementation;

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
}

/// An arbitrary GDMA channel
pub struct AnyGdmaChannel<'d> {
    info: &'static ChannelInfo,
    state: &'static ChannelState,
    _lifetime: PhantomData<&'d mut ()>,
}

impl core::fmt::Debug for AnyGdmaChannel<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AnyGdmaChannel")
            .field("channel", &self.info.channel)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AnyGdmaChannel<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "AnyGdmaChannel {{ channel: {} }}", self.info.channel)
    }
}

impl AnyGdmaChannel<'_> {
    pub(crate) unsafe fn clone_unchecked(&self) -> Self {
        Self {
            info: self.info,
            state: self.state,
            _lifetime: PhantomData,
        }
    }
}

impl crate::private::Sealed for AnyGdmaChannel<'_> {}
impl<'d> DmaChannel for AnyGdmaChannel<'d> {
    type Rx = AnyGdmaRxChannel<'d>;
    type Tx = AnyGdmaTxChannel<'d>;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AnyGdmaRxChannel(unsafe { self.clone_unchecked() }),
            AnyGdmaTxChannel(self),
        )
    }
}

/// An arbitrary GDMA RX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaRxChannel<'d>(AnyGdmaChannel<'d>);

impl<'d> DmaChannelConvert<AnyGdmaRxChannel<'d>> for AnyGdmaRxChannel<'d> {
    fn degrade(self) -> AnyGdmaRxChannel<'d> {
        self
    }
}

/// An arbitrary GDMA TX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaTxChannel<'d>(AnyGdmaChannel<'d>);

impl<'d> DmaChannelConvert<AnyGdmaTxChannel<'d>> for AnyGdmaTxChannel<'d> {
    fn degrade(self) -> AnyGdmaTxChannel<'d> {
        self
    }
}

impl crate::private::Sealed for AnyGdmaTxChannel<'_> {}
impl DmaTxChannel for AnyGdmaTxChannel<'_> {}

impl crate::private::Sealed for AnyGdmaRxChannel<'_> {}
impl DmaRxChannel for AnyGdmaRxChannel<'_> {}

impl<CH: DmaChannel, Dm: DriverMode> Channel<Dm, CH> {
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible<P: DmaEligible>(&self, _peripheral: &P) {
        // No runtime checks; GDMA channels are compatible with any peripheral
    }
}

macro_rules! impl_channel {
    // Single shared interrupt: one handler drives both the in and out paths.
    ($ch:ident, $num:literal, $interrupt_in:ident) => {
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

        impl<'d> DmaChannelConvert<AnyGdmaChannel<'d>> for $ch<'d> {
            fn degrade(self) -> AnyGdmaChannel<'d> {
                AnyGdmaChannel {
                    info: Self::info(),
                    state: Self::state(),
                    _lifetime: core::marker::PhantomData,
                }
            }
        }
        crate::dma::impl_channel_common!(AnyGdma, $ch);
    };

    // Split interrupts: separate handlers for the in and out paths.
    ($ch:ident, $num:literal, $interrupt_in:ident, $interrupt_out:ident) => {
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

        impl<'d> DmaChannelConvert<AnyGdmaChannel<'d>> for $ch<'d> {
            fn degrade(self) -> AnyGdmaChannel<'d> {
                AnyGdmaChannel {
                    info: Self::info(),
                    state: Self::state(),
                    _lifetime: core::marker::PhantomData,
                }
            }
        }
        crate::dma::impl_channel_common!(AnyGdma, $ch);
    };
}

for_each_dma_channel! {
    ($ch:ident, $num:literal, interrupt = $interrupt:ident) => {
        impl_channel!($ch, $num, $interrupt);
    };
    ($ch:ident, $num:literal, interrupt_in = $interrupt_in:ident, interrupt_out = $interrupt_out:ident) => {
        impl_channel!($ch, $num, $interrupt_in, $interrupt_out);
    };
}

for_each_peripheral! {
    (dma_eligible $(( $peri:ident, $name:ident, $id:literal )),*) => {
        crate::dma::impl_dma_eligible! {
            AnyGdmaChannel {
                $($peri => $name,)*
            }
        }
    };
}

pub(super) fn init_dma_racey() {
    DMA::regs()
        .misc_conf()
        .modify(|_, w| w.ahbm_rst_inter().set_bit());
    DMA::regs()
        .misc_conf()
        .modify(|_, w| w.ahbm_rst_inter().clear_bit());
    DMA::regs().misc_conf().modify(|_, w| w.clk_en().set_bit());

    implementation::setup();
}
