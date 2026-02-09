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
    dma::*,
    handler,
    interrupt::Priority,
    peripherals::{DMA, Interrupt, pac},
};

#[cfg_attr(dma_gdma_version = "1", path = "ahb_v1.rs")]
#[cfg_attr(dma_gdma_version = "2", path = "ahb_v2.rs")]
mod implementation;

/// An arbitrary GDMA channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaChannel<'d> {
    channel: u8,
    _lifetime: PhantomData<&'d mut ()>,
}

impl AnyGdmaChannel<'_> {
    #[cfg_attr(any(esp32c2, esp32c5), expect(unused))]
    pub(crate) unsafe fn clone_unchecked(&self) -> Self {
        Self {
            channel: self.channel,
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
            AnyGdmaRxChannel {
                channel: self.channel,
                _lifetime: PhantomData,
            },
            AnyGdmaTxChannel {
                channel: self.channel,
                _lifetime: PhantomData,
            },
        )
    }
}

/// An arbitrary GDMA RX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaRxChannel<'d> {
    channel: u8,
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d> DmaChannelConvert<AnyGdmaRxChannel<'d>> for AnyGdmaRxChannel<'d> {
    fn degrade(self) -> AnyGdmaRxChannel<'d> {
        self
    }
}

/// An arbitrary GDMA TX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaTxChannel<'d> {
    channel: u8,
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d> DmaChannelConvert<AnyGdmaTxChannel<'d>> for AnyGdmaTxChannel<'d> {
    fn degrade(self) -> AnyGdmaTxChannel<'d> {
        self
    }
}

use crate::asynch::AtomicWaker;

static TX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];
static RX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];

cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c3))] {
        use portable_atomic::AtomicBool;
        static TX_IS_ASYNC: [AtomicBool; CHANNEL_COUNT] = [const { AtomicBool::new(false) }; CHANNEL_COUNT];
        static RX_IS_ASYNC: [AtomicBool; CHANNEL_COUNT] = [const { AtomicBool::new(false) }; CHANNEL_COUNT];
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
    ($num:literal, $interrupt_in:ident $(, $interrupt_out:ident)? ) => {
        paste::paste! {
            use $crate::peripherals::[<DMA_CH $num>];
            impl [<DMA_CH $num>]<'_> {
                fn handler_in() -> Option<InterruptHandler> {
                    $crate::if_set! {
                        $({
                            // $interrupt_out is present, meaning we have split handlers
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler_in() {
                                $crate::ignore!($interrupt_out);
                                super::asynch::handle_in_interrupt::<[< DMA_CH $num >]<'static>>();
                            }
                            Some(interrupt_handler_in)
                        })?,
                        {
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler() {
                                super::asynch::handle_in_interrupt::<[< DMA_CH $num >]<'static>>();
                                super::asynch::handle_out_interrupt::<[< DMA_CH $num >]<'static>>();
                            }
                            Some(interrupt_handler)
                        }
                    }
                }

                fn isr_in() -> Option<Interrupt> {
                    Some(Interrupt::$interrupt_in)
                }

                fn handler_out() -> Option<InterruptHandler> {
                    $crate::if_set! {
                        $({
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler_out() {
                                $crate::ignore!($interrupt_out);
                                super::asynch::handle_out_interrupt::<[< DMA_CH $num >]<'static>>();
                            }
                            Some(interrupt_handler_out)
                        })?,
                        None
                    }
                }

                fn isr_out() -> Option<Interrupt> {
                    $crate::if_set! { $(Some(Interrupt::$interrupt_out))?, None }
                }
            }

            impl<'d> DmaChannel for [<DMA_CH $num>]<'d> {
                type Rx = AnyGdmaRxChannel<'d>;
                type Tx = AnyGdmaTxChannel<'d>;

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) {
                    (
                        AnyGdmaRxChannel {
                            channel: $num,
                            _lifetime: core::marker::PhantomData,
                        },
                        AnyGdmaTxChannel {
                            channel: $num,
                            _lifetime: core::marker::PhantomData,
                        },
                    )
                }
            }

            impl<'d> DmaChannelConvert<AnyGdmaChannel<'d>> for [<DMA_CH $num>]<'d> {
                fn degrade(self) -> AnyGdmaChannel<'d> {
                    AnyGdmaChannel {
                        channel: $num,
                        _lifetime: core::marker::PhantomData,
                    }
                }
            }

            impl<'d> DmaChannelConvert<AnyGdmaRxChannel<'d>> for [<DMA_CH $num>]<'d> {
                fn degrade(self) -> AnyGdmaRxChannel<'d> {
                    AnyGdmaRxChannel {
                        channel: $num,
                        _lifetime: core::marker::PhantomData,
                    }
                }
            }

            impl<'d> DmaChannelConvert<AnyGdmaTxChannel<'d>> for [<DMA_CH $num>]<'d> {
                fn degrade(self) -> AnyGdmaTxChannel<'d> {
                    AnyGdmaTxChannel {
                        channel: $num,
                        _lifetime: core::marker::PhantomData,
                    }
                }
            }

            impl DmaChannelExt for [<DMA_CH $num>]<'_> {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    AnyGdmaRxChannel {
                        channel: $num,
                        _lifetime: core::marker::PhantomData,
                    }
                }

                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    AnyGdmaTxChannel {
                        channel: $num,
                        _lifetime: core::marker::PhantomData,
                    }
                }
            }
        }
    };
}

const CHANNEL_COUNT: usize = cfg!(soc_has_dma_ch0) as usize
    + cfg!(soc_has_dma_ch1) as usize
    + cfg!(soc_has_dma_ch2) as usize
    + cfg!(soc_has_dma_ch3) as usize
    + cfg!(soc_has_dma_ch4) as usize;

cfg_if::cfg_if! {
    if #[cfg(dma_separate_in_out_interrupts)] {
        #[cfg(soc_has_dma_ch0)]
        impl_channel!(0, DMA_IN_CH0, DMA_OUT_CH0);
        #[cfg(soc_has_dma_ch1)]
        impl_channel!(1, DMA_IN_CH1, DMA_OUT_CH1);
        #[cfg(soc_has_dma_ch2)]
        impl_channel!(2, DMA_IN_CH2, DMA_OUT_CH2);
        #[cfg(soc_has_dma_ch3)]
        impl_channel!(3, DMA_IN_CH3, DMA_OUT_CH3);
        #[cfg(soc_has_dma_ch4)]
        impl_channel!(4, DMA_IN_CH4, DMA_OUT_CH4);
    } else {
        #[cfg(soc_has_dma_ch0)]
        impl_channel!(0, DMA_CH0);
        #[cfg(soc_has_dma_ch1)]
        impl_channel!(1, DMA_CH1);
        #[cfg(soc_has_dma_ch2)]
        impl_channel!(2, DMA_CH2);
        #[cfg(soc_has_dma_ch3)]
        impl_channel!(3, DMA_CH3);
        #[cfg(soc_has_dma_ch4)]
        impl_channel!(4, DMA_CH4);
    }
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
