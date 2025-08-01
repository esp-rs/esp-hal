use core::mem::ManuallyDrop;

use crate::{
    dma::{asynch::{DmaRxFuture, DmaTxFuture}, Channel, DmaChannelFor, DmaEligible, DmaRxBuffer, DmaTxBuffer}, peripherals, uart::{
        uhci::{AnyUhci, UhciInternal}, Uart
    }, Async, Blocking, DriverMode
};

/// todo
pub struct Uhci<'d, Dm>
where
    Dm: DriverMode,
{
    /// todo
    pub internal: UhciInternal<'d, Dm>,
}

impl<'d> Uhci<'d, Blocking> {
    /// todo
    pub fn new(
        uart: Uart<'d, Blocking>,
        uhci: peripherals::UHCI0<'static>,
        channel: impl DmaChannelFor<AnyUhci<'d>>,
    ) -> Self {
        let channel = Channel::new(channel.degrade());
        channel.runtime_ensure_compatible(&uhci);

        let self_uhci = UhciInternal {
            uart,
            uhci: uhci.into(),
            channel,
        };

        self_uhci.init();

        Self {
            internal: self_uhci,
        }
    }

    /// todo
    pub fn into_async(self) -> Uhci<'d, Async> {
        let internal = self.internal.into_async();
        Uhci { internal }
    }
}

impl<'d> Uhci<'d, Async> {
    /// todo
    pub async fn write_dma(
        mut self,
        mut tx_buffer: impl DmaTxBuffer,
    ) -> UhciDmaTxTransfer<'d, Async, impl DmaTxBuffer> {
        {
            unsafe {
                self.internal
                    .channel
                    .tx
                    .prepare_transfer(self.internal.uhci.dma_peripheral(), &mut tx_buffer)
                    .unwrap()
            };

            self.internal.channel.tx.start_transfer().unwrap();

            return UhciDmaTxTransfer::new(self, tx_buffer);
        }
    }

    /// todo
    pub async fn read_dma(
        mut self,
        mut rx_buffer: impl DmaRxBuffer,
    ) -> UhciDmaRxTransfer<'d, Async, impl DmaRxBuffer> {
        {
            unsafe {
                self.internal
                    .channel
                    .rx
                    .prepare_transfer(self.internal.uhci.dma_peripheral(), &mut rx_buffer)
                    .unwrap()
            };

            self.internal.channel.rx.start_transfer().unwrap();

            return UhciDmaRxTransfer::new(self, rx_buffer);
        }
    }

    /// todo
    pub fn into_blocking(self) -> Uhci<'d, Blocking> {
        let internal = self.internal.into_blocking();
        Uhci { internal }
    }
}

// Based on SpiDmaTransfer
/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct UhciDmaTxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
{
    uhci: ManuallyDrop<Uhci<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf>,
}

impl<'d, Buf> UhciDmaTxTransfer<'d, Async, Buf> {
    fn new(uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf),
        }
    }

    /// todo
    pub fn is_done(&self) -> bool {
        self.uhci.internal.channel.tx.is_done()
    }

    async fn wait_for_idle(&mut self) {
        DmaTxFuture::new(&mut self.uhci.internal.channel.tx)
            .await
            .unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf) {
        self.wait_for_idle().await;

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(&mut self) {
        // TODO: Shouldn't I here return like in wait?
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
        }
    }
}

impl<Dm, Buf> Drop for UhciDmaTxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
{
    fn drop(&mut self) {
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
            // TODO: I don't have async here, should I call the blocking (not yet existing
            // wait_for_idle?) should a drop cause the whole program to potentially
            // freeze? self.wait_for_idle();

            unsafe {
                ManuallyDrop::drop(&mut self.uhci);
                ManuallyDrop::drop(&mut self.dma_buf);
            }
        }
    }
}

// Based on SpiDmaTransfer
/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct UhciDmaRxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
{
    uhci: ManuallyDrop<Uhci<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf>,
}

impl<'d, Buf> UhciDmaRxTransfer<'d, Async, Buf> {
    fn new(uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf),
        }
    }

    /// todo
    pub fn is_done(&self) -> bool {
        self.uhci.internal.channel.tx.is_done()
    }

    async fn wait_for_idle(&mut self) {
        DmaRxFuture::new(&mut self.uhci.internal.channel.rx)
            .await
            .unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf) {
        self.wait_for_idle().await;

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(&mut self) {
        // TODO: Shouldn't I here return like in wait?
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
        }
    }
}

impl<Dm, Buf> Drop for UhciDmaRxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
{
    fn drop(&mut self) {
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
            // TODO: I don't have async here, should I call the blocking (not yet existing
            // wait_for_idle?) should a drop cause the whole program to potentially
            // freeze? self.wait_for_idle();

            unsafe {
                ManuallyDrop::drop(&mut self.uhci);
                ManuallyDrop::drop(&mut self.dma_buf);
            }
        }
    }
}
