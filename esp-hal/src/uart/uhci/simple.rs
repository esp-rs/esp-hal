use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        Channel,
        DmaChannelFor,
        DmaEligible,
        DmaRxBuffer,
        DmaTxBuffer,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    peripherals,
    uart::{
        Uart,
        uhci::{AnyUhci, UhciInternal},
    },
};

/// todo
pub struct UhciSimple<'d, Dm>
where
    Dm: DriverMode,
{
    /// todo
    pub internal: UhciInternal<'d, Dm>,
}

impl<'d> UhciSimple<'d, Blocking> {
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
    pub fn read(&mut self, rx_buffer: &mut impl DmaRxBuffer) {
        unsafe {
            self.internal
                .channel
                .rx
                .prepare_transfer(self.internal.uhci.dma_peripheral(), rx_buffer)
                .unwrap()
        };

        self.internal.channel.rx.start_transfer().unwrap();

        // Based on spi slave dma, is this a good idea? infinite loop to wait for something?
        while !self.internal.channel.rx.is_done() {}

        self.internal.channel.rx.stop_transfer();
    }

    /// todo
    pub fn write(&mut self, tx_buffer: &mut impl DmaTxBuffer) {
        unsafe {
            self.internal
                .channel
                .tx
                .prepare_transfer(self.internal.uhci.dma_peripheral(), tx_buffer)
                .unwrap()
        };

        self.internal.channel.tx.start_transfer().unwrap();

        while !self.internal.channel.tx.is_done() {}

        self.internal.channel.tx.stop_transfer();
    }

    /// todo
    pub fn into_async(self) -> UhciSimple<'d, Async> {
        let internal = self.internal.into_async();
        UhciSimple { internal }
    }
}

impl<'d> UhciSimple<'d, Async> {
    /// todo
    pub async fn read(&mut self, rx_buffer: &mut impl DmaRxBuffer) {
        let dma_future = DmaRxFuture::new(&mut self.internal.channel.rx);

        unsafe {
            dma_future
                .rx
                .prepare_transfer(self.internal.uhci.dma_peripheral(), rx_buffer)
                .unwrap()
        };

        dma_future.rx.start_transfer().unwrap();

        dma_future.await.unwrap();

        self.internal.channel.rx.stop_transfer();
    }

    /// todo
    pub async fn write(&mut self, tx_buffer: &mut impl DmaTxBuffer) {
        let dma_future = DmaTxFuture::new(&mut self.internal.channel.tx);

        unsafe {
            dma_future
                .tx
                .prepare_transfer(self.internal.uhci.dma_peripheral(), tx_buffer)
                .unwrap()
        };

        dma_future.tx.start_transfer().unwrap();

        dma_future.await.unwrap();

        self.internal.channel.tx.stop_transfer();
    }

    /// todo
    pub fn into_blocking(self) -> UhciSimple<'d, Blocking> {
        let internal = self.internal.into_blocking();
        UhciSimple::<'d, Blocking> { internal }
    }
}
