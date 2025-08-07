use esp32c6::uhci0;

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
    uart,
    uart::{
        Uart,
        uhci,
        uhci::{AnyUhci, UhciInternal},
    },
};

/// "Simple" Uhci implementation with simple Api, it's purpuse is to only (and easily) replace Uart
/// in an application and improve upon it by using DMA under the hood
/// It's api (read, write) is designed to be easily wrapped upon by a struct which will implement
/// embedded_io traits If you simply want better uart (speeds, performance, anything) you should
/// probably just use this
pub struct UhciSimple<'d, Dm>
where
    Dm: DriverMode,
{
    pub(crate) internal: UhciInternal<'d, Dm>,
}

impl<'d> UhciSimple<'d, Blocking> {
    /// Creates a new instance of UhciSimple
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

        let simple = Self {
            internal: self_uhci,
        };
        simple.init();

        simple
    }

    fn init(&self) {
        // This should be like that and never be changed (for just using as a uart wrapper)
        let reg: &uhci0::RegisterBlock = self.internal.uhci.give_uhci().register_block();

        reg.conf0().modify(|_, w| w.uart_idle_eof_en().set_bit());

        reg.conf0().modify(|_, w| w.len_eof_en().set_bit());
    }

    /// Reads from UART into the DMA buffer
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

    /// Writes from DMA buffer into UART
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

    /// Create a new instance in [crate::Async] mode.
    pub fn into_async(self) -> UhciSimple<'d, Async> {
        let internal = self.internal.into_async();
        UhciSimple { internal }
    }
}

impl<'d> UhciSimple<'d, Async> {
    /// Reads from UART into the DMA buffer
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

    /// Writes from DMA buffer into UART
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

    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> UhciSimple<'d, Blocking> {
        let internal = self.internal.into_blocking();
        UhciSimple::<'d, Blocking> { internal }
    }
}

impl<'d, Dm: DriverMode> UhciSimple<'d, Dm> {
    /// Sets the UART config for the consumer earlier uart
    pub fn set_uart_config(&mut self, uart_config: &uart::Config) -> Result<(), uart::ConfigError> {
        self.internal.set_uart_config(uart_config)
    }

    /// The limit of how much to read in a single read call. It cannot be higher than the dma
    /// buffer size, otherwise uart/dma/uhci will freeze. It cannot exceed 4095 (12 bits), above
    /// this value it will simply also split the readings
    #[allow(dead_code)]
    pub fn set_chunk_limit(&self, limit: u16) -> Result<(), uhci::Error> {
        self.internal.set_chunk_limit(limit)
    }
}
