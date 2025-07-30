use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        AnyGdmaChannel,
        Channel,
        DmaChannel,
        DmaChannelFor,
        DmaEligible,
        DmaRxBuf,
        DmaRxBuffer,
        DmaTxBuf,
        DmaTxBuffer,
        PeripheralDmaChannel,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    peripherals,
    uart::Uart,
};
use crate::uart::uhci::Error::*;

/// todo
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// todo
    AboveReadLimit,
}

crate::any_peripheral! {
    pub peripheral AnyUhci<'d> {
        Uhci0(crate::peripherals::UHCI0<'d>),
    }
}

impl<'d> DmaEligible for AnyUhci<'d> {
    #[cfg(gdma)]
    type Dma = crate::dma::AnyGdmaChannel<'d>;

    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        match &self.0 {
            any::Inner::Uhci0(_) => crate::dma::DmaPeripheral::Uhci0,
        }
    }
}

impl AnyUhci<'_> {
    /// todo
    pub fn give_uhci(&self) -> &peripherals::UHCI0<'_> {
        match &self.0 {
            any::Inner::Uhci0(x) => x,
        }
    }
}

/// todo
pub struct UhciPer<'d, Dm>
where
    Dm: DriverMode,
{
    _uart: Uart<'d, Dm>,
    uhci: AnyUhci<'static>,
    channel: Channel<Dm, PeripheralDmaChannel<AnyUhci<'d>>>,
}

impl<'d, Dm> UhciPer<'d, Dm>
where
    Dm: DriverMode,
{
    // uhci_ll_enable_bus_clock
    fn clean_turn_on(&self) {
        let reg: &esp32c6::uhci0::RegisterBlock = &self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.clk_en().set_bit());
        reg.conf0().modify(|_, w| {
            unsafe { w.bits(0) };
            w.clk_en().set_bit()
        });
        reg.conf1().modify(|_, w| unsafe { w.bits(0) });

        // For TX
        reg.escape_conf().modify(|_, w| unsafe { w.bits(0) });
    }

    // Via UHCI_RX_RST
    fn reset(&self) {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.rx_rst().set_bit());
        reg.conf0().modify(|_, w| w.rx_rst().clear_bit());
        // Tx, unsure
        reg.conf0().modify(|_, w| w.tx_rst().set_bit());
        reg.conf0().modify(|_, w| w.tx_rst().clear_bit());
    }

    /// todo
    pub fn configure(&mut self) {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();

        // Idk if there is a better way to check it, but it works
        match &self._uart.tx.uart.0 {
            super::any::Inner::Uart0(_) => {
                info!("Uhci will use uart0");
                reg.conf0().modify(|_, w| w.uart0_ce().set_bit());
            }
            super::any::Inner::Uart1(_) => {
                info!("Uhci will use uart1");
                reg.conf0().modify(|_, w| w.uart1_ce().set_bit());
            }
        }

        // If you plan to support more UHCI features, this needs to be configurable
        reg.conf0().modify(|_, w| w.uart_idle_eof_en().set_bit());

        // If you plan to support more UHCI features, this needs to be configurable
        reg.conf0().modify(|_, w| w.len_eof_en().set_bit());
    }

    /// todo
    pub fn read_limit(&mut self, mut limit: usize) -> Result<(), Error> {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        // let val = reg.pkt_thres().read().pkt_thrs().bits();
        // info!("Read limit value: {} to set: {}", val, limit);

        // limit is 12 bits
        // Above this value, it will probably split the messages, anyway, the point is below it
        // it will not freeze itself
        if limit > 4095 {
            return Err(AboveReadLimit);
        }

        reg.pkt_thres().write(|w| unsafe { w.bits(limit as u32) });
        Ok(())
    }
}

impl<'d> UhciPer<'d, Blocking> {
    /// todo
    pub fn new(
        uart: Uart<'d, Blocking>,
        uhci: peripherals::UHCI0<'static>,
        channel: impl DmaChannelFor<AnyUhci<'d>>,
    ) -> Self {
        let channel = Channel::new(channel.degrade());
        channel.runtime_ensure_compatible(&uhci);

        let self_uhci = Self {
            _uart: uart,
            uhci: uhci.into(),
            channel,
        };

        self_uhci.clean_turn_on();
        self_uhci.reset();

        self_uhci
    }

    /// todo
    pub fn read(&mut self, rx_buffer: &mut impl DmaRxBuffer) {
        unsafe {
            self.channel
                .rx
                .prepare_transfer(self.uhci.dma_peripheral(), rx_buffer)
                .unwrap()
        };

        self.channel.rx.start_transfer().unwrap();

        // info!("Is done: {}, ", self.channel.rx.is_done());

        // Based on spi slave dma, is this a good idea? infinite loop to wait for something?
        while !self.channel.rx.is_done() {}

        // info!("Is done: {}, ", self.channel.rx.is_done());

        self.channel.rx.stop_transfer();
    }

    /// todo
    pub fn write(&mut self, tx_buffer: &mut impl DmaTxBuffer) {
        // info!("tx_buffer.len() is: {}", tx_buffer.len()); // Nope

        unsafe {
            self.channel
                .tx
                .prepare_transfer(self.uhci.dma_peripheral(), tx_buffer)
                .unwrap()
        };

        self.channel.tx.start_transfer().unwrap();

        while !self.channel.tx.is_done() {}

        self.channel.tx.stop_transfer();
    }

    /// todo
    pub fn into_async(self) -> UhciPer<'d, Async> {
        UhciPer {
            _uart: self._uart.into_async(),
            uhci: self.uhci,
            channel: self.channel.into_async(),
        }
    }
}

impl<'d> UhciPer<'d, Async> {
    /// todo
    pub async fn read(&mut self, rx_buffer: &mut impl DmaRxBuffer) {
        let dma_future = DmaRxFuture::new(&mut self.channel.rx);

        unsafe {
            dma_future
                .rx
                .prepare_transfer(self.uhci.dma_peripheral(), rx_buffer)
                .unwrap()
        };

        dma_future.rx.start_transfer().unwrap();

        dma_future.await.unwrap();

        self.channel.rx.stop_transfer();
    }

    /// todo
    pub async fn write(&mut self, tx_buffer: &mut impl DmaTxBuffer) {
        let dma_future = DmaTxFuture::new(&mut self.channel.tx);

        // info!("tx_buffer.len() is: {}", tx_buffer.len()); // Nope

        unsafe {
            dma_future
                .tx
                .prepare_transfer(self.uhci.dma_peripheral(), tx_buffer)
                .unwrap()
        };

        dma_future.tx.start_transfer().unwrap();

        dma_future.await.unwrap();

        self.channel.tx.stop_transfer();
    }
}
