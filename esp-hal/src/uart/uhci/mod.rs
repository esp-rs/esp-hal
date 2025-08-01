/// todo
pub mod simple;
/// todo
pub mod normal;

use embassy_embedded_hal::SetConfig;

use crate::{
    dma::{
        Channel,
        DmaEligible,
        PeripheralDmaChannel,
    }, peripherals, uart::{self, uhci::Error::*, Uart}, Async, Blocking, DriverMode
};

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
pub struct UhciInternal<'d, Dm>
where
    Dm: DriverMode,
{
    uart: Uart<'d, Dm>,
    uhci: AnyUhci<'static>,
    channel: Channel<Dm, PeripheralDmaChannel<AnyUhci<'d>>>,
}

impl<'d, Dm> UhciInternal<'d, Dm>
where
    Dm: DriverMode,
{
    pub(crate) fn init(&self) {
        self.clean_turn_on();
        self.reset();
        self.configure();
    }

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
    fn configure(&self) {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();

        // Idk if there is a better way to check it, but it works
        match &self.uart.tx.uart.0 {
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
    #[allow(dead_code)]
    pub fn chunk_limit(&self, limit: usize) -> Result<(), Error> {
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

    /// todo
    pub fn set_uart_config(&mut self, uart_config: &uart::Config) -> Result<(), uart::ConfigError> {
        self.uart.set_config(uart_config)
    }
}

impl<'d> UhciInternal<'d, Blocking> {
    /// todo
    pub(crate) fn into_async(self) -> UhciInternal<'d, Async> {
        UhciInternal {
            uart: self.uart.into_async(),
            uhci: self.uhci,
            channel: self.channel.into_async(),
        }
    }
}

impl<'d> UhciInternal<'d, Async> {
    /// todo
    pub(crate) fn into_blocking(self) -> UhciInternal<'d, Blocking> {
        UhciInternal {
            uart: self.uart.into_blocking(),
            uhci: self.uhci,
            channel: self.channel.into_blocking(),
        }
    }
}