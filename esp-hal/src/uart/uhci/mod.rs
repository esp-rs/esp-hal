/// "Normal" Uhci implementation, which implements regular dma transfers, can be expanded upon in
/// the future with Uhci specific features
pub mod normal;
/// "Simple" Uhci implementation with simple Api, it's purpuse is to only (and easily) replace Uart
/// in an application and improve upon it by using DMA under the hood
pub mod simple;

use embassy_embedded_hal::SetConfig;
use esp32c6::uhci0;

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{Channel, DmaEligible, PeripheralDmaChannel},
    peripherals,
    uart::{self, Uart, uhci::Error::*},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// Uhci specific errors
pub enum Error {
    /// set_chunk_limit() argument is above what's possible by the hardware. It cannot exceed 4095
    /// (12 bits), above this value it will simply also split the readings
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
    /// Unwraps the enum into the peripheral below
    pub fn give_uhci(&self) -> &peripherals::UHCI0<'_> {
        match &self.0 {
            any::Inner::Uhci0(x) => x,
        }
    }
}

/// Base, internal struct for Uhci containing functions, variables for both Simple (UhciSimple) and
/// Normal (Uhci) implementations
pub(crate) struct UhciInternal<'d, Dm>
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
        self.conf_uart();
    }

    fn clean_turn_on(&self) {
        // General conf registers
        let reg: &uhci0::RegisterBlock = &self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.clk_en().set_bit());
        reg.conf0().modify(|_, w| {
            unsafe { w.bits(0) };
            w.clk_en().set_bit()
        });
        reg.conf1().modify(|_, w| unsafe { w.bits(0) });

        // For TX
        reg.escape_conf().modify(|_, w| unsafe { w.bits(0) });
    }

    fn reset(&self) {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.rx_rst().set_bit());
        reg.conf0().modify(|_, w| w.rx_rst().clear_bit());

        reg.conf0().modify(|_, w| w.tx_rst().set_bit());
        reg.conf0().modify(|_, w| w.tx_rst().clear_bit());
    }

    fn conf_uart(&self) {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();

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
    }

    #[allow(dead_code)]
    pub(crate) fn set_chunk_limit(&self, limit: u16) -> Result<(), Error> {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        // let val = reg.pkt_thres().read().pkt_thrs().bits();
        // info!("Read limit value: {} to set: {}", val, limit);

        // limit is 12 bits
        // Above this value, it will probably split the messages, anyway, the point is below (the
        // dma buffer length) it it will not freeze itself
        if limit > 4095 {
            return Err(AboveReadLimit);
        }

        reg.pkt_thres().write(|w| unsafe { w.bits(limit as u32) });
        Ok(())
    }

    #[allow(dead_code)]
    pub(crate) fn set_uart_config(
        &mut self,
        uart_config: &uart::Config,
    ) -> Result<(), uart::ConfigError> {
        self.uart.set_config(uart_config)
    }

    pub(crate) fn turn_off(&mut self) {
        info!("Running uhci internal drop!");
        self.clean_turn_on();
        self.reset();

        // Turning off the clock should be enough?
        let reg: &uhci0::RegisterBlock = &self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.clk_en().clear_bit());
    }
}

impl<'d> UhciInternal<'d, Blocking> {
    /// Create a new instance in [crate::Async] mode.
    pub(crate) fn into_async(self) -> UhciInternal<'d, Async> {
        UhciInternal {
            uart: self.uart.into_async(),
            uhci: self.uhci,
            channel: self.channel.into_async(),
        }
    }
}

impl<'d> UhciInternal<'d, Async> {
    /// Create a new instance in [crate::Blocking] mode.
    pub(crate) fn into_blocking(self) -> UhciInternal<'d, Blocking> {
        UhciInternal {
            uart: self.uart.into_blocking(),
            uhci: self.uhci,
            channel: self.channel.into_blocking(),
        }
    }
}
