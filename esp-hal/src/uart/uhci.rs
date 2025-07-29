// pub struct UhciPer<'d, Dm> where
// Dm: DriverMode, {
// uhci: esp_hal::peripherals::UHCI0<'static>,
// dma_channel: Channel<Dm, PeripheralDmaChannel<AnySpi<'d>>>,
// }

use crate::{
    Blocking,
    DriverMode,
    dma::{
        Channel,
        DmaChannelFor,
        DmaEligible,
        DmaRxBuf,
        DmaTxBuf,
        PeripheralDmaChannel,
    },
    peripherals,
    uart::Uart,
};

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
    _uart: Uart<'d, Blocking>,
    uhci: AnyUhci<'static>,
    channel: Channel<Dm, PeripheralDmaChannel<AnyUhci<'d>>>,
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

    /// todo
    pub fn configure(&mut self, buffer_rx: &mut DmaRxBuf, buffer_tx: &mut DmaTxBuf) {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.uart1_ce().set_bit());
        // We should do UHCI_UART_IDLE_EOF_EN too
        reg.conf0().modify(|_, w| w.uart_idle_eof_en().set_bit());

        // Also UHCI_LEN_EOF_EN, this shouldn't be needed, in theory
        reg.conf0().modify(|_, w| w.len_eof_en().set_bit());

        // Dma stuff
        // https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
        // Page 134
        unsafe {
            self.channel
                .rx
                .prepare_transfer(self.uhci.dma_peripheral(), buffer_rx)
                .unwrap()
        };
        // self.channel.rx.start_transfer().unwrap();

        // Tx
        unsafe {
            self.channel
                .tx
                .prepare_transfer(self.uhci.dma_peripheral(), buffer_tx)
                .unwrap()
        };
        // self.channel.tx.start_transfer().unwrap();
    }

    /// todo
    pub fn start_transfer_tx(&mut self) {
        self.channel.tx.start_transfer().unwrap();
    }

    /// todo
    pub fn start_transfer_rx(&mut self) {
        self.channel.rx.start_transfer().unwrap();
    }

    /// todo
    pub fn stop_transfer_rx(&mut self) {
        self.channel.rx.stop_transfer();
    }

    /// todo
    pub fn stop_transfer_tx(&mut self) {
        self.channel.tx.stop_transfer();
    }

    // Via UHCI_RX_RST
    fn reset(&self) {
        let reg: &esp32c6::uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.rx_rst().set_bit());
        reg.conf0().modify(|_, w| w.rx_rst().clear_bit());
    }
}
