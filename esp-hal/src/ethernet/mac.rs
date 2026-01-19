use super::types::EthRegBlock;
use super::dma::DmaRing;
use super::phy::Phy;
use crate::peripherals::ETH;

pub struct Ethernet<'a, P: Phy, const RX: usize, const TX: usize> {
    regs: EthRegBlock,
    phy: P,
    rx: &'a mut DmaRing<RX>,
    tx: &'a mut DmaRing<TX>,
}

impl<'a, P: Phy, const RX: usize, const TX: usize> Ethernet<'a, P, RX, TX> {
    pub fn new(
        eth: ETH,
        phy: P,
        rx: &'a mut DmaRing<RX>,
        tx: &'a mut DmaRing<TX>,
    ) -> Self {
        Self {
            regs: EthRegBlock::new(eth),
            phy,
            rx,
            tx,
        }
    }

    pub fn init(&mut self, mac: [u8; 6]) {
        self.rx.init(true);
        self.tx.init(false);

        let regs = self.regs.regs_mut();

        regs.reset_dma();

        regs.dma_rx_desc.write(self.rx.desc_ptr() as u32);
        regs.dma_tx_desc.write(self.tx.desc_ptr() as u32);

        self.phy.init(regs);
        while !self.phy.link_up(regs) {}

        regs.set_mac_address(mac);
        regs.enable_mac();
        regs.enable_dma();

        regs.start_rx();
        regs.start_tx();
    }

    pub fn send(&mut self, data: &[u8]) -> bool {
        let ok = self.tx.prepare_tx(data);
        if ok {
            self.regs.regs_mut().start_tx();
        }
        ok
    }

    pub fn poll(&mut self) -> Option<&[u8]> {
        self.rx.try_recv()
    }
}
