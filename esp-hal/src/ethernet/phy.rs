use super::types::EthRegisters;

pub trait Phy {
    fn init(&mut self, regs: &mut EthRegisters);
    fn link_up(&self, regs: &mut EthRegisters) -> bool;
}

pub struct Lan8720 {
    phy_addr: u8,
}

impl Lan8720 {
    pub fn new(addr: u8) -> Self {
        Self { phy_addr: addr }
    }
}

impl Phy for Lan8720 {
    fn init(&mut self, regs: &mut EthRegisters) {
        const BMCR_RESET: u16 = 1 << 15;

        regs.mdio_write(self.phy_addr, 0, BMCR_RESET);
        while regs.mdio_read(self.phy_addr, 0) & BMCR_RESET != 0 {}

        loop {
            let bmsr = regs.mdio_read(self.phy_addr, 1);
            if bmsr & (1 << 2) != 0 {
                break;
            }
        }
    }

    fn link_up(&self, regs: &mut EthRegisters) -> bool {
        let bmsr = regs.mdio_read(self.phy_addr, 1);
        bmsr & (1 << 2) != 0
    }
}
