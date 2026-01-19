use core::ptr::NonNull;
use crate::peripherals::ETH;
use crate::soc::volatile::Volatile;

const ETH_BASE: usize = 0x3FF6_9000;

#[repr(C)]
pub struct EthRegisters {
    pub mac_conf: Volatile<u32>,        // 0x000
    pub mac_addr0: Volatile<u32>,        // 0x004
    pub mac_addr1: Volatile<u32>,        // 0x008
    _r0: [u32; 9],                       // 0x00C..0x02C
    pub mac_mii_addr: Volatile<u32>,     // 0x030
    pub mac_mii_data: Volatile<u32>,     // 0x034
    _r1: [u32; 3],                       // 0x038..0x044
    pub dma_bus_mode: Volatile<u32>,     // 0x048
    pub dma_tx_poll: Volatile<u32>,      // 0x04C
    pub dma_rx_poll: Volatile<u32>,      // 0x050
    pub dma_rx_desc: Volatile<u32>,      // 0x054
    pub dma_tx_desc: Volatile<u32>,      // 0x058
    pub dma_status: Volatile<u32>,       // 0x05C
    pub dma_conf: Volatile<u32>,         // 0x060
}

pub struct EthRegBlock {
    ptr: NonNull<EthRegisters>,
}

impl EthRegBlock {
    pub fn new(_: ETH) -> Self {
        Self {
            ptr: unsafe { NonNull::new_unchecked(ETH_BASE as *mut EthRegisters) },
        }
    }

    #[inline(always)]
    pub fn regs_mut(&mut self) -> &mut EthRegisters {
        unsafe { self.ptr.as_mut() }
    }
}

impl EthRegisters {
    pub fn reset_dma(&mut self) {
        const SWR: u32 = 1 << 0;
        self.dma_bus_mode.write(SWR);
        while self.dma_bus_mode.read() & SWR != 0 {}
    }

    pub fn enable_dma(&mut self) {
        const RX_EN: u32 = 1 << 1;
        const TX_EN: u32 = 1 << 13;
        self.dma_conf.modify(|v| v | RX_EN | TX_EN);
    }

    pub fn enable_mac(&mut self) {
        const RX: u32 = 1 << 2;
        const TX: u32 = 1 << 3;
        self.mac_conf.modify(|v| v | RX | TX);
    }

    pub fn set_mac_address(&mut self, mac: [u8; 6]) {
        let low =
            (mac[3] as u32) << 24 |
            (mac[2] as u32) << 16 |
            (mac[1] as u32) << 8  |
            (mac[0] as u32);

        let high =
            (mac[5] as u32) << 8 |
            (mac[4] as u32);

        self.mac_addr0.write(low);
        self.mac_addr1.write(high);
    }

    pub fn start_rx(&mut self) {
        self.dma_rx_poll.write(1);
    }

    pub fn start_tx(&mut self) {
        self.dma_tx_poll.write(1);
    }

    pub fn mdio_write(&mut self, phy: u8, reg: u8, val: u16) {
        const MII_BUSY: u32 = 1 << 0;
        const MII_WRITE: u32 = 1 << 1;

        self.mac_mii_data.write(val as u32);

        self.mac_mii_addr.write(
            (phy as u32) << 11 |
            (reg as u32) << 6 |
            MII_WRITE |
            MII_BUSY
        );

        while self.mac_mii_addr.read() & MII_BUSY != 0 {}
    }

    pub fn mdio_read(&mut self, phy: u8, reg: u8) -> u16 {
        const MII_BUSY: u32 = 1 << 0;

        self.mac_mii_addr.write(
            (phy as u32) << 11 |
            (reg as u32) << 6 |
            MII_BUSY
        );

        while self.mac_mii_addr.read() & MII_BUSY != 0 {}

        self.mac_mii_data.read() as u16
    }
}
