use crate::soc::volatile::Volatile;
use crate::peripherals::ETH;
use core::ptr::NonNull;

#[repr(C)]
pub struct EthRegisters {
    pub mac_conf: Volatile<u32>,
    pub dma_conf: Volatile<u32>,
    pub dma_rx_desc: Volatile<u32>,
    pub dma_tx_desc: Volatile<u32>,
}

pub struct EthRegBlock {
    ptr: NonNull<EthRegisters>,
}

impl EthRegBlock {
    pub fn new(_: ETH) -> Self {
        Self {
            ptr: unsafe { NonNull::new_unchecked(0x6000_0000 as *mut _) },
        }
    }

    pub fn regs_mut(&mut self) -> &mut EthRegisters {
        unsafe { self.ptr.as_mut() }
    }
}
