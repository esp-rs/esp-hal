use crate::{
    aes::{Aes, Aes128, Aes256, AesFlavour, ALIGN_SIZE},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

impl<'d> Aes<'d> {
    pub(super) fn init(&mut self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(PeripheralEnable::Aes);
        self.write_dma(false);
    }

    fn write_dma(&mut self, enable_dma: bool) {
        match enable_dma {
            true => self.aes.dma_enable.write(|w| w.dma_enable().set_bit()),
            false => self.aes.dma_enable.write(|w| w.dma_enable().clear_bit()),
        }
    }

    pub(super) fn write_key(&mut self, key: &[u8]) {
        debug_assert!(key.len() <= 8 * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
        Self::write_to_regset(key, 8, &mut self.aes.key_0);
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
        Self::write_to_regset(block, 4, &mut self.aes.text_in_0);
    }

    pub(super) fn write_mode(&mut self, mode: u32) {
        Self::write_to_register(&mut self.aes.mode, mode);
    }

    pub(super) fn write_start(&mut self) {
        self.aes.trigger.write(|w| w.trigger().set_bit())
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.aes.state.read().state().bits() == 0
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
        Self::read_from_regset(block, 4, &self.aes.text_out_0);
    }
}

impl AesFlavour for Aes128 {
    type KeyType<'b> = &'b [u8; 16];
    const ENCRYPT_MODE: u32 = 0;
    const DECRYPT_MODE: u32 = 4;
}

impl AesFlavour for Aes256 {
    type KeyType<'b> = &'b [u8; 32];
    const ENCRYPT_MODE: u32 = 2;
    const DECRYPT_MODE: u32 = 6;
}
