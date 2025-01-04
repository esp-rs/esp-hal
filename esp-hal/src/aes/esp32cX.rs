use crate::aes::{Aes, Aes128, Aes256, AesFlavour, Mode, ALIGN_SIZE};

impl Aes<'_> {
    pub(super) fn init(&mut self) {
        self.write_dma(false);
    }

    fn write_dma(&mut self, enable_dma: bool) {
        match enable_dma {
            true => self.aes.dma_enable().write(|w| w.dma_enable().set_bit()),
            false => self.aes.dma_enable().write(|w| w.dma_enable().clear_bit()),
        };
    }

    pub(super) fn write_key(&mut self, key: &[u8]) {
        debug_assert!(key.len() <= 8 * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
        self.alignment_helper
            .volatile_write_regset(self.aes.key(0).as_ptr(), key, 8);
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
        self.alignment_helper
            .volatile_write_regset(self.aes.text_in(0).as_ptr(), block, 4);
    }

    pub(super) fn write_mode(&self, mode: Mode) {
        self.aes.mode().write(|w| unsafe { w.bits(mode as _) });
    }

    pub(super) fn write_start(&self) {
        self.aes.trigger().write(|w| w.trigger().set_bit());
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.aes.state().read().state().bits() == 0
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
        self.alignment_helper
            .volatile_read_regset(self.aes.text_out(0).as_ptr(), block, 4);
    }
}

impl AesFlavour for Aes128 {
    type KeyType<'b> = &'b [u8; 16];
}

impl AesFlavour for Aes256 {
    type KeyType<'b> = &'b [u8; 32];
}
