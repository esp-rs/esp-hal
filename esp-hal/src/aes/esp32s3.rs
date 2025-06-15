use crate::aes::{ALIGN_SIZE, Aes, Aes128, Aes256, AesFlavour, Mode};

impl Aes<'_> {
    pub(super) fn init(&mut self) {
        self.write_dma(false);
    }

    fn write_dma(&mut self, enable_dma: bool) {
        self.regs()
            .dma_enable()
            .write(|w| w.dma_enable().bit(enable_dma));
    }

    pub(super) fn write_key(&mut self, key: &[u8]) {
        let key_len = self.regs().key_iter().count();
        debug_assert!(key.len() <= key_len * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
        self.alignment_helper
            .volatile_write_regset(self.regs().key(0).as_ptr(), key, key_len);
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        let text_in_len = self.regs().text_in_iter().count();
        debug_assert_eq!(block.len(), text_in_len * ALIGN_SIZE);
        self.alignment_helper.volatile_write_regset(
            self.regs().text_in(0).as_ptr(),
            block,
            text_in_len,
        );
    }

    pub(super) fn write_mode(&self, mode: Mode) {
        self.regs().mode().write(|w| unsafe { w.bits(mode as _) });
    }

    pub(super) fn write_start(&self) {
        self.regs().trigger().write(|w| w.trigger().set_bit());
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.regs().state().read().state().bits() == 0
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        let text_out_len = self.regs().text_out_iter().count();
        debug_assert_eq!(block.len(), text_out_len * ALIGN_SIZE);
        self.alignment_helper.volatile_read_regset(
            self.regs().text_out(0).as_ptr(),
            block,
            text_out_len,
        );
    }
}

impl AesFlavour for Aes128 {
    type KeyType<'b> = &'b [u8; 16];
}

impl AesFlavour for Aes256 {
    type KeyType<'b> = &'b [u8; 32];
}
