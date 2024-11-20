use crate::{
    aes::{Aes, Aes128, Aes256, AesFlavour, ALIGN_SIZE},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

impl Aes<'_> {
    pub(super) fn init(&mut self) {
        PeripheralClockControl::enable(PeripheralEnable::Aes);
        self.write_dma(false);
    }

    fn write_dma(&mut self, enable_dma: bool) {
        self.aes
            .dma_enable()
            .write(|w| w.dma_enable().bit(enable_dma));
    }

    pub(super) fn write_key(&mut self, key: &[u8]) {
        let key_len = self.aes.key_iter().count();
        debug_assert!(key.len() <= key_len * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
        self.alignment_helper
            .volatile_write_regset(self.aes.key(0).as_ptr(), key, key_len);
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        let text_in_len = self.aes.text_in_iter().count();
        debug_assert_eq!(block.len(), text_in_len * ALIGN_SIZE);
        self.alignment_helper.volatile_write_regset(
            self.aes.text_in(0).as_ptr(),
            block,
            text_in_len,
        );
    }

    pub(super) fn write_mode(&mut self, mode: u32) {
        self.aes.mode().write(|w| unsafe { w.bits(mode) });
    }

    pub(super) fn write_start(&mut self) {
        self.aes.trigger().write(|w| w.trigger().set_bit());
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.aes.state().read().state().bits() == 0
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        let text_out_len = self.aes.text_out_iter().count();
        debug_assert_eq!(block.len(), text_out_len * ALIGN_SIZE);
        self.alignment_helper.volatile_read_regset(
            self.aes.text_out(0).as_ptr(),
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
