use crate::{
    aes::{Aes, Aes128, Aes192, Aes256, AesFlavour, Endianness, ALIGN_SIZE},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

impl<'d> Aes<'d> {
    pub(super) fn init(&mut self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(PeripheralEnable::Aes);
        self.write_endianness(
            Endianness::BigEndian,
            Endianness::BigEndian,
            Endianness::BigEndian,
            Endianness::BigEndian,
            Endianness::BigEndian,
            Endianness::BigEndian,
        );
    }

    pub(super) fn write_key(&mut self, key: &[u8]) {
        debug_assert!(key.len() <= self.aes.key_.len() * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
        Self::write_to_regset(key, self.aes.key_.len(), &mut self.aes.key_[0]);
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        debug_assert_eq!(block.len(), self.aes.text_.len() * ALIGN_SIZE);
        Self::write_to_regset(block, self.aes.text_.len(), &mut self.aes.text_[0]);
    }

    pub(super) fn write_mode(&mut self, mode: u32) {
        Self::write_to_register(&mut self.aes.mode, mode);
    }

    /// Configures how the state matrix would be laid out
    pub fn write_endianness(
        &mut self,
        input_text_word_endianess: Endianness,
        input_text_byte_endianess: Endianness,
        output_text_word_endianess: Endianness,
        output_text_byte_endianess: Endianness,
        key_word_endianess: Endianness,
        key_byte_endianess: Endianness,
    ) {
        let mut to_write = 0_u32;
        to_write |= key_byte_endianess as u32;
        to_write |= (key_word_endianess as u32) << 1;
        to_write |= (input_text_byte_endianess as u32) << 2;
        to_write |= (input_text_word_endianess as u32) << 3;
        to_write |= (output_text_byte_endianess as u32) << 4;
        to_write |= (output_text_word_endianess as u32) << 5;
        Self::write_to_register(&mut self.aes.endian, to_write);
    }

    pub(super) fn write_start(&mut self) {
        self.aes.start.write(|w| w.start().set_bit())
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.aes.idle.read().idle().bit_is_set()
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        debug_assert_eq!(block.len(), self.aes.text_.len() * ALIGN_SIZE);
        Self::read_from_regset(block, self.aes.text_.len(), &self.aes.text_[0]);
    }
}

impl AesFlavour for Aes128 {
    type KeyType<'b> = &'b [u8; 16];
    const ENCRYPT_MODE: u32 = 0;
    const DECRYPT_MODE: u32 = 4;
}

impl AesFlavour for Aes192 {
    type KeyType<'b> = &'b [u8; 24];
    const ENCRYPT_MODE: u32 = 1;
    const DECRYPT_MODE: u32 = 5;
}

impl AesFlavour for Aes256 {
    type KeyType<'b> = &'b [u8; 32];
    const ENCRYPT_MODE: u32 = 2;
    const DECRYPT_MODE: u32 = 6;
}
