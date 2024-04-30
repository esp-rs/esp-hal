use crate::{
    aes::{Aes, Endianness, ALIGN_SIZE},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

impl<'d> Aes<'d> {
    pub(super) fn init(&mut self) {
        PeripheralClockControl::enable(PeripheralEnable::Aes);
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
        let key_len = self.aes.key_iter().count();
        debug_assert!(key.len() <= key_len * ALIGN_SIZE);
        debug_assert_eq!(key.len() % ALIGN_SIZE, 0);

        if !key.is_empty() {
            for (v, reg) in key.chunks_exact(ALIGN_SIZE).zip(self.aes.key_iter()) {
                reg.write(|w| unsafe { w.bits(u32::from_ne_bytes(v.try_into().unwrap())) })
            }
        }
    }

    pub(super) fn write_block(&mut self, block: &[u8]) {
        let text_len = self.aes.text_iter().count();
        debug_assert_eq!(block.len(), text_len * ALIGN_SIZE);

        if !block.is_empty() {
            for (v, reg) in block.chunks_exact(ALIGN_SIZE).zip(self.aes.text_iter()) {
                reg.write(|w| unsafe { w.bits(u32::from_ne_bytes(v.try_into().unwrap())) })
            }
        }
    }

    pub(super) fn write_mode(&mut self, mode: u32) {
        self.aes.mode().write(|w| unsafe { w.bits(mode) });
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
        self.aes.endian().write(|w| unsafe { w.bits(to_write) });
    }

    pub(super) fn write_start(&mut self) {
        self.aes.start().write(|w| w.start().set_bit())
    }

    pub(super) fn read_idle(&mut self) -> bool {
        self.aes.idle().read().idle().bit_is_set()
    }

    pub(super) fn read_block(&self, block: &mut [u8]) {
        let text_len = self.aes.text_iter().count();
        debug_assert_eq!(block.len(), text_len * ALIGN_SIZE);

        for (chunk, reg) in block.chunks_exact_mut(ALIGN_SIZE).zip(self.aes.text_iter()) {
            let read_val: [u8; 4] = reg.read().bits().to_ne_bytes();
            chunk.copy_from_slice(&read_val);
        }
    }
}
