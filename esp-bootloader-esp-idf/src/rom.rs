pub struct Crc32 {}

impl Crc32 {
    pub fn new() -> Self {
        Self {}
    }

    pub fn crc(&self, data: &[u8]) -> u32 {
        esp_rom_sys::rom::crc::crc32_le(u32::MAX, data)
    }
}
