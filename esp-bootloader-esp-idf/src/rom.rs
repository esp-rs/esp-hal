pub struct Crc32 {}

impl Crc32 {
    pub fn new() -> Self {
        Self {}
    }

    pub fn crc(&self, data: &[u8]) -> u32 {
        extern "C" {
            fn esp_rom_crc32_le(crc: u32, buf: *const u8, len: u32) -> u32;
        }

        unsafe { esp_rom_crc32_le(0, data.as_ptr(), data.len() as u32) }
    }
}
