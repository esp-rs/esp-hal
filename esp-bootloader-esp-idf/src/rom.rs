use esp_rom_sys::rom::crc;

pub struct Crc32 {}

impl Crc32 {
    pub fn new() -> Self {
        Self {}
    }

    pub fn crc(&self, data: &[u8]) -> u32 {
        crc::crc32_le(u32::MAX, data)
    }
}

#[cfg(feature = "validation")]
pub struct Md5 {
    context: esp_rom_sys::rom::md5::Context,
}

#[cfg(feature = "validation")]
impl Md5 {
    pub fn new() -> Self {
        Self {
            context: esp_rom_sys::rom::md5::Context::new(),
        }
    }

    pub fn update(&mut self, data: &[u8]) {
        self.context.consume(data);
    }

    pub fn finalize(self) -> [u8; 16] {
        self.context.compute().0
    }
}
