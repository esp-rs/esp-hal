use crc::{Algorithm, Crc};

static ALGO_CRC32_NORMAL: Algorithm<u32> = Algorithm {
    width: 32,
    poly: 0x04c11db7,
    init: 0,
    refin: true,
    refout: true,
    xorout: 0xffffffff,
    check: 0,
    residue: 0,
};

pub struct Crc32 {
    algo: Crc<u32>,
}

impl Crc32 {
    pub fn new() -> Self {
        Self {
            algo: Crc::<u32>::new(&ALGO_CRC32_NORMAL),
        }
    }

    pub fn crc(&self, data: &[u8]) -> u32 {
        let mut digest = self.algo.digest();
        digest.update(&data);
        digest.finalize()
    }
}
