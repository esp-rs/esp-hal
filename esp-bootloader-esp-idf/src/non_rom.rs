use crc::{Algorithm, Crc};
use md5::Digest;

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

pub struct Md5 {
    context: md5::Md5,
}

impl Md5 {
    pub fn new() -> Self {
        Self {
            context: md5::Md5::new(),
        }
    }

    pub fn update(&mut self, data: &[u8]) {
        self.context.update(data);
    }

    pub fn finalize(self) -> [u8; 16] {
        let digest = self.context.finalize();
        let mut hash = [0; 16];
        hash.copy_from_slice(&digest);
        hash
    }
}
