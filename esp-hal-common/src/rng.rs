use core::convert::Infallible;

use embedded_hal::blocking::rng::Read;

use crate::pac::RNG;

#[derive(Debug)]
pub struct Rng {
    rng: RNG,
}

impl Rng {
    pub fn new(rng: RNG) -> Self {
        Self { rng }
    }
}

impl Read for Rng {
    type Error = Infallible;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in buffer.chunks_mut(4) {
            let bytes = self.rng.data.read().bits().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }

        Ok(())
    }
}
