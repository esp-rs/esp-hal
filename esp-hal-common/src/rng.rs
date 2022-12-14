//! Random number generator driver

use core::convert::Infallible;

use embedded_hal::blocking::rng::Read;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::RNG,
};

/// Random Number Generator
///
/// It should be noted that there are certain pre-conditions which must be met
/// in order for the RNG to produce *true* random numbers. The hardware RNG
/// produces true random numbers under any of the following conditions:
///
/// - RF subsystem is enabled (i.e. Wi-Fi or Bluetooth are enabled).
/// - An internal entropy source has been enabled by calling
///   `bootloader_random_enable()` and not yet disabled by calling
///   `bootloader_random_disable()`.
/// - While the ESP-IDF Second stage bootloader is running. This is because the
///   default ESP-IDF bootloader implementation calls
///   `bootloader_random_enable()` when the bootloader starts, and
///   `bootloader_random_disable()` before executing the app.
///
/// When any of these conditions are true, samples of physical noise are
/// continuously mixed into the internal hardware RNG state to provide entropy.
/// If none of the above conditions are true, the output of the RNG should be
/// considered pseudo-random only.
///
/// For more information, please refer to the ESP-IDF documentation:
/// <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/random.html>
pub struct Rng<'d> {
    rng: PeripheralRef<'d, RNG>,
}

impl<'d> Rng<'d> {
    /// Create a new random number generator instance
    pub fn new(rng: impl Peripheral<P = RNG> + 'd) -> Self {
        crate::into_ref!(rng);

        Self { rng }
    }

    #[inline]
    /// Reads currently available `u32` integer from `RNG`
    pub fn random(&mut self) -> u32 {
        self.rng.data.read().bits()
    }
}

impl Read for Rng<'_> {
    type Error = Infallible;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in buffer.chunks_mut(4) {
            let bytes = self.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }

        Ok(())
    }
}
