//! # Random Number Generator
//!
//! ## Overview
//! The Random Number Generator (RNG) Driver for ESP chips is a software module
//! that provides an interface to generate random numbers using the RNG
//! peripheral on ESP chips. This driver allows you to generate random numbers
//! that can be used for various cryptographic, security, or general-purpose
//! applications.
//!
//! The RNG peripheral on ESP chips produces random numbers based on physical
//! noise sources, which provide true random numbers under specific conditions
//! (see conditions below).
//!
//! To use the [Rng] Driver, you need to initialize it with the RNG peripheral.
//! Once initialized, you can generate random numbers by calling the `random`
//! method, which returns a 32-bit unsigned integer.
//!
//! Additionally, this driver implements the
//! [Read](embedded_hal_02::blocking::rng::Read) trait from the `embedded_hal`
//! crate, allowing you to generate random bytes by calling the `read` method.
//
//! # Important Note
//!
//! There are certain pre-conditions which must be met in order for the RNG to
//! produce *true* random numbers. The hardware RNG produces true random numbers
//! under any of the following conditions:
//!
//! - RF subsystem is enabled (i.e. Wi-Fi or Bluetooth are enabled).
//! - An internal entropy source has been enabled by calling
//!   `bootloader_random_enable()` and not yet disabled by calling
//!   `bootloader_random_disable()`.
//! - While the ESP-IDF Second stage bootloader is running. This is because the
//!   default ESP-IDF bootloader implementation calls
//!   `bootloader_random_enable()` when the bootloader starts, and
//!   `bootloader_random_disable()` before executing the app.
//!
//! When any of these conditions are true, samples of physical noise are
//! continuously mixed into the internal hardware RNG state to provide entropy.
//! If none of the above conditions are true, the output of the RNG should be
//! considered pseudo-random only.
//!
//! For more information, please refer to the
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/system/random.html)")]
//! # Examples
//!
//! ## Initialization
//!
//! ```no_run
//! let mut rng = Rng::new(peripherals.RNG);
//! ```
//!
//! ## Generate a random word (u32)
//!
//! ```no_run
//! let random: u32 = rng.random();
//! ```
//!
//! ## Fill a buffer of arbitrary size with random bytes
//!
//! ```no_run
//! let mut buffer = [0u8; 32];
//! rng.read(&mut buffer).unwrap();
//! ```

use core::marker::PhantomData;

use crate::{peripheral::Peripheral, peripherals::RNG};

/// Random number generator driver
#[derive(Clone, Copy)]
pub struct Rng {
    _phantom: PhantomData<RNG>,
}

impl Rng {
    /// Create a new random number generator instance
    pub fn new(_rng: impl Peripheral<P = RNG>) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    #[inline]
    /// Reads currently available `u32` integer from `RNG`
    pub fn random(&mut self) -> u32 {
        // SAFETY: read-only register access
        unsafe { &*crate::peripherals::RNG::PTR }
            .data()
            .read()
            .bits()
    }

    #[inline]
    /// Reads enough bytes from hardware random number generator to fill
    /// `buffer`.
    ///
    /// If any error is encountered then this function immediately returns. The
    /// contents of buf are unspecified in this case.
    ///
    /// If this function returns an error, it is unspecified how many bytes it
    /// has read, but it will never read more than would be necessary to
    /// completely fill the buffer.
    pub fn read(&mut self, buffer: &mut [u8]) {
        for chunk in buffer.chunks_mut(4) {
            let bytes = self.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::blocking::rng::Read for Rng {
    type Error = core::convert::Infallible;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read(buffer);
        Ok(())
    }
}

impl rand_core::RngCore for Rng {
    fn next_u32(&mut self) -> u32 {
        self.random()
    }

    fn next_u64(&mut self) -> u64 {
        let upper = self.random() as u64;
        let lower = self.random() as u64;

        (upper << 32) | lower
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.read(dest);
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.read(dest);
        Ok(())
    }
}
