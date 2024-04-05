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

use core::marker::PhantomData;

use crate::{peripheral::Peripheral, peripherals::RNG};

pub trait BasicRng {}
pub trait SecureRng {}

pub struct Basic;
pub struct Secure;

impl BasicRng for Basic {}
impl SecureRng for Secure {}

/// Random number generator driver
#[derive(Clone, Copy)]
pub struct Rng<T> {
    _phantom: PhantomData<(RNG, T)>,
}

impl Rng<Basic> {
    /// Create a new random number generator instance
    pub fn new(_rng: impl Peripheral<P = RNG>) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

impl Rng<Secure> {
    /// Create a new secure random number generator (or TRNG) instance
    pub fn new<'d>(
        rng: impl Peripheral<P = RNG>,
        _adc: impl Peripheral<P = crate::peripherals::ADC1> + 'd,
    ) -> Self {
        crate::soc::trng::ensure_randomness();
        Self {
            _phantom: PhantomData,
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::rng::Read for Rng<T> {
    type Error = core::convert::Infallible;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read(buffer);
        Ok(())
    }
}

impl<T> rand_core::RngCore for Rng<T> {
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

// Common RNG functionality that applies to both basic and true RNG
impl<T> Rng<T> {
    pub fn read(&mut self, buffer: &mut [u8]) {
        for chunk in buffer.chunks_mut(4) {
            let bytes = self.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }

    /// Reads currently available `u32` integer from `RNG`
    pub fn random(&mut self) -> u32 {
        // SAFETY: read-only register access
        unsafe { &*crate::peripherals::RNG::PTR }
            .data()
            .read()
            .bits()
    }
}

#[cfg(not(esp32p4))]
impl rand_core::CryptoRng for Rng<Secure> {}
