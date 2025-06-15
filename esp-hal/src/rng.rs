//! # Random Number Generator (RNG)
//!
//! ## Overview
//! The Random Number Generator (RNG) module provides an interface to generate
//! random numbers using the RNG peripheral on ESP chips. This driver allows you
//! to generate random numbers that can be used for various cryptographic,
//! security, or general-purpose applications.
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
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/system/random.html)")]
//! ## Configuration
//! To use the [Rng] Driver, you need to initialize it with the RNG peripheral.
//! Once initialized, you can generate random numbers by calling the `random`
//! method, which returns a 32-bit unsigned integer.
//!
//! ## Usage
//! The driver implements the traits from the [`rand_core`] crate.
//!
//! [`rand_core`]: https://crates.io/crates/rand_core
//!
//! ## Examples
//!
//! ### Basic RNG operation
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::rng::Rng;
//!
//! let mut rng = Rng::new(peripherals.RNG);
//!
//! // Generate a random word (u32):
//! let rand_word = rng.random();
//!
//! // Fill a buffer with random bytes:
//! let mut buf = [0u8; 16];
//! rng.read(&mut buf);
//!
//! loop {}
//! # }
//! ```
//! 
//! ### TRNG operation
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::Blocking;
//! # use esp_hal::rng::Trng;
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::peripherals::ADC1;
//! # use esp_hal::analog::adc::{AdcConfig, Attenuation, Adc};
//!
//! let mut buf = [0u8; 16];
//!
//! // ADC is not available from now
//! let mut trng = Trng::new(peripherals.RNG, peripherals.ADC1.reborrow());
//! trng.read(&mut buf);
//! let mut true_rand = trng.random();
//! let mut rng = trng.downgrade();
//! // ADC is available now
#![cfg_attr(esp32, doc = "let analog_pin = peripherals.GPIO32;")]
#![cfg_attr(not(esp32), doc = "let analog_pin = peripherals.GPIO3;")]
//! let mut adc1_config = AdcConfig::new();
//! let mut adc1_pin = adc1_config.enable_pin(
//!     analog_pin,
//!     Attenuation::_11dB
//! );
//! let mut adc1 = Adc::<ADC1, Blocking>::new(peripherals.ADC1, adc1_config);
//! let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin))?;
//! rng.read(&mut buf);
//! true_rand = rng.random();
//! let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin))?;
//! # Ok(())
//! # }
//! ```

use crate::{
    peripherals::{ADC1, RNG},
    private::Sealed,
};

/// Random number generator driver
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct Rng;

impl Rng {
    /// Create a new random number generator instance
    pub fn new(_rng: RNG<'_>) -> Self {
        Self
    }

    #[inline]
    /// Reads currently available `u32` integer from `RNG`
    pub fn random(&mut self) -> u32 {
        // SAFETY: read-only register access
        RNG::regs().data().read().bits()
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

impl Sealed for Rng {}

#[instability::unstable]
impl rand_core_06::RngCore for Rng {
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

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core_06::Error> {
        self.read(dest);
        Ok(())
    }
}

#[instability::unstable]
impl rand_core_09::RngCore for Rng {
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
}

/// True Random Number Generator (TRNG) driver
///
/// The `Trng` struct represents a true random number generator that combines
/// the randomness from the hardware RNG and an ADC. This struct provides
/// methods to generate random numbers and fill buffers with random bytes.
/// Due to pulling the entropy source from the ADC, it uses the associated
/// registers, so to use TRNG we need to "occupy" the ADC peripheral.
pub struct Trng<'d> {
    /// The hardware random number generator instance.
    pub rng: Rng,
    /// A mutable reference to the ADC1 instance.
    _adc: ADC1<'d>,
}

impl<'d> Trng<'d> {
    /// Creates a new True Random Number Generator (TRNG) instance.
    ///
    /// # Arguments
    ///
    /// * `rng` - A peripheral instance implementing the `RNG` trait.
    /// * `adc` - A mutable reference to an `Adc` instance.
    ///
    /// # Returns
    ///
    /// Returns a new `Trng` instance.
    pub fn new(rng: RNG<'_>, adc: ADC1<'d>) -> Self {
        let r#gen = Rng::new(rng);
        crate::soc::trng::ensure_randomness();
        Self {
            rng: r#gen,
            _adc: adc,
        }
    }

    /// Reads currently available `u32` integer from `TRNG`
    pub fn random(&mut self) -> u32 {
        self.rng.random()
    }

    /// Fills the provided buffer with random bytes.
    pub fn read(&mut self, buffer: &mut [u8]) {
        self.rng.read(buffer);
    }

    /// Downgrades the `Trng` instance to a `Rng` instance and releases the
    /// ADC1.
    pub fn downgrade(self) -> Rng {
        self.rng
    }
}

impl Drop for Trng<'_> {
    fn drop(&mut self) {
        crate::soc::trng::revert_trng();
    }
}

/// Implementing RngCore trait from rand_core for `Trng` structure
#[instability::unstable]
impl rand_core_06::RngCore for Trng<'_> {
    fn next_u32(&mut self) -> u32 {
        self.rng.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.rng.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.rng.fill_bytes(dest)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core_06::Error> {
        self.rng.try_fill_bytes(dest)
    }
}

#[instability::unstable]
impl rand_core_09::RngCore for Trng<'_> {
    fn next_u32(&mut self) -> u32 {
        self.rng.next_u32()
    }
    fn next_u64(&mut self) -> u64 {
        self.rng.next_u64()
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.rng.fill_bytes(dest)
    }
}

/// Implementing a CryptoRng marker trait that indicates that the generator is
/// cryptographically secure.
#[instability::unstable]
impl rand_core_06::CryptoRng for Trng<'_> {}
#[instability::unstable]
impl rand_core_09::CryptoRng for Trng<'_> {}

impl Sealed for Trng<'_> {}
