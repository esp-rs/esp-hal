#![cfg_attr(docsrs, procmacros::doc_replace(
    "analog_pin" => {
        cfg(esp32) => "let analog_pin = peripherals.GPIO32;",
        cfg(not(esp32)) => "let analog_pin = peripherals.GPIO3;"
    },
    "documentation" => concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/system/random.html)")
))]
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
//! - An ADC is used to generate entropy.
//! - An internal entropy source has been enabled by calling `bootloader_random_enable()` and not
//!   yet disabled by calling `bootloader_random_disable()`. As the default bootloader calls
//!   `bootloader_random_disable()` automatically, this option requires a custom bootloader build.
//!
//! When any of these conditions are true, samples of physical noise are
//! continuously mixed into the internal hardware RNG state to provide entropy.
//! If none of the above conditions are true, the output of the RNG should be
//! considered pseudo-random only.
//!
//! > Note that, when the Wi-Fi module is enabled, the value read from the high-speed ADC can be
//! > saturated in some extreme cases, which lowers the entropy. Thus, it is advisable to also
//! > enable the SAR ADC as the noise source for the random number generator for such cases.
//!
//! For more information, please refer to the
//! # {documentation}
//!
//! ## Configuration
//!
//! To generate pseudo-random numbers, you can create [`Rng`] at any time. To generate
//! true random numbers, you need to create an instance of [`TrngSource`]. Once you've
//! done that, you can create [`Trng`] instances at any time, as long as the [`TrngSource`]
//! is alive.
//!
//! ## Compatibility with [`rand_core`]
//!
//! The [`Rng`] and [`Trng`] drivers implement the relevant
//! traits from versions `0.6` and `0.9` of the [`rand_core`] crate.
//!
//! [`rand_core`]: https://crates.io/crates/rand_core
//!
//! ## Compatibility with [`getrandom`]
//! The driver can be used to implement a custom backend for `getrandom`.
//!
//! ### Example
//!
//! The following example demonstrates how to set up a [custom backend] using `getrandom v0.3.3`:
//!
//! ```rust, ignore
//! use getrandom::Error;
//!
//! #[no_mangle]
//! unsafe extern "Rust" fn __getrandom_v03_custom(
//!     dest: *mut u8,
//!     len: usize,
//! ) -> Result<(), Error> {
//!     unsafe { esp_hal::rng:Rng::new().read_into_raw() };
//!     Ok(())
//! }
//! ```
//!
//! [`getrandom`]: https://crates.io/crates/getrandom
//! [custom backend]: https://github.com/rust-random/getrandom/tree/v0.3.3?tab=readme-ov-file#custom-backend
//!
//! ## Examples
//!
//! ### Basic RNG operation
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::rng::Rng;
//!
//! let rng = Rng::new();
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
//! # {before_snippet}
//! # use esp_hal::Blocking;
//! # use esp_hal::peripherals::ADC1;
//! # use esp_hal::analog::adc::{AdcConfig, Attenuation, Adc};
//! #
//! use esp_hal::rng::{Trng, TrngSource};
//!
//! let mut buf = [0u8; 16];
//!
//! // ADC is not available from now
//! let trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
//!
//! let trng = Trng::try_new().unwrap(); // Unwrap is safe as we have enabled TrngSource.
//!
//! // Generate true random numbers
//! trng.read(&mut buf);
//! let true_random_number = trng.random();
//!
//! // Downgrade to Rng to allow disabling the TrngSource
//! let rng = trng.downgrade();
//!
//! // Drop the true random number source. ADC is available now.
//! core::mem::drop(trng_source);
//!
//! # {analog_pin}
//!
//! let mut adc1_config = AdcConfig::new();
//! let mut adc1_pin = adc1_config.enable_pin(analog_pin, Attenuation::_11dB);
//! let mut adc1 = Adc::<ADC1, Blocking>::new(peripherals.ADC1, adc1_config);
//! let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin))?;
//!
//! // Now we can only generate pseudo-random numbers...
//! rng.read(&mut buf);
//! let pseudo_random_number = rng.random();
//!
//! // ... but the ADC is available for use.
//! let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin))?;
//! # {after_snippet}
//! ```

mod ll;

use portable_atomic::{AtomicUsize, Ordering};

static TRNG_ENABLED: AtomicUsize = AtomicUsize::new(0);
static TRNG_USERS: AtomicUsize = AtomicUsize::new(0);

use crate::peripherals::{ADC1, RNG};

/// Ensures random numbers are cryptographically secure.
#[instability::unstable]
pub struct TrngSource<'d> {
    _rng: RNG<'d>,
    _adc: ADC1<'d>,
}

impl<'d> TrngSource<'d> {
    /// Enables the SAR ADC entropy source.
    // TODO: this is not final. A single ADC channel should be sufficient.
    #[instability::unstable]
    pub fn new(_rng: RNG<'d>, _adc: ADC1<'d>) -> Self {
        crate::soc::trng::ensure_randomness();
        unsafe { Self::increase_entropy_source_counter() }
        Self { _rng, _adc }
    }

    /// Increases the internal entropy source counter.
    ///
    /// # Panics
    ///
    /// This function panics if the internal counter overflows.
    ///
    /// # Safety
    ///
    /// This function must only be called after a new entropy source has been enabled.
    #[instability::unstable]
    pub unsafe fn increase_entropy_source_counter() {
        if TRNG_ENABLED.fetch_add(1, Ordering::Relaxed) == usize::MAX {
            panic!("TrngSource enable overflowed");
        }
    }

    /// Decreases the internal entropy source counter.
    ///
    /// This function should only be called **before** disabling an entropy source (such as the
    /// radio).
    ///
    /// This function should only be called as many times as
    /// [`TrngSource::increase_entropy_source_counter`] was called.
    ///
    /// # Panics
    ///
    /// This function panics if the internal counter underflows. Dropping the `TrngSource` will
    /// panic if this function is called more times than
    /// [`TrngSource::increase_entropy_source_counter`].
    #[instability::unstable]
    pub fn decrease_entropy_source_counter(_private: crate::private::Internal) {
        match TRNG_ENABLED.fetch_sub(1, Ordering::Relaxed) {
            0 => panic!("TrngSource is not active"),

            1 => assert!(
                TRNG_USERS.load(Ordering::Acquire) == 0,
                "TRNG cannot be disabled while it's in use"
            ),

            _ => {}
        }
    }

    /// Returns whether the TRNG is currently enabled.
    ///
    /// Note that entropy sources can be disabled at any time.
    #[instability::unstable]
    pub fn is_enabled() -> bool {
        TRNG_ENABLED.load(Ordering::Relaxed) > 0
    }

    /// Attempts to disable the TRNG.
    ///
    /// This function returns `Err(TrngSource)` if there are TRNG users.
    ///
    /// # Panics
    ///
    /// This function panics if the TRNG is not enabled (i.e. it has been disabled by calling
    /// [`TrngSource::decrease_entropy_source_counter`] incorrectly).
    #[instability::unstable]
    pub fn try_disable(self) -> Result<(), Self> {
        if TRNG_ENABLED
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |enabled| {
                assert!(enabled > 0, "TrngSource is not active");
                if TRNG_USERS.load(Ordering::Acquire) > 0 {
                    return None;
                }

                Some(enabled - 1)
            })
            .is_err()
        {
            // The TRNG is in use.
            return Err(self);
        }

        core::mem::forget(self);
        Ok(())
    }
}

impl Drop for TrngSource<'_> {
    fn drop(&mut self) {
        Self::decrease_entropy_source_counter(crate::private::Internal);
        crate::soc::trng::revert_trng();
    }
}

/// (Pseudo-)Random Number Generator
#[derive(Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct Rng;

impl Rng {
    /// Creates a new random number generator instance.
    #[inline]
    #[instability::unstable]
    pub fn new() -> Self {
        Self
    }

    /// Reads currently available `u32` integer from `RNG`.
    #[inline]
    #[instability::unstable]
    pub fn random(&self) -> u32 {
        let mut n = [0; 4];
        self.read(&mut n);
        u32::from_le_bytes(n)
    }

    /// Reads enough bytes from hardware random number generator to fill
    /// `buffer`.
    ///
    /// If any error is encountered then this function immediately returns. The
    /// contents of buf are unspecified in this case.
    #[inline]
    #[instability::unstable]
    pub fn read(&self, buffer: &mut [u8]) {
        unsafe { self.read_into_raw(buffer.as_mut_ptr(), buffer.len()) };
    }

    /// Reads enough bytes from hardware random number generator to fill
    /// `buffer`.
    ///
    /// If any error is encountered then this function immediately returns. The
    /// contents of buf are unspecified in this case.
    ///
    /// # Safety
    ///
    /// `ptr` must not be `null` and valid for writes for `len` bytes.
    #[inline]
    #[instability::unstable]
    pub unsafe fn read_into_raw(&self, ptr: *mut u8, len: usize) {
        ll::fill_ptr_range(ptr, len);
    }
}

/// Errors returned when constructing a [`Trng`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum TrngError {
    /// The [`TrngSource`] is not enabled.
    ///
    /// This error is returned by [`Trng::try_new`] when the RNG is not configured
    /// to generate true random numbers.
    TrngSourceNotEnabled,
}

/// True Random Number Generator (TRNG)
///
/// The `Trng` struct represents a true random number generator that combines
/// the randomness from the hardware RNG and an ADC. This struct provides
/// methods to generate random numbers and fill buffers with random bytes.
/// Due to pulling the entropy source from the ADC, it uses the associated
/// registers, so to use TRNG we need to "occupy" the ADC peripheral.
#[derive(Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct Trng {
    rng: Rng,
}

impl Clone for Trng {
    #[inline]
    fn clone(&self) -> Self {
        TRNG_USERS.fetch_add(1, Ordering::Acquire);
        Self { rng: self.rng }
    }
}

impl Trng {
    /// Attempts to create a new True Random Number Generator (TRNG) instance.
    ///
    /// This function returns a new `Trng` instance on success, or an error if the
    /// [`TrngSource`] is not active.
    #[inline]
    #[instability::unstable]
    pub fn try_new() -> Result<Self, TrngError> {
        TRNG_USERS.fetch_add(1, Ordering::Acquire);
        let this = Self { rng: Rng::new() };
        if TRNG_ENABLED.load(Ordering::Acquire) == 0 {
            // Dropping `this` reduces the TRNG_USERS count back (to 0 as it should be when TRNG is
            // not enabled).
            return Err(TrngError::TrngSourceNotEnabled);
        }
        Ok(this)
    }

    /// Returns a new, random `u32` integer.
    #[inline]
    #[instability::unstable]
    pub fn random(&self) -> u32 {
        self.rng.random()
    }

    /// Fills the provided buffer with random bytes.
    #[inline]
    #[instability::unstable]
    pub fn read(&self, buffer: &mut [u8]) {
        self.rng.read(buffer);
    }

    /// Downgrades the `Trng` instance to a `Rng` instance.
    #[inline]
    #[instability::unstable]
    pub fn downgrade(self) -> Rng {
        Rng::new()
    }
}

impl Drop for Trng {
    fn drop(&mut self) {
        TRNG_USERS.fetch_sub(1, Ordering::Release);
    }
}

// Implement RngCore traits

#[instability::unstable]
impl rand_core_06::RngCore for Rng {
    fn next_u32(&mut self) -> u32 {
        self.random()
    }

    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.fill_bytes(&mut bytes);
        u64::from_le_bytes(bytes)
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
        let mut bytes = [0; 8];
        self.fill_bytes(&mut bytes);
        u64::from_le_bytes(bytes)
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.read(dest);
    }
}

#[instability::unstable]
impl rand_core_06::RngCore for Trng {
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
impl rand_core_09::RngCore for Trng {
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

#[instability::unstable]
impl rand_core_06::CryptoRng for Trng {}
#[instability::unstable]
impl rand_core_09::CryptoRng for Trng {}
