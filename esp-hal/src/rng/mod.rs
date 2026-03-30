#![cfg_attr(docsrs, procmacros::doc_replace(
    "documentation" => concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/system/random.html).")
))]
//! # Random Number Generator (RNG)
//!
//! ## Overview
//!
//! The Random Number Generator (RNG) module provides an interface to generate
//! random numbers using the RNG peripheral on ESP chips. This driver allows you
//! to generate random numbers that can be used for various cryptographic,
//! security, or general-purpose applications.
//!
//! There are certain pre-conditions which must be met in order for the RNG to
//! produce *true* random numbers.
#![cfg_attr(rng_trng_supported, doc = r"For more details see [`Trng`].")]
//! The hardware RNG produces true random numbers
//! under any of the following conditions:
//!
//! - RF subsystem is enabled (i.e. Wi-Fi or Bluetooth are enabled).
//! - An ADC is used to generate entropy.
//!
//! When any of these conditions are true, samples of physical noise are
//! continuously mixed into the internal hardware RNG state to provide entropy.
//! If none of the above conditions are true, the output of the RNG should be
//! considered pseudo-random only. See [`Rng`].
//!
//! For more information, please refer to the
//! # {documentation}
//!
//! ## Example
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::rng::Rng;
//! #
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
//! ## Compatibility with [`getrandom`]
//!
//! The driver can be used directly, or integrated as a [custom backend]
//! for `getrandom` (e.g. v0.4):
//!
//! ```rust, no_run
//! # {before_snippet}
//! # mod getrandom {
//! # use core::result::Result::{self, Ok};
//! # pub struct Error;
//! #[unsafe(no_mangle)]
//! unsafe extern "Rust" fn __getrandom_v04_custom(
//!     dest: *mut u8,
//!     len: usize,
//! ) -> Result<(), Error> {
//!      unsafe { esp_hal::rng::Rng::new().read_into_raw(dest, len) };
//!      Ok(())
//! # }
//! # }
//! # {after_snippet}
//! ```
//!
//! [`getrandom`]: https://crates.io/crates/getrandom
//! [custom backend]: https://github.com/rust-random/getrandom/tree/v0.4.2?tab=readme-ov-file#custom-backend

mod ll;

#[cfg(all(feature = "unstable", rng_trng_supported))]
mod trng;

#[cfg(all(feature = "unstable", rng_trng_supported))]
pub use trng::*;

/// (Pseudo-)Random Number Generator.
///
/// To generate pseudo-random numbers, you can create [`Rng`] at any time.
#[cfg_attr(
    rng_trng_supported,
    doc = r"To generate true random numbers, see [`Trng`]."
)]
#[derive(Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Rng;

impl Rng {
    /// Creates a new random number generator instance.
    #[inline]
    pub fn new() -> Self {
        Self
    }

    #[procmacros::doc_replace]
    /// Reads currently available `u32` integer from `RNG`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rng::Rng;
    ///
    /// let rng = Rng::new();
    /// let random_number = rng.random();
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn random(&self) -> u32 {
        let mut n = [0; 4];
        self.read(&mut n);
        u32::from_le_bytes(n)
    }

    #[procmacros::doc_replace]
    /// Reads enough bytes from hardware random number generator to fill
    /// `buffer`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rng::Rng;
    ///
    /// let rng = Rng::new();
    /// let mut rng_values = [0; 10];
    /// rng.read(&mut rng_values);
    /// # {after_snippet}
    #[inline]
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

// Implement RngCore traits

/// Compatibility with `rand_core 0.6`. Documentation can be found at
/// <https://docs.rs/rand_core/0.6.4/rand_core/trait.RngCore.html>.
#[instability::unstable]
impl rand_core_06::RngCore for Rng {
    fn next_u32(&mut self) -> u32 {
        self.random()
    }

    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.read(&mut bytes);
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

/// Compatibility with `rand_core 0.9`. Documentation can be found at
/// <https://docs.rs/rand_core/0.9.5/rand_core/trait.RngCore.html>.
#[instability::unstable]
impl rand_core_09::RngCore for Rng {
    fn next_u32(&mut self) -> u32 {
        self.random()
    }
    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.read(&mut bytes);
        u64::from_le_bytes(bytes)
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.read(dest);
    }
}

/// Compatibility with `rand_core 0.10`
#[instability::unstable]
impl rand_core_010::TryRng for Rng {
    type Error = core::convert::Infallible;
    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        Ok(self.random())
    }
    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        let mut bytes = [0; 8];
        self.read(&mut bytes);
        Ok(u64::from_le_bytes(bytes))
    }
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        self.read(dest);
        Ok(())
    }
}
