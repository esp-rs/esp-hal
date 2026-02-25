#![cfg_attr(docsrs, procmacros::doc_replace(
    "documentation" => concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/system/random.html)")
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
//! To generate pseudo-random numbers, you can create [`Rng`] at any time.
#![cfg_attr(
    rng_trng_supported,
    doc = r"To generate
true random numbers, you need to create an instance of [`TrngSource`]. Once you've
done that, you can create [`Trng`] instances at any time, as long as the [`TrngSource`]
is alive."
)]
//! ## Compatibility with [`rand_core`]
//!
//! The RNG drivers implement the relevant
//! traits from versions `0.6` and `0.9` of the [`rand_core`] crate.
//!
//! [`rand_core`]: https://crates.io/crates/rand_core
//!
//! ## Compatibility with [`getrandom`]
//!
//! The driver can be used to implement a custom backend for `getrandom`.
//!
//! ## Example
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

mod ll;

#[cfg(all(feature = "unstable", rng_trng_supported))]
mod trng;

#[cfg(all(feature = "unstable", rng_trng_supported))]
pub use trng::*;

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
