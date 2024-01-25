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
//! [Read](embedded_hal::blocking::rng::Read) trait from the `embedded_hal`
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
//! For more information, please refer to the ESP-IDF documentation:  
//! <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/random.html>
//!
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

use core::{convert::Infallible, marker::PhantomData};

use crate::{peripheral::Peripheral, peripherals::RNG};

use rand_core::{CryptoRng, Error, RngCore};

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

    pub fn ensure_randomness(&self) {
        unsafe {
            #[cfg(feature = "esp32c6")]
            if (&*crate::soc::peripherals::LPWR::PTR)
                .lp_clk_conf()
                .read()
                .fast_clk_sel()
                .bit_is_set()
            {
                (&*crate::soc::peripherals::LPWR::PTR)
                    .lp_clk_conf()
                    .modify(|_, w| w.fast_clk_sel().clear_bit())
            }

            #[cfg(feature = "esp32h2")]
            if (&*crate::soc::peripherals::LPWR::PTR)
                .lp_clk_conf()
                .read()
                .fast_clk_sel()
                .bits()
                != 0b00
            {
                (&*crate::soc::peripherals::LPWR::PTR)
                    .lp_clk_conf()
                    .modify(|_, w| w.fast_clk_sel().bits(0b00))
            }

            #[cfg(feature = "esp32p4")]
            if (&*crate::soc::peripherals::LP_AON_CLKRST::PTR)
                .lp_aon_clkrst_lp_clk_conf()
                .read()
                .lp_aonclkrst_fast_clk_sel()
                .bits()
                != 0b00
            {
                (&*crate::soc::peripherals::LPWR::PTR)
                    .lp_aon_clkrst_lp_clk_conf()
                    .modify(|_, w| w.lp_aonclkrst_fast_clk_sel().bits(0b00))
            }

            #[cfg(not(any(feature = "esp32c6", feature = "esp32h2", feature = "esp32p4")))]
            if (&*crate::soc::peripherals::LPWR::PTR)
                .clk_conf()
                .read()
                .fast_clk_rtc_sel()
                .bit_is_set()
            {
                (&*crate::soc::peripherals::LPWR::PTR)
                    .clk_conf()
                    .modify(|_, w| w.fast_clk_rtc_sel().clear_bit())
            }
        }
    }
}

impl embedded_hal::blocking::rng::Read for Rng {
    type Error = Infallible;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in buffer.chunks_mut(4) {
            let bytes = self.random().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }

        Ok(())
    }
}

// Marker trait. `ensure_randomness` function helps to be sure that RNG generates really random numbers
impl CryptoRng for Rng {}

impl RngCore for Rng {
    fn next_u32(&mut self) -> u32 {
        self.ensure_randomness();
        // Directly use the existing random method to get a u32 random number
        self.random()
    }

    fn next_u64(&mut self) -> u64 {
        self.ensure_randomness();
        // Call random() twice to generate a u64 random number (сombine two u32)
        let upper = self.random() as u64;
        let lower = self.random() as u64;
        (upper << 32) | lower
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.ensure_randomness();
        // Fill the destination buffer with random bytes
        for chunk in dest.chunks_mut(4) {
            let rand_bytes = self.random().to_le_bytes();
            for (dest_byte, rand_byte) in chunk.iter_mut().zip(&rand_bytes) {
                *dest_byte = *rand_byte;
            }
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.ensure_randomness();
        // Similar implementation as fill_bytes, but encapsulated in a Result
        self.fill_bytes(dest);
        Ok(())
    }
}
