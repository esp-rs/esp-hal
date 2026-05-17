//! TRNG implementation.

use portable_atomic::{AtomicUsize, Ordering};

static TRNG_ENABLED: AtomicUsize = AtomicUsize::new(0);
static TRNG_USERS: AtomicUsize = AtomicUsize::new(0);

use super::Rng;
use crate::peripherals::{ADC1, RNG};
#[cfg(not(esp32))]
use crate::system::{GenericPeripheralGuard, Peripheral};

/// Ensures random numbers are cryptographically secure.
#[instability::unstable]
pub struct TrngSource<'d> {
    _rng: RNG<'d>,
    _adc: ADC1<'d>,
    #[cfg(not(esp32))]
    _apb_saradc_guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
}

impl<'d> TrngSource<'d> {
    /// Enables the SAR ADC entropy source.
    // TODO: this is not final. A single ADC channel should be sufficient.
    #[instability::unstable]
    pub fn new(rng: RNG<'d>, adc: ADC1<'d>) -> Self {
        #[cfg(not(esp32))]
        let apb_saradc_guard = GenericPeripheralGuard::new();

        crate::soc::trng::ensure_randomness();
        unsafe { Self::increase_entropy_source_counter() }
        Self {
            _rng: rng,
            _adc: adc,
            #[cfg(not(esp32))]
            _apb_saradc_guard: apb_saradc_guard,
        }
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
///
/// To generate true random numbers, an instance of [`TrngSource`] is required. Once created, you
/// can create [`Trng`] instances at any time, as long as the [`TrngSource`] is alive.
#[cfg_attr(docsrs, procmacros::doc_replace(
    "analog_pin" => {
        cfg(esp32) => "GPIO32",
        _ => "GPIO3"
    }
))]
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// # use esp_hal::peripherals::ADC1;
/// # use esp_hal::analog::adc::{AdcConfig, Attenuation, Adc};
/// #
/// use esp_hal::rng::{Trng, TrngSource};
///
/// let mut buf = [0u8; 16];
///
/// // ADC is not available from now
/// let trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
///
/// let trng = Trng::try_new()?;
///
/// // Generate true random numbers
/// trng.read(&mut buf);
/// let true_random_number = trng.random();
///
/// // Downgrade to Rng to allow disabling the TrngSource
/// let rng = trng.downgrade();
///
/// // Drop the true random number source. ADC is available now.
/// core::mem::drop(trng_source);
///
/// let mut adc1_config = AdcConfig::new();
/// let mut adc1_pin = adc1_config.enable_pin(peripherals.__analog_pin__, Attenuation::_11dB);
/// let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);
/// let pin_value = adc1.read_oneshot(&mut adc1_pin)?;
///
/// // Now we can only generate pseudo-random numbers...
/// rng.read(&mut buf);
/// let pseudo_random_number = rng.random();
///
/// // ... but the ADC is available for use.
/// let pin_value: u16 = adc1.read_oneshot(&mut adc1_pin)?;
/// # {after_snippet}
/// ```
#[derive(Debug)]
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
            // Dropping `this` reduces the TRNG_USERS count back (to 0 as it should be when TRNG
            // is not enabled).
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

/// Compatibility with `rand_core 0.6`. Documentation can be found at
/// <https://docs.rs/rand_core/0.6.4/rand_core/trait.RngCore.html>.
#[instability::unstable]
impl rand_core_06::RngCore for Trng {
    fn next_u32(&mut self) -> u32 {
        <Rng as rand_core_06::RngCore>::next_u32(&mut self.rng)
    }

    fn next_u64(&mut self) -> u64 {
        <Rng as rand_core_06::RngCore>::next_u64(&mut self.rng)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        <Rng as rand_core_06::RngCore>::fill_bytes(&mut self.rng, dest)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core_06::Error> {
        <Rng as rand_core_06::RngCore>::try_fill_bytes(&mut self.rng, dest)
    }
}

/// Compatibility with `rand_core 0.9`. Documentation can be found at
/// <https://docs.rs/rand_core/0.9.5/rand_core/trait.RngCore.html>.
#[instability::unstable]
impl rand_core_09::RngCore for Trng {
    fn next_u32(&mut self) -> u32 {
        <Rng as rand_core_09::RngCore>::next_u32(&mut self.rng)
    }
    fn next_u64(&mut self) -> u64 {
        <Rng as rand_core_09::RngCore>::next_u64(&mut self.rng)
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        <Rng as rand_core_09::RngCore>::fill_bytes(&mut self.rng, dest)
    }
}

/// Compatibility with `rand_core 0.6`. Documentation can be found at
/// <https://docs.rs/rand_core/0.6.4/rand_core/trait.CryptoRng.html>.
#[instability::unstable]
impl rand_core_06::CryptoRng for Trng {}
/// Compatibility with `rand_core 0.9`. Documentation can be found at
/// <https://docs.rs/rand_core/0.9.5/rand_core/trait.CryptoRng.html>.
#[instability::unstable]
impl rand_core_09::CryptoRng for Trng {}

// Non-try variants are blanket-implemented when `Error = Infallible`.

/// Compatibility with `rand_core 0.10`
#[instability::unstable]
impl rand_core_010::TryRng for Trng {
    type Error = core::convert::Infallible;
    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        <Rng as rand_core_010::TryRng>::try_next_u32(&mut self.rng)
    }
    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        <Rng as rand_core_010::TryRng>::try_next_u64(&mut self.rng)
    }
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        <Rng as rand_core_010::TryRng>::try_fill_bytes(&mut self.rng, dest)
    }
}

/// Compatibility with `rand_core 0.10`
#[instability::unstable]
impl rand_core_010::TryCryptoRng for Trng {}
