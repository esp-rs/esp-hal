//! RNG Tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use esp_hal::rng::{Rng, Trng, TrngSource};

    #[test]
    fn test_trng_without_source_is_error() {
        assert!(Trng::try_new().is_err());

        // Rng can be created anyway:
        let _rng = Rng::new();
    }

    #[test]
    fn test_trng_source_cannot_be_disabled_while_in_use() {
        let p = esp_hal::init(Default::default());
        let source = TrngSource::new(p.RNG, p.ADC1);

        let trng = Trng::try_new().unwrap();

        let _source = source.try_disable().unwrap_err();

        // Need to drop trng first
        core::mem::drop(trng);

        // Rng will not prevent disabling the TrngSource
        let _rng = Rng::new();
    }

    #[test]
    #[should_panic]
    fn test_trng_source_cannot_be_dropped_while_in_use() {
        let p = esp_hal::init(Default::default());
        let source = TrngSource::new(p.RNG, p.ADC1);

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }

    #[test]
    fn test_trng_source_can_be_dropped_if_unsafely_enabled() {
        let p = esp_hal::init(Default::default());

        let source = TrngSource::new(p.RNG, p.ADC1);

        // Unsafely increase the counter. Practically, this may be done in esp-radio.
        unsafe { TrngSource::increase_entropy_source_counter() };

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }
}
