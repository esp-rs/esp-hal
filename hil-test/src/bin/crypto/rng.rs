#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use esp_hal::rng::Rng;
    #[cfg(rng_trng_supported)]
    use esp_hal::rng::{Trng, TrngSource};

    #[test]
    #[cfg(rng_trng_supported)]
    fn test_trng_without_source_is_error() {
        assert!(Trng::try_new().is_err());

        // Rng can be created anyway:
        let _rng = Rng::new();
    }

    #[test]
    fn test_rng_returns_random_values() {
        let _p = esp_hal::init(Default::default());
        let rng = Rng::new();

        let mut rng_values = [0; 10];
        rng.read(&mut rng_values);

        assert!(rng_values.windows(2).any(|w| w[0] != w[1]));
    }

    #[test]
    #[cfg(rng_trng_supported)]
    fn test_trng_returns_random_values() {
        let p = esp_hal::init(Default::default());
        let _source = TrngSource::new(p.RNG, p.ADC1);

        let rng = Trng::try_new().unwrap();

        let mut rng_values = [0; 10];
        rng.read(&mut rng_values);

        assert!(rng_values.windows(2).any(|w| w[0] != w[1]));
    }

    #[test]
    #[cfg(rng_trng_supported)]
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
    #[cfg(rng_trng_supported)]
    fn test_trng_source_cannot_be_dropped_while_in_use() {
        let p = esp_hal::init(Default::default());
        let source = TrngSource::new(p.RNG, p.ADC1);

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }

    #[test]
    #[cfg(rng_trng_supported)]
    fn test_trng_source_can_be_dropped_if_unsafely_enabled() {
        let p = esp_hal::init(Default::default());

        let source = TrngSource::new(p.RNG, p.ADC1);

        // Unsafely increase the counter. Practically, this may be done in esp-radio.
        unsafe { TrngSource::increase_entropy_source_counter() };

        let _trng = Trng::try_new().unwrap();

        core::mem::drop(source);
    }
}
