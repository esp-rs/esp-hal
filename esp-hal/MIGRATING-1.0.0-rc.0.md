# Migration Guide from 1.0.0-rc.0 to {{currentVersion}}

## RNG changes

Random number generator objects can now be created anywhere. The `RNG` peripheral singleton
is only necessary to enable the cryptographically secure random number generator.

 - `Rng` can be constructed without any constraints.
 - `Trng` can only be constructed when it can be ensured to generate true random numbers.

A new `TrngSource` object has been added. Users can use `TrngSource::new` to create the object,
and as long as it is alive, `Trng::try_new` will return `Ok` and will provide a true random number
generator interface.

The previous way to obtain RNG object has changed like so:

```diff
-let mut rng = Rng::new(peripherals.RNG);
+let rng = Rng::new();

-let mut trng = Trng::new(peripherals.RNG, peripherals.ADC1);
+// Once:
+let trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
+// As long as `trng_source` is alive:
+let trng = Tnrg::try_new().unrwap();
```
