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

## AES changes

The `esp_hal::aes::Aes` driver has been slightly reworked:

- `Mode` has been replaced by `Operation`. Operation has `Encrypt` and `Decrypt` variants, but the key length is no longer part of the enum. The key length is specified by the key. AesDma now takes this `Operation`.
- `Aes::process` has been split into `encrypt` and `decrypt`. These functions no longer take a mode parameter.
- `AesDma::write_block` has been removed.

```diff
-aes.process(block, Mode::Encrypt128, key);
+aes.encrypt(block, key_16_bytes);

-aes.process(block, Mode::Decrypt256, key);
+aes.decrypt(block, key_32_bytes);
```
