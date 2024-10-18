# Migration Guide from 0.21.x to v0.22.x

## Removed `i2s` traits

The following traits have been removed:

- `I2sWrite`
- `I2sWriteDma`
- `I2sRead`
- `I2sReadDma`
- `I2sWriteDmaAsync`
- `I2sReadDmaAsync`

You no longer have to import these to access their respective APIs. If you used these traits
in your functions as generic parameters, you can use the `I2s` type directly instead.

For example:

```diff
-fn foo(i2s: &mut impl I2sWrite) {
+fn foo(i2s: &mut I2s<'_, I2S0, Blocking>) {
     // ...
 }
```

## Removed `async`-specific constructors

The following async-specific constuctors have been removed:

- `TwaiConfiguration::new_async` and `TwaiConfiguration::new_async_no_transceiver`

You can use the blocking counterparts, then call `into_async` on the returned peripheral instead.

```diff
-let mut config = twai::TwaiConfiguration::new_async(
+let mut config = twai::TwaiConfiguration::new(
     peripherals.TWAI0,
     loopback_pin.peripheral_input(),
     loopback_pin,
     twai::BaudRate::B1000K,
     TwaiMode::SelfTest,
-);
+).into_async();
```

## Peripheral types are now optional

You no longer have to specify the peripheral instance in the driver's type for the following
peripherals:

- SPI (both master and slave)
- TWAI

```diff
-Spi<'static, SPI2, FullDuplexMode>
+Spi<'static, FullDuplexMode>

-SpiDma<'static, SPI2, HalfDuplexMode, Blocking>
+SpiDma<'static, HalfDuplexMode, Blocking>
```

Note that you may still specify the instance if you need to. To do this, we provide `_typed`
versions of the constructors (for example: `new_typed`, `new_half_duplkex_typed`). Please note that
the peripheral instance has been moved to the last generic parameter position.

```rust
let spi: Spi<'static, FullDuplexMode, SPI2> = Spi::new_typed(peripherals.SPI2, 1.MHz(), SpiMode::Mode0);
```
