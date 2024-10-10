# Migration Guide from 0.21.x to v0.22.x

## Peripheral types are now optional

You no longer have to specify the peripheral instance in the driver's type for the following
peripherals:

- SPI (both master and slave)

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
