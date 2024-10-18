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

## Refactored `Mem2Mem` driver

The `Mem2Mem` driver has been split into an RX and TX half, allowing usage is separate tasks.
Support for `Dma(Tx|Rx)Buffer`s has also been added

```diff
- let mem2mem = Mem2Mem::new(channel, dma_peripheral, rx_descriptors, tx_descriptors).unwrap();
+ let mem2mem = Mem2Mem::new(channel, dma_peripheral);

- let transfer = mem2mem.start_transfer(&mut rx_buffer, tx_buffer).unwrap();
+ let tx_transfer = mem2mem.tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();
+ let rx_transfer = mem2mem.rx.receive(dma_rx_buf, true).map_err(|e| e.0).unwrap();
```

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
versions of the constructors (for example: `new_typed`, `new_half_duplex_typed`). Please note that
the peripheral instance has been moved to the last generic parameter position.

```rust
let spi: Spi<'static, FullDuplexMode, SPI2> = Spi::new_typed(peripherals.SPI2, 1.MHz(), SpiMode::Mode0);
```
