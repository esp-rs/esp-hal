# Migration Guide from v1.0.0-beta.0 to ?

## Peripheral singleton changes

Peripheral singletons (like `SPI2` or `GpioPin`) no longer implement `Peripheral`. The `Peripheral`
trait and `PeripheralRef` struct have been removed. The peripheral singletons instead have a
lifetime and implement the following methods:

- `steal` and `clone_unchecked` to unsafely create them.
- `reborrow` to safely create a copy with a shorter lifetime.

### Application-facing changes

Peripheral drivers no longer accept `&mut singleton`.

Use `reborrow` instead:

```diff
-let driver = Driver::new(&mut peripheral);
+let driver = Driver::new(peripheral.reborrow());
```

After dropping the driver, `peripheral` should be accessible again as it used to be previously.

### Peripheral driver changes

The `Peripheral` and `PeripheralRef` types no longer exist. The driver structs and constructors need
to be updated accordingly:

If the driver works with a single peripheral instance, for example `AES`:

```diff
 struct Driver<'d> {
-   aes: PeripheralRef<'d, AES>,
+   aes: AES<'d>,
 }
 // ...
-fn new(aes: impl Peripheral<P = AES> + 'd)
+fn new(aes: AES<'d>)
```

If a driver works with multiple peripheral instances, e.g. `SPI`:

```diff
 struct Driver<'d> {
-   spi: PeripheralRef<'d, AnySpi>,
+   spi: AnySpi<'d>,
 }
 // ...
-fn new(spi: impl Peripheral<P = impl Instance> + 'd)
+fn new(spi: impl Instance + Into<AnySpi<'d>>)
```

## GPIO changes

### Interconnect types now have a lifetime

The GPIO interconnect types now have a lifetime. This lifetime prevents them to be live for longer
than the GPIO that was used to create them.

The affected types in the `gpio::interconnect` module are:

- `InputSignal`
- `OutputSignal`
- `InputConnection`
- `OutputConnection`

## I2S driver now takes `DmaDescriptor`s later in construction

```diff
  let i2s = I2s::new(
      peripherals.I2S0,
      Standard::Philips,
      DataFormat::Data16Channel16,
      Rate::from_hz(44100),
      dma_channel,
-     rx_descriptors,
-     tx_descriptors,
  );

  let i2s_tx = i2s
      .i2s_tx
      .with_bclk(peripherals.GPIO2)
      .with_ws(peripherals.GPIO4)
      .with_dout(peripherals.GPIO5)
-     .build();
+     .build(tx_descriptors);

  let i2s_rx = i2s
      .i2s_rx
      .with_bclk(peripherals.GPIO2)
      .with_ws(peripherals.GPIO4)
      .with_din(peripherals.GPIO5)
-     .build();
+     .build(rx_descriptors);
```
