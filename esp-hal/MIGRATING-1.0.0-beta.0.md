# Migration Guide from v1.0.0-beta.0 to ?

## GPIO changes

### Interconnect types now have a lifetime

The GPIO interconnect types now have a lifetime. This lifetime prevents them to be live for longer
than the GPIO that was used to create them.

The affected types in the `gpio::interconnect` module are:

- `InputSignal`
- `OutputSignal`
- `InputConnection`
- `OutputConnection`

### Pin drivers no longer implement `Peripheral`

Peripheral drivers now take `impl PeripheralInput` and `impl PeripheralOutput` directly. These
traits are now implemented for GPIO pins (stably) and driver structs (unstably) alike.

This change means it's no longer possible to pass a reference to a GPIO driver to a peripheral
driver. For example, it's no longer possible to pass an `&mut Input` to `Spi::with_miso`.

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

## SPI Slave driver now uses the newer DMA APIs

```diff
  let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
+ let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
+ let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

- let transfer = spi.transfer(&mut rx_buffer, &mut tx_buffer)?;
+ let transfer = spi.transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)?;

- let transfer = spi.write(&mut tx_buffer)?;
+ let transfer = spi.write(dma_tx_buf.len(), dma_tx_buf)?;

- let transfer = spi.read(&mut rx_buffer)?;
+ let transfer = spi.read(dma_rx_buf.len(), dma_rx_buf)?;
```
