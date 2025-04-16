# Migration Guide from v1.0.0-beta.0 to ?

## Peripheral singleton changes

> As this is a conceptual change, we'll not list all affected types in this section. `AnyPeripheral`
> refers to all `Any*` types at the same time - `AnySpi`, `AnyUart`, etc. Similarly, `Driver` refers
> to any matching peripheral driver.

Peripheral singletons (like `SPI2` or `GpioPin`) no longer implement `Peripheral`. The `Peripheral`
trait and `PeripheralRef` struct have been removed. The peripheral singletons instead have a
lifetime and implement the following methods:

- `steal` and `clone_unchecked` to unsafely create them.
- `reborrow` to safely create a copy with a shorter lifetime.
- `degrade` has been removed in favour of `AnyPeripheral::from`.

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

## DMA changes

### DMA channel types are now consistent with other peripheral singletons

For compatibility with third party libraries, as well as for consistency with other peripherals,
the DMA channel types (e.g. `DmaChannel0`/`Spi2DmaChannel`) have been replaced by
`esp_hal::peripherals::DMA_channel<'d>` types.

```diff
-use esp_hal::gpio::DmaChannel0;
+use esp_hal::peripherals::DMA_CH0;
```

```diff
-use esp_hal::gpio::Spi2DmaChannel;
+use esp_hal::peripherals::DMA_SPI2;
```

## GPIO changes

### GPIO pins are now consistent with other peripheral singletons

For compatibility with third party libraries, as well as for consistency with other peripherals,
the GPIO pin types (`GpioPin<N>`) have been replaced by separate `esp_hal::peripherals::GPION<'d>`
types.

```diff
-use esp_hal::gpio::GpioPin;
+use esp_hal::peripherals::{GPIO2, GPIO3};

-fn my_function(gpio2: GpioPin<2>, gpio3: GpioPin<3>) {...}
+fn my_function(gpio2: GPIO2<'_>, gpio3: GPIO3<'_>) {...}
```

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

## PARL IO driver changes

### `ParlIoFullDuplex`, `ParlIoTxOnly` and `ParlIoRxOnly` have been merged into `ParlIo`

```diff
- let parl_io = ParlIoFullDuplex::new(peripherals.PARL_IO, dma_channel)?;
- let parl_io = ParlIoTxOnly::new(peripherals.PARL_IO, dma_channel)?;
- let parl_io = ParlIoRxOnly::new(peripherals.PARL_IO, dma_channel)?;
+ let parl_io = ParlIo::new(peripherals.PARL_IO, dma_channel)?;
```

### Construction no longer asks for references

```diff
 let mut parl_io_tx = parl_io
     .tx
     .with_config(
-        &mut pin_conf,
-        &mut clock_pin,
+        pin_conf,
+        clock_pin,
         0,
         SampleEdge::Normal,
         BitPackOrder::Msb,
     )?;
```

### Construction options are passed via config

```diff
+let config = RxConfig::default()
+                 .with_frequency(Rate::from_mhz(20))
+                 .with_bit_order(BitPackOrder::Msb);

 let mut parl_io_rx = parl_io
     .rx
     .with_config(
         pin_conf,
         clock_pin,
-        BitPackOrder::Msb,
-        None,
+        config,
     )?;
```

```diff
+let config = TxConfig::default()
+                 .with_frequency(Rate::from_mhz(20))
+                 .with_sample_edge(SampleEdge::Normal)
+                 .with_bit_order(BitPackOrder::Msb);

 let mut parl_io_tx = parl_io
     .tx
     .with_config(
         pin_conf,
         clock_pin,
-        0,
-        SampleEdge::Normal,
-        BitPackOrder::Msb,
+        config,
     )?;
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

## DMA memcpy driver now uses the newer DMA APIs

```diff
  let mut mem2mem = Mem2Mem::new(
      peripherals.DMA_CH0,
      dma_peripheral,
+ ).with_descriptors(
      rx_descriptors,
      tx_descriptors,
+     BurstConfig::default(),
  ).unwrap();
```

## LCD_CAM Camera changes

The data and ctrl pins of the camera have been split out into individual `with_*` methods.

```diff
- camera.with_ctrl_pins(vsync_pin, href_pin);
+ camera.with_vsync(vsync_pin).with_h_enable(href_pin);
+ config.with_vh_de_mode(VhdeMode::VsyncHsync);
```

```diff
- camera.with_ctrl_pins_and_de(vsync_pin, hsync_pin, href_pin);
+ camera.with_vsync(vsync_pin).with_hsync(hsync_pin).with_h_enable(href_pin);
+ config.with_vh_de_mode(VhdeMode::De); // Needed to enable HSYNC pin
```

```diff
- let data_pins = RxEightBits::new(
-     peripherals.GPIO11,
-     peripherals.GPIO9,
-     peripherals.GPIO8,
-     ....
- );
  let mut camera = Camera::new(
      lcd_cam.cam,
      peripherals.DMA_CH0,
-     data_pins,
-     config,
+     config.with_enable_2byte_mode(false) // If you were previously using RxEightBits (This is the default)
+     config.with_enable_2byte_mode(true) // If you were previously using RxSixteenBits
  )?;

+ camera.with_data0(peripherals.GPIO11).with_data1(peripherals.GPIO9).with_dataX();

```
