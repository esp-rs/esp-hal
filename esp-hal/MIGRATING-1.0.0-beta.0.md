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

### GPIO matrix (interconnect) has been reworked

The GPIO matrix implementation has been reworked to solve known issues. The following changes have
been made:

- `InputConnection` and `OutputConnection` have been removed and their functionality merged into `InputSignal` and `OutputSignal`.
- The `InputSignal` and `OutputSignal` types now have a lifetime. This lifetime prevents them to be live for longer than the GPIO that was used to create them.
- Because `InputSignal` and `OutputSignal` can now represent constant levels, the `number` method now returns `Option<u8>`.
- The way to invert inputs and outputs has been changed:
  - `InputSignal::inverted` has been renamed to `InputSignal::with_inverted_input`.
  - `OutputSignal::inverted` has been renamed to `OutputSignal::with_inverted_output`.
  - `InputSignal::invert` and `OutputSignal::invert` have been removed.
  - `OutputSignal` now has an `inverted_input` property, which can be changed by using `with_inverted_output`.
  - The signals have `is_{input, output}_inverted` methods to read the state that will be used when configuring the hardware.
- Users can now force a signal through the GPIO matrix.
- The `enable_input` and `enable_output` methods have been renamed to `set_input_enable` and `set_output_enable`.
- A new `PeripheralSignal` trait has been added, which allows us to no longer imply `PeripheralInput` for `PeripheralOutput` types.
- Functions that accept `PeripheralInput` **no longer accept** `PeripheralOutput` implementations.
- Removed `Input::into_peripheral_output` and `Output::peripheral_input` functions. The drivers can be converted into `Flex` which offers both ways to acquire signal types, and more.
- Various "unreasonable" signal conversions have been removed. `OutputSignal` can no longer be converted into `InputSignal`.
- `InputSignal` and `OutputSignal` now have a "frozen" state. If they are created by a pin driver, they start frozen. If they are created by splitting a GPIO pin, they start unfrozen. Frozen signals will not be configured by peripheral drivers.
- Splitting GPIO pins into signals is now unsafe.
- `Flex` can now be (unsafely) split into `Input` and `Output` drivers with `split_into_drivers`.
- Converting `AnyPin` into signals will now reset the pin's configuration. Creating an InputSignal will now enable the pin's input buffer.

### Flex API surface has been simplified

The `enable_input` method has been renamed to `set_input_enable`.

The `Flex` driver no longer provides the following functions:

- set_as_input
- set_as_output
- set_drive_strength
- set_as_open_drain
- pull_direction

The individual configurations can be set via `apply_input_config` and `apply_output_config`.
The input buffer and output driver can be separately enabled via `set_input_enable` and
`set_output_enable`.

Normally you only need to configure your pin once, after which changing modes can be done by calling
`set_input_enable` and/or `set_output_enable`.

```diff
- flex.set_as_input(pull_direction);
+ flex.apply_input_config(&InputConfig::default().with_pull(pull_direction)); // only if needed
+ flex.set_output_enable(false);
+ flex.set_input_enable(true);

- flex.set_as_output(); // or set_as_open_drain(pull_direction)
+ flex.apply_output_config(&OutputConfig::default().with_drive_mode(open_drain_or_push_pull)); // only if needed
+ flex.set_input_enable(false); // optional
+ flex.set_level(initial_level); // optional
+ flex.set_output_enable(true);
```

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

## SPI changes

`Spi::transfer` no longer returns the input buffer.

```diff
-let received = spi.transfer(&mut data[..]).unwrap();
-work_with(received);
+spi.transfer(&mut data[..]).unwrap();
+work_with(&data[..]);
```

## Configuration changes

Some configuration options are now unstable and they require the `unstable` feature to be
enabled. You can learn about a particular option in the [esp-hal documentation](https://docs.espressif.com/projects/rust/esp-hal/latest/).

The `ESP_HAL_CONFIG_PLACE_SPI_DRIVER_IN_RAM` configuration option has been renamed to `ESP_HAL_CONFIG_PLACE_SPI_MASTER_DRIVER_IN_RAM`.
