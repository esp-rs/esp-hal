# Migration Guide from v0.23.x to v1.0.0-beta.0

## `Async` drivers can no longer be sent between cores and executors

To work around this limitation, send the blocking driver, and configure it into `Async` mode
in the target context.

```diff
 #[embassy_executor::task]
-async fn interrupt_driven_task(mut spi: Spi<'static, Async>) {
+async fn interrupt_driven_task(spi: Spi<'static, Blocking>) {
+    let mut spi = spi.into_async();
     ...
 }

 let spi = Spi::new(...)
     .unwrap()
     // ...
-    .into_async();

 send_spawner.spawn(interrupt_driven_task(spi)).unwrap();
```

## RMT changes

### Configurations structs now support the builder-lite pattern

The `TxChannelConfig` and `RxChannelConfig` structs now support the builder-lite pattern.
Thus, explicit initialization of all fields can be replaced by only the necessary setter methods:

```diff
  let mut channel = rmt
      .channel0
      .configure(
          peripherals.GPIO1,
-         TxChannelConfig {
-             clk_divider: 1,
-             idle_output_level: false,
-             idle_output: false,
-             carrier_modulation: false,
-             carrier_high: 1,
-             carrier_low: 1,
-             carrier_level: false,
-         },
+          TxChannelConfig::default().with_clk_divider(1)
      )
     .unwrap();
```

### Some configuration fields now take `gpio::Level` instead of `bool`

Fields related to the carrier level in the channel configuration structs now
take the more descriptive `gpio::Level` type instead of a plain `bool`.

```diff
  let mut tx_channel = rmt
      .channel0
      .configure(
          peripherals.GPIO1,
          TxChannelConfig::default()
              .with_clk_divider(1)
-             .with_idle_output_level(false)
+             .with_idle_output_level(Level::Low)
-             .with_carrier_level(true)
+             .with_carrier_level(Level::High)
      )
     .unwrap();

  let mut rx_channel = rmt
      .channel1
      .configure(
          peripherals.GPIO2,
          RxChannelConfig::default()
              .with_clk_divider(1)
              .with_carrier_modulation(true)
              .with_carrier_high(1)
              .with_carrier_low(1)
-             .with_carrier_level(false),
+             .with_carrier_level(Level::Low),
      )
     .unwrap();
```

### `PulseCode` now uses `gpio::Level` instead of `bool` to specify output levels

The more descriptive `gpio::Level` enum is now used to specify output levels of `PulseCode`:

```diff
+ use esp_hal::gpio::Level;
+
- let code = PulseCode::new(true, 200, false, 50);
+ let code = PulseCode::new(Level::High, 200, Level::Low, 50);
```

## UART changes

Uart `write_bytes` is now blocking and return the number of bytes written. `read_bytes` will block until it fills the provided buffer with received bytes, use `read_buffered_bytes` to read the available bytes without blocking.

e.g.

```diff
- uart.write(0x42).ok();
- let read = block!(ctx.uart.read());
+ let data: [u8; 1] = [0x42];
+ uart.write_bytes(&data).unwrap();
+ let mut byte = [0u8; 1];
+ uart.read_bytes(&mut byte);
```

### UART errors have been split into `TxError` and `RxError`.

`read_*` and `write_*` functions now return different types. In practice this means you no longer
need to check for RX errors that can't be returned by `write_*`.

The error type used by `embedded-io` has been updated to reflect this. A new `IoError` enum has been
added for `embedded-io` errors associated to the unsplit `Uart` driver. On `Uart` (but not `UartRx`
or `UartTx`) TX-related trait methods return `IoError::Tx(TxError)`, while RX-related methods return
`IoError::Rx(RxError)`.

### UART halves have their configuration split too

`Uart::Config` structure now contains separate `RxConfig` and `TxConfig`:

```diff
- let config = Config::default().with_rx_fifo_full_threshold(30);
+ let config = Config::default()
+     .with_rx(RxConfig::default()
+       .with_fifo_full_threshold(30)
+ );
```

## `timer::wait` is now blocking

```diff
periodic.start(100.millis()).unwrap();
- nb::block!(periodic.wait()).unwrap();
+ periodic.wait();
```

## SPI Changes

`spi::DataMode` changed the meaning of `DataMode::Single` - it now means 3-wire SPI (using one data line).

Use `DataMode::SingleTwoDataLines` to get the previous behavior.

```diff
- DataMode::Single,
+ DataMode::SingleTwoDataLines,
```

`Spi` now offers both, `with_mosi` and `with_sio0`. Consider using `with_sio` for half-duplex SPI except for [DataMode::SingleTwoDataLines] or for a mixed-bus.

## Removed `flip-link` Feature

The `flip-link` feature is removed and replaced by the `ESP_HAL_CONFIG_FLIP_LINK` option.

Cargo.toml
```diff
- esp-hal = { version = "0.23.0", features = ["flip-link"]}
+ esp-hal = "0.23.0"
```

config/config.toml
```diff
[env]
+ ESP_HAL_CONFIG_FLIP_LINK = "true"
```

## Removed `psram-quad`/`prsram-octal` Feature

The features `psram-quad`/`prsram-octal` are replaced by a single `psram` feature and an additional config option (`ESP_HAL_CONFIG_PSRAM_MODE`).

`ESP_HAL_CONFIG_PSRAM_MODE` defaults to `quad` and (for ESP32-S3) also allows `octal`.

Cargo.toml
```diff
- esp-hal = { version = "0.23.0", features = ["psram-octal"]}
+ esp-hal = { version = "0.23.0", features = ["psram"]}
```

config/config.toml
```diff
[env]
+ ESP_HAL_CONFIG_PSRAM_MODE = "octal"
```

## PARL_IO changes
Parallel IO now uses the newer DMA Move API.

Changes on the TX side
```diff
  let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, 32000);
+ let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

- let transfer = parl_io_tx.write_dma(&tx_buffer).unwrap();
- transfer.wait().unwrap();
+ let transfer = parl_io_tx.write(dma_tx_buf.len(), dma_tx_buf).unwrap();
+ (result, parl_io_tx, dma_tx_buf) = transfer.wait();
+ result.unwrap();
```

Changes on the RX side
```diff
  let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32000, 0);
+ let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
- let transfer = parl_io_rx.read_dma(&mut rx_buffer).unwrap();
- transfer.wait().unwrap();
+ let transfer = parl_io_rx.read(Some(dma_rx_buf.len()), dma_rx_buf).unwrap();
+ (_, parl_io_rx, dma_rx_buf) = transfer.wait();
```

On the RX side, the `EofMode` is now decided at transfer time, rather than config time.
- `EofMode::ByteLen` -> `Some(<number of bytes to receive>)`
- `EofMode::EnableSignal` -> `None`

## GPIO changes

GPIO drivers now take configuration structs.

```diff
- Input::new(peripherals.GPIO0, Pull::Up);
+ Input::new(peripherals.GPIO0, InputConfig::default().with_pull_direction(Pull::Up));
 
- Output::new(peripherals.GPIO0, Level::Low);
+ Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
```

The OutputOpenDrain driver has been removed. You can use `Output` instead with
`DriveMode::OpenDrain`. The input-related methods of `OutputOpenDrain` (`level`,
`is_high`, `is_low`) are available through the (unstable) `Flex` driver.

```diff
- OutputOpenDrain::new(peripherals.GPIO0, Level::Low);
+ Output::new(
     peripherals.GPIO0,
     Level::Low,
     OutputConfig::default()
         .with_drive_mode(DriveMode::OpenDrain),
 );
```

## AES DMA driver changes
AES now uses the newer DMA move API.

```diff
  let (output, rx_descriptors, input, tx_descriptors) = dma_buffers!(32000);
+ let mut output = DmaRxBuf::new(rx_descriptors, output).unwrap();
+ let mut input = DmaTxBuf::new(tx_descriptors, input).unwrap();

  let mut aes = Aes::new(peripherals.AES).with_dma(
      dma_channel,
-     rx_descriptors,
-     tx_descriptors,
  );

  let transfer = aes
      .process(
-         &input,
-         &mut output,
+         output.len().div_ceil(16), // Number of blocks
+         output,
+         input,
          Mode::Encryption128,
          CipherMode::Ecb,
          keybuf,
      )
+     .map_err(|e| e.0)
      .unwrap();
  transfer.wait();
```

## I2C Changes

All async functions now include the `_async` postfix. Additionally the non-async functions are now available in async-mode.

```diff
- let result = i2c.write_read(0x77, &[0xaa], &mut data).await;
+ let result = i2c.write_read_async(0x77, &[0xaa], &mut data).await;
```

## ADC Changes

The ADC driver has gained a new `Async`/`Blocking` mode parameter.
NOTE: Async support is only supported in ESP32C3 and ESP32C6 for now


```diff
- Adc<'d, ADC>
+ Adc<'d, ADC, Blocking>
```

## time API changes

ESP-HAL no longer publicly exposes `fugit` and no longer exposes the concept of a `tick`.
This comes with a number of changes:

- The `RateExtU32` and similar traits are no longer used, which means `.kHz()` and similar suffix
  conversions are no longer available. A number of matching constructors are available. For example,
  instead of `1.MHz()` you need to write `Rate::from_mhz(1)`.
- Methods on `esp_hal::time` types are named differently.
  - Getters are prefixed with `as_`, e.g. `Duration::as_secs`.
  - Constructors are prefixed with `from`, e.g. `Rate::from_mhz`.
  - A number of functions that convert from a number into a time type (e.g. `Duration::from_ticks`)
    are not available. Use conversions from physical units of time, like `Duration::from_millis`.
