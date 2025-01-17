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

### SPI master driver configuration

SPI `Config::with_frequency` has been renamed to `with_clock`. This function now takes either
a `fugit::HertzU32` or `BusClockConfig`.

```diff
 let config = Config::default()
-   .with_frequency(10.MHz());
+   .with_clock(10.MHz());
```

The new structure supports configuring a clock source, although currently only `ClockSource::Apb`
is available.
