# Migration Guide from v0.23.x to v?.??.?

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
