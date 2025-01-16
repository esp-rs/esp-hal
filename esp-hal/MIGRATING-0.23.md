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

