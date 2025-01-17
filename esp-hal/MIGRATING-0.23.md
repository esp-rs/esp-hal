# Migration Guide from v0.23.x to v?.??.?

## RMT changes

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

## GPIO changes

GPIO drivers now take configuration structs, and their constructors are fallible.

```diff
- Input::new(peripherals.GPIO0, Pull::Up);
+ Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up)).unwrap();
- Output::new(peripherals.GPIO0, Level::Low);
+ Output::new(peripherals.GPIO0, OutputConfig::default().with_level(Level::Low)).unwrap();
- OutputOpenDrain::new(peripherals.GPIO0, Level::Low, Pull::Up);
+ OutputOpenDrain::new(
+     peripherals.GPIO0,
+     OutputOpenDrainConfig::default().with_level(Level::Low).with_pull(Pull::Up)
+ ).unwrap();
```
