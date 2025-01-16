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
