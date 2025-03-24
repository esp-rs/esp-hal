# Migration Guide from v1.0.0-beta.0 to ?

## `critical-section` implementation is now optional

You'll need to enable the `critical-section-impl` feature for ESP-HAL to provide an implementation.

```diff
 # in your Cargo.toml
 esp-hal = { version = "...", features = [
   # ...
+  "critical-section-impl",
 ]}
```
