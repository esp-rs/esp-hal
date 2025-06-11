# Migration Guide from 1.0.0-beta.1 to {{currentVersion}}

## AnyI2c has been moved to `esp_hal::i2c::master`

```diff
-use esp_hal::i2c::AnyI2c;
+use esp_hal::i2c::master_AnyI2c;
```
