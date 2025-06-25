# Migration Guide from 1.0.0-beta.1 to {{currentVersion}}

## AnyI2c and AnySpi have been moved to mode-specific submodules

```diff
-use esp_hal::i2c::AnyI2c;
+use esp_hal::i2c::master::AnyI2c;
```

`AnySpi` has been separated into master and slave counterparts.

```diff
-use esp_hal::spi::AnySpi;
+use esp_hal::spi::master::AnySpi;
+// or:
+use esp_hal:spi::slave::AnySpi;
```

## SPI DataMode has been moved into `esp_hal::spi::master`

```diff
-use esp_hal::spi::DataMode;
+use esp_hal::spi::master::DataMode;
```
