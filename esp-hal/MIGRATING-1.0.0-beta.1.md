# Migration Guide from 1.0.0-beta.1 to 1.0.0-rc.0

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

## RMT Channel generic parameters have changed
Instead of being parameterized by a `const CHANNEL: u8`, channels now take a generic
parameter `Raw: RawChannelAccess<Dir>` where `Dir=Tx` or `Dir=Rx` that identifies the
channel.

```diff
+use esp_hal::rmt::{ConstChannelAccess, Rx, Tx};
+
-let channel: Channel<Blocking, 0> = {
+let channel: Channel<Blocking, ConstChannelAccess<Tx, 0>> = {
     use esp_hal::rmt::TxChannelCreator;
     rmt.channel0().configure(pin, TxChannelConfig::default())
 };
-let channel: Channel<Blocking, 2> = {
+let channel: Channel<Blocking, ConstChannelAccess<Rx, 0>> = {
     use esp_hal::rmt::RxChannelCreator;
     rmt.channel2().configure(pin, RxChannelConfig::default())
 };
```

## RMT ChannelCreator traits have been re-arranged
`TxChannelCreatorAsync` and `RxChannelCreatorAsync` have been removed.
Instead, `TxChannelCreator` and `RxChannelCreator` now carry a `Dm: DriverMode`
generic parameter. `ChannelCreator<Dm, CHANNEL>` thus implements `TxChannelCreator<Dm>`
and `RxChannelCreator<Dm>`, respectively.

Additionally, the `configure` methods have been renamed to `configure_tx` and
`configure_rx` to avoid trait disambiguation issues.

```diff
+use esp_hal::rmt::{RxChannelCreator, TxChannelCreator};
+
 let rmt = rmt.into_async();
-let tx_channel = {
-    use esp_hal::rmt::TxChannelCreatorAsync;
-    rmt.channel0.configure(pin, tx_config)
-};
+ let tx_channel = rmt.channel0.configure_tx(pin, tx_config);
-let rx_channel = {
-    use esp_hal::rmt::RxChannelCreatorAsync;
-    rmt.channel2.configure(pin, rx_config)
-};
+ let rx_channel = rmt.channel2.configure_rx(pin, rx_config);
```

## `ConfigError` variants have been changed

### SPI master

- `UnsupportedFrequency` -> `FrequencyOutOfRange`

### UART

- `UnachievableBaudrate` -> `BaudrateNotAchievable`
- `UnsupportedBaudrate` -> `BaudrateNotSupported`
- `UnsupportedTimeout` -> `TimeoutTooLong`
- `UnsupportedRxFifoThreshold` -> `RxFifoThresholdNotSupported`
- `UnsupportedTxFifoThreshold` -> `TxFifoThresholdNotSupported`

### I2C master

- `FrequencyInvalid` -> `FrequencyOutOfRange`
- `TimeoutInvalid` -> `TimeoutTooLong`
