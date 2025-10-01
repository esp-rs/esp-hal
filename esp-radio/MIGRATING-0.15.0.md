# Migration Guide from 0.15.0 to {{currentVersion}}

## Initialization

The `builtin-scheduler` feature has been removed. The functionality has been moved to `esp_rtos`.
`esp_rtos` needs to be initialized before calling `esp_radio::init`. Failure to do so will result in an error.

Depending on your chosen OS, you may need to use other `esp_rtos` implementations.

Furthermore, `esp_radio::init` no longer requires `RNG` or a timer.

On Xtensa devices (ESP32/S2/S3):

```diff
-let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, Rng::new()).unwrap();
+esp_rtos::start(timg0.timer0);
+let esp_wifi_ctrl = esp_radio::init().unwrap();
```

On RISC-V devices (ESP32-C2/C3/C6/H2) you'll need to also pass `SoftwareInterrupt<0>` to `esp_rtos::start`:

```diff
-let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, Rng::new()).unwrap();
+use esp_hal::interrupt::software::SoftwareInterruptControl;
+let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
+esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);
+let esp_wifi_ctrl = esp_radio::init().unwrap();
```

## Importing

`esp_wifi` crate has been renamed to `esp_radio`

```diff
- esp-wifi = "0.15.0"
+ esp-radio = "{{currentVersion}}"
```

## `EspWifi` prefix has been removed

```diff
- use esp_wifi::EspWifiController;
+ use esp_radio::Controller;
```

## Memory allocation functions

The way to provide your own implementation of heap memory allocations (if not using `esp-alloc`) has changed.

Provide these symbols:

```diff
- pub extern "C" fn esp_wifi_free_internal_heap() ...
- pub extern "C" fn esp_wifi_allocate_from_internal_ram(size: usize) ...
- pub extern "C" fn esp_wifi_deallocate_internal_ram(ptr: *mut u8) ...
+ pub extern "C" fn malloc(size: usize) -> *mut u8 ...
+ pub extern "C" fn malloc_internal(size: usize) -> *mut u8 ...
+ pub extern "C" fn free(ptr: *mut u8) ...
+ pub extern "C" fn free_internal(ptr: *mut u8) ...
+ pub extern "C" fn calloc(number: u32, size: usize) -> *mut u8 ...
+ pub extern "C" fn calloc_internal(number: u32, size: usize) -> *mut u8 ...
+ pub extern "C" fn realloc(ptr: *mut u8, new_size: usize) -> *mut u8 ...
+ pub extern "C" fn get_free_internal_heap_size() -> usize; ...
```

## Scanning Functions

The `scan_with_config_sync_max`, `scan_with_config_sync_max`, `scan_n`, and `scan_n_async` functions have been removed. You can instead use the `scan_with_config_async` or `scan_with_config_sync` funtions while specifying a `max` value in `ScanConfig`.

## Configuration

`esp_radio::wifi::new` now takes a `WifiConfig` struct. The default configuration matches the previous build-time configuration defaults. The corresponding build-time configuration options have been removed.

```diff
 # .cargo/config.toml:
 [env]
-ESP_RADIO_CONFIG_RX_QUEUE_SIZE = 27

 # main.rs
-let (controller, ifaces) = esp_wifi::wifi::new(&ctrl, p.WIFI).unwrap();
+let config = esp_radio::wifi::WifiConfig::default()
+   .with_rx_queue_size(27);
+let (controller, ifaces) = esp_radio::wifi::new(&ctrl, p.WIFI, config).unwrap();
```

The `Configuration`, `ClientConfiguration`, `AccessPointConfiguration`, and `EapClientConfiguration` enums have been renamed to `Config`, `ClientConfig`, `AccessPointConfig`, and `EapClientConfig`:

```diff
use esp_radio::wifi::{
-    AccessPointConfiguration,
-    ClientConfiguration,
-    Configuration,
-    EapClientConfiguration,
+    AccessPointConfig,
+    ClientConfig,
+    Config,
+    EapClientConfig
}
```

Same for `set_configuration()` to `set_config()`:

```diff
- let res = controller.set_configuration(&ap_config);
+ let res = controller.set_config(&ap_config);
```

### BuilderLite pattern `AccessPointConfig` and `ClientConfig`

```diff
- let ap_config = Config::AccessPoint({
-         let mut config = AccessPointConfig::default();
-         config.ssid = "esp-radio".into();
-         config
-     });
+ let ap_config = Config::AccessPoint(AccessPointConfig::default().with_ssid("esp-radio".into()));
```

### BLE

The `BleController` can now be configured using `esp_radio::ble::Config`:

```diff
 let mut connector = BleConnector::new(
     &init,
     peripherals.BT,
+    Config::default().with_task_priority(10),
 );
```

## WifiState

`wifi_state()` is removed and `WifiState` is split into `WifiStaState` and `WifiApState`:

```diff
- if esp_radio::wifi::wifi_state() == WifiState::StaConnected { ... }
+ if esp_radio::wifi::sta_state() == WifiStaState::Connected { ... }
```

## `Mixed` mode has been renamed to `ApSta`

```diff
-    let client_config = Config::Mixed(
-        ClientConfig::default()
-            .with_ssid("ssid".into())
-            .with_password("password".into()),
-        AccessPointConfig::default().with_ssid("esp-radio".into()),
-    );
+    let client_config = Config::ApSta(
+        ClientConfig::default()
+            .with_ssid("ssid".into())
+            .with_password("password".into()),
+        AccessPointConfig::default().with_ssid("esp-radio".into()),
+    );
```

## `PowerSaveMode` is moved to `wifi` module

```diff
    controller
-        .set_power_saving(esp_radio::config::PowerSaveMode::None)
+        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
    .unwrap();
```
