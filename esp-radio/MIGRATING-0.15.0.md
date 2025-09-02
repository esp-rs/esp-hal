# Migration Guide from 0.15.0 to {{currentVersion}}

## Initialization

The `builtin-scheduler` feature has been removed. The functionality has been moved to `esp_preempt_baremetal`.
`esp_preempt_baremetal` needs to be initialized before calling `esp_radio::init`. Failure to do so will result in an error.

Depending on your chosen OS, you may need to use other `esp_preempt` implementations.

Furthermore, `esp_wifi::init` no longer requires `RNG` or a timer.

```diff
-let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, Rng::new()).unwrap();
+esp_preempt::init(timg0.timer0);
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
+ pub extern "C" fn calloc(number: u32, size: usize) -> *mut u8 ...
+ pub extern "C" fn calloc_internal(number: u32, size: usize) -> *mut u8 ...
+ pub extern "C" fn realloc(ptr: *mut u8, new_size: usize) -> *mut u8 ...
+ pub extern "C" fn get_free_internal_heap_size() -> usize; ...
```

## Scanning Functions

The `scan_with_config_sync_max`, `scan_with_config_sync_max`, `scan_n`, and `scan_n_async` functions have been removed. You can instead use the `scan_with_config_async` or `scan_with_config_sync` funtions while specifying a `max` value in `ScanConfig`.

## Configuration

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

## BuilderLite pattern `AccessPointConfig` and `ClientConfig`

```diff
- let ap_config = Config::AccessPoint({
-         let mut config = AccessPointConfig::default();
-         config.ssid = "esp-radio".into();
-         config
-     });
+ let ap_config = Config::AccessPoint(AccessPointConfig::default().with_ssid("esp-radio".into()));
```