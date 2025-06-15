# Migration Guide from 0.13.x to 0.14.0

## Initializing esp-wifi

esp-wifi can no longer be initialized with a `PeriodicTimer`.

## Initializing esp-now

An instance of `esp-now` is now available via the wifi interfaces - there are no separate constructors anymore.

```diff
-    let mut esp_now = esp_wifi::esp_now::EspNow::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();
+    let (mut controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, wifi).unwrap();
+    controller.set_mode(esp_wifi::wifi::WifiMode::Sta).unwrap();
+    controller.start().unwrap();
+
+    let mut esp_now = interfaces.esp_now;
```

## Getting the sniffer

The sniffer is now part of `Interfaces`

```diff
-    let mut sniffer = controller.take_sniffer().unwrap();
+    let mut sniffer = interfaces.sniffer;
```

## The public API is using `alloc` instead of `heapless` now

General usage doesn't change with some small exceptions.

```diff
-    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
+    let res: Result<alloc::vec::Vec<AccessPointInfo>, WifiError> = controller.scan_n(10);
```

Some code can be simplified now.
```diff
-        ssid: SSID.try_into().unwrap(),
-        password: PASSWORD.try_into().unwrap(),
+        ssid: SSID.into(),
+        password: PASSWORD.into(),
```

## `AccessPointInfo` doesn't include the `protocols` field anymore

That field was never populated and has been removed.
