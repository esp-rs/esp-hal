# Migration Guide from 0.13.x to 0.14.x

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
