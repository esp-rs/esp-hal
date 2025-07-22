# Migration Guide from 0.15.0 to {{currentVersion}}

## Initialization

`esp_wifi::init` no longer requires `RNG`.

```diff
-let esp_wifi_ctrl = init(timg0.timer0, Rng::new()).unwrap();
+let esp_wifi_ctrl = init(timg0.timer0).unwrap();
```
