# Migration Guide from 0.14.0 to {{currentVersion}}

## Deinitialization

`esp_wifi::EspWifiController::deinit` got removed and you should just drop the `EspWifiController` instead.

```diff
- esp_wifi_ctrl.deinit();
+ core::mem::drop(esp_wifi_ctrl);
```
Since `esp_wifi::deinit_unchecked` is now removed there is no unsafe way to forcefully deinit the controller.
Drop the instance of `EspWifiController` instead (see above).
