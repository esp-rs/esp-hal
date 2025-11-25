# Migration Guide from 0.17.0 to {{currentVersion}}

## The `serde` feature has been removed

You will have to provide your own datatypes that you wish to serialize/deserialize. For this, you have two options:

1. Implement the `Serialize` and `Deserialize` traits [manually](https://serde.rs/impl-serialize.html) for your custom types.
2. Implement the types and conversions to/from esp-radio types.

For example, you may want to mirror `ScanMethod`, so that you can store it in flash as a configuraton option. In this case, you could do the following:

```rust
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize)] // and possibly more
pub enum ScanMethod {
    Fast,
    AllChannels,
}

impl From<ScanMethod> for esp_radio::wifi::ScanMethod {
    fn from(scan_method: ScanMethod) -> Self {
        match scan_method {
            ScanMethod::Fast => Self::Fast,
            ScanMethod::AllChannels => Self::AllChannels,
        }
    }
}
```

## Many types/functions in the `wifi` module are now in submodules

- `PromiscuousPkt` and `Sniffer` are now located in `wifi::sniffer`.
- `AccessPointInfo` and `AccessPointConfig` are now located in `wifi::ap`.
- `ClientConfig` is now located in `wifi::sta`.
- `EapFastConfig`, `TlsPhase2Method` and `EapClientConfig` are now located in `wifi::ap::eap`.

You will need to update any imports in your project accordingly.

## `esp_radio::init()` and `Controller` is no longer available

WiFi initialization:
```diff
-    let esp_radio_ctrl = esp_radio::init().unwrap();
-    let (mut controller, interfaces) =
-        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default()).unwrap();
+    let (mut controller, interfaces) = esp_radio::wifi::new(wifi, Default::default()).unwrap();
```

BLE initialization:
```diff
-    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
-    let radio = RADIO.init(esp_radio::init().unwrap());
-    let connector = BleConnector::new(radio, bluetooth, Default::default()).unwrap();
+    let connector = BleConnector::new(bluetooth, Default::default()).unwrap();
```

## The terms `Client`, `Sta`, `Ap` have been aligned to `Station` and `AccessPoint`

```diff
-    let mut ap_device = interfaces.ap;
+    let mut ap_device = interfaces.access_point;

-    let mut sta_device = interfaces.sta;
+    let mut sta_device = interfaces.station;
```

```diff
-    let client_config = ModeConfig::ApSta(
-        ClientConfig::default()
+    let station_config = ModeConfig::AccessPointStation(
+        StationConfig::default()
```

```diff
-    let client_config = ModeConfig::Client(
-        ClientConfig::default()
+    let station_config = ModeConfig::Station(
+        StationConfig::default()
```