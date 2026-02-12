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
- The `CsiConfig` struct is now located in `wifi::csi`.
- The `ScanMethod`, `ScanTypeConfig`, and `ScanConfig` types are now located in `wifi::scan`.

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

## `Wifi::Event` uses full names for events instead of acronyms

```diff
-    impl StationWpsErPin<'_> {
+    impl StationWifiProtectedStatusEnrolleePin<'_> {
```

```diff
-    impl StationWpsErPin<'_> {
+    impl StationWifiProtectedStatusEnrolleePin<'_> {
```

## `ap_state` and `sta_state` are removed (together with the `WifiApState`/`WifiStaState`)

Use `WifiController::is_started` and `WifiController::is_connected` instead.

```diff
-      if esp_radio::wifi::station_state() == WifiStationState::Connected {
+      if matches!(controller.is_connected(), Ok(true)) {
```

## Support for non-async `start`,`stop`,`scan`,`connect` and `disconnect` in `WifiController` has been removed

WiFi is intended to be used in an async environment. 

Therefore the following functions have been removed from `WifiController`:

- `start`
- `stop`
- `scan`
- `connect`
- `disconnect`

Use the `*_async` counterparts instead. This might require you to migrate your code to use `embassy`.

The `smoltcp` feature has been removed completely. Use `embassy-net` and async instead.

## Wi-Fi Event System

The Wi-Fi event handling system has been completely refactored

#### Waiting for Disconnection

The generic `wait_for_event` has been replaced by more specific and descriptive methods. These also return more detailed information.

```diff
- use esp_radio::wifi::WifiEvent;
-
- // Wait until the station is disconnected
- controller
-     .wait_for_event(WifiEvent::StationDisconnected)
-     .await;
- println!("Station disconnected");
+ // Wait until the station is disconnected
+ let info = controller.wait_for_disconnect_async().await.ok();
+ println!("Disconnected: {:?}", info);
```

#### Handling Access Point Client Events

Previously, you would wait for `AccessPointStationConnected` or `AccessPointStationDisconnected` events. This is now handled by `wait_for_access_point_connected_event_async`.

```diff
- use esp_radio::wifi::WifiEvent;
-
- // In your AP task loop
- controller.wait_for_events(
-     WifiEvent::AccessPointStationConnected | WifiEvent::AccessPointStationDisconnected,
-     true
- ).await;
- // No way to know the details here
+ // In your AP task loop
+ let event = controller
+     .wait_for_access_point_connected_event_async()
+     .await
+     .unwrap();
+ match event {
+     esp_radio::wifi::AccessPointStationEventInfo::Connected(info) => {
+         println!("Station connected: {:?}", info);
+     }
+     esp_radio::wifi::AccessPointStationEventInfo::Disconnected(info) => {
+         println!("Station disconnected: {:?}", info);
+     }
+ }
```

## SSID Type Change

The API has been updated to use a dedicated SSID type instead of String for SSID values.

Builders now take `Into<Ssid>` instead of a String.

e.g.

```diff
 let station_config = Config::Station(
     StationConfig::default()
-        .with_ssid("MyNetwork".into())  // Used .into() to convert to String
+        .with_ssid("MyNetwork")  // Ssid implements From<&str>
         .with_password("password".into()),
 );
```

Migration Steps

1. Remove `.into()` calls: Remove .into() calls when passing SSIDs to configuration methods:
    - Change .with_ssid("network".into()) to .with_ssid("network")

2. Update SSID access: When retrieving SSIDs from structs or events, use .as_str() to get the string representation:
    - Change ssid_value to ssid_value.as_str() when you need a string slice

## Simplified WiFi Connection State Functions

The `is_connected()` and `is_started()` functions in `WifiController` have been simplified to return a plain boolean instead of a `Result<bool, WifiError>`.

This change removes the need to handle potential errors when checking connection states, making the API simpler to use.

```diff
- if matches!(controller.is_connected(), Ok(true)) {
+ if controller.is_connected() {
      println!("Station is connected");
  }

- if matches!(controller.is_started(), Ok(true)) {
+ if controller.is_started() {
      println!("Wi-Fi is started");
  }
```
