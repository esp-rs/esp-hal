# Migration Guide from 0.18.0 to {{currentVersion}}

## Wi-Fi

### `wifi::new()` replaced with `WifiController::new()`

The free function `esp_radio::wifi::new()` has been replaced by `esp_radio::wifi::WifiController::new()`.

```rust
// Before
let controller = esp_radio::wifi::new(peripherals.WIFI, Default::default())?;

// After
let controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())?;
```

### Station info types moved to `sta` module and renamed

The following types have been moved from `esp_radio::wifi` into `esp_radio::wifi::sta` and renamed:

| Before | After |
|--------|-------|
| `ConnectedStationInfo` | `sta::ConnectedInfo` |
| `DisconnectedStationInfo` | `sta::DisconnectedInfo` |

### Access point info types moved to `ap` module and renamed

The following types have been moved from `esp_radio::wifi` into `esp_radio::wifi::ap` and renamed:

| Before | After |
|--------|-------|
| `AccessPointStationConnectedInfo` | `ap::ConnectedInfo` |
| `AccessPointStationDisconnectedInfo` | `ap::DisconnectedInfo` |
| `AccessPointStationEventInfo` | `ap::EventInfo` |

### `max_connections` and `dtim_period` setters are now unstable

`AccessPointConfig::with_max_connections()` and `AccessPointConfig::with_dtim_period()` now require the `unstable` feature. If you use these setters, enable the `unstable` feature on `esp-radio`.

### `WifiController::new()` no longer returns `Interfaces`

`WifiController::new()` now returns just a `WifiController` instead of `(WifiController, Interfaces)`.

`Interface` is now a lifetime-free singleton — create it via `Interface::station()` or
`Interface::access_point()` before or after calling `WifiController::new()`. ESP-NOW and Sniffer
instances are obtained from the controller.

```rust
// Before
let (mut controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, config)?;
let wifi_interface = interfaces.station;

// After
let wifi_interface = esp_radio::wifi::Interface::station();
let mut controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, config)?;
```

For ESP-NOW and Sniffer, use the controller methods. The returned instances borrow the
controller, so the controller must outlive them:

```rust
// Before
let (_controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, config)?;
let mut sniffer = interfaces.sniffer;
let esp_now = interfaces.esp_now;

// After
let controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, config)?;
let mut sniffer = controller.sniffer();
let esp_now = controller.esp_now();
```

`Interface` is no longer `Clone` or `Copy`. Each mode (station / access point) is a
singleton — only one instance can exist at a time. Dropping it releases the slot so it
can be created again.
