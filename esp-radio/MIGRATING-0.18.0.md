# Migration Guide from 0.18.0 to {{currentVersion}}

## `wifi::new()` no longer returns `Interfaces`

`wifi::new()` now returns just a `WifiController` instead of `(WifiController, Interfaces)`.

`Interface` is now a lifetime-free singleton — create it via `Interface::station()` or
`Interface::access_point()` before or after calling `wifi::new()`. ESP-NOW and Sniffer
instances are obtained from the controller.

```rust
// Before
let (mut controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, config)?;
let wifi_interface = interfaces.station;

// After
let wifi_interface = esp_radio::wifi::Interface::station();
let mut controller = esp_radio::wifi::new(peripherals.WIFI, config)?;
```

For ESP-NOW and Sniffer, use the controller methods. The returned instances borrow the
controller, so the controller must outlive them:

```rust
// Before
let (_controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, config)?;
let mut sniffer = interfaces.sniffer;
let esp_now = interfaces.esp_now;

// After
let controller = esp_radio::wifi::new(peripherals.WIFI, config)?;
let mut sniffer = controller.sniffer();
let esp_now = controller.esp_now();
```

`Interface` is no longer `Clone` or `Copy`. Each mode (station / access point) is a
singleton — only one instance can exist at a time. Dropping it releases the slot so it
can be created again.
