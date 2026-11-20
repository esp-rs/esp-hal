# esp-provisioning

[![Crates.io](https://img.shields.io/crates/v/esp-provisioning?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-provisioning)
[![docs.rs](https://img.shields.io/docsrs/esp-provisioning?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-provisioning/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.95.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-provisioning?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

ESP-IDF Wi-Fi provisioning for bare-metal (`no_std`) Rust:

- **sec1** — Curve25519 session + proof-of-possession + AES-256-CTR
- **protocomm** — plaintext handlers for `prov-config` and `prov-scan` protobuf
- **softap** / **ble** (default) — SoftAP HTTP and BLE GATT transports with [`run`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/fn.run.html) / [`cancel`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/fn.cancel.html)

The crate returns Wi-Fi credentials with an `esp-radio` [`Ssid`](https://docs.espressif.com/projects/rust/esp-radio/latest/esp_radio/wifi/struct.Ssid.html) and a passphrase `String`; convert the passphrase with `Password::try_from` when building STA config. NVS, STA connect, and reboot stay in the application.

Compatible with the Espressif [ESP BLE Provisioning](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/provisioning/wifi_provisioning.html) apps.

## Usage

```toml
esp-provisioning = { version = "0.1.0", features = ["esp32c6", "log-04"] }
```

Or with a path dependency from this repository:

```toml
esp-provisioning = { path = "../esp-provisioning", features = ["esp32c6", "log-04"] }
```

Enable a chip feature that matches your target. SoftAP and BLE (and `unstable`) are on by default.

Typical SoftAP + BLE session (credentials only — the app still owns STA join):

```rust,ignore
use esp_provisioning::{ProvConfig, ProvError, cancel, cancel_isr, run};

let config = ProvConfig {
    pop: "abcd1234",
    softap_ssid: "PROV_esp",
    ble_name: "PROV_esp",
    // ap_gateway, timeout, controller, ap_device, ble_connector, spawner, rng, rng_ble …
    ..
};

match run(config).await {
    Ok(creds) => {
        // creds.ssid: esp_radio::wifi::Ssid
        // creds.password: heapless::String<64> — Password::try_from(creds.password.as_str()) for STA
    }
    Err(ProvError::Cancelled) => { /* cancel() or cancel_isr() */ }
    Err(ProvError::Timeout) | Err(ProvError::Internal) => {}
}
```

Call `cancel()` from task context or `cancel_isr()` from an ISR to stop an in-flight `run`. Neither tears down Wi-Fi or Bluetooth.

### Example

See [`examples/wifi/embassy_provisioning`](../examples/wifi/embassy_provisioning): menu SoftAP+BLE shell, Esc / hold-C cancel, NVS save, and STA test.

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
