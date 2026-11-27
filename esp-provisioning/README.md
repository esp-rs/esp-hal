# esp-provisioning

[![Crates.io](https://img.shields.io/crates/v/esp-provisioning?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-provisioning)
[![docs.rs](https://img.shields.io/docsrs/esp-provisioning?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-provisioning/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.95.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-provisioning?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

ESP-IDF Wi-Fi provisioning for bare-metal (`no_std`) Rust:

- **sec1** — Curve25519 session + proof-of-possession + AES-256-CTR
- **protocomm** — `wifi_provisioning` handlers for `prov-config`, `prov-scan`, and `prov-ctrl`
- **softap** / **ble** (default) — one serve future per transport

The application owns the Wi-Fi controller, the `embassy-net` stacks, the BLE connector, the RNG, and the station join/verify loop. Run [`softap::serve`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/softap/fn.serve.html) and/or [`ble::serve`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/ble/fn.serve.html) alongside a verify loop that awaits [`Session::credentials`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/struct.Session.html#method.credentials), attempts the station join + DHCP, and calls [`Session::report`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/struct.Session.html#method.report) with [`StaState::Connected`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/enum.StaState.html#variant.Connected) or [`StaState::Failed`](https://docs.espressif.com/projects/rust/esp-provisioning/latest/esp_provisioning/enum.StaState.html#variant.Failed).

On failure, the transports remain live so the Espressif phone app can send `prov-ctrl` (`CtrlReset`) and submit new credentials on the same link. On success, the transports deliver the `Connected` `GetStatus` response before finishing. Dropping a serve future cancels that transport.

Credentials stay in RAM for the session; the crate does not write flash and never logs passphrases. Credentials use an `esp-radio` [`Ssid`](https://docs.espressif.com/projects/rust/esp-radio/latest/esp_radio/wifi/struct.Ssid.html) and a passphrase `String`; convert the passphrase with `Password::try_from` when building STA config. NVS storage and reboot stay in the application.

Protobuf sources are vendored from ESP-IDF v5.4.2 `wifi_provisioning` and `protocomm` (`components/wifi_provisioning/proto` and `components/protocomm/proto`; not `network_provisioning`). Generated code is committed, so a firmware build does not need `protoc`.

Compatible with the Espressif [ESP BLE Provisioning](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/provisioning/provisioning.html) apps.

## Usage

```toml
esp-provisioning = { version = "0.1.0", features = ["esp32c6", "log-04", "unstable"] }
```

Or with a path dependency from this repository:

```toml
esp-provisioning = { path = "../esp-provisioning", features = ["esp32c6", "log-04", "unstable"] }
```

Enable a chip feature that matches your target. `softap` and `ble` are on by default. They do not turn on this crate's `unstable` feature; the application enables `unstable` (and `esp-radio/unstable`) itself.

```rust,ignore
use esp_provisioning::{Config, Session, StaFailReason, StaState, ble, softap};

let config = Config::default();
let session = Session::new();

// Pre-scan in Station mode, bring up AP+STA (if using SoftAP) and BleConnector (if using BLE),
// then run softap::serve / ble::serve concurrently with your verify loop:
//   loop {
//       let creds = session.credentials().await;
//       match try_connect_and_dhcp(&mut controller, &sta_stack, &creds, &aps).await {
//           Ok((ipv4, info)) => {
//               session.report(StaState::Connected { ipv4, info });
//               break;
//           }
//           Err(reason) => session.report(StaState::Failed(reason)),
//       }
//   }
```

`Config` is a builder (`with_pop`, `with_softap_ssid`, `with_ble_name`, `with_gateway`) plus `Default` (`abcd1234`, SSID `PROV_esp`, BLE name `PROV_esp`, gateway `192.168.4.1`).

### Example

[`examples/wifi/embassy_provisioning`](../examples/wifi/embassy_provisioning) pre-scans nearby APs, starts SoftAP and BLE, verifies the provisioned credentials against the router (with DHCP and multi-attempt retry), and prints the obtained IPv4 address.

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
