# esp-radio

[![Crates.io](https://img.shields.io/crates/v/esp-radio?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-radio)
[![docs.rs](https://img.shields.io/docsrs/esp-radio?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-radio/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.88.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-radio?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A WiFi, BLE and ESP-NOW driver for Espressif microcontrollers.

This package also includes low-level [IEEE 802.15.4] driver for the ESP32-C6 and ESP32-H2

Note that this crate currently requires you to enable the `unstable` feature on `esp-hal`.

[IEEE 802.15.4]: https://en.wikipedia.org/wiki/IEEE_802.15.4

## Current support

If a cell contains an em dash (&mdash;) this means that the particular feature is not present for a chip. A check mark (✓) means that some driver implementation exists.

|          | `Wi-Fi`| `BLE` | `Coex (Wi-Fi + BLE)` | `ESP-NOW` | `IEEE 802.15.4`
| :------: | :--------------------------------------------------: | :-------------------------------------------------: | :--------------------------------------------------: | :-----: | :-----: |
|  ESP32   |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    | &mdash; |
| ESP32-C2 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    | &mdash; |    
| ESP32-C3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    | &mdash; |
| ESP32-C6 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |    ✓    |
| ESP32-H2 |                       &mdash;                        |                          ✓                          |                       &mdash;                        | &mdash; |    ✓    |
| ESP32-S2 |                          ✓                           |                       &mdash;                       |                       &mdash;                        |    ✓    | &mdash; |
| ESP32-S3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    | &mdash; |

## Missing / To be done

- Support for non-open SoftAP

## Bluetooth stack
We recommend using [`TrouBLE`] as the Bluetooth stack. You can find detailed examples [here].

[`TrouBLE`]: https://github.com/embassy-rs/trouble/tree/main
[here]: https://github.com/embassy-rs/trouble/tree/main/examples/esp32

## Driver version

This uses the WiFi drivers from https://github.com/esp-rs/esp-wireless-drivers-3rdparty

v5.1.2-602-gdb1e54a0c5-dirty commit db1e54a0c537d8b2cc2bd109ee88b50e1ca0ea80

https://github.com/esp-rs/esp-wireless-drivers-3rdparty/ (commit ca2809144cf6d2f89d413f1d415f1c4454ee6249)

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
