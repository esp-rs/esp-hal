# esp-wifi

[![Crates.io](https://img.shields.io/crates/v/esp-wifi?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-wifi)
[![docs.rs](https://img.shields.io/docsrs/esp-wifi?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.esp-rs.org/esp-hal)
![MSRV](https://img.shields.io/badge/MSRV-1.76-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-wifi?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A WiFi, BLE and ESP-NOW driver for Espressif microcontrollers.

## Current support

If a cell contains an em dash (&mdash;) this means that the particular feature is not present for a chip. A check mark (✓) means that some driver implementation exists. A Tilde (&tilde;) means it is implemented but buggy. An empty cell means that the feature is present in the chip but not implemented yet.

|          | [Wifi](https://github.com/esp-rs/esp-wifi/issues/94) | [BLE](https://github.com/esp-rs/esp-wifi/issues/93) | [Coex](https://github.com/esp-rs/esp-wifi/issues/92) | ESP-NOW |
| :------: | :--------------------------------------------------: | :-------------------------------------------------: | :--------------------------------------------------: | :-----: |
|  ESP32   |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C2 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C6 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-H2 |                       &mdash;                        |                          ✓                          |                       &mdash;                        | &mdash; |
| ESP32-S2 |                          ✓                           |                       &mdash;                       |                       &mdash;                        |    ✓    |
| ESP32-S3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |

Minimum supported Rust compiler version: 1.72.0.0

## Usage

### Importing

Ensure that the right features are enabled for your chip. See [Examples](https://github.com/esp-rs/esp-hal/tree/main/examples#examples) for more examples.

```toml
[dependencies.esp-wifi]
# A supported chip needs to be specified, as well as specific use-case features
features = ["esp32s3", "wifi", "esp-now"]
```

### Link configuration

Make sure to include the rom functions for your target:

```toml
# .cargo/config.toml
rustflags = [
    "-C", "link-arg=-Tlinkall.x",
    "-C", "link-arg=-Trom_functions.x",
]
```

At the time of writing, you will already have the `linkall` flag if you used `cargo generate`. Generating from a template does not include the `rom_functions` flag.

### Optimization Level

It is necessary to build with optimization level 2 or 3 since otherwise, it might not even be able to connect or advertise.

To make it work also for your debug builds add this to your `Cargo.toml`

```toml
[profile.dev.package.esp-wifi]
opt-level = 3
```

### Xtensa considerations

Within this crate, `CCOMPARE0` CPU timer is used for timing, ensure that in your application you are not using this CPU timer.

## USB-SERIAL-JTAG

When using USB-SERIAL-JTAG (for example by selecting `jtag-serial` in [`esp-println`](https://crates.io/crates/esp-println)) you have to activate the feature `phy-enable-usb`.

Don't use this feature if you are _not_ using USB-SERIAL-JTAG as it might reduce WiFi performance.

## Tuning

The defaults used by `esp-wifi` and the examples are rather conservative. It is possible to change a few of the important settings.

See [Tuning](./tuning.md) for details

## Missing / To be done

- Support for non-open SoftAP

## Directory Structure

- `src/timer/`: systimer code used for timing and task switching
- `src/preemt/`: a bare minimum RISCV and Xtensa round-robin task scheduler
- `src/compat/`: code needed to emulate enough of an (RT)OS to use the driver
  - `common.rs`: basics like semaphores and recursive mutexes
  - `timer_compat.rs`: code to emulate timer related functionality
- `examples/*.rs`: examples

## Driver version

This uses the WiFi drivers from https://github.com/esp-rs/esp-wireless-drivers-3rdparty

v5.1.2-602-gdb1e54a0c5-dirty commit db1e54a0c537d8b2cc2bd109ee88b50e1ca0ea80

https://github.com/esp-rs/esp-wireless-drivers-3rdparty/ (commit ca2809144cf6d2f89d413f1d415f1c4454ee6249)

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
