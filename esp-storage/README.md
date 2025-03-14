# esp-storage

[![Crates.io](https://img.shields.io/crates/v/esp-storage?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-storage)
[![docs.rs](https://img.shields.io/docsrs/esp-storage?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-storage/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-storage?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

This implements [`embedded-storage`](https://github.com/rust-embedded-community/embedded-storage) traits to access unencrypted ESP32 flash.

## Current support

ESP32, ESP32-C2, ESP32-C3, ESP32-C6, ESP32-H2, ESP32-S2 and ESP32-S3 are supported in `esp-storage`

## Important

For ESP32 it is necessary to build with [optimization level](https://doc.rust-lang.org/cargo/reference/profiles.html#opt-level) 2 or 3.

To make it work also for `debug` builds add this to your `Cargo.toml`

```toml
[profile.dev.package.esp-storage]
opt-level = 3
```

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
