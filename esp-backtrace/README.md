# esp-backtrace - backtrace for ESP32 bare-metal

[![Crates.io](https://img.shields.io/crates/v/esp-backtrace?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-backtrace)
[![docs.rs](https://img.shields.io/docsrs/esp-backtrace?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-backtrace/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.86.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-backtrace?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Supports the ESP32, ESP32-C2/C3/C6, ESP32-H2, ESP32-P4, and ESP32-S2/S3. Optional panic handler is included, which can be enabled via its respective feature.

Please note that when targeting a RISC-V device, you **need** to force frame pointers (i.e. `"-C", "force-frame-pointers",` in your `.cargo/config.toml`); this is **not** required for Xtensa.

You can get an array of backtrace addresses (currently limited to 10) via `arch::backtrace()` if
you want to create a backtrace yourself (i.e. not using the panic handler).

When using the panic handler make sure to include `use esp_backtrace as _;`.

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
