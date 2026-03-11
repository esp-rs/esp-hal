# esp-rom-sys

[![Crates.io](https://img.shields.io/crates/v/esp-rom-sys?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-rom-sys)
[![docs.rs](https://img.shields.io/docsrs/esp-rom-sys?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-rom-sys/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-rom-sys?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

ROM code support. This is an implementation detail of the esp-hal ecosystem crates.

This includes the definition of ROM code function addresses.

For some targets (ESP32 currently) we will use `libesp_rom.a` from ESP-IDF. The code here corresponds to [ESP-IDF v5.3.1](https://github.com/espressif/esp-idf/blob/v5.3.1/components/esp_rom/patches/esp_rom_spiflash.c)

Since there can be only one version of this crate used in a dependency tree this is expected to never see a major version bump. (i.e. all releases are patch releases)

That said, we cannot remove anything here. Degrading a symbol from hard linkage to weak linkage is allowed.

## [Documentation](https://docs.espressif.com/projects/rust/esp-rom-sys/latest/)

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
