# esp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-hal)
![Crates.io](https://img.shields.io/crates/l/esp-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

The `esp-hal` package aims to provide a safe, idiomatic Hardware Abstraction Layer (HAL) for the entire family of ESP32 devices from Espressif.

This package implements both blocking and, when able, asynchronous drivers for the various peripherals. At this time the blocking APIs are used by default, with all asynchronous functionality gated behind the `async` feature. See the package documentation for more information on its features.

Most traits defined by the [embedded-hal] family of packages are implemented as applicable.

[embedded-hal]: https://github.com/rust-embedded/embedded-hal

## [Documentation]

[documentation]: https://docs.rs/esp-hal/

## Usage

Before using `esp-hal`, ensure that you have configured your [development environment] correctly, and the [required tooling] has been installed.

When starting a new project using `esp-hal`, we strongly recommend you generate a project skeleton using [cargo-generate] and [esp-template]. This will take much of the guesswork out of the process and give you a starting point to build an application from.

Much of the functionality available is feature-gated, so be sure to refer to the documentation to read about all available Cargo features.

[development environment]: https://esp-rs.github.io/book/installation/index.html
[required tooling]: https://esp-rs.github.io/book/tooling/espflash.html
[cargo-generate]: https://github.com/cargo-generate/cargo-generate/
[esp-template]: https://github.com/esp-rs/esp-template/

### Supporting Packages

A number of additional packages are available which add additional functionality beyond the HAL.

Within this repository, the [esp-lp-hal] package provides support for the (ultra-)low-power RISC-V coprocessors found aboard the ESP32-C6, ESP32-S2, and ESP32-S3.

There is also the [esp-wifi] package, which provides support for Bluetooth and Wi-Fi.

For additional libraries, you can check the [list of repositories] in the [esp-rs organization].

[esp-lp-hal]: ../esp-lp-hal/
[esp-wifi]: https://github.com/esp-rs/esp-wifi
[list of repositories]: https://github.com/orgs/esp-rs/repositories
[esp-rs organization]: https://github.com/esp-rs

## Examples

Examples demonstrating the use of various peripherals and features of the HAL are available in the [examples] package.

[examples]: ../examples/

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
