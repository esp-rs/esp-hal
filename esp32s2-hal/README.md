# esp32s2-hal

[![Crates.io](https://img.shields.io/crates/v/esp32s2-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp32s2-hal)
[![docs.rs](https://img.shields.io/docsrs/esp32s2-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp32s2-hal)
![Crates.io](https://img.shields.io/crates/l/esp32s2-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

`no_std` HAL for the ESP32-S2 from Espressif. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the Xtensa ISA, which is not officially supported by the Rust compiler. In order to develop for this device, you must use the Rust compiler fork with Xtensa support found at [esp-rs/rust](https://github.com/esp-rs/rust). Refer to the [Getting Started](#getting-started) section below for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32s2-hal/

## Getting Started

### Installing the Rust Compiler

```sh
cargo install espup
espup install
# Unix
. $HOME/export-esp.sh
# Windows does not require sourcing any file
```

See the [Installation chapter of The Rust on ESP Book](https://esp-rs.github.io/book/installation/installation.html#installing-rust-for-espressif-socs) for more details.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
