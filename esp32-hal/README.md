# esp32-hal

[![Crates.io](https://img.shields.io/crates/v/esp32-hal.svg)](https://crates.io/crates/esp32-hal)
[![Docs](https://docs.rs/esp32-hal/badge.svg)](https://docs.rs/esp32-hal/)
![Crates.io](https://img.shields.io/crates/l/esp32-hal)

`no_std` HAL for the ESP32 from Espressif. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the Xtensa ISA, which is not officially supported by the Rust compiler. In order to develop for this device you must use the Rust compiler fork with Xtensa support found at [esp-rs/rust](https://github.com/esp-rs/rust). Refer to the [Getting Stared](#getting-started) section below for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32-hal/

## Getting Started

### Installing the Rust Compiler

Pre-built compilers are available for most common operating systems and architectures via the [esp-rs/rust-build](https://github.com/esp-rs/rust-build) repository. This repository additionally provides scripts to simplify the installation process.

For _Linux_ or _macOS_:

```shell
$ curl -LO https://raw.githubusercontent.com/esp-rs/rust-build/main/install-rust-toolchain.sh
$ chmod +x install-rust-toolchain.sh
$ ./install-rust-toolchain.sh
```

For _Windows_:

```shell
PS> Invoke-WebRequest https://raw.githubusercontent.com/esp-rs/rust-build/main/Install-RustToolchain.ps1 -OutFile Install-RustToolchain.ps1
PS> .\Install-RustToolchain.ps1
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
